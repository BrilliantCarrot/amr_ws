#!/usr/bin/env python3
"""
monte_carlo.py — W12 Monte Carlo 통합 테스트 자동화 스크립트

[사전 준비]
  1. bringup을 먼저 수동으로 실행해두어야 함:
     ros2 launch scenarios bringup.launch.py \
       use_planner:=true goal_x:=0.0 goal_y:=4.5 \
       world:=monte_carlo_room mock_link:=true drop_rate:=0.0

  2. slam_toolbox(localization 모드)도 먼저 실행:
     ros2 launch localization slam.launch.py

  3. 이 스크립트 실행:
     python3 monte_carlo.py

[동작 원리]
  매 trial마다:
    1. 로봇 위치 초기화 (ign service set_pose)
    2. 노드들 재시작
    3. 미션 감시 (목표 도달 / 충돌 / 타임아웃)
    4. 시나리오 이벤트 주입 (통신 차단 / localization lost)
    5. 결과 기록

[결과]
  ~/amr_ws/monte_carlo_results.csv 로 저장
"""

import subprocess
import time
import csv
import os
import signal
import threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from amr_msgs.msg import MinObstacleDistance, SafetyStatus, PoseRmse
import random

# ================================================================
# ★★★ 설정 섹션 — 여기만 수정 ★★★
# ================================================================

# --- 디버그 모드 ---
# True: 노드 터미널 출력 표시 (처음 테스트 시 권장)
# False: 모든 출력 숨김 (본 실험 시)
DEBUG_MODE = True  # ← 추가

# --- 반복 횟수 ---
NUM_TRIALS = 50

# --- 타임아웃 ---
TRIAL_TIMEOUT_SEC = 120.0   # 이 시간 초과 시 해당 trial 실패 처리 [s]

# --- 목표점 ---
GOAL_X = 0.0
GOAL_Y = 4.5

# --- 성공 판정 ---
GOAL_TOLERANCE_M   = 0.3    # 목표 도달 판정 거리 [m]
COLLISION_THRESH_M = -0.2   # 충돌 판정 min_clearance 임계값 [m]

# --- 통신 차단 이벤트 ---
COMM_FAIL_ENABLED      = True   # True: 활성화 / False: 비활성화
COMM_FAIL_START_SEC    = 15.0   # trial 시작 후 이벤트 시작 시각 [s]
COMM_FAIL_DURATION_SEC = 3.0    # 통신 차단 지속 시간 [s] (이후 자동 복구)

# --- localization lost 이벤트 ---
LOC_LOST_ENABLED       = True   # True: 활성화 / False: 비활성화
LOC_LOST_START_SEC     = 25.0   # trial 시작 후 pkill 시각 [s]
LOC_LOST_RECOVERY_SEC  = 1.0   # pkill 후 slam_toolbox 재시작까지 대기 [s]
# 예: t=55s에 pkill → t=65s에 slam.launch.py 재시작

# --- 동적 장애물 지연 시작 ---
OBS_CONTROLLER_ENABLED   = True   # True: 활성화 / False: 비활성화
OBS_CONTROLLER_START_MIN = 30.0   # 시작 최소 시각 [s]
OBS_CONTROLLER_START_MAX = 40.0   # 시작 최대 시각 [s]

# --- 결과 저장 경로 ---
RESULT_CSV = os.path.expanduser("~/amr_ws/monte_carlo_results.csv")

# --- 로봇 스폰 위치 (set_pose로 초기화) ---
ROBOT_INIT_X = 0.0
ROBOT_INIT_Y = 0.0
ROBOT_INIT_Z = 0.1

# ================================================================
# 노드 실행 명령어 목록
# (bringup, slam_toolbox는 이미 실행 중이므로 제외)
# ================================================================
NODE_COMMANDS = [
    # ekf_node (odom-only, 비교 기준)
    ["ros2", "run", "estimation", "ekf_node"],
    # map_ekf_node (LiDAR fusion, MPC 입력)
    ["ros2", "run", "estimation", "map_ekf_node"],
    # localization_monitor_node
    ["ros2", "run", "localization", "localization_monitor_node"],
    # watchdog_node
    ["ros2", "run", "safety", "watchdog_node"],
    # state_machine_node
    ["ros2", "run", "safety", "state_machine_node"],
    # pose_rmse_node
    ["ros2", "run", "estimation", "pose_rmse_node"],
    # obstacle_tracker_node
    ["ros2", "run", "control_mpc", "obstacle_tracker_node"],
    # obstacle_controller_node (동적 장애물 제어)
    # ["ros2", "run", "scenarios", "obstacle_controller_node.py"],
    # mpc_node
    ["ros2", "run", "control_mpc", "mpc_node",
     "--ros-args",
     "-p", "use_global_planner:=true",
     "-p", f"v_ref_:=0.1",
     "-p", f"v_max:=0.2"],
]

# ================================================================
# 미션 감시 노드 — rclpy로 토픽 구독
# ================================================================
class MissionMonitor(Node):
    """
    미션 진행 상황을 실시간으로 감시하는 ROS2 노드.
    - /map_ekf/odom: 로봇 현재 위치 → 목표 도달 판정
    - /metrics/min_obstacle_distance: 충돌 판정
    - /safety/state: SAFE_STOP 감시
    """
    def __init__(self):
        super().__init__("monte_carlo_monitor")

        # 상태 변수
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.min_clearance = 9999.0
        self.mean_rmse = 0.0 # 멤버 변수 + 구독 추가
        self.rmse_samples = []
        self.safety_state = 0       # 0=NORMAL, 2=SAFE_STOP
        self.mission_done = False   # 목표 도달 여부
        self.collision    = False   # 충돌 여부
        self.safe_stop_count = 0    # SAFE_STOP 전환 횟수
        self.safe_stop_time  = None # SAFE_STOP 진입 시각
        self.recovery_time   = None # SAFE_STOP → NORMAL 복귀 소요 시간

        # 구독
        self.odom_sub = self.create_subscription(
            Odometry, "/map_ekf/odom",
            self._odom_cb, 10)
        self.dist_sub = self.create_subscription(
            MinObstacleDistance, "/metrics/min_obstacle_distance",
            self._dist_cb, 10)
        self.safety_sub = self.create_subscription(
            SafetyStatus, "/safety/state",
            self._safety_cb, 10)
        self.create_subscription(PoseRmse, "/metrics/pose_rmse",
                                        self._rmse_cb, 10)        

    def _odom_cb(self, msg):
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y
        # 목표 도달 판정
        dist = ((self.cur_x - GOAL_X)**2 + (self.cur_y - GOAL_Y)**2)**0.5
        if dist < GOAL_TOLERANCE_M:
            self.mission_done = True

    def _dist_cb(self, msg):
        if msg.min_distance_m < self.min_clearance:
            self.min_clearance = msg.min_distance_m
        # 충돌 판정
        if msg.min_distance_m < COLLISION_THRESH_M:
            self.collision = True

    def _rmse_cb(self, msg):
        self.rmse_samples.append(msg.rmse_total)
        if self.rmse_samples:
            self.mean_rmse = sum(self.rmse_samples) / len(self.rmse_samples)

    def _safety_cb(self, msg):
        prev = self.safety_state
        self.safety_state = msg.state
        # # NORMAL → SAFE_STOP 전환
        # if prev != 2 and msg.state == 2:
        #     self.safe_stop_count += 1
        #     self.safe_stop_time = time.time()
        # # SAFE_STOP → NORMAL 복귀
        # if prev == 2 and msg.state == 0 and self.safe_stop_time is not None:
        #     self.recovery_time = time.time() - self.safe_stop_time
        if prev != 2 and msg.state == 2:
            self.safe_stop_count += 1
            self.safe_stop_time = time.time()
            print(f"  [Monitor] ⚠ SAFE_STOP 진입 감지 (count={self.safe_stop_count})")
        if prev == 2 and msg.state == 0 and self.safe_stop_time is not None:
            self.recovery_time = time.time() - self.safe_stop_time
            print(f"  [Monitor] ✅ NORMAL 복귀 감지 (recovery={self.recovery_time:.1f}s)")

    def reset(self):
        """매 trial 시작 전 상태 초기화"""
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.min_clearance = 9999.0
        self.safety_state  = 0
        self.mission_done  = False
        self.collision     = False
        self.safe_stop_count = 0
        self.safe_stop_time  = None
        self.recovery_time   = None
        self.mean_rmse     = 0.0
        self.rmse_samples  = []

# ================================================================
# 헬퍼 함수
# ================================================================

def run_cmd(cmd, wait=True, timeout=None):
    """명령어 실행. wait=False이면 백그라운드 프로세스 반환."""
    # bash -c로 감싸서 ROS2 환경 source 보장
    # subprocess는 ~/.bashrc를 자동으로 읽지 않으므로 명시적으로 source 필요
    bash_cmd = ["/bin/bash", "-c",
                "source /opt/ros/humble/setup.bash && "
                "source /home/lyj/amr_ws/install/setup.bash && "
                + " ".join(cmd)]
    stdout = None if DEBUG_MODE else subprocess.DEVNULL
    stderr = None if DEBUG_MODE else subprocess.DEVNULL
    if wait:
        subprocess.run(cmd, timeout=timeout,
                       stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)
    else:
        return subprocess.Popen(cmd,
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)


def reset_robot_pose():
    """로봇을 초기 위치(0,0)으로 set_pose 이동."""
    req = (
        f'name: "amr_robot" '
        f'position {{x: {ROBOT_INIT_X} y: {ROBOT_INIT_Y} z: {ROBOT_INIT_Z}}} '
        f'orientation {{w: 1.0}}'
    )
    subprocess.run([
        "ign", "service",
        "-s", "/world/simple_room/set_pose",
        "--reqtype", "ignition.msgs.Pose",
        "--reptype", "ignition.msgs.Boolean",
        "--timeout", "3000",
        "--req", req,
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5.0)


def set_drop_rate(rate: float):
    """mock_link_node drop_rate 파라미터 변경."""

    subprocess.run([
        "ros2", "param", "set",
        "/mock_link_node", "drop_rate", str(rate)
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)



def kill_slam():
    """slam_toolbox 프로세스 종료."""
    set_drop_rate(1.0)
    subprocess.run(["pkill", "-f", "slam_toolbox"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def restart_slam():
    # """slam_toolbox localization 모드 재시작. 백그라운드로 실행."""
    # proc = subprocess.Popen([
    #     "ros2", "launch", "localization", "slam.launch.py"
    # ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    # time.sleep(3.0)
    # set_drop_rate(0.0)
    # return proc
    """slam_toolbox 재시작. 통신 복구는 미션 루프에서 타이머로 처리."""
    proc = subprocess.Popen([
        "ros2", "launch", "localization", "slam.launch.py"
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return proc


def start_nodes():
    """NODE_COMMANDS의 모든 노드를 백그라운드로 시작. 프로세스 리스트 반환."""
    procs = []
    for cmd in NODE_COMMANDS:
        p = run_cmd(cmd, wait=False)
        procs.append(p)
        time.sleep(0.3)  # 노드 순차 기동 (짧은 대기)
    return procs


def stop_nodes(procs):
    """모든 노드 프로세스 종료."""
    for p in procs:
        try:
            p.terminate()
            p.wait(timeout=3.0)
        except Exception:
            try:
                p.kill()
            except Exception:
                pass
    # ros2 run으로 실행된 노드 잔여 프로세스 정리
    subprocess.run(["pkill", "-f", "ekf_node"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "map_ekf_node"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "localization_monitor_node"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "watchdog_node"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "state_machine_node"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "pose_rmse_node"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)    
    subprocess.run(["pkill", "-f", "obstacle_tracker_node"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "obstacle_controller_node.py"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "mpc_node"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "param set"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)    


# ================================================================
# 한 trial 실행
# ================================================================

def run_trial(trial_id: int, monitor: MissionMonitor) -> dict:
    """
    한 번의 trial을 실행하고 결과 딕셔너리를 반환.
    결과 키: trial, result, duration, min_clearance,
             safe_stop_count, recovery_time_sec, events
    """
    print(f"\n{'='*50}")
    print(f"[Trial {trial_id:02d}/{NUM_TRIALS}] 시작")
    print(f"{'='*50}")

    # --- 1. 초기화 ---
    monitor.reset()

    # 매 trial 시작 시 slam 완전 재시작 (이전 trial 오염 상태 초기화)
    print(f"  [Trial {trial_id}] slam 재시작 중...")
    subprocess.run(["pkill", "-f", "slam_toolbox"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)
    subprocess.Popen(
        ["ros2", "launch", "localization", "slam.launch.py"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(5.0)


    reset_robot_pose()
    time.sleep(1.0)  # set_pose 반영 대기

    # drop_rate 초기화
    set_drop_rate(0.0)

    # --- 2. 노드 시작 ---
    print(f"  [Trial {trial_id}] 노드 시작 중...")
    procs = start_nodes()
    time.sleep(8.0)  # 노드 초기화 대기 (map_ekf, localization 안정화)

    # run_trial() 안, t_start 선언 위에
    obs_start_sec = round(random.uniform(
        OBS_CONTROLLER_START_MIN, OBS_CONTROLLER_START_MAX), 1)
    obs_started = False

    # --- 3. 미션 감시 루프 ---
    t_start   = time.time()
    events    = []          # 이 trial에서 주입된 이벤트 목록
    slam_proc = None        # slam 재시작 프로세스 핸들
    comm_fired = False      # 통신 이벤트 발동 여부
    loc_fired  = False      # localization 이벤트 발동 여부
    loc_killed = False      # pkill 완료 여부

    print(f"  [Trial {trial_id}] 미션 감시 시작 (timeout={TRIAL_TIMEOUT_SEC}s)")

    while True:
        elapsed = time.time() - t_start

        # rclpy 콜백 처리 (비블로킹)
        rclpy.spin_once(monitor, timeout_sec=0.05)

        # ── 목표 도달 → 성공 ──────────────────────────────────
        if monitor.mission_done:
            duration = time.time() - t_start
            print(f"  [Trial {trial_id}] ✅ 목표 도달! ({duration:.1f}s)")
            result = "SUCCESS"
            set_drop_rate(1.0)
            break

        # ── 충돌 → 실패 ───────────────────────────────────────
        if monitor.collision:
            duration = time.time() - t_start
            print(f"  [Trial {trial_id}] ❌ 충돌 감지! "
                  f"min_clearance={monitor.min_clearance:.3f}m ({duration:.1f}s)")
            result = "COLLISION"
            break

        # ── 타임아웃 → 실패 ───────────────────────────────────
        if elapsed > TRIAL_TIMEOUT_SEC:
            duration = elapsed
            print(f"  [Trial {trial_id}] ⏱ 타임아웃! ({duration:.1f}s)")
            result = "TIMEOUT"
            break

        # ── 동적 장애물 지연 시작 ─────────────────────────────────
        if OBS_CONTROLLER_ENABLED and not obs_started:
            if elapsed >= obs_start_sec:
                print(f"  [Trial {trial_id}] 🟢 동적 장애물 시작 (t={elapsed:.1f}s)")
                p = subprocess.Popen(
                    ["/bin/bash", "-c",
                    "source /opt/ros/humble/setup.bash && "
                    "source /home/lyj/amr_ws/install/setup.bash && "
                    "ros2 run scenarios obstacle_controller_node.py"],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                procs.append(p)
                obs_started = True

        # ── 이벤트 1: 통신 차단 ───────────────────────────────
        if COMM_FAIL_ENABLED and not comm_fired:
            if elapsed >= COMM_FAIL_START_SEC:
                print(f"  [Trial {trial_id}] 📡 통신 차단 주입 "
                      f"(t={elapsed:.1f}s, {COMM_FAIL_DURATION_SEC}s간)")
                set_drop_rate(1.0)
                events.append(f"COMM_FAIL@{elapsed:.1f}s")
                comm_fired = True
                monitor.safe_stop_count += 1
                monitor.safe_stop_time = time.time()                

                # COMM_FAIL_DURATION_SEC 후 복구 (별도 스레드)
                def restore_comm():
                    time.sleep(COMM_FAIL_DURATION_SEC)
                    set_drop_rate(0.0)
                    print(f"  [Trial {trial_id}] 📡 통신 복구")

                threading.Thread(target=restore_comm, daemon=True).start()

        # ── 이벤트 2: localization lost ───────────────────────
        if LOC_LOST_ENABLED and not loc_fired:
            if elapsed >= LOC_LOST_START_SEC:
                print(f"  [Trial {trial_id}] 🗺 localization lost 주입 "
                      f"(t={elapsed:.1f}s)")
                kill_slam()
                events.append(f"LOC_LOST@{elapsed:.1f}s")
                loc_fired  = True
                loc_killed = True
                monitor.safe_stop_count += 1
                monitor.safe_stop_time = time.time()                

        # slam 재시작 (pkill 후 LOC_LOST_RECOVERY_SEC 경과)
        # if loc_killed and slam_proc is None:
        #     if elapsed >= LOC_LOST_START_SEC + LOC_LOST_RECOVERY_SEC:
        #         print(f"  [Trial {trial_id}] 🗺 slam 재시작 "
        #               f"(t={elapsed:.1f}s)")
        #         slam_proc = restart_slam()
        #         events.append(f"SLAM_RESTART@{elapsed:.1f}s")
        #         if monitor.safe_stop_time is not None:
        #             monitor.recovery_time = time.time() - monitor.safe_stop_time
        #             print(f"  [Monitor] ✅ 재획득 시간: {monitor.recovery_time:.1f}s")

        # ── slam 재시작 ───────────────────────────────────────
        if loc_killed and slam_proc is None:
            if elapsed >= LOC_LOST_START_SEC + LOC_LOST_RECOVERY_SEC:
                print(f"  [Trial {trial_id}] 🗺 slam 재시작 (t={elapsed:.1f}s)")
                slam_proc = restart_slam()
                events.append(f"SLAM_RESTART@{elapsed:.1f}s")

                # 통신 복구를 3초 후 비블로킹으로 처리
                # loc_start_sec을 명시적으로 캡처해서 클로저 버그 방지
                def delayed_comm_restore(t_id=trial_id, mon=monitor):
                    time.sleep(3.0)
                    set_drop_rate(0.0)
                    print(f"  [Trial {t_id}] 📡 통신 복구 (slam 안정화 후)")
                    if mon.safe_stop_time is not None:
                        mon.recovery_time = time.time() - mon.safe_stop_time
                        print(f"  [Monitor] ✅ 재획득 시간: {mon.recovery_time:.1f}s")

                threading.Thread(target=delayed_comm_restore, daemon=True).start()

        time.sleep(0.1)

    # --- 4. 정리
    stop_nodes(procs)
    time.sleep(1.0)
    set_drop_rate(0.0)   # 통신 복구
    if slam_proc is not None:
        try:
            slam_proc.terminate()
        except Exception:
            pass

    if loc_killed and trial_id < NUM_TRIALS:
        print(f"  [Trial {trial_id}] 🗺 다음 trial용 slam 재시작")
        subprocess.Popen(
            ["ros2", "launch", "localization", "slam.launch.py"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(5.0)

    time.sleep(2.0)  # 노드 완전 종료 대기

    # --- 5. 결과 딕셔너리 ---
    return {
        "trial"            : trial_id,
        "result"           : result,
        "duration_sec"     : round(duration, 1),
        "min_clearance_m"  : round(monitor.min_clearance, 3),
        "mean_rmse_m"       : round(monitor.mean_rmse, 4),        
        "safe_stop_count"  : monitor.safe_stop_count,
        "recovery_time_sec": round(monitor.recovery_time, 1)
                             if monitor.recovery_time is not None else "-",
        "events"           : "|".join(events) if events else "none",
    }


# ================================================================
# 메인
# ================================================================

def main():
    rclpy.init()
    monitor = MissionMonitor()

    results = []

    print("=" * 60)
    print("  W12 Monte Carlo 통합 테스트")
    print(f"  trials={NUM_TRIALS}, timeout={TRIAL_TIMEOUT_SEC}s")
    print(f"  COMM_FAIL: {'ON' if COMM_FAIL_ENABLED else 'OFF'} "
          f"(t={COMM_FAIL_START_SEC}s, dur={COMM_FAIL_DURATION_SEC}s)")
    print(f"  LOC_LOST:  {'ON' if LOC_LOST_ENABLED else 'OFF'} "
          f"(t={LOC_LOST_START_SEC}s, recovery={LOC_LOST_RECOVERY_SEC}s)")
    print("=" * 60)

    for trial_id in range(1, NUM_TRIALS + 1):
        result = run_trial(trial_id, monitor)
        results.append(result)

        # 중간 통계 출력
        successes = sum(1 for r in results if r["result"] == "SUCCESS")
        print(f"  → 중간 성공률: {successes}/{trial_id} "
              f"({successes/trial_id*100:.1f}%)")

    # --- 최종 결과 저장 ---
    with open(RESULT_CSV, "w", newline="") as f:
        fieldnames = ["trial", "result", "duration_sec",
                      "min_clearance_m", "mean_rmse_m", "safe_stop_count",
                      "recovery_time_sec", "events"]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)

    # --- 최종 통계 출력 ---
    total     = len(results)
    successes = sum(1 for r in results if r["result"] == "SUCCESS")
    collisions= sum(1 for r in results if r["result"] == "COLLISION")
    timeouts  = sum(1 for r in results if r["result"] == "TIMEOUT")

    durations = [r["duration_sec"] for r in results
                 if r["result"] == "SUCCESS"]
    avg_dur   = sum(durations) / len(durations) if durations else 0

    clearances = [r["min_clearance_m"] for r in results]
    avg_clr    = sum(clearances) / len(clearances) if clearances else 0

    recovery_times = [r["recovery_time_sec"] for r in results
                      if isinstance(r["recovery_time_sec"], float)]
    avg_rec = (sum(recovery_times) / len(recovery_times)
               if recovery_times else 0)

    print("\n" + "=" * 60)
    print("  ★ Monte Carlo 최종 결과 ★")
    print("=" * 60)
    print(f"  총 시행:      {total}회")
    print(f"  성공:         {successes}회 ({successes/total*100:.1f}%)")
    print(f"  충돌 실패:    {collisions}회")
    print(f"  타임아웃 실패:{timeouts}회")
    print(f"  평균 소요시간:{avg_dur:.1f}s (성공 기준)")
    print(f"  평균 min_clearance: {avg_clr:.3f}m")
    print(f"  평균 재획득 시간:   {avg_rec:.1f}s")
    print(f"  결과 저장: {RESULT_CSV}")
    print("=" * 60)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
