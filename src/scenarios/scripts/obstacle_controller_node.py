#!/usr/bin/env python3
"""
obstacle_controller_node.py — W9 동적 장애물 위치 제어 노드

[역할]
  ign service /world/simple_room/set_pose 를 통해
  static model의 pose를 주기적으로 갱신함.

[장애물 3종]
  dyn_obs_1  (초록): x=0.5 고정, y축 왕복 0.30 m/s
  dyn_obs_2  (파란): x=1.5 고정, y축 왕복 0.16 m/s (위상 반전)
  sudden_obs (노랑): 남쪽 벽 밖(y=-3.5) 대기 → t=8s에 경로 정면(y=0.0) 출현
                     (지상 대기: 지하 방식이 SceneBroadcaster 오류 유발)
"""

import rclpy
from rclpy.node import Node
import subprocess
import threading


class ObstacleControllerNode(Node):

    def __init__(self):
        super().__init__('obstacle_controller_node')
        self.t_start = self.get_clock().now()
        self.ready = False
        self._obs_turn = 0  # 콜백마다 0→1→0 번갈아 호출

        self.ready_timer = self.create_timer(3.0, self._on_ready_callback)
        self.control_timer = self.create_timer(0.2, self._control_callback)  # 5Hz

        self.get_logger().info('ObstacleControllerNode 시작 — 3초 후 장애물 제어 시작 예정')

    # ---------------------------------------------------------------
    # _on_ready_callback() — 3s 후 제어 활성화 (단발성)
    # ---------------------------------------------------------------
    def _on_ready_callback(self):
        """Gazebo 초기화 완료 확인 후 제어 활성화"""
        self.ready = True
        self.ready_timer.cancel()   # 단발성: 완료 후 즉시 취소
        self.get_logger().info(
            'Gazebo 초기화 대기 완료 — 5Hz 장애물 pose 갱신 시작'
        )

    # ---------------------------------------------------------------
    # _set_pose_async() — 비동기 pose 갱신
    #
    # [설계]
    #   ign service 프로세스 기동: 50~150ms 소요
    #   5Hz 타이머(200ms)에서 3개 동기 호출 시 타이머 지연 발생
    #   → daemon thread로 비동기 처리, 타이머 블로킹 없음
    # ---------------------------------------------------------------
    def _set_pose_sync(self, model_name: str, x: float, y: float, z: float):
        """ign service 동기 순차 호출 (스레드 없음 — 동시 호출 방지)"""
        req = (
            f'name: "{model_name}" '
            f'position {{x: {x:.4f} y: {y:.4f} z: {z:.4f}}} '
            f'orientation {{w: 1.0}}'
        )
        try:
            subprocess.run(
                [
                    'ign', 'service',
                    '-s', '/world/simple_room/set_pose',
                    '--reqtype', 'ignition.msgs.Pose',
                    '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '500',   # 500 → 2000ms
                    '--req', req,
                ],
                capture_output=True,
                timeout=1.0,             # 1.0 → 3.0s
            )
        except subprocess.TimeoutExpired:
            self.get_logger().warn(
                f'set_pose timeout: {model_name}',
                throttle_duration_sec=3.0
            )
        except Exception as e:
            self.get_logger().error(f'set_pose 오류 [{model_name}]: {e}')

    # ---------------------------------------------------------------
    # _control_callback() — 5Hz 제어 루프
    # ---------------------------------------------------------------
    def _control_callback(self):
        if not self.ready:
            return

        elapsed = (self.get_clock().now() - self.t_start).nanoseconds / 1e9

        # dyn_obs_1: y=1.5 고정, x=-2.5 ~ +2.5 왕복 (0.2 m/s)
        # period = 5m / 0.2 m/s = 25s
        period1 = 125.0
        phase1 = (elapsed % period1) / period1
        if phase1 < 0.5:
            x1 = -2.5 + (phase1 / 0.5) * 5.0
        else:
            x1 = 2.5 - ((phase1 - 0.5) / 0.5) * 5.0

        # daemon thread로 비동기 실행 — ROS2 실행기 블로킹 방지
        threading.Thread(
            target=self._set_pose_sync,
            args=('dyn_obs_1', x1, 3.5, 0.4),
            daemon=True
        ).start()

        # self._set_pose_sync('dyn_obs_1', x1, 1.5, 0.4)

        # dyn_obs_2: y=2.5 고정, x=+2.5 ~ -2.5 왕복 (위상 반전)
        # period2 = 25.0
        # phase2 = ((elapsed + 12.5) % period2) / period2
        # if phase2 < 0.5:
        #     x2 = -2.5 + (phase2 / 0.5) * 5.0
        # else:
        #     x2 = 2.5 - ((phase2 - 0.5) / 0.5) * 5.0
        # self._set_pose_sync('dyn_obs_2', x2, 2.5, 0.4)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # 이미 shutdown된 경우 무시


if __name__ == '__main__':
    main()