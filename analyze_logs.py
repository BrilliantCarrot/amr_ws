import re
import statistics
import os

def analyze_log(file_path):
    solve_times = []
    vs = []
    ws = []
    costs = []

    # 정규표현식: solve, v, w, cost 값 추출
    pattern = re.compile(r"solve=([0-9.]+)ms.*?v=([-0-9.]+)\s+w=([-0-9.]+)\s+\|\s+cost=([-0-9.]+)")

    if not os.path.exists(file_path):
        return None

    with open(file_path, 'r', encoding='utf-8') as f:
        for line in f:
            match = pattern.search(line)
            if match:
                solve_times.append(float(match.group(1)))
                vs.append(float(match.group(2)))
                ws.append(float(match.group(3)))
                costs.append(float(match.group(4)))

    if not solve_times:
        return None

    return {
        'solve_mean': statistics.mean(solve_times),
        'solve_max': max(solve_times),
        'w_max_abs': max(abs(w) for w in ws),
        'w_std': statistics.stdev(ws) if len(ws) > 1 else 0.0, # 조향 떨림(Jitter) 지표
        'cost_mean': statistics.mean(costs)
    }

# 분석할 로그 파일 목록 (파일명이 다르면 여기에 맞게 수정)
files = {
    "1. Baseline": "tune_test1_baseline.txt",
    "2. Aggressive": "tune_test2_aggressive.txt",
    "3. Smooth": "tune_test3_smooth.txt",
    "4. Horizon": "tune_test4_horizon.txt"
}

print("="*105)
print(f"{'Scenario':<15} | {'Avg Solve(ms)':<15} | {'Max Solve(ms)':<15} | {'Max |w|':<10} | {'w StdDev(Jitter)':<18} | {'Avg Cost':<15}")
print("-" * 105)

for name, path in files.items():
    res = analyze_log(path)
    if res:
        print(f"{name:<15} | {res['solve_mean']:<15.2f} | {res['solve_max']:<15.2f} | {res['w_max_abs']:<10.2f} | {res['w_std']:<18.2f} | {res['cost_mean']:<15.2f}")
    else:
        print(f"{name:<15} | File not found or empty ({path})")
print("="*105)