# 🤖 amr_ws: Defense-Grade Ground Autonomy Framework

> **GPS-Denied 환경에서의 신뢰성 있는 주행을 위한 LiDAR 로컬라이제이션 및 MPC 기반 AMR 제어 프레임워크**

본 프로젝트는 일반적인 상업용 AMR을 넘어 특수 목적(GPS 불가능 환경)에서도 운용 가능한 수준의 높은 신뢰성과 안전성을 목표로 합니다.

---

## 🎯 Project Highlights
* **GPS-Denied Autonomy:** LiDAR 기반 Localization(slam_toolbox)과 EKF 센서 퓨전을 통해 GPS 없이도 정밀한 위치 추정을 수행합니다.
* **MPC-based Control:** OSQP 솔버를 이용한 모델 예측 제어(MPC)를 통해 제약 조건(속도/가속도/조향)을 준수하며 최적의 경로를 추종합니다.
* **Defense-Grade Reliability:** 통신 지연, 센서 데이터 손실 등 극한 상황에 대비한 **Fail-safe State Machine**과 **Watchdog** 시스템을 구축합니다.
* **Performance Metrics:** 단순히 "동작"하는 것을 넘어, RMSE(오차), Latency(지연), 제어 주기 등을 수치화하여 성능을 증명합니다.

---

## 🛠️ Technology Stack
* **OS/Middleware:** Ubuntu 22.04 / ROS2 Humble (rclcpp)
* **Language:** Modern C++ (17/20), Python 3.10
* **Optimization:** OSQP (Quadratic Programming), Eigen
* **Estimation/SLAM:** EKF (robot_localization), slam_toolbox
* **Simulation:** Gazebo Fortress, RViz2

---

## 📅 Development Roadmap (12-Week Plan)

### Phase 1: Foundation & Estimation (Weeks 1-4)
* 아키텍처 설계 및 GitHub 환경 구축
* UGV 동역학 모델링 및 기본 EKF(IMU+Odom) 융합

### Phase 2: LiDAR SLAM & Localization (Weeks 5-8)
* slam_toolbox 기반 2D SLAM 및 Map-based Localization 구현
* GPS-denied 환경에서의 Global Correction 알고리즘 검증

### Phase 3: Advanced Control & Safety (Weeks 9-12)
* Map Frame 기반 MPC 설계 및 정적/동적 장애물 회피 구현
* Fail-safe State Machine 통합 및 통신/센서 이상 상황 대응 테스트

---

## 📊 Performance Indicators (KPI)
* **Tracking Error:** Reference Trajectory 대비 RMSE 최소화
* **Control Frequency:** MPC 제어 주기 50Hz 안정 유지
* **Safety:** 극한 상황(Sensor Dropout) 발생 시 100% 안전 정지 보장