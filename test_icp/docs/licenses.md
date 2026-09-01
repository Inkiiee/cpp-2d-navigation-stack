# 사용 라이브러리 라이선스 정리


## 요약

| # | 라이브러리 | 라이선스 | 필수 여부 | 상업적 사용 |
|---|-----------|---------|----------|------------|
| 1 | ROS 2 (rclcpp, sensor_msgs, nav_msgs, geometry_msgs) | Apache 2.0 | 필수 | ✅ 허용 |
| 2 | Eigen3 | MPL 2.0 | 필수 | ✅ 허용 |
| 3 | Qt 6 (Core, Gui, Widgets) | LGPL 3.0 / GPL 3.0 / 상업 | 필수 | ⚠️ 조건부 |
| 4 | g2o | BSD 2-Clause | 선택 | ✅ 허용 |
| 5 | nanoflann | BSD 2-Clause | 필수 (내장) | ✅ 허용 |
| 6 | ament_cmake | Apache 2.0 | 빌드 도구 | ✅ 허용 |
| 7 | ament_lint_auto / ament_lint_common | Apache 2.0 | 테스트 전용 | ✅ 허용 |


## 상세 정보

---

### 1. ROS 2 (Humble Hawksbill)

- **구성 요소**: `rclcpp`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `ament_cmake`
- **라이선스**: Apache License 2.0
- **저작권**: Open Robotics
- **링크**: https://github.com/ros2

#### Apache 2.0 주요 조건
- 소스 코드 공개 의무 **없음**
- 저작권 표시 및 라이선스 고지 필요
- 특허 사용 허가 포함
- 수정본에 변경 사항 명시 필요

---

### 2. Eigen3

- **라이선스**: Mozilla Public License 2.0 (MPL 2.0)
- **저작권**: Benoît Jacob, Gaël Guennebaud 외
- **링크**: https://eigen.tuxfamily.org
- **프로젝트 내 사용**: SVD 분해, 희소 행렬 연산, LDLT 분해, 포즈 그래프 최적화

#### MPL 2.0 주요 조건
- Eigen 소스 파일 자체를 수정하면 해당 파일만 공개 의무
- Eigen을 사용하는 자체 코드의 공개 의무 **없음**
- 저작권 표시 필요
- 상업적 사용 허용

---

### 3. Qt 6 (Core, Gui, Widgets)

- **라이선스**: LGPL 3.0 / GPL 3.0 / 상업 라이선스 (선택)
- **저작권**: The Qt Company
- **링크**: https://www.qt.io/licensing
- **프로젝트 내 사용**: GUI 시각화 (QPainter, QWidget), 시그널-슬롯 (QObject), 스레딩 (QThread)

#### LGPL 3.0 주요 조건 (오픈소스 사용 시)
- Qt 라이브러리를 **동적 링크**하면 자체 코드 공개 의무 없음
- Qt 소스 수정 시 수정 부분 공개 필요
- 사용자가 Qt 라이브러리를 교체할 수 있어야 함
- LGPL 고지 및 Qt 라이선스 텍스트 포함 필요

#### ⚠️ 주의사항
- **정적 링크** 시 LGPL 조건이 복잡해짐 (오브젝트 파일 제공 필요)
- 상업용 배포 시 LGPL 조건 준수 또는 상업 라이선스 구매 권장
- `CMAKE_AUTOMOC` 사용으로 moc 생성 코드가 포함됨

---

### 4. g2o (General Graph Optimization)

- **라이선스**: BSD 2-Clause ("Simplified")
- **저작권**: Giorgio Grisetti, Rainer Kümmerle, Wolfram Burgard 외
- **링크**: https://github.com/RainerKuemmerle/g2o
- **프로젝트 내 사용**: 포즈 그래프 최적화 (Levenberg-Marquardt), 선택적 사용 (`TEST_ICP_HAS_G2O`)
- **사용 모듈**: `g2o_core`, `g2o_stuff`, `g2o_types_slam2d`, `g2o_solver_eigen`

#### BSD 2-Clause 주요 조건
- 소스 코드 공개 의무 **없음**
- 저작권 표시 및 라이선스 고지 필요
- 매우 자유로운 라이선스

---

### 5. nanoflann

- **라이선스**: BSD 2-Clause
- **저작권**: Jose Luis Blanco-Claraco, Pranjal Kumar Rai
- **버전**: 1.2.3 (프로젝트에 내장)
- **링크**: https://github.com/jlblancoc/nanoflann
- **프로젝트 내 사용**: KDTree 최근접 탐색 (ICP 대응점 매칭)
- **포함 방식**: 헤더 파일 직접 포함 (`include/nanoflann.h`)

#### BSD 2-Clause 주요 조건
- 소스 코드 공개 의무 **없음**
- 저작권 표시 및 라이선스 고지 필요

---

### 6. ament_lint_auto / ament_lint_common

- **라이선스**: Apache License 2.0
- **용도**: 테스트 전용 (`<test_depend>`)
- **링크**: https://github.com/ament/ament_lint
- 배포 바이너리에 포함되지 않으므로 라이선스 고지 불필요

---


## 라이선스 호환성

```
Apache 2.0 (ROS2)  ──┐
BSD 2-Clause (g2o)  ──┤
BSD 2-Clause (nanoflann)──┼──▶  LGPL 3.0 호환  ──▶  배포 가능
MPL 2.0 (Eigen)  ─────┘
LGPL 3.0 (Qt)  ──────────┘
```

모든 라이선스가 LGPL 3.0과 호환되므로, Qt의 LGPL 조건만 준수하면 전체 프로젝트를 배포할 수 있다.


## 배포 시 필수 조치

1. **라이선스 파일 포함**: 각 라이브러리의 LICENSE/COPYING 파일을 배포물에 포함
2. **저작권 고지**: README 또는 About 화면에 사용 라이브러리 목록과 저작권 표시
3. **Qt 동적 링크**: Qt를 동적 링크(`.so` / `.dll`)로 사용하여 LGPL 조건 충족
4. **nanoflann 고지**: 내장된 헤더 파일 상단의 BSD 라이선스 텍스트 유지
5. **Eigen 고지**: MPL 2.0 라이선스 텍스트 포함
