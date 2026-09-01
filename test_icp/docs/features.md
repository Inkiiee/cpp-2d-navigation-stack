# 기능 정리

이 문서는 `test_icp` SLAM 시스템의 기능을 모듈별로 정리한다.


## 1. 센서 데이터 수신

### 1.1 LiDAR 스캔 수신 (`LaserScan`)

| 항목 | 내용 |
|------|------|
| 토픽 | `/scan` |
| 메시지 타입 | `sensor_msgs/msg/LaserScan` |
| QoS | `SensorDataQoS` |
| 처리 | 극좌표 → 직교좌표 변환, deskew 적용 후 Bridge 전달 |

**극좌표 → 직교좌표 변환**:
```
x = cos(angle) × range
y = sin(angle) × range
```

유효 범위 필터링: `isfinite(r) && r >= range_min && r <= range_max`

### 1.2 오도메트리 수신 (`OdomLoader`)

| 항목 | 내용 |
|------|------|
| 토픽 | `/odom` |
| 메시지 타입 | `nav_msgs/msg/Odometry` |
| 처리 | 쿼터니언→오일러 변환 후 Bridge 전달 |

### 1.3 IMU 수신 (`ImuLoader`)

| 항목 | 내용 |
|------|------|
| 토픽 | `/imu` |
| 메시지 타입 | `sensor_msgs/msg/Imu` |
| 처리 | 쿼터니언→yaw 변환 후 Bridge 전달 |


## 2. 센서 브리지 (`Bridge`)

ROS2 데이터를 Qt signal/slot 시스템으로 전달하는 중개 계층이다.

### 2.1 스캔 Deskew (Motion Compensation)

| 항목 | 내용 |
|------|------|
| 입력 | LaserScan 메시지 + odom 히스토리 |
| 출력 | 왜곡 보정된 (x, y) 점군 |
| 방법 | 각 빔 시점의 pose를 보간하여 ref 프레임으로 역변환 |

- odom/IMU 히스토리를 최대 400 샘플까지 유지
- 선형 보간으로 임의 시점의 pose/yaw 추정
- 보간 실패 시 raw 극좌표 변환으로 fallback

### 2.2 Signal 목록

| Signal | 용도 |
|--------|------|
| `scanDataReceived(xs, ys)` | 보정된 scan 점군 전달 |
| `odomDataReceived(x, y, z, rx, ry, rz)` | 오일러 변환된 odom 전달 |
| `imuHeadingReceived(yaw)` | IMU heading 전달 |


## 3. 온라인 위치 추정 (`ScanMatchBackend`)

매 LiDAR 프레임마다 현재 로봇 pose를 추정하고, 맵을 관리한다.

### 3.1 오도메트리 적분

| 항목 | 내용 |
|------|------|
| 방법 | odom 변위를 현재 heading 기준으로 world 프레임에 투영 |
| 갱신 대상 | `map_x`, `map_y`, `map_theta` |

```
local_dx = cos(odom_θ) × Δx + sin(odom_θ) × Δy
local_dy = -sin(odom_θ) × Δx + cos(odom_θ) × Δy
map_x += cos(map_θ) × local_dx - sin(map_θ) × local_dy
map_y += sin(map_θ) × local_dx + cos(map_θ) × local_dy
map_θ += Δθ
```

### 3.2 스캔 매칭 파이프라인

```
scan 수신
  ├─ 이동량 부족 → local_map 누적만
  └─ 이동량 충분 ─┬─ reference map 구성 (local_map + world_map 근처 점)
                   ├─ CSM 필요? ─ Yes → LUT 빌드 + CSM → NDT 정밀 보정
                   │              └ No  → NDT만 수행
                   ├─ 점프 제한 (α=0.3 가중 평균)
                   └─ pose 갱신 + local_map 누적
```

### 3.3 거리 게이팅

| 파라미터 | 값 | 설명 |
|----------|-----|------|
| `kMinTravelDistance` | 0.01m | 최소 이동 거리 |
| `kMinTravelAngle` | 0.005rad | 최소 회전 각도 |

이동량이 부족하면 스캔 매칭을 건너뛰고 누적만 수행한다.

### 3.4 CSM 호출 정책

| 조건 | 처리 |
|------|------|
| 이동 > 0.15m 또는 회전 > 0.1rad 또는 LUT 무효 | CSM → NDT |
| 그 외 | NDT만 |

### 3.5 점프 제한

매칭 결과가 odom 예측과 크게 차이나면 급격한 보정을 방지한다.

| 조건 | 처리 |
|------|------|
| 이동 > 0.3m 또는 회전 > 0.15rad | α=0.3으로 가중 평균 |

### 3.6 Submap 생성

| 파라미터 | 값 |
|----------|-----|
| 생성 주기 | 10 프레임마다 |

생성 절차:
1. local_map의 점군을 world 좌표로 읽음
2. world_map에 occupancy 반영 (`updateOccupancyMap`)
3. 현재 pose의 역변환을 적용해 local 좌표 submap으로 저장
4. pose graph에 노드 추가
5. 이전 노드와 odom 엣지 연결
6. local_map 초기화

### 3.7 캐싱 전략

| 캐시 | 무효화 조건 | 용도 |
|------|------------|------|
| `cached_lut_` | 맵 변경 또는 CSM 호출 시 | LUT 재활용 |
| `cached_world_x/y_` | 맵 변경 또는 0.5m 이상 이동 | world_map ref 점군 |


## 4. 스캔 매칭 알고리즘 (`ScanMatcher`)

### 4.1 구현된 알고리즘

| 알고리즘 | 함수 | 용도 |
|----------|------|------|
| SVD ICP (Point-to-Point) | `runICP` | SVD 기반 정합 |
| Gauss-Newton ICP (Point-to-Point) | `runGauseNewtonICP` | 비선형 최적화 기반 |
| Gauss-Newton ICP (Point-to-Plane) | `runGauseNewtonICP2` | 법선 기반 정밀 정합 |
| NDT | `runNDT` | 정규 분포 기반 정합 |
| Multi-Resolution NDT | `runNDTAndGetBestPose` | coarse-to-fine NDT |
| CSM | `runCSM` | 격자 전수 탐색 |

### 4.2 기본 파라미터

| 파라미터 | ICP/GN-ICP | NDT | CSM |
|----------|-----------|-----|-----|
| maxIter | 100 | 30~100 | N/A |
| epsilon | 1e-3 | 1e-6 | N/A |
| k (이웃 수) | 1 | N/A | N/A |
| resolution | N/A | 0.1 | 0.02 |
| step | N/A | 0.05 | N/A |
| search_xy | N/A | N/A | 0.3~0.5m |
| search_theta | N/A | N/A | 0.2~0.35rad |

### 4.3 품질 평가

| 지표 | 함수 | 설명 |
|------|------|------|
| RMSE | `cal_rmse` | 최근접점까지의 평균 제곱근 오차 |
| Inlier Ratio | `cal_inlier_ratio` | 임계 거리 이내 대응점 비율 |
| CSM Score | `scoreCandidate` | LUT 기반 매칭 점수 |


## 5. Occupancy Grid Map (`MapBackend`)

### 5.1 핵심 기능

| 기능 | 함수 | 설명 |
|------|------|------|
| 점 추가 | `addPos` | hit 카운트 증가 |
| 점유 업데이트 | `updateOccupancyMap` | hit + ray tracing miss |
| 점 조회 | `getPos` | 전체 또는 static 셀만 |
| 인접 조회 | `getAdjacentPos` | 반경 내 점만 |
| 다운샘플 조회 | `getAdjacentPosDownsampled` | 반경 내 + 해상도 간소화 |
| 맵 재구성 | `rebuildFromSubmaps` | 전체 submap으로 맵 재생성 |

### 5.2 Static Cell 판정

```
occupied_ratio = hit_count / (hit_count + miss_count)
static = (occupied_ratio >= 0.55)
```

### 5.3 Ray Tracing 규칙

- hit 셀에서는 miss를 찍지 않음
- 기존 장애물(hit ≥ 3)을 관통하는 ray는 중단
- hit/miss 각각 상한 20으로 최근 관측에 비중

### 5.4 맵 재구성 최적화

`rebuildFromSubmaps`는 flat 2D 배열 기반으로 Bresenham ray tracing을 수행한다.
hash map 대신 배열을 사용해 O(1) 셀 접근을 달성하고, 마지막에 한 번만 hash map으로 변환한다.


## 6. 루프 클로저 (`LoopDetecter`)

### 6.1 탐지 조건

| 파라미터 | 값 | 설명 |
|----------|-----|------|
| `kLoopDetectionStride` | 2 | 2 프레임마다 탐지 |
| `kMinLoopIndexGap` | 12 | 최소 인덱스 간격 |
| `kCandidateDistanceThreshold` | 1.5m | 후보 거리 임계값 |
| `kMaxCandidatesForRefinement` | 3 | 최대 검증 후보 수 |

### 6.2 검증 파이프라인

```
1. 초기 score 확인 (avg_score >= 0.12)
2. CSM으로 coarse 정합
3. NDT로 fine 정합
4. RMSE < 0.5 AND avg_score > 0.3 → loop edge 등록
```

### 6.3 최적화 트리거

| 파라미터 | 값 |
|----------|-----|
| `kOptimizeEveryNLoopEdges` | 2 |

loop edge가 2개 누적될 때마다 `loopOptimize()` 실행.


## 7. Pose Graph (`PoseGraph`)

### 7.1 데이터 구조

| 요소 | 설명 |
|------|------|
| Node | `(tx, ty, theta)` — 각 submap의 global pose |
| Edge (odom) | 연속 노드 간 상대 변환, info=(1,1,1) |
| Edge (loop) | loop closure 상대 변환, info=(100,100,100) |

### 7.2 최적화 경로

| 경로 | 조건 | solver |
|------|------|--------|
| g2o | `TEST_ICP_HAS_G2O` 정의 시 | Levenberg-Marquardt + LinearSolverEigen |
| fallback | g2o 없을 때 | Sparse Gauss-Newton + SimplicialLDLT |

### 7.3 반복 횟수 제한

```
max_iter = max(3, min(iter, 5000 / N))
```

노드가 많아질수록 반복을 줄여 실시간성을 유지한다.

### 7.4 Robust Kernel

g2o 경로에서 loop edge에 Huber robust kernel (δ=1.0)을 적용하여 잘못된 loop closure의 영향을 완화한다.


## 8. 시각화 (`Painter`)

### 8.1 렌더링 레이어

| 레이어 | 내용 | 색상 |
|--------|------|------|
| `world_pixmap` | world map (static 셀) | 검정 |
| `lader_pixmap` | 최근 LiDAR scan | 초록 |
| `pixmap` | trajectory + 현재 heading | 빨강 |

### 8.2 좌표 매핑

world 좌표 → pixel 좌표 변환 (`worldToPixel`):

| 파라미터 | 값 |
|----------|-----|
| world 범위 | -16m ~ +16m |
| pixel 크기 | 400×400 |

### 8.3 최적화

| 기법 | 설명 |
|------|------|
| World map throttle | 5 프레임마다 world_pixmap 갱신 |
| Scan drop | paint_busy_ 중이면 scan 드롭 |
| Toggle | Lidar 레이어 on/off 버튼 |

### 8.4 로봇 화살표

5×13 점으로 구성된 방향 화살표를 현재 heading으로 회전하여 표시한다.


## 9. 텔레옵 (`MyTelNode` + `KeyInputMon`)

### 9.1 키 매핑

| 키 | 동작 | 범위 |
|----|------|------|
| W/w | 속도 증가 (+0.01) | ≤ 0.22 m/s |
| X/x | 속도 감소 (-0.01) | ≥ -0.22 m/s |
| A/a | 회전 증가 (+0.1) | ≤ 2.84 rad/s |
| D/d | 회전 감소 (-0.1) | ≥ -2.84 rad/s |
| S/s | 정지 (속도=0, 회전=0) | — |
| Q/q | 종료 | — |

### 9.2 퍼블리시

| 항목 | 내용 |
|------|------|
| 토픽 | `cmd_vel` |
| 메시지 타입 | `geometry_msgs/msg/Twist` |
| 주기 | 118ms |


## 10. 쓰레드 구조

### 10.1 쓰레드 배치

| 쓰레드 | 객체 | 역할 |
|--------|------|------|
| t1 (std::thread) | LaserScan, OdomLoader, ImuLoader | ROS2 spin |
| t2 (std::thread) | KeyInputMon | 키보드 입력 |
| t3 (std::thread) | MyTelNode | cmd_vel 퍼블리시 |
| QThread 1 | ScanMatchBackend | 스캔 매칭 + 맵 관리 |
| QThread 2 | LoopDetecter | 루프 탐지 |
| QThread 3 | SlamSystem | 맵 재구성 |
| GUI thread | Painter | Qt 렌더링 |

### 10.2 동기화

| 뮤텍스 | 보호 대상 |
|--------|----------|
| `shared_data_mutex_` | world_map, sub_maps, pose_graph, map_x/y/theta |
| `PoseGraph::mutex_` | pose_history, edges |
| `scan_mutex_` | pending_scan 버퍼 |
| `pose_mutex_` (Bridge) | odom_history, imu_history |

### 10.3 Signal-Slot 연결

| 발신 | Signal | 수신 | Slot | 연결 |
|------|--------|------|------|------|
| Bridge | scanDataReceived | ScanMatchBackend | lidarUpdate | Queued |
| Bridge | odomDataReceived | ScanMatchBackend | odomUpdate | Queued |
| ScanMatchBackend | predictedPose | Painter | predictedPoseUpdate | Queued |
| ScanMatchBackend | scanUpdated | Painter | scanUpdate | Queued |
| ScanMatchBackend | subMapUpdated | LoopDetecter | detectLoop | Queued |
| ScanMatchBackend | rebuildMapRequested | SlamSystem | rebuildMap | Queued |
| LoopDetecter | optimizedPoseUpdated | ScanMatchBackend | poseOptimized | Queued |
