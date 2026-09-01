# 점유 격자 지도 (Occupancy Grid Map)


## 개요

점유 격자(occupancy grid)는 환경을 일정한 크기의 격자(cell)로 나누고, 각 셀이 **장애물로 점유되었는지** 확률적으로 표현하는 지도 방식이다.

1989년 Moravec & Elfes가 제안했다.


## 격자 구조

### 기본 파라미터

| 파라미터 | 의미 | 이 프로젝트 기본값 |
|---------|------|-------------------|
| resolution | 셀 한 변의 길이 (m) | 0.05 m |
| width × height | 격자 크기 | 동적 확장 |
| origin | 격자 (0,0) 셀의 월드 좌표 | 로봇 초기 위치 기준 |

### 좌표 변환

월드 좌표 $(x_w, y_w)$ → 격자 인덱스 $(i, j)$:

$$
i = \left\lfloor \frac{x_w - x_{origin}}{res} \right\rfloor, \quad
j = \left\lfloor \frac{y_w - y_{origin}}{res} \right\rfloor
$$


## 확률 모델

### 셀 상태

각 셀 $m$은 점유 확률 $P(m)$을 가진다:

| 상태 | 확률 범위 | 의미 |
|------|----------|------|
| Occupied | $P(m) > 0.5$ | 장애물 존재 |
| Free | $P(m) < 0.5$ | 비어 있음 |
| Unknown | $P(m) = 0.5$ | 정보 없음 |


## 로그 오즈 (Log-Odds)

### 동기

확률을 직접 곱하면 수치적으로 불안정하다 (0에 가까운 값끼리 곱셈). 로그 오즈를 사용하면 **덧셈**으로 변환되어 안정적이다.

### 정의

$$
l(m) = \log \frac{P(m)}{1 - P(m)}
$$

| $P(m)$ | $l(m)$ |
|--------|--------|
| 0.5 (unknown) | 0 |
| 0.9 (occupied) | +2.2 |
| 0.1 (free) | -2.2 |
| 1.0 | $+\infty$ |
| 0.0 | $-\infty$ |

### 역변환

$$
P(m) = 1 - \frac{1}{1 + \exp(l(m))}
$$


## 베이즈 업데이트

### 센서 관측 모델

레이저 빔 한 개가 도달했을 때:

- **Hit 셀** (빔 끝점): 점유 확률 증가
- **Miss 셀** (빔 경로): 점유 확률 감소

### 로그 오즈 업데이트

$$
l(m \mid z_{1:t}) = l(m \mid z_{1:t-1}) + l(m \mid z_t) - l_0
$$

- $l(m \mid z_t)$: 현재 관측에 의한 로그 오즈
- $l_0 = \log \frac{P_0}{1 - P_0}$: 사전 확률 (보통 $P_0 = 0.5$이면 $l_0 = 0$)

단순화하면:

$$
l_{new} = l_{old} + \Delta l
$$

| 관측 | $\Delta l$ | 의미 |
|------|-----------|------|
| Hit | $+l_{occ}$ | 점유 확률 증가 |
| Miss | $-l_{free}$ | 점유 확률 감소 |

$l_{occ}$와 $l_{free}$는 센서 신뢰도에 따라 설정하는 파라미터다.


## Clamping

로그 오즈 값이 무한히 커지면 한번 장애물로 판정된 셀이 영원히 바뀌지 않는다. 이를 방지하기 위해 범위를 제한한다:

$$
l_{min} \leq l(m) \leq l_{max}
$$


## Ray Tracing

각 레이저 빔에 대해:

1. 센서 위치 → 격자 인덱스 $(i_s, j_s)$
2. Hit 지점 → 격자 인덱스 $(i_h, j_h)$
3. **Bresenham 알고리즘**으로 $(i_s, j_s)$에서 $(i_h, j_h)$까지 지나는 셀 열거
4. 경로 셀: miss 업데이트 ($l \leftarrow l - l_{free}$)
5. 끝점 셀: hit 업데이트 ($l \leftarrow l + l_{occ}$)

```
  관측 한 번의 업데이트:
  
  ┌───┬───┬───┬───┬───┐
  │   │   │   │   │+▓│  +▓ = hit update (+l_occ)
  ├───┼───┼───┼───┼───┤
  │   │   │-░│-░│   │  -░ = miss update (-l_free)
  ├───┼───┼───┼───┼───┤
  │ S │-░│   │   │   │  S  = sensor position
  └───┴───┴───┴───┴───┘
```


## 서브맵 (Submap) 전략

### 동기

하나의 거대한 전역 맵을 유지하면:
- 메모리 비효율 (대부분의 셀이 unknown)
- 포즈 그래프 최적화 후 맵 전체를 재구축해야 함

### 서브맵 방식

1. 일정 거리/시간마다 새로운 서브맵 생성
2. 각 서브맵은 로컬 좌표계의 작은 occupancy grid
3. 포즈 그래프 최적화 후 각 서브맵을 보정된 포즈로 재배치
4. 전역 맵 = 모든 서브맵의 합성

```
  서브맵 1        서브맵 2        서브맵 3
  ┌────┐        ┌────┐        ┌────┐
  │ ■  │  --->  │  ■ │  --->  │■   │
  │  ■ │        │ ■  │        │ ■  │
  └────┘        └────┘        └────┘
     T₁            T₂            T₃
     
  전역 맵 = T₁·서브맵1 + T₂·서브맵2 + T₃·서브맵3
```


## Hit/Miss 카운트 방식

이 프로젝트에서는 로그 오즈 대신 **hit/miss 카운트**를 사용하는 방식도 구현되어 있다:

$$
P(m) = \frac{n_{hit}}{n_{hit} + n_{miss}}
$$

- 장점: 구현이 단순, 직관적
- 단점: 베이즈 최적성이 보장되지 않음, clamping 불필요


## ROS2 맵 메시지

```
nav_msgs/msg/OccupancyGrid:
  header: stamp, frame_id
  info:
    resolution: float32     # 셀 크기 (m)
    width: uint32           # 가로 셀 수
    height: uint32          # 세로 셀 수
    origin: Pose            # (0,0) 셀의 월드 좌표
  data: int8[]              # 셀 값 배열
```

셀 값:
- `-1`: unknown
- `0`: free
- `100`: occupied
- `0~100`: 점유 확률 (%)


## 시각화

Qt Painter에서 셀 값을 색상으로 변환:
- 점유 (occupied) → 검은색
- 비어 있음 (free) → 흰색
- 미탐색 (unknown) → 회색


## 코드 위치

- `map_backend.h` — 격자 맵 데이터 구조 (`std::unordered_map<PairHash>` 기반)
- `map_backend.cpp` — hit/miss 업데이트, Bresenham ray tracing, 서브맵 합성
- `slam.cpp` — 스캔 데이터로 맵 업데이트 호출
- `painter.cpp` — occupancy grid 시각화 (`drawWorldMap`)
