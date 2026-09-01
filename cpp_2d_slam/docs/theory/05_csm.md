# CSM (Correlative Scan Matching)


## 개요

CSM은 가능한 모든 후보 pose를 격자 위에서 **전수 탐색(brute-force)**하여 가장 높은 점수의 pose를 찾는 스캔 매칭 알고리즘이다.

Karto SLAM (SRI International)에서 사용된 방식이며, Olson (2009)의 Real-Time Correlative Scan Matching에 기초한다.


## ICP/NDT와의 비교

| 항목 | ICP/NDT | CSM |
|------|---------|-----|
| 방식 | 반복적 최적화 (gradient 기반) | 전수 탐색 (enumerate) |
| 초기값 의존성 | **높음** (지역 최소해 위험) | **낮음** (탐색 범위 내 전역 최적) |
| 정밀도 | 높음 | 격자 해상도에 의존 |
| 속도 | 빠름 (수렴 시) | 느림 (탐색 범위에 비례) |
| 역할 | Fine alignment | **Coarse alignment** |

→ 이 프로젝트에서는 CSM을 coarse 초기값으로 사용하고, NDT로 fine 보정하는 2단계 구조를 사용한다.


## 원리

### 1단계: Lookup Table (LUT) 구성

레퍼런스 점군을 2D 격자에 가우시안으로 "번져서(smear)" 저장한다.

격자 해상도 $r$, smear 파라미터 $\sigma$에 대해:

$$
\text{LUT}(g_x, g_y) = \max_i \exp\left(-\frac{d_i^2}{2\sigma^2}\right)
$$

여기서 $d_i$는 격자 중심과 레퍼런스 점 $i$ 사이의 거리다.

#### 가우시안 커널 (Gaussian Kernel)

각 레퍼런스 점 주변에 가우시안 모양의 "방울"을 찍는다:

$$
G(\Delta x, \Delta y) = \exp\left(-\frac{(\Delta x)^2 + (\Delta y)^2}{2\sigma^2}\right)
$$

커널 반경: $3\sigma / r$ 셀 (99.7% 커버)

```
LUT 시각화 (1D 단면):
  1.0  ┃     ╱╲
       ┃    ╱  ╲
  0.5  ┃   ╱    ╲
       ┃  ╱      ╲
  0.0  ┗━╱━━━━━━━━╲━━━
         점 위치
```

#### max 연산

같은 셀에 여러 점의 커널이 겹치면 **최댓값**을 취한다. 점이 밀집된 영역이 과도하게 높은 점수를 받는 것을 방지하기 위함이다.


### 2단계: 후보 Pose 평가

scan 점군을 후보 pose $(t_x + \Delta x, t_y + \Delta y, \theta + \Delta\theta)$로 변환하고, 변환된 점들의 LUT 값을 합산:

$$
\text{score}(\xi) = \sum_{i=1}^N \text{LUT}\left(\cos\theta' s_x^i - \sin\theta' s_y^i + t_x', \;\; \sin\theta' s_x^i + \cos\theta' s_y^i + t_y'\right)
$$

score가 가장 높은 $\xi$가 최적 결과다.


### 3단계: Coarse-to-Fine 탐색

전수 탐색의 계산량을 줄이기 위해 2단계로 나눈다.

#### Coarse Pass

| 파라미터 | 의미 |
|----------|------|
| `search_xy` | XY 탐색 범위 (±0.3~0.5m) |
| `search_theta` | θ 탐색 범위 (±0.2~0.35rad) |
| `coarse_xy_res` | XY 격자 간격 (0.05m) |
| `coarse_angle_res` | θ 간격 (0.0175rad ≈ 1°) |

- scan을 3개당 1개로 **다운샘플**하여 속도 향상
- 최적 coarse pose를 찾음

#### Fine Pass

| 파라미터 | 의미 |
|----------|------|
| 탐색 범위 | coarse 간격 이내 (±`coarse_xy_res`, ±`coarse_angle_res`) |
| `fine_xy_res` | 0.005m |
| `fine_angle_res` | 0.00175rad ≈ 0.1° |

- **전체 scan 점** 사용
- coarse 최적값 주변에서 정밀 탐색

```
Coarse (넓고 성김):         Fine (좁고 조밀):
┌─────────────────┐        ┌───┐
│  ·  ·  ·  ·  · │        │···│
│  ·  ·  ★  ·  · │  →     │·★·│
│  ·  ·  ·  ·  · │        │···│
└─────────────────┘        └───┘
```


## 정수 좌표 최적화

Inner loop에서 부동소수점 연산을 줄이기 위해 **사전 회전(pre-rotation)**과 **정수 격자 좌표**를 사용한다.

각 $\theta$ 후보에 대해:
1. 모든 scan 점을 회전하여 정수 격자 좌표로 변환 (1회)
2. $(dx, dy)$ 루프에서는 정수 offset만 더함 (부동소수점 곱셈 없음)

```cpp
// Pre-rotation (θ당 1회)
grid_x[i] = round((cos_t * scan_x[i] - sin_t * scan_y[i]) * inv_res);

// Inner loop (dx, dy당 — 정수 연산만)
int gx = grid_x[i] + off_x;
score += lut_data[gy * W + gx];
```


## 탐색 공간 크기

총 후보 수:

$$
N_{\text{candidates}} = \frac{2 \cdot \text{search\_xy}}{\text{xy\_res}} \times \frac{2 \cdot \text{search\_xy}}{\text{xy\_res}} \times \frac{2 \cdot \text{search\_theta}}{\text{angle\_res}}
$$

Coarse 예시: $(0.6/0.05)^2 \times (0.4/0.0175) \approx 144 \times 23 \approx 3,300$ 후보

Fine 예시: $(0.1/0.005)^2 \times (0.035/0.00175) \approx 400 \times 20 \approx 8,000$ 후보

각 후보에서 scan 점 수만큼 LUT 조회가 필요하므로, 다운샘플링과 정수 좌표 최적화가 중요하다.


## 코드 위치

- `scan_match.cpp::buildLookupTable` — LUT 구성
- `scan_match.cpp::scoreCandidate` — 단일 후보 평가
- `scan_match.cpp::runCSM` — Coarse-to-Fine 탐색
