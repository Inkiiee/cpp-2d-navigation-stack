# ICP (Iterative Closest Point)


## 개요

ICP는 두 점군(point cloud) 사이의 최적 변환(회전 + 평행이동)을 반복적으로 찾는 알고리즘이다.

1992년 Besl & McKay가 제안했으며, 로봇 SLAM과 3D 모델 정합의 핵심 알고리즘이다.


## 기본 아이디어

1. Source 점군의 각 점에 대해 Target 점군에서 **가장 가까운 점(closest point)**을 찾는다
2. 이 대응 관계를 이용해 **최적 변환**을 계산한다
3. Source 점군에 변환을 적용한다
4. 수렴할 때까지 1~3을 **반복**한다

```
Source 점군 A ──┐
               ├──→ 대응점 탐색 → 변환 계산 → 적용 → 수렴? ──→ 종료
Target 점군 B ──┘          ↑                           │ No
                           └───────────────────────────┘
```


## 변형 1: Point-to-Point ICP (SVD 기반)

가장 기본적인 형태로, 대응점 사이의 유클리드 거리를 최소화한다.

### 비용 함수

$$
\min_{R, \mathbf{t}} \sum_{i=1}^{N} \| R \mathbf{a}_i + \mathbf{t} - \mathbf{b}_i \|^2
$$

- $\mathbf{a}_i$: source 점
- $\mathbf{b}_i$: $\mathbf{a}_i$의 최근접 target 점

### 풀이 (SVD 방법)

각 반복에서:

1. **중심 계산**: $\bar{\mathbf{a}} = \frac{1}{N}\sum \mathbf{a}_i$, $\bar{\mathbf{b}} = \frac{1}{N}\sum \mathbf{b}_i$

2. **Cross-covariance**: $H = \sum (\mathbf{a}_i - \bar{\mathbf{a}})(\mathbf{b}_i - \bar{\mathbf{b}})^T$

3. **SVD 분해**: $H = U\Sigma V^T$

4. **최적 회전**: $R = VU^T$ (반사 보정: $\det < 0$이면 $V$ 마지막 열 부호 반전)

5. **최적 평행이동**: $\mathbf{t} = \bar{\mathbf{b}} - R\bar{\mathbf{a}}$

6. **점 갱신**: $\mathbf{a}_i \leftarrow R\mathbf{a}_i + \mathbf{t}$

변환을 동차 행렬로 누적: $T_{\text{total}} = T_k \cdot T_{k-1} \cdots T_1$

### 수렴 조건

- 평균 오차의 상대 변화: $\frac{|e_{k-1} - e_k|}{e_{k-1}} < \epsilon$
- 또는 오차 자체가 매우 작음: $e_k < 0.01\epsilon$

### 각도 제한

최종 회전 각도가 $30°$ ($\approx 0.524$ rad) 이상이면 0으로 리셋한다. LiDAR 스캔 간 급격한 회전은 잘못된 대응에 의한 것일 가능성이 높기 때문이다.


## 변형 2: Point-to-Point ICP (Gauss-Newton)

SVD 대신 비선형 최적화로 직접 $(t_x, t_y, \theta)$를 최적화한다.

### 최적화 변수

$$
\xi = (t_x, t_y, \theta) \in \mathbb{R}^3
$$

### 잔차

각 대응 점쌍 $(\mathbf{p}_i, \mathbf{q}_i)$에 대해:

$$
\mathbf{e}_i = R(\theta) \mathbf{p}_i + \mathbf{t} - \mathbf{q}_i \in \mathbb{R}^2
$$

### 야코비안 (2×3)

$$
J_i = \frac{\partial \mathbf{e}_i}{\partial \xi} = \begin{bmatrix} 1 & 0 & -\sin\theta \cdot p_x - \cos\theta \cdot p_y \\ 0 & 1 & \cos\theta \cdot p_x - \sin\theta \cdot p_y \end{bmatrix}
$$

세 번째 열은 $\frac{\partial (R\mathbf{p})}{\partial \theta}$이다.

### 갱신

$$
H = \sum J_i^T J_i, \quad \mathbf{b} = \sum J_i^T \mathbf{e}_i
$$

$$
\Delta\xi = -H^{-1}\mathbf{b}, \quad \xi \leftarrow \xi + \Delta\xi
$$

### SVD ICP와의 비교

| 항목 | SVD ICP | GN ICP |
|------|---------|--------|
| 회전 표현 | 행렬 R 직접 계산 | 각도 θ로 파라미터화 |
| 초기값 | 불필요 (닫힌 해) | 필요 (반복법) |
| 속도 | 매 반복 SVD 분해 | 매 반복 3×3 선형 시스템 |
| 장점 | 전역 최적 (대응 고정 시) | 초기값 있으면 빠른 수렴 |


## 변형 3: Point-to-Plane ICP (Gauss-Newton)

대응점의 **표면 법선(normal)** 방향으로의 거리만 최소화한다.

### 직관

점과 평면 사이의 거리가 점과 점 사이의 거리보다 더 의미 있는 정합 지표인 경우가 많다. 특히 매끈한 벽면을 따라 미끄러지는 방향의 불필요한 제약을 줄여준다.

```
Point-to-Point:           Point-to-Plane:
    •←──────•                •
                              ↓ (법선 방향만)
    ─────────             ─────────── (표면)
```

### 법선 추정

KNN으로 이웃 점을 구하고, 이웃 점들의 **공분산 행렬**을 고유분해:

$$
\Sigma = \sum_j (\mathbf{n}_j - \bar{\mathbf{n}})(\mathbf{n}_j - \bar{\mathbf{n}})^T
$$

$$
\Sigma = Q \Lambda Q^T
$$

**최소 고유값**에 대응하는 고유벡터 = **법선 벡터** $\mathbf{n}$

왜? 점들이 직선(surface) 위에 분포하면, 직선 방향으로 분산이 크고 법선 방향으로 분산이 작다.

### 잔차 (스칼라)

$$
e_i = \mathbf{n}_i^T (R(\theta)\mathbf{p}_i + \mathbf{t} - \mathbf{q}_i)
$$

### 야코비안 (1×3)

$$
J_i = \begin{bmatrix} n_x & n_y & (-\sin\theta \cdot p_x - \cos\theta \cdot p_y) n_x + (\cos\theta \cdot p_x - \sin\theta \cdot p_y) n_y \end{bmatrix}
$$

### Step size 제한

갱신 $\Delta\xi$의 각 성분이 `step`을 넘지 않도록 클리핑한다. Gauss-Newton은 선형 근사이므로 큰 스텝에서 발산할 수 있기 때문이다.


## 대응점 탐색 (Correspondence)

ICP의 성능은 대응점 탐색에 크게 좌우된다.

| 방법 | 설명 | 복잡도 |
|------|------|--------|
| Brute force | 모든 쌍 비교 | $O(N^2)$ |
| **KDTree** | 공간 분할 검색 | $O(N \log N)$ |

이 프로젝트에서는 **nanoflann KDTree**로 $k$-nearest neighbor를 탐색하고, $k$개 이웃의 평균을 대응점으로 사용한다.


## ICP의 한계

| 한계 | 설명 |
|------|------|
| 지역 최소해 | 초기값에 따라 다른 해에 수렴 |
| 부분 겹침 | 한쪽에만 존재하는 점이 잘못된 대응 생성 |
| 대칭 환경 | 복도 등에서 길이 방향 불확실 |

→ 이 프로젝트에서는 **odom prediction**이나 **CSM**으로 좋은 초기값을 먼저 구하고 ICP/NDT를 적용하여 지역 최소해 문제를 완화한다.


## 코드 위치

- `scan_match.cpp::runICP` — SVD 기반 Point-to-Point ICP
- `scan_match.cpp::runGauseNewtonICP` — GN 기반 Point-to-Point ICP
- `scan_match.cpp::runGauseNewtonICP2` — GN 기반 Point-to-Plane ICP
