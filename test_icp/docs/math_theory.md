# 수학 이론 정리

이 문서는 `test_icp` SLAM 시스템에서 사용하는 수학 이론을 정리한다.


## 1. 2D 좌표 변환 (Rigid Body Transform)

### 1.1 회전 행렬 (Rotation Matrix)

2D 평면에서 각도 $\theta$만큼 회전하는 회전 행렬은 다음과 같다.

$$
R(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}
$$

**사용 위치**: `slam_basic.cpp::rotationAndTranslation`, `scan_match.cpp::rotation`

### 1.2 Rigid Body Transform (회전 + 평행이동)

점 $\mathbf{p} = (x, y)$를 회전 $\theta$와 평행이동 $(t_x, t_y)$로 변환:

$$
\mathbf{p'} = R(\theta) \cdot \mathbf{p} + \mathbf{t} = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix} \begin{bmatrix} x \\ y \end{bmatrix} + \begin{bmatrix} t_x \\ t_y \end{bmatrix}
$$

동차 좌표(homogeneous coordinates)로 표현하면:

$$
T = \begin{bmatrix} \cos\theta & -\sin\theta & t_x \\ \sin\theta & \cos\theta & t_y \\ 0 & 0 & 1 \end{bmatrix}, \quad \begin{bmatrix} x' \\ y' \\ 1 \end{bmatrix} = T \cdot \begin{bmatrix} x \\ y \\ 1 \end{bmatrix}
$$

**사용 위치**: `slam_basic.cpp::rotationAndTranslation`, `runICP`의 `T_total` 누적

### 1.3 역변환 (Inverse Transform)

Pose $T = (t_x, t_y, \theta)$의 역변환:

$$
T^{-1} = \begin{bmatrix} R^T & -R^T \mathbf{t} \\ 0 & 1 \end{bmatrix}
$$

구체적으로:

$$
t_x^{inv} = -(\cos\theta \cdot t_x + \sin\theta \cdot t_y)
$$

$$
t_y^{inv} = -(-\sin\theta \cdot t_x + \cos\theta \cdot t_y)
$$

$$
\theta^{inv} = -\theta
$$

**사용 위치**: `slam_basic.cpp::inversePose`

### 1.4 상대 Pose 계산 (Relative Pose)

Pose A에서 Pose B까지의 상대 변환:

$$
\Delta \mathbf{t} = R(\theta_A)^T \cdot (\mathbf{t}_B - \mathbf{t}_A)
$$

$$
\Delta\theta = \theta_B - \theta_A
$$

즉, A의 local 좌표계에서 본 B의 위치와 방향차이다.

**사용 위치**: `slam_basic.cpp::relativePose`


## 2. 쿼터니언 → 오일러 변환 (Quaternion to Euler)

ROS2의 오도메트리/IMU 데이터는 쿼터니언 $(q_x, q_y, q_z, q_w)$으로 회전을 표현한다.

### 2.1 Yaw (Z축 회전) 추출

$$
\text{yaw} = \text{atan2}(2(q_w q_z + q_x q_y), \; 1 - 2(q_y^2 + q_z^2))
$$

### 2.2 Pitch (Y축 회전) 추출

$$
\text{pitch} = \arcsin(\text{clamp}(2(q_w q_y - q_z q_x), \; -1, \; 1))
$$

### 2.3 Roll (X축 회전) 추출

$$
\text{roll} = \text{atan2}(2(q_w q_x + q_y q_z), \; 1 - 2(q_x^2 + q_y^2))
$$

**사용 위치**: `bridge.cpp::quaternionToYaw`, `bridge.cpp::emitOdomDataReceived`


## 3. 각도 정규화 (Angle Normalization)

각도를 $[-\pi, \pi]$ 범위로 정규화:

$$
\text{normalize}(\alpha) = \alpha - 2\pi \cdot \text{round}\left(\frac{\alpha}{2\pi}\right)
$$

구현에서는 반복 빼기 방식:

```
while (α > π):  α -= 2π
while (α < -π): α += 2π
```

**사용 위치**: `slam_basic.cpp::normalizeAngle`, `bridge.cpp::normalizeAngle`


## 4. ICP (Iterative Closest Point)

두 점군(point cloud) 사이의 최적 변환을 반복적으로 찾는 알고리즘이다.

### 4.1 Point-to-Point ICP (SVD 기반)

**목적**: source 점군 $A$를 target 점군 $B$에 정합(align)하는 $R, \mathbf{t}$를 찾는다.

**매 반복에서의 풀이**:

1. KDTree로 각 source 점의 최근접 target 점 탐색
2. 두 점군의 중심(centroid) 계산:

$$
\bar{\mathbf{a}} = \frac{1}{N}\sum_{i} \mathbf{a}_i, \quad \bar{\mathbf{b}} = \frac{1}{N}\sum_{i} \mathbf{b}_i
$$

3. 중심 정렬 후 cross-covariance 행렬 $H$ 계산:

$$
H = \sum_{i} (\mathbf{a}_i - \bar{\mathbf{a}})(\mathbf{b}_i - \bar{\mathbf{b}})^T
$$

4. SVD 분해: $H = U \Sigma V^T$

5. 최적 회전: $R = V U^T$
   - 만약 $\det(R) < 0$이면 $V$의 마지막 열 부호 반전

6. 최적 평행이동: $\mathbf{t} = \bar{\mathbf{b}} - R \bar{\mathbf{a}}$

**수렴 조건**: 평균 오차의 상대 변화량이 $\epsilon$ 미만

**사용 위치**: `scan_match.cpp::runICP`

### 4.2 Point-to-Point ICP (Gauss-Newton)

비선형 최소자승법으로 $\xi = (t_x, t_y, \theta)$를 직접 최적화한다.

**에러 함수**: 각 점 $i$에 대해

$$
\mathbf{e}_i = R(\theta) \mathbf{p}_i + \mathbf{t} - \mathbf{q}_i
$$

여기서 $\mathbf{q}_i$는 최근접 target 점이다.

**야코비안** (2×3):

$$
J_i = \begin{bmatrix} 1 & 0 & -\sin\theta \cdot p_x^i - \cos\theta \cdot p_y^i \\ 0 & 1 & \cos\theta \cdot p_x^i - \sin\theta \cdot p_y^i \end{bmatrix}
$$

**정규방정식** (Gauss-Newton):

$$
H \Delta\xi = -\mathbf{b}
$$

$$
H = \sum_i J_i^T J_i, \quad \mathbf{b} = \sum_i J_i^T \mathbf{e}_i
$$

$$
\Delta\xi = -H^{-1}\mathbf{b}
$$

LDLT 분해로 $\Delta\xi$를 풀고, $\xi \leftarrow \xi + \Delta\xi$로 갱신한다.

**사용 위치**: `scan_match.cpp::runGauseNewtonICP`

### 4.3 Point-to-Plane ICP (Gauss-Newton)

목표 점에서의 **법선 벡터**를 이용해 정합한다.

**법선 추정**: K-nearest neighbors의 공분산 행렬에 대한 고유분해(eigendecomposition)에서 최소 고유값에 대응하는 고유벡터를 법선으로 사용.

$$
\Sigma = \sum_j (\mathbf{n}_j - \bar{\mathbf{n}})(\mathbf{n}_j - \bar{\mathbf{n}})^T
$$

$$
\Sigma = Q \Lambda Q^T \implies \mathbf{n} = Q \text{의 최소 고유값 열}
$$

**에러 함수** (스칼라):

$$
e_i = \mathbf{n}_i^T \left( R(\theta)\mathbf{p}_i + \mathbf{t} - \mathbf{q}_i \right)
$$

**야코비안** (1×3):

$$
J_i = \begin{bmatrix} n_x & n_y & (-\sin\theta \cdot p_x - \cos\theta \cdot p_y) n_x + (\cos\theta \cdot p_x - \sin\theta \cdot p_y) n_y \end{bmatrix}
$$

이후 Gauss-Newton과 동일하게 풀되, step size 제한을 적용한다.

**사용 위치**: `scan_match.cpp::runGauseNewtonICP2`


## 5. NDT (Normal Distributions Transform)

점군을 격자 셀(grid cell)로 나누고, 각 셀의 점 분포를 정규 분포로 모델링한다.

### 5.1 셀 통계량 계산

각 셀 $c$에 속한 점들 $\{\mathbf{x}_j\}$에 대해:

**평균**:

$$
\boldsymbol{\mu}_c = \frac{1}{|c|} \sum_j \mathbf{x}_j
$$

**공분산**:

$$
\Sigma_c = \frac{1}{|c|-1} \sum_j (\mathbf{x}_j - \boldsymbol{\mu}_c)(\mathbf{x}_j - \boldsymbol{\mu}_c)^T
$$

퇴화(degenerate) 방지: $\det(\Sigma_c) < 10^{-6}$이면 해당 셀 제외.

### 5.2 비용 함수

변환된 source 점 $\mathbf{p}'_i = R(\theta)\mathbf{p}_i + \mathbf{t}$가 속한 셀의 정규 분포에 대한 마할라노비스 거리(Mahalanobis distance):

$$
\mathcal{L} = \sum_i \mathbf{r}_i^T \Sigma_c^{-1} \mathbf{r}_i, \quad \mathbf{r}_i = \mathbf{p}'_i - \boldsymbol{\mu}_c
$$

### 5.3 야코비안 (NDT)

최적화 변수는 $\xi = (\theta, t_x, t_y)$ 순서다.

$$
J_i = \frac{\partial \mathbf{p}'_i}{\partial \xi} = \begin{bmatrix} -\sin\theta \cdot p_x - \cos\theta \cdot p_y & 1 & 0 \\ \cos\theta \cdot p_x - \sin\theta \cdot p_y & 0 & 1 \end{bmatrix}
$$

**정규방정식**:

$$
H = \sum_i J_i^T \Sigma_c^{-1} J_i, \quad \mathbf{b} = \sum_i J_i^T \Sigma_c^{-1} \mathbf{r}_i
$$

$$
\Delta\xi = H^{-1}\mathbf{b}
$$

step size 클리핑 후 $\xi \leftarrow \xi - \Delta\xi$로 갱신한다.

### 5.4 멀티 해상도 NDT

coarse-to-fine 전략으로 해상도 $[1.0, 0.5, 0.3, 0.1, 0.05]$을 순차 적용.
이전 해상도의 결과를 다음 해상도의 초기값으로 사용한다.

**사용 위치**: `scan_match.cpp::runNDT`, `scan_match.cpp::runNDTAndGetBestPose`


## 6. CSM (Correlative Scan Matching)

격자 기반 전수 탐색(brute-force search)으로 최적 pose를 찾는다.

### 6.1 Lookup Table (LUT)

레퍼런스 점군을 가우시안 커널로 smear하여 2D 격자에 저장한다.

$$
\text{LUT}(g_x, g_y) = \max_i \exp\left(-\frac{d_i^2}{2\sigma^2}\right)
$$

여기서 $d_i$는 격자 중심과 레퍼런스 점 사이의 거리, $\sigma$는 smear 파라미터다.

### 6.2 후보 평가

각 후보 pose $(t_x + \Delta x, t_y + \Delta y, \theta + \Delta\theta)$에 대해 scan 점들을 변환 후 LUT 값을 합산:

$$
\text{score} = \sum_i \text{LUT}(\cos\theta' \cdot s_x^i - \sin\theta' \cdot s_y^i + t_x', \;\; \sin\theta' \cdot s_x^i + \cos\theta' \cdot s_y^i + t_y')
$$

### 6.3 Coarse-to-Fine 탐색

1. **Coarse pass**: 넓은 범위, 큰 간격, 다운샘플된 scan (every 3rd point)
2. **Fine pass**: coarse 최적값 주변, 작은 간격, 전체 scan 사용

정수 좌표 변환으로 inner loop에서 부동소수점 연산을 최소화한다.

**사용 위치**: `scan_match.cpp::buildLookupTable`, `scan_match.cpp::runCSM`


## 7. Pose Graph 최적화

### 7.1 그래프 구조

- **노드**: 각 submap의 global pose $\xi_i = (t_x^i, t_y^i, \theta^i)$
- **오도메트리 엣지**: 연속 노드 간 상대 변환 (낮은 정보 행렬)
- **루프 엣지**: loop closure로 감지된 상대 변환 (높은 정보 행렬)

### 7.2 에러 함수

엣지 $(i, j)$에 대한 에러:

$$
\mathbf{e}_{ij} = \text{relative}(\xi_i, \xi_j) - \mathbf{z}_{ij}
$$

여기서 $\text{relative}$는 예측된 상대 pose, $\mathbf{z}_{ij}$는 측정된 상대 pose다.

### 7.3 정보 행렬 (Information Matrix)

각 엣지에 weight를 부여하는 대각 행렬:

$$
\Omega_{ij} = \text{diag}(\omega_{tx}, \omega_{ty}, \omega_\theta)
$$

loop 엣지는 일반적으로 큰 값 (100), odom 엣지는 작은 값 (1)을 사용한다.

### 7.4 Gauss-Newton 최적화 (Fallback)

전체 비용 함수:

$$
\mathcal{F} = \sum_{(i,j)} \mathbf{e}_{ij}^T \Omega_{ij} \mathbf{e}_{ij}
$$

야코비안 $A = \frac{\partial \mathbf{e}}{\partial \xi_i}$, $B = \frac{\partial \mathbf{e}}{\partial \xi_j}$를 각 엣지에서 계산하고:

$$
H = \sum_{(i,j)} \begin{bmatrix} A^T \Omega A & A^T \Omega B \\ B^T \Omega A & B^T \Omega B \end{bmatrix}, \quad \mathbf{b} = \sum_{(i,j)} \begin{bmatrix} A^T \Omega \mathbf{e} \\ B^T \Omega \mathbf{e} \end{bmatrix}
$$

#### 야코비안 A, B의 구체적 형태

$$
A = \begin{bmatrix} -\cos\theta_i & -\sin\theta_i & \sin\theta_i \cdot dx - \cos\theta_i \cdot dy \\ \sin\theta_i & -\cos\theta_i & \cos\theta_i \cdot dx + \sin\theta_i \cdot dy \\ 0 & 0 & -1 \end{bmatrix}
$$

$$
B = \begin{bmatrix} \cos\theta_i & \sin\theta_i & 0 \\ -\sin\theta_i & \cos\theta_i & 0 \\ 0 & 0 & 1 \end{bmatrix}
$$

$H$는 **sparse matrix** (`Eigen::SparseMatrix`)로 구성하고, `SimplicialLDLT`로 해를 구한다.
첫 번째 노드는 앵커링(고정)을 위해 대각에 큰 값($10^6$)을 추가한다.

**사용 위치**: `my_pose_graph.cpp::loopOptimize` (fallback 경로)

### 7.5 g2o 최적화 경로

`TEST_ICP_HAS_G2O` 매크로가 정의되면 g2o 라이브러리를 사용한다.

- **solver**: `OptimizationAlgorithmLevenberg` + `LinearSolverEigen`
- **vertex**: `VertexSE2` (2D pose)
- **edge**: `EdgeSE2` (상대 pose 제약)
- **robust kernel**: loop edge에 `RobustKernelHuber` ($\delta = 1.0$)
  - Huber 비용: $\rho(e) = \begin{cases} \frac{1}{2}e^2 & |e| \leq \delta \\ \delta(|e| - \frac{1}{2}\delta) & |e| > \delta \end{cases}$

**사용 위치**: `my_pose_graph.cpp::loopOptimize` (g2o 경로)


## 8. Occupancy Grid Map

### 8.1 격자 인덱싱

연속 좌표 $(x, y)$를 해상도 $r$의 격자 인덱스로 변환:

$$
g_x = \lceil x / r \rceil, \quad g_y = \lceil y / r \rceil
$$

### 8.2 Hit / Miss 모델

각 셀은 `hit_count`과 `miss_count`를 유지한다.

- **hit**: scan endpoint가 도달한 셀
- **miss**: 센서에서 endpoint까지 ray가 통과한 셀 (Bresenham ray tracing)

점유 확률 근사:

$$
P(\text{occupied}) \approx \frac{\text{hit\_count}}{\text{hit\_count} + \text{miss\_count}}
$$

Static 셀 판정: $P(\text{occupied}) \geq 0.55$

상한값(kMaxCount=20)으로 최근 관측에 더 높은 비중을 부여한다.

### 8.3 Bresenham 직선 알고리즘 (Ray Tracing)

센서 위치에서 endpoint까지 2D 격자 위의 직선을 효율적으로 trace한다.

정수 연산만으로 격자 셀을 순회:

```
dx = |x1 - x0|, dy = |y1 - y0|
err = dx - dy
while (x0, y0) ≠ (x1, y1):
    process(x0, y0)
    err2 = 2 * err
    if err2 > -dy:  err -= dy, x0 += sx
    if err2 <  dx:  err += dx, y0 += sy
```

**사용 위치**: `map_backend.cpp::markFreeRay`, `map_backend.cpp::rebuildFromSubmaps`


## 9. KDTree (K-Dimensional Tree)

최근접 이웃 탐색(Nearest Neighbor Search)을 위한 공간 분할 자료구조다.

### 9.1 구성

`nanoflann` 헤더 온리 라이브러리를 사용. L2 거리 기반 2D KDTree를 leaf size=10으로 구성한다.

### 9.2 거리 척도

$$
d^2(\mathbf{p}, \mathbf{q}) = (p_x - q_x)^2 + (p_y - q_y)^2
$$

### 9.3 KNN 탐색

query 점에서 가장 가까운 $k$개 점을 탐색한다. ICP/NDT에서 대응점(correspondence) 찾기에 사용된다.

**사용 위치**: `scan_match.cpp::buildKDTree`, `scan_match.cpp::knnSearch`


## 10. 스캔 Deskew (Motion Compensation)

LiDAR 스캔 중 로봇이 이동하면 각 빔의 시간이 다르므로 왜곡이 발생한다.

### 10.1 보간 (Interpolation)

각 빔의 타임스탬프에서의 로봇 pose를 선형 보간(LERP)으로 추정:

**위치 보간**:

$$
\mathbf{p}(t) = \mathbf{p}_a + \frac{t - t_a}{t_b - t_a}(\mathbf{p}_b - \mathbf{p}_a)
$$

**각도 보간** (wrap-around 고려):

$$
\theta(t) = \text{normalize}\left(\theta_a + \frac{t - t_a}{t_b - t_a} \cdot \text{normalize}(\theta_b - \theta_a)\right)
$$

### 10.2 Deskew 절차

1. 스캔 종료 시점의 reference pose $T_{\text{ref}}$를 구한다
2. 각 빔의 시점에서의 pose $T_i$를 보간한다
3. 빔을 $T_i$의 world 좌표로 변환한다
4. $T_{\text{ref}}$의 local 좌표계로 역변환한다

$$
\mathbf{p}^{\text{ref}}_i = R(\theta_{\text{ref}})^T \left( T_i \cdot \mathbf{p}^{\text{local}}_i - \mathbf{t}_{\text{ref}} \right)
$$

**사용 위치**: `bridge.cpp::deskewScan`


## 11. RMSE (Root Mean Square Error)

두 점군 사이의 정합 품질 측정:

$$
\text{RMSE} = \sqrt{\frac{1}{N}\sum_{i=1}^{N} \|\mathbf{p}'_i - \text{NN}(\mathbf{p}'_i)\|^2}
$$

여기서 $\text{NN}(\cdot)$은 KDTree 기반 최근접 점이다.

**사용 위치**: `scan_match.cpp::cal_rmse`


## 12. 해싱 (Hash Function)

`std::pair<int,int>` 키를 위한 해시 함수:

$$
h(a, b) = \text{hash}(a) \oplus (\text{hash}(b) \times 2654435761)
$$

$2654435761 \approx 2^{32} / \phi$ (피보나치 해싱 상수)로, 해시 충돌을 줄인다.

**사용 위치**: `slam_basic.h::PairHash`
