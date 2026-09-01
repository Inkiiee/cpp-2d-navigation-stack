# NDT (Normal Distributions Transform)


## 개요

NDT는 점군을 격자 셀(grid cell)로 분할하고, 각 셀의 점 분포를 **정규 분포(Gaussian)**로 모델링하여 스캔 매칭을 수행하는 알고리즘이다.

2003년 Biber & Straßer가 제안했다.


## ICP와의 차이

| 항목 | ICP | NDT |
|------|-----|-----|
| Target 표현 | 개별 점 | 셀 내 정규 분포 |
| 대응 탐색 | KDTree로 최근접 점 | 점이 속한 셀의 분포 |
| 계산 비용 | 점 수에 비례 | 셀 수에 비례 (보통 훨씬 적음) |
| 노이즈 견딤 | 개별 점 노이즈에 민감 | 분포로 평균화되므로 견고 |


## 동작 원리

### 1단계: 격자 분할 + 통계량 계산

Target 점군을 해상도 $r$의 격자로 나누고, 각 셀 $c$의 통계를 구한다.

**셀 인덱싱**:

$$
g_x = \lceil x / r \rceil, \quad g_y = \lceil y / r \rceil
$$

**평균**:

$$
\boldsymbol{\mu}_c = \frac{1}{|c|} \sum_{j \in c} \mathbf{x}_j
$$

**공분산**:

$$
\Sigma_c = \frac{1}{|c|-1} \sum_{j \in c} (\mathbf{x}_j - \boldsymbol{\mu}_c)(\mathbf{x}_j - \boldsymbol{\mu}_c)^T
$$

셀에 점이 2개 미만이면 제외한다. $\det(\Sigma_c) < 10^{-6}$이면 퇴화(degenerate)로 판단하여 제외한다.

### 정규 분포의 의미

각 셀은 "이 영역에 점이 있을 확률"을 가우시안으로 모델링:

$$
p(\mathbf{x} | c) = \frac{1}{2\pi|\Sigma_c|^{1/2}} \exp\left(-\frac{1}{2}(\mathbf{x} - \boldsymbol{\mu}_c)^T \Sigma_c^{-1} (\mathbf{x} - \boldsymbol{\mu}_c)\right)
$$


### 2단계: 비용 함수 정의

Source 점 $\mathbf{p}_i$를 현재 변환 $\xi = (\theta, t_x, t_y)$로 변환:

$$
\mathbf{p}'_i = R(\theta)\mathbf{p}_i + \mathbf{t}
$$

변환된 점이 속한 셀의 **마할라노비스 거리(Mahalanobis distance)**를 비용으로 사용:

$$
\mathcal{L}(\xi) = \sum_i (\mathbf{p}'_i - \boldsymbol{\mu}_c)^T \Sigma_c^{-1} (\mathbf{p}'_i - \boldsymbol{\mu}_c)
$$

#### 마할라노비스 거리란?

점과 분포 중심 사이의 거리를 공분산으로 보정한 것이다.

$$
d_M(\mathbf{x}, \boldsymbol{\mu}) = \sqrt{(\mathbf{x} - \boldsymbol{\mu})^T \Sigma^{-1} (\mathbf{x} - \boldsymbol{\mu})}
$$

직관적으로:
- 분산이 큰 방향: 같은 거리라도 비용이 **작음** (불확실한 방향이므로 관대)
- 분산이 작은 방향: 같은 거리라도 비용이 **큼** (확실한 방향이므로 엄격)

```
유클리드 거리:         마할라노비스 거리:
    ○ (동심원)             ⬮ (타원)
```


### 3단계: Gauss-Newton 최적화

비용 함수를 $\xi$에 대해 Gauss-Newton으로 최소화한다.

**잔차**: $\mathbf{r}_i = \mathbf{p}'_i - \boldsymbol{\mu}_c \in \mathbb{R}^2$

**야코비안** (2×3):

주의: NDT에서의 변수 순서는 $(\theta, t_x, t_y)$이다.

$$
J_i = \frac{\partial \mathbf{p}'_i}{\partial \xi} = \begin{bmatrix} -\sin\theta \cdot p_x - \cos\theta \cdot p_y & 1 & 0 \\ \cos\theta \cdot p_x - \sin\theta \cdot p_y & 0 & 1 \end{bmatrix}
$$

**정규 방정식** (가중 Gauss-Newton):

$$
H = \sum_i J_i^T \Sigma_c^{-1} J_i
$$

$$
\mathbf{b} = \sum_i J_i^T \Sigma_c^{-1} \mathbf{r}_i
$$

$$
\Delta\xi = H^{-1}\mathbf{b}
$$

**갱신**: $\xi \leftarrow \xi - \Delta\xi$ (gradient descent 방향)

### Step size 제한

- $\|\Delta\xi\| > 1.0$이면 단위 벡터로 스케일링 (큰 점프 방지)
- $\theta$ 변화: `step × 0.3` 이내로 클리핑
- $t_x, t_y$ 변화: `step` 이내로 클리핑


## 멀티 해상도 NDT

하나의 해상도로는 coarse/fine 정합을 동시에 만족하기 어렵다.

**전략**: 큰 해상도 → 작은 해상도 순으로 NDT를 반복 적용

$$
\text{resolutions} = [1.0, \; 0.5, \; 0.3, \; 0.1, \; 0.05] \;\text{m}
$$

각 해상도의 결과를 다음 해상도의 초기값으로 전달한다. NDT 결과의 RMSE가 원래 pose의 RMSE보다 개선되면 즉시 리턴한다.

| 해상도 | 역할 | 셀 크기 |
|--------|------|---------|
| 1.0m | 큰 이동 보정 | 넓음 |
| 0.5m | 중간 보정 | |
| 0.3m | | |
| 0.1m | 정밀 보정 | |
| 0.05m | 미세 정합 | 좁음 |


## 해상도에 따른 특성

| 해상도 | 장점 | 단점 |
|--------|------|------|
| 큰 셀 | 넓은 수렴 영역, 빠름 | 정밀도 낮음 |
| 작은 셀 | 정밀 정합 | 좁은 수렴 영역, 셀이 비거나 퇴화 |

멀티 해상도는 이 trade-off를 coarse-to-fine으로 해결한다.


## 코드 위치

- `scan_match.cpp::runNDT` — 단일 해상도 NDT
- `scan_match.cpp::runNDTAndGetBestPose` — 멀티 해상도 NDT
