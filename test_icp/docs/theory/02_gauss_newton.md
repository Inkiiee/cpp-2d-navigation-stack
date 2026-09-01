# Gauss-Newton 최적화


## 배경: 비선형 최소자승 문제

많은 공학 문제는 다음 형태로 귀결된다.

$$
\min_{\mathbf{x}} \sum_{i=1}^{N} \| \mathbf{e}_i(\mathbf{x}) \|^2
$$

여기서 $\mathbf{e}_i(\mathbf{x})$는 잔차(residual)이고, $\mathbf{x}$는 최적화 변수다.

$\mathbf{e}_i$가 $\mathbf{x}$에 대해 비선형이면 닫힌 해가 없으므로, **반복적 선형화**로 풀어야 한다.


## 테일러 전개 기반 선형화

잔차를 현재 추정값 $\mathbf{x}_k$ 근처에서 1차 테일러 전개한다:

$$
\mathbf{e}_i(\mathbf{x}_k + \Delta \mathbf{x}) \approx \mathbf{e}_i(\mathbf{x}_k) + J_i \Delta \mathbf{x}
$$

여기서 $J_i = \frac{\partial \mathbf{e}_i}{\partial \mathbf{x}} \bigg|_{\mathbf{x}_k}$는 야코비안(Jacobian) 행렬이다.


## 정규 방정식 (Normal Equation)

선형화된 잔차를 비용에 대입하면:

$$
\sum_i \| \mathbf{e}_i + J_i \Delta\mathbf{x} \|^2
$$

이를 $\Delta\mathbf{x}$에 대해 미분하여 0으로 놓으면:

$$
\left(\sum_i J_i^T J_i\right) \Delta\mathbf{x} = -\sum_i J_i^T \mathbf{e}_i
$$

간단히 쓰면:

$$
H \Delta\mathbf{x} = -\mathbf{b}
$$

- $H = \sum_i J_i^T J_i$: 근사 Hessian 행렬 (항상 양의 반정치)
- $\mathbf{b} = \sum_i J_i^T \mathbf{e}_i$: gradient 벡터


## 반복 갱신

$$
\mathbf{x}_{k+1} = \mathbf{x}_k + \Delta\mathbf{x}
$$

$\|\Delta\mathbf{x}\| < \epsilon$이면 수렴으로 판단하고 종료한다.


## Gauss-Newton vs Newton's Method

| 항목 | Newton 법 | Gauss-Newton 법 |
|------|----------|----------------|
| Hessian | 실제 2차 미분 $\nabla^2 f$ 필요 | $J^T J$로 근사 (2차 미분 불필요) |
| 계산 비용 | 높음 | 낮음 |
| 수렴 조건 | 2차 수렴 | 잔차가 작을 때 2차 수렴에 가까움 |
| 적용 | 일반 비선형 최적화 | **최소자승 문제 전용** |

Gauss-Newton은 Hessian의 실제 2차항 $\sum_i \mathbf{e}_i \otimes \nabla^2 \mathbf{e}_i$를 무시한다.
잔차 $\mathbf{e}_i$가 최적해 근처에서 작으면 이 근사가 정확하다.


## Levenberg-Marquardt (LM)

Gauss-Newton이 발산하거나 $H$가 특이(singular)할 때를 보완한 변형이다.

$$
(H + \lambda I) \Delta\mathbf{x} = -\mathbf{b}
$$

- $\lambda$ 크면: gradient descent에 가까움 (안전하지만 느림)
- $\lambda$ 작으면: Gauss-Newton에 가까움 (빠르지만 불안정할 수 있음)

$\lambda$를 비용 감소량에 따라 동적으로 조절한다.


## 선형 시스템 풀기

### LDLT 분해

$H$가 양의 반정치(positive semi-definite)이므로 Cholesky 변형인 LDLT 분해를 쓴다:

$$
H = L D L^T
$$

- $L$: 하삼각 행렬 (대각은 1)
- $D$: 대각 행렬

Cholesky보다 수치적으로 안정하고 $H$의 대각이 0일 수 있는 경우에도 동작한다.

### 풀이 순서

$$
L \mathbf{y} = -\mathbf{b} \quad \text{(forward substitution)}
$$

$$
D \mathbf{z} = \mathbf{y} \quad \text{(대각 나눗셈)}
$$

$$
L^T \Delta\mathbf{x} = \mathbf{z} \quad \text{(back substitution)}
$$

시간 복잡도: $O(n^3)$ 분해 + $O(n^2)$ 풀이


## 프로젝트에서의 사용

### Point-to-Point ICP (Gauss-Newton)

최적화 변수: $\xi = (t_x, t_y, \theta)$ (3차원)

잔차: $\mathbf{e}_i = R(\theta)\mathbf{p}_i + \mathbf{t} - \mathbf{q}_i$ (2차원)

야코비안 (2×3):

$$
J_i = \begin{bmatrix} 1 & 0 & -\sin\theta \cdot p_x - \cos\theta \cdot p_y \\ 0 & 1 & \cos\theta \cdot p_x - \sin\theta \cdot p_y \end{bmatrix}
$$

3×3 Hessian $H$를 구성하고 LDLT로 $\Delta\xi$를 구한다.

### Point-to-Plane ICP

잔차가 스칼라: $e_i = \mathbf{n}_i^T(R\mathbf{p}_i + \mathbf{t} - \mathbf{q}_i)$

야코비안 (1×3)이므로 $J_i^T J_i$는 여전히 3×3이다.

### NDT

잔차에 공분산 역행렬 $\Sigma^{-1}$이 곱해진 형태:

$$
H = \sum_i J_i^T \Sigma_c^{-1} J_i
$$

이는 **가중(weighted) Gauss-Newton**이며, 각 셀의 불확실성에 따라 기여도가 달라진다.

### Pose Graph 최적화 (fallback)

노드 수 $N$에 대해 $3N \times 3N$ sparse Hessian을 구성한다.
`SimplicialLDLT`로 sparse LDLT 분해를 수행한다.


## 수렴 특성

| 상황 | 수렴 속도 |
|------|----------|
| 잔차가 0에 가까움 | 2차 수렴 (quadratic) |
| 잔차가 큼 | 1차 수렴 (linear) 또는 발산 가능 |
| 초기값이 해에서 멀음 | 수렴 보장 없음 |

→ 그래서 ICP/NDT는 좋은 초기값(odom prediction, CSM 결과)을 먼저 구하고 Gauss-Newton을 적용하는 구조를 갖는다.


## 코드 위치

- `scan_match.cpp::runGauseNewtonICP` — Point-to-Point GN-ICP
- `scan_match.cpp::runGauseNewtonICP2` — Point-to-Plane GN-ICP
- `scan_match.cpp::runNDT` — NDT (가중 GN)
- `my_pose_graph.cpp::loopOptimize` — Pose Graph (sparse GN, g2o LM)
