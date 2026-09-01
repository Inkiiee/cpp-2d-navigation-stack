# Pose Graph 최적화


## 개요

Pose Graph 최적화는 SLAM에서 **누적 드리프트(drift)**를 줄이기 위한 후처리 기법이다.

로봇의 궤적을 "그래프"로 표현하고, loop closure 등의 추가 제약을 이용해 전체 궤적을 동시에 보정한다.


## 그래프 구조

### 노드 (Node)

각 시점의 로봇 pose:

$$
\xi_i = (t_x^i, t_y^i, \theta^i) \in \mathbb{R}^3
$$

### 엣지 (Edge)

두 노드 사이의 **상대 변환 제약**:

$$
\mathbf{z}_{ij} = (z_{tx}, z_{ty}, z_\theta)
$$

엣지 종류:
- **오도메트리 엣지**: 연속 노드 간 ($i \to i+1$), 낮은 확신
- **루프 엣지**: loop closure로 발견된 비연속 쌍, 높은 확신

```
Pose Graph 예시:

  ○───○───○───○───○
  1   2   3   4   5
              │       │
              └───────┘  ← loop edge (3→5)
```


## 정보 행렬 (Information Matrix)

각 엣지의 "확신도"를 나타내는 대각 행렬:

$$
\Omega_{ij} = \begin{bmatrix} \omega_{tx} & 0 & 0 \\ 0 & \omega_{ty} & 0 \\ 0 & 0 & \omega_\theta \end{bmatrix}
$$

값이 클수록 해당 제약을 더 믿는다.

| 엣지 종류 | $\omega$ 값 | 의미 |
|----------|------------|------|
| 오도메트리 | 1 | 적당히 믿음 |
| 루프 클로저 | 100 | 강하게 믿음 |


## 에러 함수

엣지 $(i, j)$에 대한 잔차:

$$
\mathbf{e}_{ij} = \text{relative}(\xi_i, \xi_j) - \mathbf{z}_{ij}
$$

여기서 $\text{relative}(\xi_i, \xi_j)$는 현재 estimate에서의 상대 pose:

$$
\Delta t_x = \cos\theta_i (t_x^j - t_x^i) + \sin\theta_i (t_y^j - t_y^i)
$$

$$
\Delta t_y = -\sin\theta_i (t_x^j - t_x^i) + \cos\theta_i (t_y^j - t_y^i)
$$

$$
\Delta\theta = \theta_j - \theta_i
$$

## 전체 비용 함수

$$
\mathcal{F}(\Xi) = \sum_{(i,j)} \mathbf{e}_{ij}^T \Omega_{ij} \mathbf{e}_{ij}
$$

$\Xi = (\xi_1, \xi_2, \ldots, \xi_N)$는 모든 노드의 포즈를 하나로 쌓은 벡터다.


## 최적화: Gauss-Newton (Fallback)

### 야코비안

각 엣지에 대해 에러의 야코비안 $A$, $B$를 구한다:

$$
A = \frac{\partial \mathbf{e}_{ij}}{\partial \xi_i}, \quad B = \frac{\partial \mathbf{e}_{ij}}{\partial \xi_j}
$$

구체적 형태:

$$
A = \begin{bmatrix} -\cos\theta_i & -\sin\theta_i & \sin\theta_i \cdot dx - \cos\theta_i \cdot dy \\ \sin\theta_i & -\cos\theta_i & \cos\theta_i \cdot dx + \sin\theta_i \cdot dy \\ 0 & 0 & -1 \end{bmatrix}
$$

$$
B = \begin{bmatrix} \cos\theta_i & \sin\theta_i & 0 \\ -\sin\theta_i & \cos\theta_i & 0 \\ 0 & 0 & 1 \end{bmatrix}
$$

### Hessian 구성

전체 Hessian $H$는 $3N \times 3N$ sparse 행렬이다:

$$
H = \sum_{(i,j)} \begin{bmatrix} A^T\Omega A & A^T\Omega B \\ B^T\Omega A & B^T\Omega B \end{bmatrix}
$$

각 엣지가 $3 \times 3$ 블록 4개를 $(fi, fi), (ti, ti), (fi, ti), (ti, fi)$ 위치에 기여한다.

```
Hessian의 sparse 구조 (5 노드, 5 엣지):

  ■ ■ . . .
  ■ ■ ■ . .
  . ■ ■ ■ .   ← 대부분 비어있음 (sparse)
  . . ■ ■ ■
  . . . ■ ■
```

### 앵커링

첫 번째 노드를 고정하지 않으면 시스템이 특이(singular)해진다 (자유 이동 가능). 대각에 큰 값($10^6$)을 추가:

$$
H_{0:2, 0:2} \mathrel{+}= 10^6 \cdot I_3
$$

### 풀이

$$
H \Delta\Xi = -\mathbf{b}
$$

`Eigen::SimplicialLDLT`로 sparse LDLT 분해하여 풀고:

$$
\xi_i \leftarrow \xi_i + \Delta\xi_i
$$


## 최적화: g2o (Levenberg-Marquardt)

`TEST_ICP_HAS_G2O` 매크로 정의 시 g2o 라이브러리를 사용한다.

### 구조

| 요소 | g2o 타입 | 설명 |
|------|---------|------|
| Vertex | `VertexSE2` | 2D pose $(x, y, \theta)$ |
| Edge | `EdgeSE2` | 상대 pose 제약 |
| Solver | `LinearSolverEigen` | 선형 시스템 풀기 |
| Algorithm | `OptimizationAlgorithmLevenberg` | LM 최적화 |

### Robust Kernel (Huber)

loop edge에 Huber robust kernel을 적용한다:

$$
\rho(e) = \begin{cases} \frac{1}{2}e^2 & |e| \leq \delta \\ \delta(|e| - \frac{1}{2}\delta) & |e| > \delta \end{cases}
$$

$\delta = 1.0$

#### 왜 필요한가?

잘못된 loop closure (false positive)가 들어오면 큰 에러를 생성한다. 일반 최소자승은 이 에러에 과도하게 끌린다. Huber kernel은 큰 잔차의 영향을 선형($|e|$)으로 제한하여 이상값(outlier)에 강건해진다.

```
비용 함수 비교:
  │
  │     ╱ L2 (제곱)
  │    ╱
  │   ╱  ╱ Huber (선형으로 전환)
  │  ╱  ╱
  │ ╱ ╱
  ├╱╱──────── e
```


## 반복 횟수 제한

노드 수 $N$이 커지면 1회 반복 비용도 커지므로:

$$
\text{max\_iter} = \max(3, \; \min(\text{iter}, \; 5000 / N))
$$

| 노드 수 | 최대 반복 |
|---------|----------|
| 100 | 20 |
| 500 | 10 |
| 1000 | 5 |
| 2000 | 3 |


## Dense vs Sparse

| 방식 | 메모리 | 계산 복잡도 | 현재 구현 |
|------|--------|-----------|----------|
| Dense (MatrixXd + LDLT) | $O(N^2)$ | $O(N^3)$ | **제거됨** |
| Sparse (SparseMatrix + SimplicialLDLT) | $O(E)$ | $O(N)$~$O(N^{1.5})$ | **현재 fallback** |
| g2o (sparse + LM) | $O(E)$ | $O(N)$~$O(N^{1.5})$ | **현재 기본** |


## 코드 위치

- `my_pose_graph.cpp::loopOptimize` — Sparse Gauss-Newton (fallback) + g2o LM
- `my_pose_graph.cpp::errorComputeUnlocked` — 에러 계산
- `loop_detecter.cpp::detectLoop` — 최적화 트리거
