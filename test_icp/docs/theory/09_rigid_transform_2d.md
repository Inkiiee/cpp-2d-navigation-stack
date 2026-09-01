# 2D 강체 변환 (2D Rigid Body Transform)


## 개요

강체 변환(rigid body transform)은 물체의 형상을 보존하면서 위치와 방향을 바꾸는 변환이다. 2D에서는 **회전**과 **평행이동**의 조합으로 구성된다.


## 수학적 정의

### 2D 회전 행렬

각도 $\theta$만큼의 반시계 방향 회전:

$$
R(\theta) = \begin{pmatrix}
\cos\theta & -\sin\theta \\
\sin\theta & \cos\theta
\end{pmatrix}
$$

성질:
- $\det(R) = 1$ (방향 보존)
- $R^T = R^{-1}$ (직교 행렬)
- $R(\alpha) R(\beta) = R(\alpha + \beta)$ (회전 합성)

### 강체 변환

점 $\mathbf{p} = (p_x, p_y)^T$에 대한 변환:

$$
\mathbf{p}' = R(\theta)\,\mathbf{p} + \mathbf{t}
$$

$$
\begin{pmatrix} p'_x \\ p'_y \end{pmatrix} =
\begin{pmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{pmatrix}
\begin{pmatrix} p_x \\ p_y \end{pmatrix} +
\begin{pmatrix} t_x \\ t_y \end{pmatrix}
$$


## 동차 좌표 (Homogeneous Coordinates)

### 동기

회전+평행이동을 단일 행렬 곱으로 표현하면 변환 합성이 간단해진다.

### 표현

2D 점 $(x, y)$를 3D 벡터 $(x, y, 1)$로 확장:

$$
T = \begin{pmatrix}
\cos\theta & -\sin\theta & t_x \\
\sin\theta & \cos\theta & t_y \\
0 & 0 & 1
\end{pmatrix}
$$

$$
\begin{pmatrix} p'_x \\ p'_y \\ 1 \end{pmatrix} =
T \begin{pmatrix} p_x \\ p_y \\ 1 \end{pmatrix}
$$

### 변환 합성

두 변환 $T_1$, $T_2$를 순서대로 적용:

$$
T_{total} = T_2 \cdot T_1
$$

주의: 오른쪽에서 왼쪽 순서로 적용된다 ($T_1$ 먼저, $T_2$ 나중).


## 역변환

$$
T^{-1} = \begin{pmatrix}
\cos\theta & \sin\theta & -(t_x \cos\theta + t_y \sin\theta) \\
-\sin\theta & \cos\theta & (t_x \sin\theta - t_y \cos\theta) \\
0 & 0 & 1
\end{pmatrix}
$$

즉:

$$
T^{-1} = \begin{pmatrix}
R^T & -R^T \mathbf{t} \\
\mathbf{0}^T & 1
\end{pmatrix}
$$


## 상대 포즈 (Relative Pose)

### 문제

포즈 A에서 본 포즈 B의 상대적 위치/방향은?

### 풀이

$$
T_{A \to B} = T_A^{-1} \cdot T_B
$$

```
World
  │
  ├── T_A → 로봇 A
  │
  └── T_B → 로봇 B

T_{A→B} = T_A⁻¹ · T_B  →  A 기준에서 B의 상대 포즈
```

### 2D Pose로 표현

포즈 $A = (x_A, y_A, \theta_A)$, $B = (x_B, y_B, \theta_B)$:

$$
\begin{aligned}
\Delta x &= (x_B - x_A)\cos\theta_A + (y_B - y_A)\sin\theta_A \\
\Delta y &= -(x_B - x_A)\sin\theta_A + (y_B - y_A)\cos\theta_A \\
\Delta\theta &= \theta_B - \theta_A
\end{aligned}
$$


## SLAM에서의 활용

### 1. Scan 변환

로봇 로컬 좌표의 스캔 포인트를 월드 좌표로 변환:

$$
\mathbf{p}_{world} = R(\theta_{robot})\,\mathbf{p}_{local} + \mathbf{t}_{robot}
$$

### 2. ICP 결과 적용

ICP가 구한 $(\Delta x, \Delta y, \Delta\theta)$로 포즈 업데이트:

$$
T_{new} = T_{icp} \cdot T_{old}
$$

### 3. 포즈 그래프 엣지

노드 $i$와 $j$ 사이의 관측값:

$$
\mathbf{z}_{ij} = T_i^{-1} \cdot T_j
$$

### 4. 오도메트리 적분

이전 포즈에 odometry 증분을 누적:

$$
T_{k+1} = T_k \cdot T_{\Delta odom}
$$


## 각도 정규화

각도가 $[-\pi, \pi)$ 범위를 벗어나면 예기치 않은 결과가 발생한다.

$$
\theta_{normalized} = \text{atan2}(\sin\theta, \cos\theta)
$$

이 공식은 모든 범위의 각도를 $(-\pi, \pi]$로 변환한다.


## Eigen에서의 구현

```cpp
// 2D 변환 행렬 (3x3)
Eigen::Matrix3d T;
T << cos(th), -sin(th), tx,
     sin(th),  cos(th), ty,
     0,        0,        1;

// 점 변환
Eigen::Vector2d p_local(x, y);
Eigen::Vector2d p_world = T.block<2,2>(0,0) * p_local + T.block<2,1>(0,2);

// 변환 합성
Eigen::Matrix3d T_total = T2 * T1;

// 역변환
Eigen::Matrix3d T_inv = T.inverse();
```


## 코드 위치

- `slam_basic.h` — `MyPose` 구조체 $(x, y, \theta)$
- `scan_match.cpp` — ICP에서 스캔 포인트 변환 (`cos(th)`, `sin(th)` 적용)
- `slam.cpp` — 포즈 업데이트, 오도메트리 적분
- `my_pose_graph.cpp` — 상대 포즈 계산 (엣지 constraint)
- `painter.cpp` — 월드→픽셀 좌표 변환 (강체 변환의 특수 경우)
