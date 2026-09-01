# 쿼터니언과 오일러 각 (Quaternion & Euler Angles)


## 개요

3D 회전을 표현하는 두 가지 대표적 방법. 이 프로젝트는 2D SLAM이지만 ROS2 메시지 체계에서 orientation을 쿼터니언으로 전달하므로 변환이 필요하다.


## 오일러 각 (Euler Angles)

### 정의

세 축에 대한 순차적 회전으로 자세를 표현한다:

$$
\text{Roll}(\phi),\quad \text{Pitch}(\theta),\quad \text{Yaw}(\psi)
$$

| 이름 | 축 | 의미 |
|------|-----|------|
| Roll ($\phi$) | X축 | 좌우 기울기 |
| Pitch ($\theta$) | Y축 | 앞뒤 기울기 |
| Yaw ($\psi$) | Z축 | 방위각 (heading) |

### 2D 로봇에서

2D 평면 로봇은 $\phi = 0$, $\theta = 0$이고, $\psi$ (yaw)만 의미가 있다.

$$
\text{Pose}_{2D} = (x, y, \psi)
$$

### 짐벌 락 (Gimbal Lock)

오일러 각의 치명적 단점. Pitch가 $\pm 90°$일 때 Roll과 Yaw의 회전축이 겹쳐 자유도 하나를 잃는다.

$$
\theta = \pm\frac{\pi}{2} \Rightarrow \text{Roll과 Yaw 구분 불가}
$$

2D 로봇에서는 Pitch가 항상 0이므로 짐벌 락이 발생하지 않지만, 3D 확장 시 문제가 된다.


## 쿼터니언 (Quaternion)

### 정의

4개의 스칼라로 3D 회전을 표현하는 초복소수(hypercomplex number):

$$
\mathbf{q} = w + xi + yj + zk = (w, x, y, z)
$$

여기서 $i^2 = j^2 = k^2 = ijk = -1$ (해밀턴 규칙)

### 단위 쿼터니언

회전을 표현하려면 **단위 쿼터니언** (unit quaternion)이어야 한다:

$$
\|\mathbf{q}\| = \sqrt{w^2 + x^2 + y^2 + z^2} = 1
$$

### 축-각 표현과의 관계

축 $\hat{\mathbf{n}} = (n_x, n_y, n_z)$ 주위로 $\alpha$ 만큼 회전:

$$
\mathbf{q} = \left(\cos\frac{\alpha}{2},\; n_x \sin\frac{\alpha}{2},\; n_y \sin\frac{\alpha}{2},\; n_z \sin\frac{\alpha}{2}\right)
$$

### 장점

| 특성 | 오일러 각 | 쿼터니언 |
|------|----------|----------|
| 파라미터 수 | 3 | 4 (단위 구속 1개) |
| 짐벌 락 | 있음 | 없음 |
| 보간 (SLERP) | 불연속 | 부드러움 |
| 회전 합성 | 삼각함수 계산 | 곱셈 |
| 직관성 | 높음 | 낮음 |


## 변환 공식

### 오일러 → 쿼터니언

$$
\begin{aligned}
w &= \cos\frac{\phi}{2}\cos\frac{\theta}{2}\cos\frac{\psi}{2} + \sin\frac{\phi}{2}\sin\frac{\theta}{2}\sin\frac{\psi}{2} \\
x &= \sin\frac{\phi}{2}\cos\frac{\theta}{2}\cos\frac{\psi}{2} - \cos\frac{\phi}{2}\sin\frac{\theta}{2}\sin\frac{\psi}{2} \\
y &= \cos\frac{\phi}{2}\sin\frac{\theta}{2}\cos\frac{\psi}{2} + \sin\frac{\phi}{2}\cos\frac{\theta}{2}\sin\frac{\psi}{2} \\
z &= \cos\frac{\phi}{2}\cos\frac{\theta}{2}\sin\frac{\psi}{2} - \sin\frac{\phi}{2}\sin\frac{\theta}{2}\cos\frac{\psi}{2}
\end{aligned}
$$

### 2D 단순화 (Roll=0, Pitch=0, Yaw=$\psi$)

$$
\mathbf{q} = \left(\cos\frac{\psi}{2},\; 0,\; 0,\; \sin\frac{\psi}{2}\right)
$$

Z축 회전만 존재하므로 $x = y = 0$이다.

### 쿼터니언 → Yaw 추출

$$
\psi = \text{atan2}\big(2(wz + xy),\; 1 - 2(y^2 + z^2)\big)
$$

2D 단순화 ($x = y = 0$):

$$
\psi = \text{atan2}(2wz,\; 1 - 2z^2) = 2 \cdot \text{atan2}(z, w)
$$


## 쿼터니언 연산

### 회전 합성 (곱셈)

$$
\mathbf{q}_1 \otimes \mathbf{q}_2 = \begin{pmatrix}
w_1 w_2 - x_1 x_2 - y_1 y_2 - z_1 z_2 \\
w_1 x_2 + x_1 w_2 + y_1 z_2 - z_1 y_2 \\
w_1 y_2 - x_1 z_2 + y_1 w_2 + z_1 x_2 \\
w_1 z_2 + x_1 y_2 - y_1 x_2 + z_1 w_2
\end{pmatrix}
$$

주의: 쿼터니언 곱은 **비교환적** ($\mathbf{q}_1 \otimes \mathbf{q}_2 \neq \mathbf{q}_2 \otimes \mathbf{q}_1$)

### 역회전

$$
\mathbf{q}^{-1} = \frac{(w, -x, -y, -z)}{\|\mathbf{q}\|^2}
$$

단위 쿼터니언이면 $\mathbf{q}^{-1} = (w, -x, -y, -z)$ (켤레와 동일)


## ROS2에서의 사용

ROS2 메시지에서 orientation은 항상 쿼터니언이다:

```
geometry_msgs/msg/Quaternion:
  float64 x
  float64 y
  float64 z
  float64 w
```

Odometry, IMU 등의 메시지에서 orientation을 받아 yaw로 변환:

```cpp
double yaw = 2.0 * atan2(q.z, q.w);  // 2D 전용 단순 변환
```


## 코드 위치

- `odom_receive_node.cpp` — 오도메트리 쿼터니언 → yaw 변환
- `imu_receive_node.cpp` — IMU 쿼터니언 → yaw 변환
- `slam_basic.h` — `MyPose` 구조체의 각도는 radian yaw
