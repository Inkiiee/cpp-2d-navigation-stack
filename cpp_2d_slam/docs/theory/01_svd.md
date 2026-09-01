# SVD (Singular Value Decomposition, 특이값 분해)


## 정의

임의의 $m \times n$ 행렬 $A$는 다음과 같이 분해할 수 있다.

$$
A = U \Sigma V^T
$$

- $U$: $m \times m$ 직교 행렬 (left singular vectors)
- $\Sigma$: $m \times n$ 대각 행렬 (singular values $\sigma_1 \geq \sigma_2 \geq \cdots \geq 0$)
- $V$: $n \times n$ 직교 행렬 (right singular vectors)

직교 행렬의 성질: $U^T U = I$, $V^T V = I$


## 기하학적 의미

SVD는 선형 변환 $A$를 세 단계로 분해한다.

1. $V^T$: 입력 공간의 회전 (또는 반사)
2. $\Sigma$: 각 축 방향 스케일링
3. $U$: 출력 공간의 회전 (또는 반사)

즉, **모든 선형 변환은 "회전 → 스케일 → 회전"으로 분해할 수 있다**는 것이다.


## 계산 원리

### 고유값 분해와의 관계

$A^T A$와 $A A^T$의 고유값 분해로부터 SVD를 구할 수 있다.

$$
A^T A = V \Sigma^T \Sigma V^T = V \text{diag}(\sigma_i^2) V^T
$$

$$
A A^T = U \Sigma \Sigma^T U^T = U \text{diag}(\sigma_i^2) U^T
$$

- $V$의 열: $A^T A$의 고유벡터
- $U$의 열: $A A^T$의 고유벡터
- $\sigma_i$: $A^T A$ 고유값의 양의 제곱근

### 2×2 행렬의 경우

이 프로젝트에서 ICP의 cross-covariance 행렬 $H$는 $2 \times 2$이므로:

$$
H = \begin{bmatrix} h_{11} & h_{12} \\ h_{21} & h_{22} \end{bmatrix} = U \Sigma V^T
$$

Eigen 라이브러리의 `JacobiSVD`를 사용하여 계산한다.


## ICP에서의 활용

Point-to-Point ICP에서 두 점군의 최적 회전을 구하는 데 SVD가 핵심적으로 사용된다.

### 문제 정의

source 점군 $\{a_i\}$와 target 점군 $\{b_i\}$의 대응 관계가 주어졌을 때, 다음을 최소화하는 $R$과 $\mathbf{t}$를 구한다:

$$
\min_{R, \mathbf{t}} \sum_{i=1}^{N} \| R \mathbf{a}_i + \mathbf{t} - \mathbf{b}_i \|^2
$$

### 풀이

**1단계**: 두 점군의 중심 계산

$$
\bar{\mathbf{a}} = \frac{1}{N}\sum_i \mathbf{a}_i, \quad \bar{\mathbf{b}} = \frac{1}{N}\sum_i \mathbf{b}_i
$$

**2단계**: 중심 정렬 후 cross-covariance 행렬 $H$ 구성

$$
H = \sum_i (\mathbf{a}_i - \bar{\mathbf{a}})(\mathbf{b}_i - \bar{\mathbf{b}})^T
$$

**3단계**: SVD 분해

$$
H = U \Sigma V^T
$$

**4단계**: 최적 회전

$$
R = V U^T
$$

만약 $\det(R) < 0$이면 (반사가 발생), $V$의 마지막 열 부호를 반전시킨다:

$$
V' = [v_1, -v_2], \quad R = V' U^T
$$

이렇게 하면 $\det(R) = 1$이 보장되어 순수 회전만 남는다.

**5단계**: 최적 평행이동

$$
\mathbf{t} = \bar{\mathbf{b}} - R \bar{\mathbf{a}}
$$

### 왜 SVD가 최적해를 주는가

cross-covariance $H$의 SVD에서 $R = VU^T$는 $\text{trace}(R^T H)$를 최대화한다.

$$
\text{trace}(R^T H) = \text{trace}(U V^T \cdot U \Sigma V^T) = \text{trace}(\Sigma)
$$

이것이 최대가 되는 이유: $\Sigma$의 대각 원소(singular values)가 이미 최대값이고, 직교 변환은 trace를 줄일 수만 있기 때문이다.

이는 곧 오차 제곱합을 최소화하는 것과 동치이다.


## 프로젝트에서의 코드 위치

- `scan_match.cpp::runICP` — `Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, ComputeFullU | ComputeFullV)`
