# KDTree (K-Dimensional Tree)


## 개요

KDTree는 $k$차원 공간의 점들을 효율적으로 탐색하기 위한 **이진 공간 분할(binary space partitioning)** 자료구조다.

1975년 Bentley가 제안했다.


## 동기

$N$개 점에서 query 점의 최근접 이웃을 찾는 문제:

| 방법 | 복잡도 | N=10,000 |
|------|--------|----------|
| Brute force | $O(N)$ per query | 10,000 비교 |
| KDTree | $O(\log N)$ 평균 per query | ~13 비교 |

SLAM에서 ICP 매 반복 × 모든 점에 대해 최근접을 찾으므로, KDTree 없이는 실시간 처리가 불가능하다.


## 구조

### 트리 빌드

각 레벨에서 한 축을 선택하여 점들을 반으로 나눈다.

1. 현재 레벨의 축을 선택 (2D: x축 → y축 → x축 → ...)
2. 해당 축으로 중앙값(median)을 기준으로 분할
3. 왼쪽 자식: 중앙값보다 작은 점들
4. 오른쪽 자식: 중앙값보다 큰 점들
5. leaf node 크기가 임계값 이하면 종료

```
        y
        │   ┌──── split x=5 ────┐
        │   │                    │
  4 ─── │ ──A── split y=4 ─── ──B── split y=3
        │   │↙     ↘            │↙     ↘
  2 ─── │  [1,2]  [3,6]       [7,3]  [8,5]
        │
        └─────────────────────── x
            1  3  5  7  8
```

### Leaf Size

이 프로젝트에서는 leaf size = 10 (`KDTreeSingleIndexAdaptorParams(10)`).

leaf node가 10개 이하의 점을 가지면 더 이상 분할하지 않고 brute-force로 탐색한다. 너무 작으면 트리가 깊어지고, 너무 크면 leaf에서 비교가 많아진다.


## 최근접 탐색 (NN Search)

### 알고리즘

1. query 점이 속할 영역을 따라 leaf까지 내려간다
2. leaf 내 점들과 거리를 비교하여 현재 최근접을 갱신
3. 부모로 올라가면서:
   - 현재 최근접 거리보다 **분할 경계까지의 거리가 짧으면** 반대쪽 자식도 탐색
   - 그렇지 않으면 가지치기(pruning)

```
query 점 Q에서 탐색:

    ┌─────────┬─────────┐
    │         │    ·    │
    │    ·    │  · Q    │  Q의 후보가 같은 쪽에만 있으면
    │         │    ↓    │  반대쪽은 탐색하지 않음 (pruning)
    │    ·    │ nearest │
    └─────────┴─────────┘
```

### K-Nearest Neighbors (KNN)

최근접 1개 대신 가장 가까운 $k$개를 유지한다. max-heap으로 $k$번째 거리를 추적하여 pruning 기준으로 사용한다.


## 거리 척도

이 프로젝트에서는 L2 (유클리드 거리)의 **제곱**을 사용한다:

$$
d^2(\mathbf{p}, \mathbf{q}) = (p_x - q_x)^2 + (p_y - q_y)^2
$$

제곱근을 계산하지 않는 이유: 순서 비교에서 $\sqrt{\cdot}$는 단조 함수이므로 생략해도 결과가 같고, 연산을 절약할 수 있다.


## nanoflann

이 프로젝트에서 사용하는 KDTree 구현체는 **nanoflann**이다.

| 특성 | 내용 |
|------|------|
| 언어 | C++ header-only |
| 의존성 | 없음 |
| 차원 | 컴파일 타임 고정 (2D) |
| 어댑터 | `PointCloud` 구조체에 인터페이스 함수 구현 |

어댑터 인터페이스:
```cpp
size_t kdtree_get_point_count() const;          // 점 수
double kdtree_get_pt(size_t idx, int dim) const; // idx번째 점의 dim축 좌표
double kdtree_distance(const double* p, size_t idx, size_t) const; // 거리 제곱
```


## 시간 복잡도

| 연산 | 평균 | 최악 |
|------|------|------|
| 빌드 | $O(N \log N)$ | $O(N \log N)$ |
| NN 탐색 | $O(\log N)$ | $O(N)$ (고차원 시) |
| KNN 탐색 | $O(k \log N)$ | $O(kN)$ |

2D에서는 최악 케이스가 거의 발생하지 않아 매우 효율적이다.


## ICP에서의 역할

ICP 매 반복에서:
1. Source 점 → KDTree에서 Target의 최근접 점 탐색 (대응 관계 생성)
2. 대응 관계로 최적 변환 계산
3. Source 점 갱신

KDTree를 한 번만 빌드하고 모든 반복에서 재사용한다 (`CloudTree` RAII 패턴).


## 코드 위치

- `slam_basic.h` — `PointCloud` 어댑터 + `KDTree` typedef
- `scan_match.h` — `CloudTree` (RAII helper: PointCloud + KDTree 동시 관리)
- `scan_match.cpp::buildKDTree` — 트리 빌드
- `scan_match.cpp::knnSearch` — KNN 탐색
