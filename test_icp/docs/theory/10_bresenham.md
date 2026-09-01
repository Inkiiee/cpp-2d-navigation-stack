# 브레젠험 직선 알고리즘 (Bresenham's Line Algorithm)


## 개요

1962년 Jack Bresenham이 제안한, **정수 연산만으로** 두 점 사이의 직선을 래스터화하는 알고리즘이다. 부동소수점 연산 없이 어떤 격자 셀을 직선이 통과하는지 계산한다.


## 동기

Occupancy Grid에서 레이저 빔의 경로를 추적(ray tracing)하려면, 센서 위치부터 hit 지점까지 직선이 지나는 모든 격자 셀을 알아야 한다.

```
  Hit!
   ×
  /        빔이 지나간 셀 = miss (비어 있음)
 /         빔이 도착한 셀 = hit (장애물)
/
○ 센서
```

부동소수점으로 기울기를 계산하면 느리고 오차가 누적된다. Bresenham은 정수 덧셈과 비교만으로 이를 해결한다.


## 기본 원리

### 기울기가 0~1인 경우 (|slope| ≤ 1)

시작점 $(x_0, y_0)$에서 끝점 $(x_1, y_1)$까지:

$$
\Delta x = x_1 - x_0, \quad \Delta y = y_1 - y_0
$$

**이상적** 직선의 y좌표:

$$
y = y_0 + \frac{\Delta y}{\Delta x}(x - x_0)
$$

하지만 격자에서는 $y$가 정수여야 한다. 핵심 아이디어: **오차(error)**를 누적하여 $y$를 한 칸 올릴지 결정한다.


### 판별식 (Decision Variable)

$$
d = 2\Delta y - \Delta x \quad \text{(초기값)}
$$

각 x 스텝에서:
- $d < 0$: y를 유지, $d \leftarrow d + 2\Delta y$
- $d \geq 0$: y를 1 증가, $d \leftarrow d + 2\Delta y - 2\Delta x$

### 예시

$(0, 0)$ → $(5, 3)$: $\Delta x = 5$, $\Delta y = 3$, $d_0 = 2(3) - 5 = 1$

| Step | x | y | d | 판정 |
|------|---|---|----|----|
| 0 | 0 | 0 | 1 | $d \geq 0$ → y++ |
| 1 | 1 | 1 | -3 | $d < 0$ → y 유지 |
| 2 | 2 | 1 | 3 | $d \geq 0$ → y++ |
| 3 | 3 | 2 | -1 | $d < 0$ → y 유지 |
| 4 | 4 | 2 | 5 | $d \geq 0$ → y++ |
| 5 | 5 | 3 | — | 도착 |

```
  3 │          ×
  2 │      ■  ■
  1 │  ■  ■
  0 │○
    └──────────
      0  1  2  3  4  5
```


## 일반화: 모든 기울기

기본 알고리즘은 기울기 0~1만 처리한다. 나머지 경우를 처리하려면:

| 조건 | 처리 |
|------|------|
| $\Delta x < 0$ | x, y를 swap하여 항상 왼쪽→오른쪽 |
| $\|\Delta y\| > \|\Delta x\|$ | x와 y 역할 교환 (y가 주축) |
| $\Delta y < 0$ | y 증감 방향을 -1로 설정 |

이렇게 8방향(octant) 모두를 커버한다.


## 의사 코드

```
function bresenham(x0, y0, x1, y1):
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = sign(x1 - x0)    // +1 or -1
    sy = sign(y1 - y0)    // +1 or -1
    
    steep = (dy > dx)
    if steep:
        swap(dx, dy)
    
    d = 2 * dy - dx
    x = x0, y = y0
    
    for i = 0 to dx:
        visit(x, y)        // 이 셀을 처리
        
        if steep:
            y += sy
        else:
            x += sx
            
        if d >= 0:
            if steep: x += sx
            else:     y += sy
            d -= 2 * dx
        d += 2 * dy
```


## 복잡도

$$
O(\max(|\Delta x|, |\Delta y|))
$$

각 스텝에서 정수 덧셈 2번 + 비교 1번. 부동소수점 연산 **0회**.


## Occupancy Grid에서의 적용

### Ray Tracing 과정

센서에서 hit 지점까지 Bresenham으로 직선을 그리면:

1. **경로 위의 셀** (중간 셀들) → **miss**: 빔이 통과했으므로 비어 있음
2. **끝점의 셀** (hit 지점) → **hit**: 장애물이 존재

```
  ┌───┬───┬───┬───┬───┐
  │   │   │   │   │ H │  H = hit (occupied)
  ├───┼───┼───┼───┼───┤
  │   │   │ M │ M │   │  M = miss (free)
  ├───┼───┼───┼───┼───┤
  │ S │ M │   │   │   │  S = sensor (free)
  └───┴───┴───┴───┴───┘
```

### 좌표 변환

월드 좌표 → 격자 인덱스:

$$
i = \left\lfloor \frac{x - x_{origin}}{resolution} \right\rfloor, \quad
j = \left\lfloor \frac{y - y_{origin}}{resolution} \right\rfloor
$$

Bresenham은 이 정수 인덱스 위에서 동작한다.


## DDA와의 비교

| 특성 | Bresenham | DDA (Digital Differential Analyzer) |
|------|-----------|-------------------------------------|
| 연산 | 정수 덧셈 | 부동소수점 덧셈 |
| 속도 | 빠름 | 중간 |
| 정확도 | 정확히 동일 | 부동소수점 오차 누적 가능 |
| 구현 복잡도 | 중간 | 낮음 |

SLAM의 맵 업데이트는 매 프레임 수백~수천 빔에 대해 수행하므로 Bresenham의 정수 연산 장점이 크다.


## 코드 위치

- `map_backend.cpp` — `bresenham()` 함수: occupancy grid의 hit/miss 셀 마킹
- `map_backend.h` — 격자 맵 데이터 구조와 해상도 설정
