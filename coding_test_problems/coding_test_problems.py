"""
==============================================================
  코딩 테스트 대비 - Python 수학 계산 & 정렬 문제 모음
==============================================================
"""

from functools import cmp_to_key
import heapq

# ============================================================
# [수학 계산 문제 1] 소수 판별 (Prime Number Check)
# ------------------------------------------------------------
# 문제: 주어진 정수 n이 소수인지 판별하라.
# 입력: n = 17
# 출력: True
# 시간복잡도: O(√n)
# ============================================================

def is_prime(n: int) -> bool:
    # 0, 1, 음수는 소수가 아니다.
    if n < 2:
        return False
    # 2는 유일한 짝수 소수다.
    if n == 2:
        return True
    # 2를 제외한 짝수는 소수가 아니다.
    if n % 2 == 0:
        return False
    # 홀수 약수만 확인해도 충분하다. (i*i <= n)
    for i in range(3, int(n**0.5) + 1, 2):
        if n % i == 0:
            return False
    return True

# 테스트
print("=== [수학 1] 소수 판별 ===")
print(is_prime(17))   # True
print(is_prime(18))   # False
print(is_prime(2))    # True


# ============================================================
# [수학 계산 문제 2] 에라토스테네스의 체 (소수 목록)
# ------------------------------------------------------------
# 문제: 2 이상 n 이하의 모든 소수를 구하라.
# 입력: n = 30
# 출력: [2, 3, 5, 7, 11, 13, 17, 19, 23, 29]
# 시간복잡도: O(n log log n)
# ============================================================

def sieve_of_eratosthenes(n: int) -> list[int]:
    # n이 2보다 작으면 소수 목록은 빈 리스트다.
    if n < 2:
        return []

    # 인덱스를 숫자로 보고, 값(True/False)을 소수 여부로 본다.
    is_prime_arr = [True] * (n + 1)
    is_prime_arr[0] = is_prime_arr[1] = False
    for i in range(2, int(n**0.5) + 1):
        if is_prime_arr[i]:
            # i*i 미만의 배수는 이미 더 작은 소수에서 지워졌다.
            for j in range(i*i, n + 1, i):
                is_prime_arr[j] = False
    return [i for i in range(2, n + 1) if is_prime_arr[i]]

print("\n=== [수학 2] 에라토스테네스의 체 ===")
print(sieve_of_eratosthenes(30))
# [2, 3, 5, 7, 11, 13, 17, 19, 23, 29]


# ============================================================
# [수학 계산 문제 3] 최대공약수 & 최소공배수 (GCD & LCM)
# ------------------------------------------------------------
# 문제: 두 수 a, b의 GCD와 LCM을 구하라.
# 입력: a=12, b=18
# 출력: GCD=6, LCM=36
# 시간복잡도: O(log(min(a,b)))
# ============================================================

def gcd(a: int, b: int) -> int:
    # 음수 입력이 와도 최대공약수는 양수로 정규화한다.
    a, b = abs(a), abs(b)
    while b:
        a, b = b, a % b
    return a

def lcm(a: int, b: int) -> int:
    # 둘 중 하나라도 0이면 최소공배수는 0이다.
    if a == 0 or b == 0:
        return 0
    return abs(a * b) // gcd(a, b)

print("\n=== [수학 3] GCD & LCM ===")
print(f"GCD(12, 18) = {gcd(12, 18)}")   # 6
print(f"LCM(12, 18) = {lcm(12, 18)}")   # 36


# ============================================================
# [수학 계산 문제 4] 피보나치 수열 (동적 프로그래밍)
# ------------------------------------------------------------
# 문제: n번째 피보나치 수를 구하라.
# 입력: n = 10
# 출력: 55
# 시간복잡도: O(n)  /  공간복잡도: O(1)
# ============================================================

def fibonacci(n: int) -> int:
    # 일반적으로 음수 인덱스 피보나치는 다루지 않으므로 예외 처리.
    if n < 0:
        raise ValueError("n은 0 이상의 정수여야 합니다.")

    if n <= 1:
        return n

    # 직전 두 항만 유지하면 되므로 공간복잡도 O(1) 달성.
    a, b = 0, 1
    for _ in range(2, n + 1):
        a, b = b, a + b
    return b

print("\n=== [수학 4] 피보나치 수열 ===")
print([fibonacci(i) for i in range(11)])
# [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]


# ============================================================
# [수학 계산 문제 5] 거듭제곱 나머지 (빠른 거듭제곱)
# ------------------------------------------------------------
# 문제: a^b mod m을 구하라.
# 입력: a=2, b=10, m=1000
# 출력: 24
# 시간복잡도: O(log b)
# ============================================================

def fast_power(a: int, b: int, m: int) -> int:
    if m <= 0:
        raise ValueError("mod m은 1 이상의 정수여야 합니다.")
    if b < 0:
        raise ValueError("지수 b는 0 이상의 정수여야 합니다.")

    result = 1
    a %= m
    while b > 0:
        if b % 2 == 1:       # b가 홀수면 현재 a를 곱함
            result = result * a % m
        a = a * a % m        # a를 제곱
        b //= 2
    return result

# Python 내장: pow(a, b, m) 과 동일
print("\n=== [수학 5] 빠른 거듭제곱 ===")
print(fast_power(2, 10, 1000))   # 24
print(pow(2, 10, 1000))          # 24 (내장 함수 비교)


# ============================================================
# [수학 계산 문제 6] 조합 (nCr mod p)
# ------------------------------------------------------------
# 문제: n개 중 r개를 선택하는 경우의 수를 구하라. (mod 1e9+7)
# 입력: n=10, r=3
# 출력: 120
# 시간복잡도: O(n)
# ============================================================

def combination(n: int, r: int, mod: int = 10**9 + 7) -> int:
    # 잘못된 입력은 0으로 처리한다. (문제 정의에 맞춘 방어 코드)
    if n < 0 or r < 0:
        return 0
    if r > n:
        return 0
    if mod <= 1:
        raise ValueError("mod는 2 이상의 정수여야 합니다.")

    r = min(r, n - r)  # 대칭성 활용
    num = 1
    den = 1
    for i in range(r):
        num = num * (n - i) % mod
        den = den * (i + 1) % mod
    return num * pow(den, mod - 2, mod) % mod  # 페르마 소정리 역원

print("\n=== [수학 6] 조합 nCr ===")
print(combination(10, 3))   # 120
print(combination(5, 2))    # 10


# ============================================================
# [정렬 문제 1] 버블 정렬 (Bubble Sort)
# ------------------------------------------------------------
# 문제: 정수 배열을 오름차순으로 정렬하라.
# 입력: [64, 34, 25, 12, 22, 11, 90]
# 출력: [11, 12, 22, 25, 34, 64, 90]
# 시간복잡도: O(n²)
# ============================================================

def bubble_sort(arr: list[int]) -> list[int]:
    # 원본 보존을 위해 복사본에서만 정렬.
    arr = arr[:]
    n = len(arr)
    for i in range(n):
        swapped = False
        for j in range(0, n - i - 1):
            if arr[j] > arr[j + 1]:
                arr[j], arr[j + 1] = arr[j + 1], arr[j]
                swapped = True
        if not swapped:   # 이미 정렬된 경우 조기 종료
            break
    return arr

print("\n=== [정렬 1] 버블 정렬 ===")
print(bubble_sort([64, 34, 25, 12, 22, 11, 90]))


# ============================================================
# [정렬 문제 2] 선택 정렬 (Selection Sort)
# ------------------------------------------------------------
# 시간복잡도: O(n²)  /  특징: swap 횟수 최소
# ============================================================

def selection_sort(arr: list[int]) -> list[int]:
    arr = arr[:]
    n = len(arr)
    for i in range(n):
        min_idx = i
        for j in range(i + 1, n):
            if arr[j] < arr[min_idx]:
                min_idx = j
        arr[i], arr[min_idx] = arr[min_idx], arr[i]
    return arr

print("\n=== [정렬 2] 선택 정렬 ===")
print(selection_sort([64, 25, 12, 22, 11]))


# ============================================================
# [정렬 문제 3] 삽입 정렬 (Insertion Sort)
# ------------------------------------------------------------
# 시간복잡도: O(n²)  / 거의 정렬된 경우 O(n)
# ============================================================

def insertion_sort(arr: list[int]) -> list[int]:
    arr = arr[:]
    for i in range(1, len(arr)):
        key = arr[i]
        j = i - 1
        while j >= 0 and arr[j] > key:
            arr[j + 1] = arr[j]
            j -= 1
        arr[j + 1] = key
    return arr

print("\n=== [정렬 3] 삽입 정렬 ===")
print(insertion_sort([12, 11, 13, 5, 6]))


# ============================================================
# [정렬 문제 4] 병합 정렬 (Merge Sort)
# ------------------------------------------------------------
# 시간복잡도: O(n log n)  /  안정 정렬
# ============================================================

def merge_sort(arr: list[int]) -> list[int]:
    # 길이가 0 또는 1이면 이미 정렬 상태.
    if len(arr) <= 1:
        return arr
    mid = len(arr) // 2
    left = merge_sort(arr[:mid])
    right = merge_sort(arr[mid:])
    return merge(left, right)

def merge(left: list[int], right: list[int]) -> list[int]:
    result = []
    i = j = 0
    while i < len(left) and j < len(right):
        if left[i] <= right[j]:
            result.append(left[i]); i += 1
        else:
            result.append(right[j]); j += 1
    result.extend(left[i:])
    result.extend(right[j:])
    return result

print("\n=== [정렬 4] 병합 정렬 ===")
print(merge_sort([38, 27, 43, 3, 9, 82, 10]))


# ============================================================
# [정렬 문제 5] 퀵 정렬 (Quick Sort)
# ------------------------------------------------------------
# 시간복잡도: 평균 O(n log n), 최악 O(n²)
# ============================================================

def quick_sort(arr: list[int]) -> list[int]:
    if len(arr) <= 1:
        return arr
    # 중간 원소를 피벗으로 선택해 구현 단순화.
    pivot = arr[len(arr) // 2]
    left  = [x for x in arr if x < pivot]
    mid   = [x for x in arr if x == pivot]
    right = [x for x in arr if x > pivot]
    return quick_sort(left) + mid + quick_sort(right)

print("\n=== [정렬 5] 퀵 정렬 ===")
print(quick_sort([3, 6, 8, 10, 1, 2, 1]))


# ============================================================
# [정렬 문제 6] 계수 정렬 (Counting Sort)
# ------------------------------------------------------------
# 문제: 값의 범위가 제한적일 때 O(n+k) 정렬
# 입력: [4,2,2,8,3,3,1]
# 출력: [1, 2, 2, 3, 3, 4, 8]
# 시간복잡도: O(n + k)  k = 최대값
# ============================================================

def counting_sort(arr: list[int]) -> list[int]:
    if not arr:
        return []

    # 음수도 처리할 수 있도록 offset(최솟값)을 사용한다.
    min_val = min(arr)
    max_val = max(arr)
    offset = -min_val
    count = [0] * (max_val - min_val + 1)

    for x in arr:
        count[x + offset] += 1

    result = []
    for idx, cnt in enumerate(count):
        # idx를 실제 값으로 되돌린다.
        result.extend([idx - offset] * cnt)
    return result

print("\n=== [정렬 6] 계수 정렬 ===")
print(counting_sort([4, 2, 2, 8, 3, 3, 1]))


# ============================================================
# [정렬 응용 문제 7] 기준 정렬 (커스텀 key 활용)
# ------------------------------------------------------------
# 문제: 학생 (이름, 점수) 리스트를 점수 내림차순,
#       점수가 같으면 이름 오름차순으로 정렬하라.
# ============================================================

students = [
    ("Alice", 88), ("Bob", 95), ("Charlie", 88),
    ("Diana", 72), ("Eve", 95)
]

def sort_students(students: list[tuple[str, int]]) -> list[tuple[str, int]]:
    # 1순위: 점수 내림차순(-score), 2순위: 이름 오름차순(name)
    return sorted(students, key=lambda x: (-x[1], x[0]))

print("\n=== [정렬 7] 커스텀 정렬 ===")
for name, score in sort_students(students):
    print(f"  {name}: {score}")


# ============================================================
# [정렬 응용 문제 8] K번째로 큰 수 (힙 활용)
# ------------------------------------------------------------
# 문제: 배열에서 K번째로 큰 수를 구하라.
# 입력: arr=[3,2,1,5,6,4], k=2
# 출력: 5
# 시간복잡도: O(n log k)
# ============================================================

def kth_largest(arr: list, k: int) -> int:
    if not arr:
        raise ValueError("arr는 비어 있으면 안 됩니다.")
    if k <= 0 or k > len(arr):
        raise ValueError("k는 1 이상 len(arr) 이하여야 합니다.")

    # 크기 k의 최소 힙을 유지하면, 힙의 루트가 항상 k번째 큰 값이다.
    min_heap = []
    for num in arr:
        heapq.heappush(min_heap, num)
        if len(min_heap) > k:
            heapq.heappop(min_heap)
    return min_heap[0]

print("\n=== [정렬 8] K번째로 큰 수 (힙) ===")
print(kth_largest([3, 2, 1, 5, 6, 4], 2))   # 5
print(kth_largest([3, 2, 3, 1, 2, 4, 5, 5, 6], 4))  # 4


# ============================================================
# [정렬 응용 문제 9] 구간 병합 (Merge Intervals)
# ------------------------------------------------------------
# 문제: 겹치는 구간들을 합쳐라.
# 입력: [[1,3],[2,6],[8,10],[15,18]]
# 출력: [[1,6],[8,10],[15,18]]
# 시간복잡도: O(n log n)
# ============================================================

def merge_intervals(intervals: list[list[int]]) -> list[list[int]]:
    if not intervals:
        return []

    # 원본 훼손을 막기 위해 복사 + 정렬.
    sorted_intervals = sorted((start, end) for start, end in intervals)
    merged = [[sorted_intervals[0][0], sorted_intervals[0][1]]]

    for start, end in sorted_intervals[1:]:
        if start <= merged[-1][1]:        # 겹치면 합침
            merged[-1][1] = max(merged[-1][1], end)
        else:
            merged.append([start, end])
    return merged

print("\n=== [정렬 9] 구간 병합 ===")
print(merge_intervals([[1,3],[2,6],[8,10],[15,18]]))
# [[1, 6], [8, 10], [15, 18]]


# ============================================================
# [정렬 응용 문제 10] 가장 큰 수 만들기
# ------------------------------------------------------------
# 문제: 정수 배열의 수를 이어 붙여 만들 수 있는
#       가장 큰 수를 문자열로 반환하라.
# 입력: [3, 30, 34, 5, 9]
# 출력: "9534330"
# 시간복잡도: O(n log n)
# ============================================================

def largest_number(nums: list[int]) -> str:
    if not nums:
        return ""

    nums = list(map(str, nums))

    def compare(a, b):
        if a + b > b + a:   # "93" vs "39" → "93" 우선
            return -1
        elif a + b < b + a:
            return 1
        return 0

    nums.sort(key=cmp_to_key(compare))
    result = "".join(nums)
    return "0" if result[0] == "0" else result   # 모두 0인 경우 처리

print("\n=== [정렬 10] 가장 큰 수 만들기 ===")
print(largest_number([3, 30, 34, 5, 9]))    # 9534330
print(largest_number([10, 2]))              # 210


# ============================================================
# 📊 알고리즘 시간복잡도 요약
# ============================================================
print("""
╔══════════════════════════════════════════════════════╗
║            알고리즘 시간복잡도 정리                    ║
╠════════════════════╦════════════╦════════════════════╣
║ 알고리즘            ║ 시간복잡도  ║ 특징               ║
╠════════════════════╬════════════╬════════════════════╣
║ 소수 판별           ║ O(√n)      ║ 단일 판별           ║
║ 에라토스테네스의 체  ║ O(n log n) ║ 범위 소수 목록      ║
║ 유클리드 GCD        ║ O(log n)   ║ 재귀/반복 모두 가능 ║
║ 빠른 거듭제곱        ║ O(log b)   ║ pow(a,b,m) 내장    ║
╠════════════════════╬════════════╬════════════════════╣
║ 버블/선택/삽입 정렬  ║ O(n²)      ║ 구현 단순           ║
║ 병합 정렬           ║ O(n log n) ║ 안정 정렬           ║
║ 퀵 정렬             ║ O(n log n) ║ 평균 가장 빠름      ║
║ 계수 정렬           ║ O(n+k)     ║ 값 범위 제한 시     ║
╚════════════════════╩════════════╩════════════════════╝
""")
