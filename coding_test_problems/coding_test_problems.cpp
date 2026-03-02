/*
==============================================================
  코딩 테스트 대비 - C++ 수학 계산 & 정렬 문제 모음
==============================================================
*/

#include <iostream>
#include <vector>
#include <algorithm>
#include <queue>
#include <functional>
#include <string>
#include <cmath>
#include <numeric>

using namespace std;

using IntPair = pair<int, int>;

// ============================================================
// [수학 1] 소수 판별 (Prime Number Check)
// 시간복잡도: O(√n)
// ============================================================

bool is_prime(int n) {
    // 0, 1, 음수는 소수가 아니다.
    if (n < 2) return false;
    // 2는 유일한 짝수 소수.
    if (n == 2) return true;
    // 2를 제외한 짝수는 소수가 아니다.
    if (n % 2 == 0) return false;
    // 매 반복마다 sqrt를 계산하지 않도록 한 번만 구해둔다.
    const int limit = static_cast<int>(sqrt(n));
    for (int i = 3; i <= limit; i += 2)
        if (n % i == 0) return false;
    return true;
}

// ============================================================
// [수학 2] 에라토스테네스의 체
// 시간복잡도: O(n log log n)
// ============================================================

vector<int> sieve_of_eratosthenes(int n) {
    // n < 2면 소수가 존재하지 않는다.
    if (n < 2) return {};

    // index == 숫자 값, true면 현재까지 "소수 후보"라는 의미.
    vector<bool> is_prime_arr(n + 1, true);
    is_prime_arr[0] = is_prime_arr[1] = false;
    const int limit = static_cast<int>(sqrt(n));
    for (int i = 2; i <= limit; i++) {
        if (is_prime_arr[i])
            // i의 배수는 모두 합성수 처리.
            for (int j = i * i; j <= n; j += i)
                is_prime_arr[j] = false;
    }
    vector<int> primes;
    primes.reserve(n / 10);  // 대략적인 추정치로 재할당 횟수 감소.
    for (int i = 2; i <= n; i++)
        if (is_prime_arr[i]) primes.push_back(i);
    return primes;
}

// ============================================================
// [수학 3] 최대공약수 & 최소공배수 (GCD & LCM)
// 시간복잡도: O(log(min(a,b)))
// ============================================================

long long gcd(long long a, long long b) {
    // 음수 입력도 안정적으로 처리하기 위해 절대값 사용.
    a = llabs(a);
    b = llabs(b);
    while (b) { a %= b; swap(a, b); }
    return a;
}

long long lcm(long long a, long long b) {
    // 둘 중 하나라도 0이면 최소공배수는 0.
    if (a == 0 || b == 0) return 0;
    return a / gcd(a, b) * b;   // 오버플로 방지: 먼저 나누기
}

// ============================================================
// [수학 4] 피보나치 수열 (DP, 공간 O(1))
// 시간복잡도: O(n)
// ============================================================

long long fibonacci(int n) {
    // 음수 인덱스는 본 예제 범위에서 허용하지 않는다.
    if (n < 0) return 0;
    if (n <= 1) return n;
    long long a = 0, b = 1;
    for (int i = 2; i <= n; i++) {
        long long tmp = a + b;
        a = b; b = tmp;
    }
    return b;
}

// ============================================================
// [수학 5] 빠른 거듭제곱 (Fast Power / Binary Exponentiation)
// 시간복잡도: O(log b)
// ============================================================

long long fast_power(long long a, long long b, long long mod) {
    // mod <= 0이면 수학적 의미가 모호하므로 0 반환.
    if (mod <= 0) return 0;

    long long result = 1;
    a %= mod;
    while (b > 0) {
        if (b & 1) result = result * a % mod;   // b가 홀수
        a = a * a % mod;
        b >>= 1;
    }
    return result;
}

// ============================================================
// [수학 6] 조합 nCr mod p (페르마 소정리 역원)
// 시간복잡도: O(n)
// ============================================================

const long long MOD = 1e9 + 7;

long long combination(int n, int r) {
    if (n < 0 || r < 0 || r > n) return 0;
    // nCr == nC(n-r) 성질을 이용해 반복 횟수 최적화.
    r = min(r, n - r);
    long long num = 1, den = 1;
    for (int i = 0; i < r; i++) {
        num = num * (n - i) % MOD;
        den = den * (i + 1) % MOD;
    }
    return num % MOD * fast_power(den, MOD - 2, MOD) % MOD;
}

// ============================================================
// [정렬 1] 버블 정렬 (Bubble Sort)
// 시간복잡도: O(n²)
// ============================================================

vector<int> bubble_sort(vector<int> arr) {
    const int n = static_cast<int>(arr.size());
    for (int i = 0; i < n; i++) {
        bool swapped = false;
        for (int j = 0; j < n - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                swap(arr[j], arr[j + 1]);
                swapped = true;
            }
        }
        if (!swapped) break;   // 이미 정렬됨 → 조기 종료
    }
    return arr;
}

// ============================================================
// [정렬 2] 선택 정렬 (Selection Sort)
// 시간복잡도: O(n²)
// ============================================================

vector<int> selection_sort(vector<int> arr) {
    const int n = static_cast<int>(arr.size());
    for (int i = 0; i < n; i++) {
        int min_idx = i;
        for (int j = i + 1; j < n; j++)
            if (arr[j] < arr[min_idx]) min_idx = j;
        swap(arr[i], arr[min_idx]);
    }
    return arr;
}

// ============================================================
// [정렬 3] 삽입 정렬 (Insertion Sort)
// 시간복잡도: O(n²) / 거의 정렬된 경우 O(n)
// ============================================================

vector<int> insertion_sort(vector<int> arr) {
    const int n = static_cast<int>(arr.size());
    for (int i = 1; i < n; i++) {
        int key = arr[i];
        int j = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
    return arr;
}

// ============================================================
// [정렬 4] 병합 정렬 (Merge Sort)
// 시간복잡도: O(n log n) / 안정 정렬
// ============================================================

vector<int> merge_two(const vector<int>& left, const vector<int>& right) {
    vector<int> result;
    result.reserve(left.size() + right.size());
    int i = 0, j = 0;
    while (i < (int)left.size() && j < (int)right.size()) {
        if (left[i] <= right[j]) result.push_back(left[i++]);
        else                     result.push_back(right[j++]);
    }
    while (i < (int)left.size())  result.push_back(left[i++]);
    while (j < (int)right.size()) result.push_back(right[j++]);
    return result;
}

vector<int> merge_sort(vector<int> arr) {
    if (arr.size() <= 1) return arr;
    const int mid = static_cast<int>(arr.size()) / 2;
    vector<int> left  = merge_sort(vector<int>(arr.begin(), arr.begin() + mid));
    vector<int> right = merge_sort(vector<int>(arr.begin() + mid, arr.end()));
    return merge_two(left, right);
}

// ============================================================
// [정렬 5] 퀵 정렬 (Quick Sort)
// 시간복잡도: 평균 O(n log n) / 최악 O(n²)
// ============================================================

vector<int> quick_sort(vector<int> arr) {
    if (arr.size() <= 1) return arr;
    // 중앙값 기반 pivot 선택(단순 구현).
    const int pivot = arr[arr.size() / 2];
    vector<int> left, mid, right;
    left.reserve(arr.size());
    mid.reserve(arr.size());
    right.reserve(arr.size());
    for (int x : arr) {
        if (x < pivot)      left.push_back(x);
        else if (x == pivot) mid.push_back(x);
        else                right.push_back(x);
    }
    auto l = quick_sort(left);
    auto r = quick_sort(right);
    l.insert(l.end(), mid.begin(), mid.end());
    l.insert(l.end(), r.begin(), r.end());
    return l;
}

// ============================================================
// [정렬 6] 계수 정렬 (Counting Sort)
// 시간복잡도: O(n + k)
// ============================================================

vector<int> counting_sort(vector<int> arr) {
    if (arr.empty()) return arr;
    const int min_val = *min_element(arr.begin(), arr.end());
    const int max_val = *max_element(arr.begin(), arr.end());

    // 음수 데이터도 처리하기 위해 offset(min_val)을 사용한다.
    const int offset = -min_val;
    vector<int> count(max_val - min_val + 1, 0);
    for (int x : arr) count[x + offset]++;

    vector<int> result;
    result.reserve(arr.size());
    for (int val = min_val; val <= max_val; val++) {
        const int frequency = count[val + offset];
        for (int c = 0; c < frequency; c++) {
            result.push_back(val);
        }
    }
    return result;
}

// ============================================================
// [정렬 응용 7] 커스텀 정렬 (점수 내림차순, 이름 오름차순)
// ============================================================

void sort_students() {
    vector<pair<string, int>> students = {
        {"Alice", 88}, {"Bob", 95}, {"Charlie", 88},
        {"Diana", 72}, {"Eve", 95}
    };

    // 1순위: 점수 내림차순, 2순위: 이름 오름차순.
    sort(students.begin(), students.end(), [](const auto& a, const auto& b) {
        if (a.second != b.second) return a.second > b.second;
        return a.first < b.first;
    });

    cout << "\n=== [정렬 7] 커스텀 정렬 ===" << endl;
    for (const auto& [name, score] : students)
        cout << "  " << name << ": " << score << endl;
}

// ============================================================
// [정렬 응용 8] K번째로 큰 수 (최소 힙)
// 시간복잡도: O(n log k)
// ============================================================

int kth_largest(const vector<int>& arr, int k) {
    // 잘못된 요청(예: k <= 0, k > n)은 INT_MIN으로 신호 처리.
    if (k <= 0 || k > static_cast<int>(arr.size())) return numeric_limits<int>::min();

    priority_queue<int, vector<int>, greater<int>> min_heap;  // 크기 k 유지용 최소 힙
    for (int num : arr) {
        min_heap.push(num);
        if (static_cast<int>(min_heap.size()) > k) min_heap.pop();
    }
    return min_heap.top();
}

// ============================================================
// [정렬 응용 9] 구간 병합 (Merge Intervals)
// 시간복잡도: O(n log n)
// ============================================================

vector<IntPair> merge_intervals(vector<IntPair> intervals) {
    if (intervals.empty()) return {};

    sort(intervals.begin(), intervals.end());   // 시작점 기준 정렬
    vector<IntPair> merged;
    merged.reserve(intervals.size());
    merged.push_back(intervals[0]);

    // 첫 원소는 이미 merged에 넣었으므로 2번째 원소부터 순회.
    for (size_t i = 1; i < intervals.size(); i++) {
        const auto& [start, end] = intervals[i];
        if (start <= merged.back().second)
            merged.back().second = max(merged.back().second, end);
        else
            merged.push_back({start, end});
    }
    return merged;
}

// ============================================================
// [정렬 응용 10] 가장 큰 수 만들기
// 시간복잡도: O(n log n)
// ============================================================

string largest_number(const vector<int>& nums) {
    if (nums.empty()) return "0";

    vector<string> strs;
    strs.reserve(nums.size());
    for (int n : nums) strs.push_back(to_string(n));
    sort(strs.begin(), strs.end(), [](const string& a, const string& b) {
        return a + b > b + a;   // "93" vs "39" → "93" 먼저
    });
    string result;
    for (const auto& s : strs) result += s;
    return (result[0] == '0') ? "0" : result;
}

// ============================================================
// 헬퍼: 벡터 출력
// ============================================================

void print_vec(const vector<int>& v) {
    cout << "[";
    for (int i = 0; i < static_cast<int>(v.size()); i++) {
        cout << v[i];
        if (i + 1 < static_cast<int>(v.size())) cout << ", ";
    }
    cout << "]" << endl;
}

// ============================================================
// main
// ============================================================

int main() {
    ios_base::sync_with_stdio(false);
    cin.tie(nullptr);

    // boolalpha를 켜면 true/false를 1/0 대신 문자열로 출력한다.
    cout << boolalpha;

    // ── 수학 1: 소수 판별 ──
    cout << "=== [수학 1] 소수 판별 ===" << endl;
    cout << is_prime(17) << endl;   // true
    cout << is_prime(18) << endl;   // false
    cout << is_prime(2)  << endl;   // true

    // ── 수학 2: 에라토스테네스의 체 ──
    cout << "\n=== [수학 2] 에라토스테네스의 체 ===" << endl;
    print_vec(sieve_of_eratosthenes(30));

    // ── 수학 3: GCD & LCM ──
    cout << "\n=== [수학 3] GCD & LCM ===" << endl;
    cout << "GCD(12, 18) = " << gcd(12, 18) << endl;   // 6
    cout << "LCM(12, 18) = " << lcm(12, 18) << endl;   // 36

    // ── 수학 4: 피보나치 ──
    cout << "\n=== [수학 4] 피보나치 수열 ===" << endl;
    cout << "[";
    for (int i = 0; i <= 10; i++) {
        cout << fibonacci(i);
        if (i < 10) cout << ", ";
    }
    cout << "]" << endl;

    // ── 수학 5: 빠른 거듭제곱 ──
    cout << "\n=== [수학 5] 빠른 거듭제곱 ===" << endl;
    cout << fast_power(2, 10, 1000) << endl;   // 24

    // ── 수학 6: 조합 ──
    cout << "\n=== [수학 6] 조합 nCr ===" << endl;
    cout << combination(10, 3) << endl;   // 120
    cout << combination(5, 2)  << endl;   // 10

    // ── 정렬 1~6 ──
    const vector<int> arr1 = {64, 34, 25, 12, 22, 11, 90};

    cout << "\n=== [정렬 1] 버블 정렬 ===" << endl;
    print_vec(bubble_sort(arr1));

    cout << "\n=== [정렬 2] 선택 정렬 ===" << endl;
    print_vec(selection_sort({64, 25, 12, 22, 11}));

    cout << "\n=== [정렬 3] 삽입 정렬 ===" << endl;
    print_vec(insertion_sort({12, 11, 13, 5, 6}));

    cout << "\n=== [정렬 4] 병합 정렬 ===" << endl;
    print_vec(merge_sort({38, 27, 43, 3, 9, 82, 10}));

    cout << "\n=== [정렬 5] 퀵 정렬 ===" << endl;
    print_vec(quick_sort({3, 6, 8, 10, 1, 2, 1}));

    cout << "\n=== [정렬 6] 계수 정렬 ===" << endl;
    print_vec(counting_sort({4, 2, 2, 8, 3, 3, 1}));
    print_vec(counting_sort({-3, 0, 2, -1, 2, -3}));  // 음수 예시

    // ── 정렬 응용 7: 커스텀 ──
    sort_students();

    // ── 정렬 응용 8: K번째 큰 수 ──
    cout << "\n=== [정렬 8] K번째로 큰 수 (힙) ===" << endl;
    cout << kth_largest({3, 2, 1, 5, 6, 4}, 2) << endl;            // 5
    cout << kth_largest({3, 2, 3, 1, 2, 4, 5, 5, 6}, 4) << endl;   // 4

    // ── 정렬 응용 9: 구간 병합 ──
    cout << "\n=== [정렬 9] 구간 병합 ===" << endl;
    const auto merged = merge_intervals({{1,3},{2,6},{8,10},{15,18}});
    for (const auto& [s, e] : merged)
        cout << "[" << s << ", " << e << "] ";
    cout << endl;

    // ── 정렬 응용 10: 가장 큰 수 ──
    cout << "\n=== [정렬 10] 가장 큰 수 만들기 ===" << endl;
    cout << largest_number({3, 30, 34, 5, 9}) << endl;   // 9534330
    cout << largest_number({10, 2})           << endl;   // 210

    cout << R"(
╔══════════════════════════════════════════════════════╗
║            알고리즘 시간복잡도 정리                    ║
╠════════════════════╦════════════╦════════════════════╣
║ 알고리즘            ║ 시간복잡도  ║ 특징               ║
╠════════════════════╬════════════╬════════════════════╣
║ 소수 판별           ║ O(√n)      ║ sqrt() 활용         ║
║ 에라토스테네스의 체  ║ O(n log n) ║ vector<bool> 효율  ║
║ 유클리드 GCD        ║ O(log n)   ║ std::gcd (C++17)   ║
║ 빠른 거듭제곱        ║ O(log b)   ║ 비트 연산 b >>= 1  ║
╠════════════════════╬════════════╬════════════════════╣
║ 버블/선택/삽입 정렬  ║ O(n²)      ║ swap() 활용        ║
║ 병합 정렬           ║ O(n log n) ║ 안정 정렬           ║
║ 퀵 정렬             ║ O(n log n) ║ pivot 분할         ║
║ 계수 정렬           ║ O(n+k)     ║ max_element 활용   ║
╚════════════════════╩════════════╩════════════════════╝
)" << endl;

    return 0;
}
