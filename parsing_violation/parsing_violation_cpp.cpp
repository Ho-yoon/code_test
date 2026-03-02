/*
====================================================================
  Navifra · SVE (Robotics) 코딩 테스트 대비
  A. 로그/데이터 파싱 & 집계
  B. 규칙 위반 탐지 시뮬레이션
  C++ 구현
====================================================================
  컴파일: g++ -std=c++17 -O2 -o parsing_violation parsing_violation.cpp
====================================================================
*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <regex>
#include <optional>
#include <variant>
#include <algorithm>
#include <numeric>
#include <functional>
#include <iomanip>
#include <cmath>
#include <cassert>
#include <random>

// ────────────────────────────────────────────────────────────────
// 터미널 컬러 유틸
// ────────────────────────────────────────────────────────────────

#define RESET  "\033[0m"
#define BOLD   "\033[1m"
#define GREEN  "\033[32m"
#define CYAN   "\033[36m"
#define YELLOW "\033[33m"
#define RED    "\033[31m"

static void section(const std::string& title) {
    std::string bar(54, '=');
    std::cout << "\n" << BOLD << CYAN
              << bar << "\n  " << title << "\n" << bar
              << RESET << "\n";
}
static void ok  (const std::string& l, const std::string& v = "") {
    std::cout << GREEN << "  [✓] " << RESET << l
              << (v.empty() ? "" : ": " + std::string(BOLD) + v + RESET) << "\n";
}
static void warn(const std::string& l, const std::string& v = "") {
    std::cout << YELLOW << "  [!] " << RESET << l
              << (v.empty() ? "" : ": " + std::string(BOLD) + v + RESET) << "\n";
}
static void err (const std::string& l, const std::string& v = "") {
    std::cout << RED << "  [✗] " << RESET << l
              << (v.empty() ? "" : ": " + std::string(BOLD) + v + RESET) << "\n";
}


// ====================================================================
// A-1. ROS 스타일 텍스트 로그 파싱 & 집계
// ====================================================================
//
//  로그 형식:  [timestamp] [LEVEL] [node_name]: message
//
//  시간 복잡도
//    parse_log             : O(N)
//    aggregate_by_node     : O(N)
//    error_rate_per_second : O(N)
// ====================================================================

struct LogEntry {
    double      timestamp;
    std::string level;    // INFO / WARN / ERROR / FATAL
    std::string node;
    std::string message;
};

// ──────────────────────────────────────────────────────────────────
// parse_log  — 정규식 기반 텍스트 파싱
// 시간 복잡도: O(N)   N = 라인 수
// ──────────────────────────────────────────────────────────────────
std::vector<LogEntry> parseLog(const std::string& raw) {
    // [1.200] [WARN ] [lidar_node       ]: Point cloud drop ...
    static const std::regex RE(
        R"((\[\d+\.\d+\]\s+\[[A-Z]+\s*\]\s+\[[^\]]+\]:\s+.+))"
    );

    std::vector<LogEntry> entries;
    std::istringstream ss(raw);
    std::string line;

    // Adjusted regex to match the provided format more robustly.
    // Original regex: R"(\[(\d+\.\d+)\]\s+\[([A-Z]+)\s*\]\s+\[([^\]]+)\]:\s+(.+))"
    // The issue might be in how regex groups are captured or the whitespace handling. Let's make it simpler.
    // Re-evaluating the original regex for parsing issue, it seems correct for direct parsing.
    // The syntax error is due to Colab trying to run C++ as python.
    // The current fix for the regex is not needed if the initial error was related to C++ not Python
    // I will revert to the original regex as it is more specific for parsing.

    static const std::regex actual_RE(
        R"(\[(\d+\.\d+)\]\s+\[([A-Z]+)\s*\]\s+\[([^\]]+)\]:\s+(.+))"
    );

    while (std::getline(ss, line)) {
        if (line.empty()) continue;
        std::smatch m;
        if (std::regex_match(line, m, actual_RE)) {
            LogEntry e;
            e.timestamp = std::stod(m[1]);
            e.level     = m[2];
            e.node      = m[3];
            e.message   = m[4];

            // 앞뒤 공백 제거 (node 필드)
            auto trim = [](std::string s) {
                s.erase(s.begin(),
                    std::find_if(s.begin(), s.end(),
                        [](unsigned char c){ return !std::isspace(c); }));
                s.erase(std::find_if(s.rbegin(), s.rend(),
                    [](unsigned char c){ return !std::isspace(c); }).base(),
                    s.end());
                return s;
            };
            e.level = trim(e.level);
            e.node  = trim(e.node);
            entries.push_back(std::move(e));
        }
    }
    return entries;
}

// ──────────────────────────────────────────────────────────────────
// aggregate_by_node — 노드별 레벨 빈도 집계
// 반환: { node → { level → count } }
// 시간 복잡도: O(N)
// ──────────────────────────────────────────────────────────────────
using NodeStats = std::map<std::string, std::map<std::string, int>>;

NodeStats aggregateByNode(const std::vector<LogEntry>& entries) {
    NodeStats stats;
    for (const auto& e : entries)
        stats[e.node][e.level]++;
    return stats;
}

// ──────────────────────────────────────────────────────────────────
// firstFatal — 첫 번째 FATAL 이벤트
// 시간 복잡도: O(N)
// ──────────────────────────────────────────────────────────────────
std::optional<LogEntry> firstFatal(const std::vector<LogEntry>& entries) {
    for (const auto& e : entries)
        if (e.level == "FATAL") return e;
    return std::nullopt;
}

// ──────────────────────────────────────────────────────────────────
// errorRatePerSecond — 1초 버킷별 ERROR+FATAL 수
// 시간 복잡도: O(N)
// ──────────────────────────────────────────────────────────────────
std::map<int,int> errorRatePerSecond(const std::vector<LogEntry>& entries) {
    std::map<int,int> bucket;
    for (const auto& e : entries)
        if (e.level == "ERROR" || e.level == "FATAL")
            bucket[static_cast<int>(e.timestamp)]++;
    return bucket;
}


// ====================================================================
// A-2. CSV 시나리오 결과 파싱 & 합격 판정
// ====================================================================
//
//  CSV 컬럼:
//    scenario_id, category, max_lateral_err_m,
//    max_speed_kmh, collision, test_duration_s
//
//  판정 기준
//    max_lateral_err_m <= 0.30
//    max_speed_kmh     <= 80.0
//    collision         == false
//
//  시간 복잡도: O(N)
// ====================================================================

struct ScenarioResult {
    std::string scenario_id;
    std::string category;
    float       max_lateral_err;
    float       max_speed;
    bool        collision;
    float       test_duration;
    bool        passed;
    std::vector<std::string> fail_reasons;

    static constexpr float LATERAL_LIMIT = 0.30f;
    static constexpr float SPEED_LIMIT   = 80.0f;
};

// ──────────────────────────────────────────────────────────────────
// parseScenarioCsv — 헤더 파싱 후 각 행 판정
// 시간 복잡도: O(N)
// ──────────────────────────────────────────────────────────────────

// CSV 한 행을 ',' 기준으로 분리
static std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> cols;
    std::stringstream ss(line);
    std::string tok;
    while (std::getline(ss, tok, ','))
        cols.push_back(tok);
    return cols;
}

// 문자열 앞뒤 공백·개행 제거
static std::string trim(const std::string& s) {
    size_t b = s.find_first_not_of(" \t\r\n");
    size_t e = s.find_last_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    return s.substr(b, e - b + 1);
}

std::vector<ScenarioResult> parseScenarioCsv(const std::string& raw) {
    std::vector<ScenarioResult> results;
    std::istringstream ss(raw);
    std::string line;
    bool header = true;

    // 컬럼 인덱스 (헤더에서 동적 탐색)
    int idx_id=-1, idx_cat=-1, idx_lat=-1, idx_spd=-1,
        idx_col=-1, idx_dur=-1;

    while (std::getline(ss, line)) {
        line = trim(line);
        if (line.empty()) continue;
        auto cols = splitCsv(line);

        if (header) {                         // 헤더 행 처리
            for (int i = 0; i < (int)cols.size(); ++i) {
                std::string h = trim(cols[i]);
                if (h == "scenario_id")        idx_id  = i;
                else if (h == "category")       idx_cat = i;
                else if (h == "max_lateral_err_m") idx_lat = i;
                else if (h == "max_speed_kmh")  idx_spd = i;
                else if (h == "collision")       idx_col = i;
                else if (h == "test_duration_s") idx_dur = i;
            }
            header = false;
            continue;
        }

        if ((int)cols.size() <= idx_dur) continue;

        ScenarioResult r;
        r.scenario_id    = trim(cols[idx_id]);
        r.category       = trim(cols[idx_cat]);
        r.max_lateral_err= std::stof(trim(cols[idx_lat]));
        r.max_speed      = std::stof(trim(cols[idx_spd]));
        r.collision      = (trim(cols[idx_col]) == "True");
        r.test_duration  = std::stof(trim(cols[idx_dur]));

        // 판정
        if (r.max_lateral_err > ScenarioResult::LATERAL_LIMIT)
            r.fail_reasons.push_back(
                "lateral_err " + std::to_string(r.max_lateral_err).substr(0,4)
                + "m > " + std::to_string(ScenarioResult::LATERAL_LIMIT).substr(0,4) + "m");
        if (r.max_speed > ScenarioResult::SPEED_LIMIT)
            r.fail_reasons.push_back(
                "speed " + std::to_string(r.max_speed).substr(0,4)
                + "km/h > " + std::to_string(ScenarioResult::SPEED_LIMIT).substr(0,4) + "km/h");
        if (r.collision)
            r.fail_reasons.push_back("collision detected");

        r.passed = r.fail_reasons.empty();
        results.push_back(std::move(r));
    }
    return results;
}

struct AggStats {
    int total, passed;
    double pass_rate;
    std::map<std::string, std::pair<int,int>> by_category; // {pass, total}
};

AggStats aggregateScenarioStats(const std::vector<ScenarioResult>& results) {
    AggStats s{};
    s.total  = static_cast<int>(results.size());
    s.passed = 0;
    for (const auto& r : results) {
        if (r.passed) ++s.passed;
        auto& p = s.by_category[r.category];
        if (r.passed) ++p.first;
        ++p.second;
    }
    s.pass_rate = s.total ? 100.0 * s.passed / s.total : 0.0;
    return s;
}


// ====================================================================
// A-3. 슬라이딩 윈도우 — 센서 Latency 이상 탐지
// ====================================================================
//
//  윈도우 크기 W 내 평균 latency > threshold 인 시작 인덱스 탐색
//
//  시간 복잡도: O(N)   슬라이딩 합산 (naive O(N×W) 아님)
// ====================================================================

std::vector<int> findHighLatencyWindows(
    const std::vector<double>& latencies,
    int    window,
    double threshold_ms
) {
    int n = static_cast<int>(latencies.size());
    if (n < window) return {};

    // 첫 윈도우 합산
    double sum = std::accumulate(latencies.begin(),
                                 latencies.begin() + window, 0.0);
    std::vector<int> bad;
    if (sum / window > threshold_ms) bad.push_back(0);

    for (int i = 1; i <= n - window; ++i) {
        sum += latencies[i + window - 1] - latencies[i - 1];  // O(1) 갱신
        if (sum / window > threshold_ms) bad.push_back(i);
    }
    return bad;
}


// ====================================================================
// B-1. 차량 거동 규칙 위반 탐지
// ====================================================================
//
//  규칙
//    R1. 속도 초과        speed > speed_limit
//    R2. 급가속           acceleration > 3.0 m/s²
//    R3. 급감속           acceleration < -5.0 m/s²
//    R4. 횡방향 오차 초과 |lateral_err| > 0.30 m
//    R5. 안전 거리 부족   dist_to_front < 2.0s × speed
//
//  시간 복잡도: O(N × R) = O(N)   R=5 상수
// ====================================================================

enum class ViolationType {
    SPEED_LIMIT,
    RAPID_ACCEL,
    HARD_BRAKE,
    LATERAL_ERR,
    SAFETY_DIST,
};

static std::string vtypeName(ViolationType v) {
    switch(v) {
        case ViolationType::SPEED_LIMIT: return "R1_속도초과";
        case ViolationType::RAPID_ACCEL: return "R2_급가속";
        case ViolationType::HARD_BRAKE:  return "R3_급감속";
        case ViolationType::LATERAL_ERR: return "R4_횡방향오차";
        case ViolationType::SAFETY_DIST: return "R5_안전거리부족";
    }
    return "UNKNOWN";
}

struct VehicleFrame {
    double t;
    double speed;           // [m/s]
    double acceleration;    // [m/s²]
    double lateral_err;     // [m]
    double dist_to_front;   // [m]
    double speed_limit;     // [m/s]
};

struct Violation {
    double        t;
    ViolationType vtype;
    std::string   detail;
};

// ──────────────────────────────────────────────────────────────────
// detectVehicleViolations
// 시간 복잡도: O(N)
// ──────────────────────────────────────────────────────────────────
std::vector<Violation> detectVehicleViolations(
    const std::vector<VehicleFrame>& frames
) {
    constexpr double ACCEL_LIMIT  =  3.0;
    constexpr double BRAKE_LIMIT  = -5.0;
    constexpr double LAT_LIMIT    =  0.30;
    constexpr double HEADWAY_TIME =  2.0;   // [s]

    std::vector<Violation> viols;
    auto push = [&](double t, ViolationType vt, const std::string& d) {
        viols.push_back({t, vt, d});
    };

    auto fmt = [](double v, int prec=2) {
        std::ostringstream os;
        os << std::fixed << std::setprecision(prec) << v;
        return os.str();
    };

    for (const auto& f : frames) {
        if (f.speed > f.speed_limit)
            push(f.t, ViolationType::SPEED_LIMIT,
                 "speed=" + fmt(f.speed) + " > limit=" + fmt(f.speed_limit) + " m/s");

        if (f.acceleration > ACCEL_LIMIT)
            push(f.t, ViolationType::RAPID_ACCEL,
                 "accel=" + fmt(f.acceleration) + " > " + fmt(ACCEL_LIMIT) + " m/s²");

        if (f.acceleration < BRAKE_LIMIT)
            push(f.t, ViolationType::HARD_BRAKE,
                 "accel=" + fmt(f.acceleration) + " < " + fmt(BRAKE_LIMIT) + " m/s²");

        if (std::abs(f.lateral_err) > LAT_LIMIT)
            push(f.t, ViolationType::LATERAL_ERR,
                 "|lateral|=" + fmt(std::abs(f.lateral_err))
                 + " > " + fmt(LAT_LIMIT) + " m");

        double safe = HEADWAY_TIME * f.speed;
        if (f.dist_to_front < safe)
            push(f.t, ViolationType::SAFETY_DIST,
                 "dist=" + fmt(f.dist_to_front) + "m < safe=" + fmt(safe) + "m");
    }
    return viols;
}

// ──────────────────────────────────────────────────────────────────
// summarizeViolations — 유형별 빈도 집계   O(N)
// ──────────────────────────────────────────────────────────────────
std::map<std::string,int> summarizeViolations(
    const std::vector<Violation>& viols
) {
    std::map<std::string,int> cnt;
    for (const auto& v : viols) cnt[vtypeName(v.vtype)]++;
    return cnt;
}


// ====================================================================
// B-2. FSM 기반 금지 이벤트 패턴 탐지
// ====================================================================
//
//  금지 패턴
//    P1. OBSTACLE_DETECTED 후 1.0s 이내 BRAKE 없음
//    P2. LANE_CHANGE → LANE_CHANGE  (연속 차선 변경)
//    P3. SPEED_UP ×3회 연속
//
//  구현: 상태 변수 기반 단일 패스 FSM
//  시간 복잡도: O(N)
// ====================================================================

struct Event {
    double      t;
    std::string etype;
};

struct PatternViolation {
    double      t;
    std::string pattern;
    std::string detail;
};

// ──────────────────────────────────────────────────────────────────
// detectPatternViolations
// 시간 복잡도: O(N)
// ──────────────────────────────────────────────────────────────────
std::vector<PatternViolation> detectPatternViolations(
    const std::vector<Event>& events
) {
    constexpr double BRAKE_WINDOW = 1.0;   // [s]

    std::vector<PatternViolation> results;
    auto push = [&](double t, const std::string& pat, const std::string& d) {
        results.push_back({t, pat, d});
    };
    auto fmtT = [](double t) {
        std::ostringstream os; os << std::fixed << std::setprecision(2) << t;
        return os.str();
    };

    // FSM 상태
    bool   waitingBrake      = false;
    double obstacleT         = 0.0;
    int    consecutiveSpeedup = 0;
    std::string prevEtype    = "";

    for (const auto& e : events) {

        // P1 타임아웃 체크 (현재 이벤트 처리 전)
        if (waitingBrake && (e.t - obstacleT) > BRAKE_WINDOW) {
            push(obstacleT, "P1_장애물후제동없음",
                 "OBSTACLE@" + fmtT(obstacleT)
                 + "s → no BRAKE within " + fmtT(BRAKE_WINDOW) + "s");
            waitingBrake = false;
        }

        if (e.etype == "OBSTACLE_DETECTED") {
            waitingBrake = true;
            obstacleT    = e.t;
        } else if (e.etype == "BRAKE" && waitingBrake) {
            waitingBrake = false;             // 정상 처리
        }

        // P2: 연속 LANE_CHANGE
        if (e.etype == "LANE_CHANGE" && prevEtype == "LANE_CHANGE")
            push(e.t, "P2_연속차선변경",
                 "LANE_CHANGE×2 @" + fmtT(e.t) + "s");

        // P3: 연속 SPEED_UP 3회 이상
        if (e.etype == "SPEED_UP") {
            ++consecutiveSpeedup;
            if (consecutiveSpeedup >= 3)
                push(e.t, "P3_연속가속3회",
                     "SPEED_UP×" + std::to_string(consecutiveSpeedup)
                     + " @" + fmtT(e.t) + "s");
        } else {
            consecutiveSpeedup = 0;
        }

        prevEtype = e.etype;
    }

    // 스트림 종료 후 미처리 OBSTACLE
    if (waitingBrake)
        push(obstacleT, "P1_장애물후제동없음",
             "OBSTACLE@" + fmtT(obstacleT) + "s → stream ended without BRAKE");

    return results;
}


// ====================================================================
// B-3. 센서 타임스탬프 연속성 위반 탐지
// ====================================================================
//
//  탐지 유형
//    GAP     : dt >= expected_dt × 2
//    JITTER  : dt < expected_dt × (1-tol) || dt > expected_dt × (1+tol)
//    REORDER : dt <= 0  (역전 or 중복)
//
//  시간 복잡도: O(N)
// ====================================================================

struct SensorAnomaly {
    int         index;
    double      t;
    std::string atype;    // GAP / JITTER / REORDER
    std::string detail;
};

std::vector<SensorAnomaly> detectSensorAnomalies(
    const std::vector<double>& timestamps,
    double expected_hz,
    double tolerance = 0.2
) {
    double dt_expected  = 1.0 / expected_hz;
    double dt_min       = dt_expected * (1.0 - tolerance);
    double dt_max       = dt_expected * (1.0 + tolerance);
    double gap_threshold= dt_expected * 2.0;

    auto fmtMs = [](double s) {
        std::ostringstream os;
        os << std::fixed << std::setprecision(1) << s * 1000.0;
        return os.str();
    };

    std::vector<SensorAnomaly> anomalies;
    for (int i = 1; i < (int)timestamps.size(); ++i) {
        double dt = timestamps[i] - timestamps[i-1];
        auto fmtT = [&](int idx) {
            std::ostringstream os;
            os << std::fixed << std::setprecision(4) << timestamps[idx];
            return os.str();
        };

        if (dt <= 0.0) {
            anomalies.push_back({i, timestamps[i], "REORDER",
                "t[" + std::to_string(i)   + "]=" + fmtT(i)
                + " <= t[" + std::to_string(i-1) + "]=" + fmtT(i-1)});
        } else if (dt >= gap_threshold) {
            anomalies.push_back({i, timestamps[i], "GAP",
                "dt=" + fmtMs(dt) + "ms >> expected="
                + fmtMs(dt_expected) + "ms"});
        } else if (dt < dt_min || dt > dt_max) {
            anomalies.push_back({i, timestamps[i], "JITTER",
                "dt=" + fmtMs(dt) + "ms out of ["
                + fmtMs(dt_min) + ", " + fmtMs(dt_max) + "]ms"});
        }
    }
    return anomalies;
}


// ====================================================================
// 샘플 데이터
// ====================================================================

static const std::string SAMPLE_LOG = R"(
[1.000] [INFO ] [lidar_node       ]: LiDAR initialized, 64 channels
[1.050] [INFO ] [camera_node      ]: Camera stream started
[1.200] [WARN ] [lidar_node       ]: Point cloud drop detected (seq=42)
[1.350] [ERROR] [perception_node  ]: Object detection timeout (50ms exceeded)
[1.400] [INFO ] [planning_node    ]: Waypoint published, count=12
[1.600] [WARN ] [lidar_node       ]: Point cloud drop detected (seq=87)
[1.700] [ERROR] [control_node     ]: Lateral deviation 0.45m > threshold 0.30m
[1.800] [INFO ] [lidar_node       ]: Point cloud restored
[2.000] [ERROR] [perception_node  ]: Object detection timeout (55ms exceeded)
[2.100] [WARN ] [control_node     ]: Braking force 0.82g near limit 0.80g
[2.300] [ERROR] [control_node     ]: Lateral deviation 0.38m > threshold 0.30m
[2.500] [INFO ] [planning_node    ]: Re-routing triggered
[2.700] [FATAL] [safety_monitor   ]: Emergency stop activated
[2.701] [INFO ] [vehicle_node     ]: Brake applied, speed=0.0 km/h
)";

static const std::string SAMPLE_CSV = R"(scenario_id,category,max_lateral_err_m,max_speed_kmh,collision,test_duration_s
SC001,lane_keep,0.21,72.3,False,45.2
SC002,emergency_stop,0.08,65.0,False,12.1
SC003,obstacle_avoidance,0.35,58.4,False,30.7
SC004,intersection,0.18,45.2,True,22.5
SC005,lane_change,0.29,78.1,False,38.9
SC006,parking,0.12,15.0,False,60.3
SC007,highway_merge,0.41,88.2,False,55.0
SC008,emergency_stop,0.06,60.5,False,10.8
)";


// ====================================================================
// main — 테스트 & 동작 검증
// ====================================================================

int main() {
    std::cout << BOLD
              << "\n  Navifra SVE (Robotics) — 파싱/집계/위반탐지 대비 (C++)\n"
              << RESET;

    // ================================================================
    section("A-1. ROS 로그 파싱 & 집계");
    // ================================================================

    auto entries = parseLog(SAMPLE_LOG);
    ok("파싱된 로그 수", std::to_string(entries.size()));

    auto stats = aggregateByNode(entries);
    std::cout << "\n  " << std::left
              << std::setw(22) << "노드명"
              << std::setw(6)  << "INFO"
              << std::setw(6)  << "WARN"
              << std::setw(7)  << "ERROR"
              << std::setw(7)  << "FATAL" << "\n";
    std::cout << "  " << std::string(48, '-') << "\n";
    for (const auto& [node, lvls] : stats) {
        auto get = [&](const std::string& k) {
            auto it = lvls.find(k);
            return it != lvls.end() ? it->second : 0;
        };
        std::cout << "  " << std::left << std::setw(22) << node
                  << std::setw(6)  << get("INFO")
                  << std::setw(6)  << get("WARN")
                  << std::setw(7)  << get("ERROR")
                  << std::setw(7)  << get("FATAL") << "\n";
    }

    if (auto f = firstFatal(entries)) {
        std::ostringstream os;
        os << std::fixed << std::setprecision(1) << f->timestamp;
        warn("첫 FATAL", "t=" + os.str() + "s  [" + f->node + "]: " + f->message);
    }

    auto rate = errorRatePerSecond(entries);
    std::ostringstream rateStr;
    rateStr << "{";
    bool first_rate = true;
    for (auto& [sec, cnt] : rate) {
        if (!first_rate) rateStr << ", ";
        rateStr << sec << ":" << cnt;
        first_rate = false;
    }
    rateStr << "}";
    std::cout << "\n  초별 ERROR/FATAL 수: " << rateStr.str() << "\n";

    // ================================================================
    section("A-2. 시나리오 CSV 파싱 & 합격 판정");
    // ================================================================

    auto scenarios = parseScenarioCsv(SAMPLE_CSV);
    std::cout << "\n  " << std::left
              << std::setw(9)  << "시나리오"
              << std::setw(22) << "카테고리"
              << std::setw(7)  << "결과"
              << "실패 이유\n";
    std::cout << "  " << std::string(65, '-') << "\n";

    for (const auto& r : scenarios) {
        std::string tag = r.passed
            ? std::string(GREEN) + "PASS" + RESET
            : std::string(RED)   + "FAIL" + RESET;
        std::string reason = r.fail_reasons.empty() ? "-" :
            [&]() {
                std::string s;
                for (size_t i=0; i < r.fail_reasons.size(); ++i) {
                    if (i) s += "; ";
                    s += r.fail_reasons[i];
                }
                return s;
            }();
        std::cout << "  " << std::left
                  << std::setw(9)  << r.scenario_id
                  << std::setw(22) << r.category
                  << tag << "   "
                  << reason << "\n";
    }

    auto agg = aggregateScenarioStats(scenarios);
    std::ostringstream prStr;
    prStr << std::fixed << std::setprecision(1) << agg.pass_rate;
    std::cout << "\n  전체: " << agg.passed << "/" << agg.total
              << " 통과  (" << BOLD << prStr.str() << "%" << RESET << ")\n";

    // ================================================================
    section("A-3. 슬라이딩 윈도우 — 센서 Latency 이상 탐지");
    // ================================================================

    {
        std::mt19937 rng(0);
        std::normal_distribution<double> nd(10.0, 2.0);
        std::vector<double> latencies(50);
        for (auto& v : latencies) v = nd(rng);
        for (int i = 20; i < 30; ++i) latencies[i] += 15.0;   // 인위적 지연

        auto bad = findHighLatencyWindows(latencies, 5, 15.0);
        std::ostringstream bs;
        bs << "[";
        for (size_t i = 0; i < bad.size(); ++i) {
            if (i) bs << ", ";
            bs << bad[i];
        }
        bs << "]";
        ok("고지연 윈도우 인덱스 (window=5, threshold=15ms)", bs.str());
    }

    // ================================================================
    section("B-1. 차량 거동 규칙 위반 탐지");
    // ================================================================

    {
        std::vector<VehicleFrame> frames = {
            {0.0, 10.0,  0.5,  0.10, 25.0, 13.9},
            {0.1, 14.5,  3.5,  0.12, 22.0, 13.9},   // R1+R2
            {0.2, 14.0, -6.0,  0.35, 18.0, 13.9},   // R1+R3+R4
            {0.3, 13.0, -1.0,  0.10,  5.0, 13.9},   // R5 (safe=26m)
            {0.4, 12.0,  0.0,  0.05, 30.0, 13.9},
        };
        auto viols   = detectVehicleViolations(frames);
        auto summary = summarizeViolations(viols);

        std::cout << "\n  발생 위반 총 " << viols.size() << "건:\n";
        auto fmtT = [](double t) {
            std::ostringstream os; os << std::fixed << std::setprecision(1) << t;
            return os.str();
        };
        for (const auto& v : viols)
            err("t=" + fmtT(v.t) + "s  " + vtypeName(v.vtype), v.detail);

        std::ostringstream ss;
        ss << "{";
        bool first = true;
        for (auto& [k,v] : summary) {
            if (!first) ss << ", ";
            ss << k << ":" << v;
            first = false;
        }
        ss << "}";
        std::cout << "\n  유형별 집계: " << ss.str() << "\n";
    }

    // ================================================================
    section("B-2. FSM 기반 금지 패턴 탐지");
    // ================================================================

    {
        std::vector<Event> events = {
            {0.0, "SPEED_UP"},
            {0.5, "SPEED_UP"},
            {1.0, "SPEED_UP"},            // P3
            {1.5, "LANE_CHANGE"},
            {1.8, "LANE_CHANGE"},          // P2
            {2.0, "OBSTACLE_DETECTED"},
            {2.3, "SPEED_UP"},
            {3.5, "BRAKE"},               // P1: 1.5s 경과 후 제동
            {4.0, "OBSTACLE_DETECTED"},   // P1: stream 종료 전 BRAKE 없음
        };
        auto pviols = detectPatternViolations(events);
        std::cout << "\n  탐지된 패턴 위반 " << pviols.size() << "건:\n";
        auto fmtT = [](double t) {
            std::ostringstream os; os << std::fixed << std::setprecision(2) << t;
            return os.str();
        };
        for (const auto& pv : pviols)
            err("t=" + fmtT(pv.t) + "s  " + pv.pattern, pv.detail);
    }

    // ================================================================
    section("B-3. 센서 타임스탬프 연속성 위반 탐지");
    // ================================================================

    {
        // 10Hz 기준 타임스탬프 (100ms 간격)
        std::vector<double> ts(30);
        for (int i = 0; i < 30; ++i) ts[i] = i * 0.1;
        ts[8]  = ts[7]  + 0.005;   // JITTER
        ts[15] = ts[14] - 0.001;   // REORDER
        ts[22] = ts[21] + 0.35;    // GAP

        auto anomalies = detectSensorAnomalies(ts, 10.0, 0.2);
        std::cout << "\n  탐지된 센서 이상 " << anomalies.size()
                  << "건 (10Hz, ±20%):\n";
        for (const auto& a : anomalies)
            warn("idx=" + std::to_string(a.index) + "  [" + a.atype + "]", a.detail);
    }

    std::cout << "\n" << BOLD << GREEN << "  모든 테스트 완료 ✓\n" << RESET << "\n";
    return 0;
}
