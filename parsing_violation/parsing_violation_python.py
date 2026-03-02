"""
====================================================================
  Navifra · SVE (Robotics) 코딩 테스트 대비
  A. 로그/데이터 파싱 & 집계
  B. 규칙 위반 탐지 시뮬레이션
====================================================================
  컴파일/실행: python3 parsing_violation.py
====================================================================
"""

import re
import csv
import json
import heapq
from io import StringIO
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
from collections import defaultdict, deque
from enum import Enum
import math
import random

# ────────────────────────────────────────────────────────────────
# 공통 유틸
# ────────────────────────────────────────────────────────────────

RESET  = "\033[0m"
BOLD   = "\033[1m"
GREEN  = "\033[32m"
CYAN   = "\033[36m"
YELLOW = "\033[33m"
RED    = "\033[31m"

def section(title: str):
    bar = "═" * 54
    print(f"\n{BOLD}{CYAN}{bar}\n  {title}\n{bar}{RESET}")

def ok(label: str, val: str = ""):
    print(f"{GREEN}  [✓]{RESET} {label}" + (f": {BOLD}{val}{RESET}" if val else ""))

def warn(label: str, val: str = ""):
    print(f"{YELLOW}  [!]{RESET} {label}" + (f": {BOLD}{val}{RESET}" if val else ""))

def err(label: str, val: str = ""):
    print(f"{RED}  [✗]{RESET} {label}" + (f": {BOLD}{val}{RESET}" if val else ""))


# ====================================================================
# A. 로그/데이터 파싱 & 집계
# ====================================================================
#
#  자율주행 검증 엔지니어 실무 맥락
#    - ROS 2 bag / rosbag 로그에서 이벤트 추출
#    - 센서 latency / 드롭 통계 집계
#    - 시나리오 결과 CSV 파싱 → 합격/불합격 판정
#
#  시간복잡도 요약
#    단순 파싱    : O(N)     N = 라인 수
#    정렬 집계    : O(N log N)
#    슬라이딩 윈도우 통계 : O(N)
# ====================================================================

# ──────────────────────────────────────────────────────────────────
# A-1. ROS 스타일 텍스트 로그 파싱
# ──────────────────────────────────────────────────────────────────
# 시나리오: 자율주행 시스템 로그에서 레벨별 이벤트를 추출하고
#           노드별 ERROR/WARN 빈도를 집계하라.
#
# 로그 형식:
#   [timestamp] [LEVEL] [node_name]: message
#
# 시간 복잡도: O(N)
# ──────────────────────────────────────────────────────────────────

# 샘플 로그 (실제 시험에서는 파일로 제공)
SAMPLE_LOG = """\
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
"""

@dataclass
class LogEntry:
    timestamp: float
    level:     str          # INFO / WARN / ERROR / FATAL
    node:      str
    message:   str

# 정규식: [timestamp] [LEVEL] [node]: message
_LOG_RE = re.compile(
    r"\[(?P<ts>\d+\.\d+)\]\s+"
    r"\[(?P<level>[A-Z]+)\s*\]\s+"
    r"\[(?P<node>[^\]]+)\]:\s+"
    r"(?P<msg>.+)"
)

def parse_log(raw: str) -> List[LogEntry]:
    """
    텍스트 로그 → LogEntry 리스트
    시간 복잡도: O(N)
    """
    entries = []
    for line in raw.strip().splitlines():
        m = _LOG_RE.match(line.strip())
        if m:
            entries.append(LogEntry(
                timestamp=float(m.group("ts")),
                level=m.group("level").strip(),
                node=m.group("node").strip(),
                message=m.group("msg").strip(),
            ))
    return entries

def aggregate_by_node(entries: List[LogEntry]) -> Dict[str, Dict[str, int]]:
    """
    노드별 레벨 빈도 집계
    반환: { node: { level: count } }
    시간 복잡도: O(N)
    """
    stats: Dict[str, Dict[str, int]] = defaultdict(lambda: defaultdict(int))
    for e in entries:
        stats[e.node][e.level] += 1
    return stats

def first_fatal(entries: List[LogEntry]) -> Optional[LogEntry]:
    """첫 번째 FATAL 이벤트 탐색   O(N)"""
    for e in entries:
        if e.level == "FATAL":
            return e
    return None

def error_rate_per_second(entries: List[LogEntry]) -> Dict[int, int]:
    """
    1초 단위 버킷별 ERROR+FATAL 이벤트 수
    시간 복잡도: O(N)
    """
    bucket: Dict[int, int] = defaultdict(int)
    for e in entries:
        if e.level in ("ERROR", "FATAL"):
            bucket[int(e.timestamp)] += 1
    return dict(sorted(bucket.items()))


# ──────────────────────────────────────────────────────────────────
# A-2. CSV 시나리오 결과 파싱 & 합격 판정
# ──────────────────────────────────────────────────────────────────
# 시나리오: 시뮬레이터가 출력한 시나리오 결과 CSV를 파싱하여
#           각 항목의 합격/불합격 여부와 전체 통과율을 계산하라.
#
# CSV 컬럼:
#   scenario_id, category, max_lateral_err_m,
#   max_speed_kmh, collision, test_duration_s
#
# 판정 기준:
#   max_lateral_err_m <= 0.30   (허용 횡방향 오차)
#   max_speed_kmh     <= 80.0   (최고속도 제한)
#   collision         == False
#
# 시간 복잡도: O(N)
# ──────────────────────────────────────────────────────────────────

SAMPLE_CSV = """\
scenario_id,category,max_lateral_err_m,max_speed_kmh,collision,test_duration_s
SC001,lane_keep,0.21,72.3,False,45.2
SC002,emergency_stop,0.08,65.0,False,12.1
SC003,obstacle_avoidance,0.35,58.4,False,30.7
SC004,intersection,0.18,45.2,True,22.5
SC005,lane_change,0.29,78.1,False,38.9
SC006,parking,0.12,15.0,False,60.3
SC007,highway_merge,0.41,88.2,False,55.0
SC008,emergency_stop,0.06,60.5,False,10.8
"""

@dataclass
class ScenarioResult:
    scenario_id:      str
    category:         str
    max_lateral_err:  float
    max_speed:        float
    collision:        bool
    test_duration:    float
    passed:           bool = field(init=False)
    fail_reasons:     List[str] = field(default_factory=list, init=False)

    LATERAL_LIMIT = 0.30
    SPEED_LIMIT   = 80.0

    def __post_init__(self):
        reasons = []
        if self.max_lateral_err > self.LATERAL_LIMIT:
            reasons.append(
                f"lateral_err {self.max_lateral_err:.2f}m > {self.LATERAL_LIMIT}m")
        if self.max_speed > self.SPEED_LIMIT:
            reasons.append(
                f"speed {self.max_speed:.1f}km/h > {self.SPEED_LIMIT}km/h")
        if self.collision:
            reasons.append("collision detected")
        self.fail_reasons = reasons
        self.passed = len(reasons) == 0

def parse_scenario_csv(raw: str) -> List[ScenarioResult]:
    """
    CSV → ScenarioResult 리스트
    시간 복잡도: O(N)
    """
    reader = csv.DictReader(StringIO(raw.strip()))
    results = []
    for row in reader:
        results.append(ScenarioResult(
            scenario_id=row["scenario_id"],
            category=row["category"],
            max_lateral_err=float(row["max_lateral_err_m"]),
            max_speed=float(row["max_speed_kmh"]),
            collision=row["collision"].strip().lower() == "true",
            test_duration=float(row["test_duration_s"]),
        ))
    return results

def aggregate_scenario_stats(results: List[ScenarioResult]) -> dict:
    """
    전체 통과율 + 카테고리별 통과율 집계
    시간 복잡도: O(N)
    """
    total   = len(results)
    passed  = sum(1 for r in results if r.passed)
    by_cat: Dict[str, List[bool]] = defaultdict(list)
    for r in results:
        by_cat[r.category].append(r.passed)

    cat_stats = {
        cat: {
            "pass": sum(v),
            "total": len(v),
            "rate": f"{sum(v)/len(v)*100:.0f}%"
        }
        for cat, v in by_cat.items()
    }
    return {
        "total": total,
        "passed": passed,
        "pass_rate": f"{passed/total*100:.1f}%",
        "by_category": cat_stats,
    }


# ──────────────────────────────────────────────────────────────────
# A-3. 슬라이딩 윈도우 — 센서 지연(latency) 이상 탐지
# ──────────────────────────────────────────────────────────────────
# 시나리오: 연속된 LiDAR 타임스탬프 시퀀스에서
#           윈도우 크기 W 내 평균 latency가 임계치를 초과하는
#           구간(시작 인덱스)을 모두 찾아라.
#
# 시간 복잡도: O(N)   (슬라이딩 합산)
# ──────────────────────────────────────────────────────────────────

def find_high_latency_windows(
    latencies_ms: List[float],
    window:       int,
    threshold_ms: float,
) -> List[int]:
    """
    반환: 윈도우 평균 > threshold_ms 인 시작 인덱스 리스트
    시간 복잡도: O(N)
    """
    n = len(latencies_ms)
    if n < window:
        return []

    # 첫 윈도우 합 초기화
    win_sum = sum(latencies_ms[:window])
    bad_windows = []

    if win_sum / window > threshold_ms:
        bad_windows.append(0)

    for i in range(1, n - window + 1):
        win_sum += latencies_ms[i + window - 1] - latencies_ms[i - 1]
        if win_sum / window > threshold_ms:
            bad_windows.append(i)

    return bad_windows


# ====================================================================
# B. 규칙 위반 탐지 시뮬레이션
# ====================================================================
#
#  자율주행 검증 엔지니어 실무 맥락
#    - 차량 거동 시퀀스에서 교통 규칙 위반 탐지
#    - 센서 데이터 연속성 위반 탐지
#    - 이벤트 시퀀스에서 금지 패턴 탐색 (유한 오토마톤)
#
# ====================================================================

# ──────────────────────────────────────────────────────────────────
# B-1. 차량 거동 규칙 위반 탐지
# ──────────────────────────────────────────────────────────────────
# 규칙 집합:
#   R1. 속도 초과        speed > speed_limit
#   R2. 급가속           acceleration > 3.0 m/s²
#   R3. 급감속           deceleration < -5.0 m/s²  (emergency brake)
#   R4. 횡방향 오차 초과 |lateral_err| > 0.30 m
#   R5. 최소 안전 거리   distance_to_front < 2.0 s * speed
#
# 시간 복잡도: O(N × R)   N=프레임 수, R=규칙 수 (상수)
# ──────────────────────────────────────────────────────────────────

class ViolationType(Enum):
    SPEED_LIMIT   = "R1_속도초과"
    RAPID_ACCEL   = "R2_급가속"
    HARD_BRAKE    = "R3_급감속"
    LATERAL_ERR   = "R4_횡방향오차"
    SAFETY_DIST   = "R5_안전거리부족"

@dataclass
class VehicleFrame:
    t:             float    # timestamp [s]
    speed:         float    # [m/s]
    acceleration:  float    # [m/s²]
    lateral_err:   float    # [m]  양수=우측 이탈
    dist_to_front: float    # [m]  전방 차량까지 거리
    speed_limit:   float    # [m/s]

@dataclass
class Violation:
    t:         float
    vtype:     ViolationType
    detail:    str

def detect_vehicle_violations(frames: List[VehicleFrame]) -> List[Violation]:
    """
    프레임 시퀀스 → 규칙 위반 목록
    시간 복잡도: O(N)
    """
    ACCEL_LIMIT  =  3.0   # m/s²
    BRAKE_LIMIT  = -5.0   # m/s²
    LAT_LIMIT    =  0.30  # m
    HEADWAY_TIME =  2.0   # s (time-gap 기준)

    violations = []
    for f in frames:
        if f.speed > f.speed_limit:
            violations.append(Violation(
                f.t, ViolationType.SPEED_LIMIT,
                f"speed={f.speed:.1f} > limit={f.speed_limit:.1f} m/s"
            ))
        if f.acceleration > ACCEL_LIMIT:
            violations.append(Violation(
                f.t, ViolationType.RAPID_ACCEL,
                f"accel={f.acceleration:.2f} > {ACCEL_LIMIT} m/s²"
            ))
        if f.acceleration < BRAKE_LIMIT:
            violations.append(Violation(
                f.t, ViolationType.HARD_BRAKE,
                f"accel={f.acceleration:.2f} < {BRAKE_LIMIT} m/s²"
            ))
        if abs(f.lateral_err) > LAT_LIMIT:
            violations.append(Violation(
                f.t, ViolationType.LATERAL_ERR,
                f"|lateral|={abs(f.lateral_err):.2f} > {LAT_LIMIT} m"
            ))
        safe_dist = HEADWAY_TIME * f.speed
        if f.dist_to_front < safe_dist:
            violations.append(Violation(
                f.t, ViolationType.SAFETY_DIST,
                f"dist={f.dist_to_front:.1f}m < safe={safe_dist:.1f}m"
            ))
    return violations

def summarize_violations(violations: List[Violation]) -> Dict[str, int]:
    """위반 유형별 빈도 집계   O(N)"""
    counts: Dict[str, int] = defaultdict(int)
    for v in violations:
        counts[v.vtype.value] += 1
    return dict(counts)


# ──────────────────────────────────────────────────────────────────
# B-2. 유한 오토마톤(FSM) 기반 금지 시퀀스 탐지
# ──────────────────────────────────────────────────────────────────
# 시나리오: 자율주행 이벤트 스트림에서 다음 금지 패턴을 탐지하라.
#
#   금지 패턴 1: OBSTACLE_DETECTED → (< 1.0s) 안에 BRAKE 없음
#   금지 패턴 2: LANE_CHANGE → LANE_CHANGE  (연속 차선 변경)
#   금지 패턴 3: SPEED_UP → SPEED_UP → SPEED_UP  (3회 연속 가속)
#
# 접근법: 각 패턴마다 간단한 FSM / 슬라이딩 카운터로 O(N) 탐지
#
# 시간 복잡도: O(N)
# ──────────────────────────────────────────────────────────────────

@dataclass
class Event:
    t:    float
    etype: str   # OBSTACLE_DETECTED / BRAKE / LANE_CHANGE / SPEED_UP / ...

@dataclass
class PatternViolation:
    t:       float
    pattern: str
    detail:  str

def detect_pattern_violations(events: List[Event]) -> List[PatternViolation]:
    """
    FSM 기반 이벤트 스트림 금지 패턴 탐지
    시간 복잡도: O(N)
    """
    results = []
    n = len(events)

    # ── 패턴 1: OBSTACLE_DETECTED 후 1.0s 내 BRAKE 없음
    #    FSM: waiting_brake=False → OBSTACLE_DETECTED 시 True + 타임스탬프 기록
    #         BRAKE 수신 시 False / 1.0s 초과 시 위반
    waiting_brake  = False
    obstacle_t     = 0.0
    BRAKE_WINDOW   = 1.0  # [s]

    # ── 패턴 3: 연속 SPEED_UP 3회
    consecutive_speedup = 0

    prev_etype = ""

    for i, e in enumerate(events):
        # 패턴 1 타임아웃 체크
        if waiting_brake and (e.t - obstacle_t) > BRAKE_WINDOW:
            results.append(PatternViolation(
                obstacle_t,
                "P1_장애물후제동없음",
                f"OBSTACLE@{obstacle_t:.2f}s → no BRAKE within {BRAKE_WINDOW}s"
            ))
            waiting_brake = False

        if e.etype == "OBSTACLE_DETECTED":
            waiting_brake = True
            obstacle_t = e.t
        elif e.etype == "BRAKE" and waiting_brake:
            waiting_brake = False   # 정상 처리

        # 패턴 2: 연속 LANE_CHANGE
        if e.etype == "LANE_CHANGE" and prev_etype == "LANE_CHANGE":
            results.append(PatternViolation(
                e.t, "P2_연속차선변경",
                f"LANE_CHANGE×2 @{e.t:.2f}s"
            ))

        # 패턴 3: 연속 SPEED_UP 3회
        if e.etype == "SPEED_UP":
            consecutive_speedup += 1
            if consecutive_speedup >= 3:
                results.append(PatternViolation(
                    e.t, "P3_연속가속3회",
                    f"SPEED_UP×{consecutive_speedup} @{e.t:.2f}s"
                ))
        else:
            consecutive_speedup = 0

        prev_etype = e.etype

    # 마지막 OBSTACLE 처리 안 된 경우 처리
    if waiting_brake:
        results.append(PatternViolation(
            obstacle_t, "P1_장애물후제동없음",
            f"OBSTACLE@{obstacle_t:.2f}s → stream ended without BRAKE"
        ))

    return results


# ──────────────────────────────────────────────────────────────────
# B-3. 센서 데이터 연속성 위반 탐지 (타임스탬프 gap / 역전)
# ──────────────────────────────────────────────────────────────────
# 시나리오: LiDAR / Camera 타임스탬프 시퀀스에서 아래를 탐지하라.
#   - gap   : 연속 타임스탬프 간격이 expected_dt * 2 이상
#   - jitter : 간격이 expected_dt ± tolerance 범위 밖
#   - reorder: 타임스탬프 역전 (t[i] <= t[i-1])
#
# 시간 복잡도: O(N)
# ──────────────────────────────────────────────────────────────────

@dataclass
class SensorAnomaly:
    index:  int
    t:      float
    atype:  str    # "GAP" / "JITTER" / "REORDER"
    detail: str

def detect_sensor_anomalies(
    timestamps:   List[float],
    expected_hz:  float,       # 예: 10.0 → 100ms 주기
    tolerance:    float = 0.2, # ±20%
) -> List[SensorAnomaly]:
    """
    타임스탬프 시퀀스 이상 탐지
    시간 복잡도: O(N)
    """
    dt_expected = 1.0 / expected_hz
    dt_min = dt_expected * (1 - tolerance)
    dt_max = dt_expected * (1 + tolerance)
    gap_threshold = dt_expected * 2.0

    anomalies = []
    for i in range(1, len(timestamps)):
        dt = timestamps[i] - timestamps[i-1]
        if dt <= 0:
            anomalies.append(SensorAnomaly(
                i, timestamps[i], "REORDER",
                f"t[{i}]={timestamps[i]:.4f} <= t[{i-1}]={timestamps[i-1]:.4f}"
            ))
        elif dt >= gap_threshold:
            anomalies.append(SensorAnomaly(
                i, timestamps[i], "GAP",
                f"dt={dt*1000:.1f}ms >> expected={dt_expected*1000:.1f}ms"
            ))
        elif not (dt_min <= dt <= dt_max):
            anomalies.append(SensorAnomaly(
                i, timestamps[i], "JITTER",
                f"dt={dt*1000:.1f}ms out of [{dt_min*1000:.1f}, {dt_max*1000:.1f}]ms"
            ))
    return anomalies


# ====================================================================
# 테스트 / 동작 검증
# ====================================================================

if __name__ == "__main__":

    print(f"{BOLD}\n  Navifra SVE (Robotics) — 파싱/집계/위반탐지 대비{RESET}")

    # ================================================================
    section("A-1. ROS 로그 파싱 & 집계")
    # ================================================================

    entries = parse_log(SAMPLE_LOG)
    ok(f"파싱된 로그 수", str(len(entries)))

    stats = aggregate_by_node(entries)
    print(f"\n  {'노드명':<22} {'INFO':>5} {'WARN':>5} {'ERROR':>6} {'FATAL':>6}")
    print(f"  {'─'*22} {'─'*5} {'─'*5} {'─'*6} {'─'*6}")
    for node, lvls in sorted(stats.items()):
        print(f"  {node:<22} "
              f"{lvls.get('INFO',0):>5} "
              f"{lvls.get('WARN',0):>5} "
              f"{lvls.get('ERROR',0):>6} "
              f"{lvls.get('FATAL',0):>6}")

    fatal = first_fatal(entries)
    if fatal:
        warn(f"첫 FATAL", f"t={fatal.timestamp}s  [{fatal.node}]: {fatal.message}")

    rate = error_rate_per_second(entries)
    print(f"\n  초별 ERROR/FATAL 수: {dict(rate)}")

    # ================================================================
    section("A-2. 시나리오 CSV 파싱 & 합격 판정")
    # ================================================================

    scenarios = parse_scenario_csv(SAMPLE_CSV)
    print(f"\n  {'시나리오':<8} {'카테고리':<22} {'결과':<6} 실패 이유")
    print(f"  {'─'*8} {'─'*22} {'─'*6} {'─'*30}")
    for r in scenarios:
        tag = f"{GREEN}PASS{RESET}" if r.passed else f"{RED}FAIL{RESET}"
        reason = "; ".join(r.fail_reasons) if r.fail_reasons else "-"
        print(f"  {r.scenario_id:<8} {r.category:<22} {tag}   {reason}")

    agg = aggregate_scenario_stats(scenarios)
    print(f"\n  전체: {agg['passed']}/{agg['total']} 통과  "
          f"({BOLD}{agg['pass_rate']}{RESET})")

    # ================================================================
    section("A-3. 슬라이딩 윈도우 — 센서 Latency 이상 탐지")
    # ================================================================

    random.seed(0)
    latencies = [random.gauss(10, 2) for _ in range(50)]
    # 구간 [20:30] 에 인위적 지연 주입
    for i in range(20, 30):
        latencies[i] += 15
    bad = find_high_latency_windows(latencies, window=5, threshold_ms=15.0)
    ok(f"고지연 윈도우 인덱스 (window=5, threshold=15ms)", str(bad))

    # ================================================================
    section("B-1. 차량 거동 규칙 위반 탐지")
    # ================================================================

    frames = [
        VehicleFrame(t=0.0, speed=10.0, acceleration=0.5,  lateral_err=0.10, dist_to_front=25.0, speed_limit=13.9),
        VehicleFrame(t=0.1, speed=14.5, acceleration=3.5,  lateral_err=0.12, dist_to_front=22.0, speed_limit=13.9),  # R1+R2
        VehicleFrame(t=0.2, speed=14.0, acceleration=-6.0, lateral_err=0.35, dist_to_front=18.0, speed_limit=13.9),  # R1+R3+R4
        VehicleFrame(t=0.3, speed=13.0, acceleration=-1.0, lateral_err=0.10, dist_to_front=5.0,  speed_limit=13.9),  # R5 (safe=26m)
        VehicleFrame(t=0.4, speed=12.0, acceleration=0.0,  lateral_err=0.05, dist_to_front=30.0, speed_limit=13.9),
    ]
    viols = detect_vehicle_violations(frames)
    summary = summarize_violations(viols)
    print(f"\n  발생 위반 총 {len(viols)}건:")
    for v in viols:
        err(f"t={v.t:.1f}s  {v.vtype.value}", v.detail)
    print(f"\n  유형별 집계: {summary}")

    # ================================================================
    section("B-2. FSM 기반 금지 패턴 탐지")
    # ================================================================

    events = [
        Event(0.0,  "SPEED_UP"),
        Event(0.5,  "SPEED_UP"),
        Event(1.0,  "SPEED_UP"),           # P3: 연속 가속 3회
        Event(1.5,  "LANE_CHANGE"),
        Event(1.8,  "LANE_CHANGE"),         # P2: 연속 차선 변경
        Event(2.0,  "OBSTACLE_DETECTED"),
        Event(2.3,  "SPEED_UP"),            # BRAKE 없이 가속
        Event(3.5,  "BRAKE"),               # P1: 1.0s 초과 후 제동
        Event(4.0,  "OBSTACLE_DETECTED"),   # P1: stream 종료 전 BRAKE 없음
    ]
    pviols = detect_pattern_violations(events)
    print(f"\n  탐지된 패턴 위반 {len(pviols)}건:")
    for pv in pviols:
        err(f"t={pv.t:.2f}s  {pv.pattern}", pv.detail)

    # ================================================================
    section("B-3. 센서 타임스탬프 연속성 위반 탐지")
    # ================================================================

    # 10Hz 기준 타임스탬프 생성 (100ms 간격)
    ts = [i * 0.1 for i in range(30)]
    ts[8]  = ts[7] + 0.005   # jitter
    ts[15] = ts[14] - 0.001  # reorder (역전)
    ts[22] = ts[21] + 0.35   # gap (3.5× expected)

    anomalies = detect_sensor_anomalies(ts, expected_hz=10.0, tolerance=0.2)
    print(f"\n  탐지된 센서 이상 {len(anomalies)}건 (10Hz, ±20%):")
    for a in anomalies:
        warn(f"idx={a.index}  [{a.atype}]", a.detail)

    print(f"\n{BOLD}{GREEN}  모든 테스트 완료 ✓{RESET}\n")
