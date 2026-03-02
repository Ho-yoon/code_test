"""
====================================================================
  Navifra · Software Verification Engineer (Robotics) 코딩 테스트 대비
====================================================================

우선순위 1 : 그래프 탐색 (BFS / DFS)
우선순위 2 : 우선순위 큐 / Dijkstra
우선순위 3 : ROS 2 — LiDAR 데이터 처리

자율주행 검증 엔지니어 맥락에서 출제 가능한 문제들로 구성
====================================================================
"""

# ====================================================================
# [1순위] 그래프 탐색 — BFS / DFS
# ====================================================================
#
# 시간복잡도 요약
#   BFS / DFS (인접 리스트) : O(V + E)
#   BFS / DFS (인접 행렬)   : O(V²)
#
# 자율주행 연관 출제 유형
#   - 격자(grid) 맵에서 경로 존재 여부 확인
#   - 연결된 장애물 영역(connected components) 계산
#   - 시나리오 상태 전이 그래프 탐색
# ====================================================================

from collections import defaultdict, deque
from typing import Deque, Dict, List, Optional, Tuple
import heapq


# ------------------------------------------------------------------
# 1-A. 기본 BFS — 격자 맵 최단 경로 (장애물 포함)
# ------------------------------------------------------------------
# 시나리오: 자율주행 차량이 occupancy grid 에서 시작점→목적지
#           최소 이동 칸 수를 구하라.
#
# 시간 복잡도: O(R × C)   (R=행, C=열)
# 공간 복잡도: O(R × C)
# ------------------------------------------------------------------

def bfs_shortest_path(
    grid: List[List[int]],
    start: Tuple[int, int],
    end: Tuple[int, int],
) -> int:
    """
    grid: 0=이동 가능, 1=장애물
    반환: 최단 이동 칸 수, 경로 없으면 -1
    """
    # 입력 유효성: 비어 있는 격자는 경로 탐색 자체가 불가능하다.
    if not grid or not grid[0]:
        return -1

    # 행/열 길이를 지역 변수로 캐싱하면 반복문에서 가독성과 성능이 모두 좋아진다.
    R, C = len(grid), len(grid[0])

    # 시작점/도착점이 격자 범위를 벗어나면 잘못된 입력이므로 실패 처리한다.
    if not (0 <= start[0] < R and 0 <= start[1] < C):
        return -1
    if not (0 <= end[0] < R and 0 <= end[1] < C):
        return -1

    # 시작/도착 칸이 장애물인 경우에는 경로가 존재할 수 없다.
    if grid[start[0]][start[1]] == 1 or grid[end[0]][end[1]] == 1:
        return -1

    visited = [[False] * C for _ in range(R)]
    queue: Deque[Tuple[int, int, int]] = deque()
    queue.append((start[0], start[1], 0))   # (row, col, steps)
    visited[start[0]][start[1]] = True
    DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 상/하/좌/우 4방향 이동

    while queue:
        r, c, steps = queue.popleft()
        if (r, c) == end:
            return steps
        for dr, dc in DIRS:
            nr, nc = r + dr, c + dc
            # 조건을 순서대로 분리하면 디버깅할 때 어떤 조건에서 탈락했는지 파악하기 쉽다.
            if not (0 <= nr < R and 0 <= nc < C):
                continue
            if visited[nr][nc]:
                continue
            if grid[nr][nc] == 1:
                continue

            visited[nr][nc] = True
            queue.append((nr, nc, steps + 1))
    return -1


# ------------------------------------------------------------------
# 1-B. BFS — 연결 장애물 영역 수 세기 (Connected Components)
# ------------------------------------------------------------------
# 시나리오: LiDAR 포인트 클라우드를 2D 격자로 투영했을 때
#           독립적인 장애물 클러스터 개수를 세어라.
#
# 시간 복잡도: O(R × C)
# ------------------------------------------------------------------

def count_obstacle_clusters(grid: List[List[int]]) -> int:
    """grid: 1=장애물, 0=빈 공간 — 연결된 장애물 덩어리 수 반환"""
    if not grid or not grid[0]:
        return 0

    R, C = len(grid), len(grid[0])
    visited = [[False] * C for _ in range(R)]
    DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    count = 0

    def bfs(sr: int, sc: int) -> None:
        # 하나의 시작 장애물 셀에서 BFS를 수행하여 같은 클러스터를 전부 방문 처리한다.
        queue: Deque[Tuple[int, int]] = deque([(sr, sc)])
        visited[sr][sc] = True
        while queue:
            r, c = queue.popleft()
            for dr, dc in DIRS:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < R and 0 <= nc < C):
                    continue
                if visited[nr][nc] or grid[nr][nc] == 0:
                    continue

                visited[nr][nc] = True
                queue.append((nr, nc))

    for r in range(R):
        for c in range(C):
            if grid[r][c] == 1 and not visited[r][c]:
                bfs(r, c)
                count += 1
    return count


# ------------------------------------------------------------------
# 1-C. DFS — 사이클 감지 (시나리오 상태 전이 검증)
# ------------------------------------------------------------------
# 시나리오: 자율주행 소프트웨어의 상태 전이 다이어그램에
#           무한 루프(사이클)가 있는지 검사하라. (방향 그래프)
#
# 시간 복잡도: O(V + E)
# ------------------------------------------------------------------

def has_cycle_directed(n: int, edges: List[Tuple[int, int]]) -> bool:
    """
    n     : 노드 수 (0 ~ n-1)
    edges : [(from, to), ...]
    반환  : 사이클 존재 여부
    """
    if n <= 0:
        return False

    graph: List[List[int]] = [[] for _ in range(n)]
    for u, v in edges:
        # 잘못된 인덱스는 탐색 시 런타임 에러를 만들 수 있으므로 사전 필터링한다.
        if not (0 <= u < n and 0 <= v < n):
            continue
        graph[u].append(v)

    # 0=미방문, 1=방문중(스택에 있음), 2=완료
    state = [0] * n

    def dfs(node: int) -> bool:
        state[node] = 1
        for nxt in graph[node]:
            if state[nxt] == 1:          # 현재 경로에서 재방문 → 사이클
                return True
            if state[nxt] == 0 and dfs(nxt):
                return True
        state[node] = 2
        return False

    for i in range(n):
        if state[i] == 0:
            if dfs(i):
                return True
    return False


# ====================================================================
# [2순위] 우선순위 큐 / Dijkstra
# ====================================================================
#
# 시간복잡도 요약
#   Dijkstra (heapq + 인접 리스트) : O((V + E) log V)
#   Bellman-Ford                    : O(V × E)  — 음수 가중치 허용
#
# 자율주행 연관 출제 유형
#   - 도로 네트워크 최단 경로
#   - 비용(연료, 시간) 최소 경로
#   - A* 탐색 전처리
# ====================================================================

INF = float('inf')


# ------------------------------------------------------------------
# 2-A. Dijkstra — 단일 출발지 최단 경로
# ------------------------------------------------------------------
# 시나리오: 도로 네트워크(가중치=소요시간)에서 차량 출발지부터
#           모든 노드까지의 최단 시간을 구하라.
#
# 시간 복잡도: O((V + E) log V)
# 공간 복잡도: O(V + E)
# ------------------------------------------------------------------

def dijkstra(
    n: int,
    edges: List[Tuple[int,int,float]],   # (from, to, weight)
    src: int,
) -> List[float]:
    """
    반환: dist[i] = src → i 최단 거리 (도달 불가 시 INF)
    """
    if n <= 0:
        return []
    if not (0 <= src < n):
        return [INF] * n

    graph: Dict[int, List[Tuple[float, int]]] = defaultdict(list)
    for u, v, w in edges:
        if not (0 <= u < n and 0 <= v < n):
            continue
        if w < 0:
            # Dijkstra의 전제(비음수 간선)를 보존하기 위해 음수 가중치는 무시한다.
            continue
        graph[u].append((w, v))

    dist = [INF] * n
    dist[src] = 0
    heap = [(0.0, src)]                  # (cost, node)

    while heap:
        cost, u = heapq.heappop(heap)
        if cost > dist[u]:               # 이미 더 짧은 경로 처리됨
            continue
        for w, v in graph[u]:
            new_cost = cost + w
            if new_cost < dist[v]:
                dist[v] = new_cost
                heapq.heappush(heap, (new_cost, v))
    return dist


# ------------------------------------------------------------------
# 2-B. Dijkstra with Path Reconstruction (경로 추적)
# ------------------------------------------------------------------
# 시나리오: 계획된 경로의 waypoint 시퀀스를 실제로 출력하라.
#
# 시간 복잡도: O((V + E) log V)
# ------------------------------------------------------------------

def dijkstra_with_path(
    n: int,
    edges: List[Tuple[int,int,float]],
    src: int,
    dst: int,
) -> Tuple[float, List[int]]:
    """반환: (최단 거리, 경로 노드 리스트)"""
    if n <= 0 or not (0 <= src < n and 0 <= dst < n):
        return INF, []

    graph: Dict[int, List[Tuple[float, int]]] = defaultdict(list)
    for u, v, w in edges:
        if not (0 <= u < n and 0 <= v < n):
            continue
        if w < 0:
            continue
        graph[u].append((w, v))

    dist = [INF] * n
    prev = [-1] * n
    dist[src] = 0
    heap = [(0.0, src)]

    while heap:
        cost, u = heapq.heappop(heap)
        if cost > dist[u]:
            continue
        for w, v in graph[u]:
            nc = cost + w
            if nc < dist[v]:
                dist[v] = nc
                prev[v] = u
                heapq.heappush(heap, (nc, v))

    if dist[dst] == INF:
        return INF, []

    # prev 배열을 따라 역추적한 뒤 reverse하면 src -> dst 순서 경로가 된다.
    path: List[int] = []
    cur = dst
    while cur != -1:
        path.append(cur)
        cur = prev[cur]
    path.reverse()
    return dist[dst], path


# ------------------------------------------------------------------
# 2-C. 우선순위 큐 — k번째 가장 가까운 장애물
# ------------------------------------------------------------------
# 시나리오: LiDAR 포인트 클라우드에서 차량으로부터
#           k번째로 가까운 장애물 포인트를 찾아라.
#
# 시간 복잡도: O(N log k)   — 최대 힙 크기 k 유지
# ------------------------------------------------------------------

def kth_closest_obstacle(
    points: List[Tuple[float,float,float]],   # (x, y, z)
    origin: Tuple[float,float,float],
    k: int,
) -> Optional[Tuple[float,float,float]]:
    """반환: k번째로 가까운 포인트 (없으면 None)"""
    if k <= 0:
        return None

    ox, oy, oz = origin
    max_heap = []                        # Python은 최소 힙 → 음수로 최대 힙 구현

    for x, y, z in points:
        dist_sq = (x-ox)**2 + (y-oy)**2 + (z-oz)**2
        heapq.heappush(max_heap, (-dist_sq, x, y, z))
        if len(max_heap) > k:
            heapq.heappop(max_heap)      # 가장 먼 것 제거

    if len(max_heap) < k:
        return None
    # 음수 거리 중 최솟값(가장 큰 음수 절댓값)이 실제로는 k번째 최근접 거리다.
    _, x, y, z = min(max_heap)
    return (x, y, z)


# ====================================================================
# [3순위] ROS 2 — LiDAR 데이터 처리 노드
# ====================================================================
#
# 실제 ROS 2 환경이 없어도 면접에서 코드 구조와 로직 설명이 핵심.
# 실행 가능 여부보다 "어떻게 설계했는가"를 보여주는 것이 중요.
#
# 핵심 포인트
#   - rclpy Node 상속 구조
#   - 토픽 Subscriber / Publisher 패턴
#   - sensor_msgs/msg/PointCloud2 처리
#   - 간단한 장애물 필터링 로직 (거리 임계값)
# ====================================================================

ROS2_NODE_CODE = '''
#!/usr/bin/env python3
"""
ROS 2 LiDAR 처리 노드
========================
역할  : /lidar/points 토픽 구독 → 근접 장애물 필터링 → /obstacles 퍼블리시
의존  : rclpy, sensor_msgs, std_msgs, numpy
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct


# ----------------------------------------------------------------
# PointCloud2 <-> NumPy 변환 유틸
# 시간복잡도: O(N)  N = 포인트 수
# ----------------------------------------------------------------

def pointcloud2_to_numpy(msg: PointCloud2) -> np.ndarray:
    """
    sensor_msgs/PointCloud2 → (N, 3) float32 배열 (x, y, z)
    """
    point_step = msg.point_step          # 포인트 하나의 바이트 크기
    data = np.frombuffer(msg.data, dtype=np.uint8)
    n_points = msg.width * msg.height

    # x, y, z 오프셋 추출 (기본 Velodyne/OS1 레이아웃: 0, 4, 8)
    offsets = {field.name: field.offset for field in msg.fields}
    x_off, y_off, z_off = offsets['x'], offsets['y'], offsets['z']

    points = np.zeros((n_points, 3), dtype=np.float32)
    for i in range(n_points):
        base = i * point_step
        points[i, 0] = struct.unpack_from('f', msg.data, base + x_off)[0]
        points[i, 1] = struct.unpack_from('f', msg.data, base + y_off)[0]
        points[i, 2] = struct.unpack_from('f', msg.data, base + z_off)[0]

    return points                        # shape: (N, 3)


def numpy_to_pointcloud2(
    points: np.ndarray,
    frame_id: str,
    stamp,
) -> PointCloud2:
    """(N, 3) float32 배열 → sensor_msgs/PointCloud2"""
    msg = PointCloud2()
    msg.header = Header()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id

    msg.height = 1
    msg.width = points.shape[0]
    msg.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12                  # 3 × 4 bytes
    msg.row_step = msg.point_step * msg.width
    msg.data = points.astype(np.float32).tobytes()
    msg.is_dense = True
    return msg


# ----------------------------------------------------------------
# LiDAR 처리 노드
# ----------------------------------------------------------------

class LidarObstacleDetector(Node):
    """
    구독 : /lidar/points  (sensor_msgs/PointCloud2)
    발행 : /obstacles      (sensor_msgs/PointCloud2) — 필터링된 장애물 포인트
           /obstacle_count (std_msgs/Int32)          — 클러스터 수 (간략)

    파라미터
    --------
    max_range     : float  — 처리 최대 거리 [m] (기본 30.0)
    min_height    : float  — 지면 제거 최소 높이 [m] (기본 -0.5)
    max_height    : float  — 천장 제거 최대 높이 [m] (기본 3.0)
    voxel_size    : float  — 다운샘플 복셀 크기 [m]  (기본 0.1)
    """

    def __init__(self):
        super().__init__('lidar_obstacle_detector')

        # ROS 2 파라미터 선언 및 로드
        self.declare_parameter('max_range',  30.0)
        self.declare_parameter('min_height', -0.5)
        self.declare_parameter('max_height',  3.0)
        self.declare_parameter('voxel_size',  0.1)

        self.max_range  = self.get_parameter('max_range').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.voxel_size = self.get_parameter('voxel_size').value

        # Subscriber / Publisher
        self.sub = self.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.lidar_callback,
            10,                          # QoS depth
        )
        self.pub_obstacles = self.create_publisher(
            PointCloud2, '/obstacles', 10
        )

        self.get_logger().info('LidarObstacleDetector 노드 시작')

    # ------------------------------------------------------------
    # 메인 콜백
    # 시간복잡도: O(N)  N = 입력 포인트 수
    # ------------------------------------------------------------

    def lidar_callback(self, msg: PointCloud2) -> None:
        """LiDAR 포인트 수신 → 필터링 → 발행"""
        try:
            points = pointcloud2_to_numpy(msg)          # (N, 3)

            # 1. 거리 필터 — O(N)
            filtered = self._range_filter(points)

            # 2. 높이 필터 (지면 + 천장 제거) — O(N)
            filtered = self._height_filter(filtered)

            # 3. 복셀 다운샘플 (처리 부하 감소) — O(N log N)
            filtered = self._voxel_downsample(filtered)

            # 4. 결과 발행
            out_msg = numpy_to_pointcloud2(
                filtered,
                frame_id=msg.header.frame_id,
                stamp=msg.header.stamp,
            )
            self.pub_obstacles.publish(out_msg)

            self.get_logger().debug(
                f'입력: {len(points)}pt → 출력: {len(filtered)}pt'
            )

        except Exception as e:
            self.get_logger().error(f'콜백 오류: {e}')

    # ------------------------------------------------------------
    # 필터 메서드
    # ------------------------------------------------------------

    def _range_filter(self, points: np.ndarray) -> np.ndarray:
        """
        원점(차량)으로부터 max_range 이내 포인트만 보존
        시간복잡도: O(N)
        """
        dist_sq = points[:, 0]**2 + points[:, 1]**2   # XY 평면 거리²
        mask = dist_sq <= self.max_range ** 2
        return points[mask]

    def _height_filter(self, points: np.ndarray) -> np.ndarray:
        """
        z축(높이) 범위 필터 — 지면 노이즈 및 천장 제거
        시간복잡도: O(N)
        """
        mask = (points[:, 2] >= self.min_height) & \
               (points[:, 2] <= self.max_height)
        return points[mask]

    def _voxel_downsample(self, points: np.ndarray) -> np.ndarray:
        """
        복셀 그리드 다운샘플링 — 각 복셀 내 포인트를 평균으로 대표
        시간복잡도: O(N log N)  (정렬 기반 그룹핑)
        """
        if len(points) == 0:
            return points

        voxel_indices = np.floor(points / self.voxel_size).astype(np.int32)

        # 복셀 인덱스를 정수 키로 변환 후 그룹 평균
        keys = (voxel_indices[:, 0].astype(np.int64) * 1_000_003 +
                voxel_indices[:, 1].astype(np.int64) * 1_009 +
                voxel_indices[:, 2].astype(np.int64))

        sort_idx = np.argsort(keys)
        sorted_keys = keys[sort_idx]
        sorted_points = points[sort_idx]

        # 동일 복셀 구간의 평균 계산
        _, first_idx = np.unique(sorted_keys, return_index=True)
        last_idx = np.append(first_idx[1:], len(sorted_keys))

        downsampled = np.array([
            sorted_points[s:e].mean(axis=0)
            for s, e in zip(first_idx, last_idx)
        ], dtype=np.float32)

        return downsampled


# ----------------------------------------------------------------
# 엔트리 포인트
# ----------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
'''


# ====================================================================
# 테스트 / 사용 예시
# ====================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("  코딩 테스트 대비 — 동작 검증")
    print("=" * 60)

    # --- 1-A. BFS 최단 경로 ---
    grid = [
        [0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1],
        [1, 0, 1, 0, 0],
        [0, 0, 0, 1, 0],
        [0, 1, 0, 0, 0],
    ]
    result = bfs_shortest_path(grid, (0,0), (4,4))
    print(f"\n[1-A] BFS 최단 경로 (0,0)→(4,4): {result}칸")

    # --- 1-B. 장애물 클러스터 ---
    obs_grid = [
        [1, 1, 0, 0, 1],
        [1, 0, 0, 1, 1],
        [0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0],
    ]
    clusters = count_obstacle_clusters(obs_grid)
    print(f"[1-B] 장애물 클러스터 수: {clusters}개")

    # --- 1-C. 사이클 감지 ---
    edges_cycle    = [(0,1),(1,2),(2,0),(2,3)]
    edges_no_cycle = [(0,1),(1,2),(0,2)]
    print(f"[1-C] 사이클 감지: {has_cycle_directed(4, edges_cycle)} (사이클 있음)")
    print(f"[1-C] 사이클 감지: {has_cycle_directed(3, edges_no_cycle)} (사이클 없음)")

    # --- 2-A. Dijkstra ---
    road_edges = [
        (0,1,4.0),(0,2,1.0),(2,1,2.0),
        (1,3,1.0),(2,3,5.0),(3,4,3.0),
    ]
    dist = dijkstra(5, road_edges, src=0)
    print(f"\n[2-A] Dijkstra 최단 거리 (src=0): {dist}")

    # --- 2-B. 경로 추적 ---
    d, path = dijkstra_with_path(5, road_edges, src=0, dst=4)
    print(f"[2-B] 0→4 최단 경로: {path}, 거리: {d}")

    # --- 2-C. k번째 가까운 장애물 ---
    import random
    random.seed(42)
    lidar_pts = [(random.uniform(-30,30), random.uniform(-30,30), 0.0)
                 for _ in range(1000)]
    kth = kth_closest_obstacle(lidar_pts, (0.0,0.0,0.0), k=3)
    if kth:
        dist_k = (kth[0]**2 + kth[1]**2)**0.5
        print(f"[2-C] 3번째 가까운 포인트: {kth}, 거리: {dist_k:.3f}m")

    # --- 3. ROS 2 노드 코드 출력 안내 ---
    print("\n" + "=" * 60)
    print("  [3순위] ROS 2 LiDAR 노드 코드")
    print("  → 변수 ROS2_NODE_CODE 에 저장됨")
    print("  → lidar_obstacle_detector.py 로 저장 후 colcon build")
    print("=" * 60)
    print("\n코딩 테스트 대비 완료 ✓")
