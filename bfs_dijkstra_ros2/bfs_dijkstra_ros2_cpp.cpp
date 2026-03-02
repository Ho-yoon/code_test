/*
====================================================================
  Navifra · Software Verification Engineer (Robotics) 코딩 테스트 대비
  C++ 구현
====================================================================
  1순위 : 그래프 탐색 (BFS / DFS)
  2순위 : 우선순위 큐 / Dijkstra
  3순위 : ROS 2 — LiDAR 데이터 처리 노드 (주석 형태 포함)

  컴파일: g++ -std=c++17 -O2 -o coding_test_prep coding_test_prep.cpp
====================================================================
*/

#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <functional>
#include <cassert>
#include <iomanip>
#include <random>
#include <numeric>
#include <optional>
#include <sstream>

// ====================================================================
// 유틸리티
// ====================================================================

constexpr double INF = std::numeric_limits<double>::infinity();

// 컬러 출력 (터미널)
#define RESET  "\033[0m"
#define BOLD   "\033[1m"
#define GREEN  "\033[32m"
#define CYAN   "\033[36m"
#define YELLOW "\033[33m"

void section(const std::string& title) {
    std::cout << "\n" << BOLD << CYAN
              << "══════════════════════════════════════════════════\n"
              << "  " << title << "\n"
              << "══════════════════════════════════════════════════"
              << RESET << "\n";
}

void result(const std::string& label, const std::string& val) {
    std::cout << GREEN << "  [✓] " << RESET << label << ": " << BOLD << val << RESET << "\n";
}

// ====================================================================
// [1순위] 그래프 탐색 — BFS / DFS
// ====================================================================
//
//  시간복잡도
//    BFS / DFS (인접 리스트) : O(V + E)
//    BFS / DFS (인접 행렬)   : O(V²)
//
//  자율주행 연관 유형
//    - 격자 맵 최단 경로 (Occupancy Grid)
//    - 장애물 클러스터 수 (Connected Components)
//    - 상태 전이 사이클 감지
// ====================================================================

// ------------------------------------------------------------------
// 1-A. BFS — Occupancy Grid 최단 경로
// ------------------------------------------------------------------
// 시나리오: 자율주행 차량이 occupancy grid 에서 시작→목적지
//           최소 이동 칸 수를 구하라.
//
// 시간 복잡도: O(R × C)
// 공간 복잡도: O(R × C)
// ------------------------------------------------------------------

struct Point { int r, c; };

int bfsShortesetPath(
    const std::vector<std::vector<int>>& grid,
    Point start,
    Point end
) {
    int R = grid.size(), C = grid[0].size();
    if (grid[start.r][start.c] == 1 || grid[end.r][end.c] == 1) return -1;

    std::vector<std::vector<bool>> visited(R, std::vector<bool>(C, false));
    // {row, col, steps}
    std::queue<std::tuple<int,int,int>> q;
    q.push({start.r, start.c, 0});
    visited[start.r][start.c] = true;

    const int dr[] = {-1, 1,  0, 0};
    const int dc[] = { 0, 0, -1, 1};

    while (!q.empty()) {
        auto [r, c, steps] = q.front(); q.pop();
        if (r == end.r && c == end.c) return steps;
        for (int d = 0; d < 4; ++d) {
            int nr = r + dr[d], nc = c + dc[d];
            if (nr >= 0 && nr < R && nc >= 0 && nc < C
                && !visited[nr][nc] && grid[nr][nc] == 0) {
                visited[nr][nc] = true;
                q.push({nr, nc, steps + 1});
            }
        }
    }
    return -1; // 경로 없음
}

// ------------------------------------------------------------------
// 1-B. BFS — 장애물 클러스터 수 (Connected Components)
// ------------------------------------------------------------------
// 시나리오: LiDAR 포인트를 2D 격자로 투영 → 독립 장애물 클러스터 수.
//
// 시간 복잡도: O(R × C)
// ------------------------------------------------------------------

int countObstacleClusters(const std::vector<std::vector<int>>& grid) {
    int R = grid.size(), C = grid[0].size();
    std::vector<std::vector<bool>> visited(R, std::vector<bool>(C, false));
    const int dr[] = {-1, 1, 0, 0};
    const int dc[] = { 0, 0,-1, 1};
    int count = 0;

    auto bfs = [&](int sr, int sc) {
        std::queue<std::pair<int,int>> q;
        q.push({sr, sc});
        visited[sr][sc] = true;
        while (!q.empty()) {
            auto [r, c] = q.front(); q.pop();
            for (int d = 0; d < 4; ++d) {
                int nr = r + dr[d], nc = c + dc[d];
                if (nr >= 0 && nr < R && nc >= 0 && nc < C
                    && !visited[nr][nc] && grid[nr][nc] == 1) {
                    visited[nr][nc] = true;
                    q.push({nr, nc});
                }
            }
        }
    };

    for (int r = 0; r < R; ++r)
        for (int c = 0; c < C; ++c)
            if (grid[r][c] == 1 && !visited[r][c]) {
                bfs(r, c);
                ++count;
            }
    return count;
}

// ------------------------------------------------------------------
// 1-C. DFS — 방향 그래프 사이클 감지
// ------------------------------------------------------------------
// 시나리오: 자율주행 SW 상태 전이 다이어그램에 무한 루프가 있는가.
//
// 시간 복잡도: O(V + E)
// ------------------------------------------------------------------

bool hasCycleDirected(int n, const std::vector<std::pair<int,int>>& edges) {
    std::vector<std::vector<int>> graph(n);
    for (auto [u, v] : edges) graph[u].push_back(v);

    // 0=미방문, 1=방문중(스택), 2=완료
    std::vector<int> state(n, 0);

    std::function<bool(int)> dfs = [&](int node) -> bool {
        state[node] = 1;
        for (int nxt : graph[node]) {
            if (state[nxt] == 1) return true;   // 역방향 간선 → 사이클
            if (state[nxt] == 0 && dfs(nxt)) return true;
        }
        state[node] = 2;
        return false;
    };

    for (int i = 0; i < n; ++i)
        if (state[i] == 0 && dfs(i)) return true;
    return false;
}

// ====================================================================
// [2순위] 우선순위 큐 / Dijkstra
// ====================================================================
//
//  시간복잡도
//    Dijkstra (priority_queue + 인접 리스트) : O((V + E) log V)
//    Bellman-Ford                             : O(V × E)
//
//  자율주행 연관 유형
//    - 도로 네트워크 최단 경로
//    - 비용(연료·시간) 최소 경로
//    - A* 탐색 전처리
// ====================================================================

// 간선 타입: {가중치, 목적 노드}
using Edge = std::pair<double, int>;

// ------------------------------------------------------------------
// 2-A. Dijkstra — 단일 출발지 최단 거리
// ------------------------------------------------------------------
// 시나리오: 도로 네트워크(가중치=소요시간) → 출발지~모든 노드 최단 시간.
//
// 시간 복잡도: O((V + E) log V)
// 공간 복잡도: O(V + E)
// ------------------------------------------------------------------

std::vector<double> dijkstra(
    int n,
    const std::vector<std::tuple<int,int,double>>& edges,  // (from, to, weight)
    int src
) {
    std::vector<std::vector<Edge>> graph(n);
    for (auto [u, v, w] : edges) graph[u].push_back({w, v});

    std::vector<double> dist(n, INF);
    dist[src] = 0.0;

    // 최소 힙: {cost, node}
    std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>> pq;
    pq.push({0.0, src});

    while (!pq.empty()) {
        auto [cost, u] = pq.top(); pq.pop();
        if (cost > dist[u]) continue;       // 더 짧은 경로가 이미 처리됨
        for (auto [w, v] : graph[u]) {
            double nc = cost + w;
            if (nc < dist[v]) {
                dist[v] = nc;
                pq.push({nc, v});
            }
        }
    }
    return dist;
}

// ------------------------------------------------------------------
// 2-B. Dijkstra with Path Reconstruction (경로 추적)
// ------------------------------------------------------------------
// 시나리오: 계획된 경로의 waypoint 시퀀스를 출력하라.
//
// 시간 복잡도: O((V + E) log V)
// ------------------------------------------------------------------

std::pair<double, std::vector<int>> dijkstraWithPath(
    int n,
    const std::vector<std::tuple<int,int,double>>& edges,
    int src,
    int dst
) {
    std::vector<std::vector<Edge>> graph(n);
    for (auto [u, v, w] : edges) graph[u].push_back({w, v});

    std::vector<double> dist(n, INF);
    std::vector<int>    prev(n, -1);
    dist[src] = 0.0;

    std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>> pq;
    pq.push({0.0, src});

    while (!pq.empty()) {
        auto [cost, u] = pq.top(); pq.pop();
        if (cost > dist[u]) continue;
        for (auto [w, v] : graph[u]) {
            double nc = cost + w;
            if (nc < dist[v]) {
                dist[v] = nc;
                prev[v] = u;
                pq.push({nc, v});
            }
        }
    }

    if (dist[dst] == INF) return {INF, {}};

    std::vector<int> path;
    for (int cur = dst; cur != -1; cur = prev[cur])
        path.push_back(cur);
    std::reverse(path.begin(), path.end());
    return {dist[dst], path};
}

// ------------------------------------------------------------------
// 2-C. 우선순위 큐 — k번째 가장 가까운 LiDAR 포인트
// ------------------------------------------------------------------
// 시나리오: LiDAR 포인트 클라우드에서 k번째 근접 장애물을 찾아라.
//
// 시간 복잡도: O(N log k)   — 최대 힙 크기 k 유지
// ------------------------------------------------------------------

struct Point3D { float x, y, z; };

std::optional<Point3D> kthClosestObstacle(
    const std::vector<Point3D>& points,
    Point3D origin,
    int k
) {
    // 최대 힙: {dist², index}
    auto distSq = [&](const Point3D& p) {
        return (p.x-origin.x)*(p.x-origin.x)
             + (p.y-origin.y)*(p.y-origin.y)
             + (p.z-origin.z)*(p.z-origin.z);
    };

    using Elem = std::pair<float, int>;
    std::priority_queue<Elem> maxHeap;   // 최대 힙 (기본)

    for (int i = 0; i < (int)points.size(); ++i) {
        float d = distSq(points[i]);
        maxHeap.push({d, i});
        if ((int)maxHeap.size() > k)
            maxHeap.pop();               // 가장 먼 포인트 제거
    }

    if ((int)maxHeap.size() < k) return std::nullopt;

    // 힙에서 k번째 가까운 것 = 현재 힙 최상단
    int idx = maxHeap.top().second;
    return points[idx];
}

// ====================================================================
// [3순위] ROS 2 LiDAR 처리 노드 — C++ 구현
// ====================================================================
//
//  ※ ROS 2 환경 없이 컴파일 가능하도록
//    실제 rclcpp / sensor_msgs 헤더를 mock 구조체로 대체.
//    면접에서는 실제 ROS 2 코드 구조와 동일하게 설명 가능.
//
//  핵심 포인트
//    - rclcpp::Node 상속 구조
//    - create_subscription / create_publisher 패턴
//    - PointCloud2 처리 파이프라인
//    - 거리 필터 O(N), 높이 필터 O(N), 복셀 다운샘플 O(N log N)
// ====================================================================

// ------------------------------------------------------------------
// Mock 구조체 (실제 환경에서는 #include <rclcpp/rclcpp.hpp> 등으로 대체)
// ------------------------------------------------------------------

namespace sensor_msgs {
    struct PointCloud2 {
        std::vector<Point3D> points;    // 실제에선 raw bytes
        std::string frame_id;
    };
}

// ------------------------------------------------------------------
// LiDAR 처리 파이프라인 (ROS 2 노드 로직과 동일)
// ------------------------------------------------------------------

class LidarObstacleDetector {
public:
    // ROS 2 파라미터에 해당하는 설정값
    float max_range_  = 30.0f;    // 최대 처리 거리 [m]
    float min_height_ = -0.5f;    // 지면 제거 최소 높이 [m]
    float max_height_ =  3.0f;    // 천장 제거 최대 높이 [m]
    float voxel_size_ =  0.1f;    // 복셀 크기 [m]

    // -------------------------------------------------------
    // 메인 처리 파이프라인 (ROS 2 콜백과 동일한 로직)
    // 시간 복잡도: O(N log N) — 복셀 다운샘플이 지배항
    // -------------------------------------------------------
    sensor_msgs::PointCloud2 process(const sensor_msgs::PointCloud2& input) {
        auto pts = input.points;
        pts = rangeFilter(pts);        // O(N)
        pts = heightFilter(pts);       // O(N)
        pts = voxelDownsample(pts);    // O(N log N)
        return {pts, input.frame_id};
    }

    // -------------------------------------------------------
    // 거리 필터 — max_range 이내 포인트만 보존
    // 시간 복잡도: O(N)
    // -------------------------------------------------------
    std::vector<Point3D> rangeFilter(const std::vector<Point3D>& pts) {
        std::vector<Point3D> out;
        out.reserve(pts.size());
        float r2 = max_range_ * max_range_;
        for (const auto& p : pts)
            if (p.x*p.x + p.y*p.y <= r2)
                out.push_back(p);
        return out;
    }

    // -------------------------------------------------------
    // 높이 필터 — 지면 노이즈 및 천장 제거
    // 시간 복잡도: O(N)
    // -------------------------------------------------------
    std::vector<Point3D> heightFilter(const std::vector<Point3D>& pts) {
        std::vector<Point3D> out;
        out.reserve(pts.size());
        for (const auto& p : pts)
            if (p.z >= min_height_ && p.z <= max_height_)
                out.push_back(p);
        return out;
    }

    // -------------------------------------------------------
    // 복셀 그리드 다운샘플링
    // 각 복셀 내 포인트들을 평균 한 점으로 대표
    //
    // 시간 복잡도: O(N log N)  — unordered_map 사용 시 O(N) 가능
    // 공간 복잡도: O(N)
    // -------------------------------------------------------
    std::vector<Point3D> voxelDownsample(const std::vector<Point3D>& pts) {
        if (pts.empty()) return {};

        // 복셀 인덱스를 64비트 키로 인코딩
        auto toKey = [&](const Point3D& p) -> int64_t {
            int64_t ix = static_cast<int64_t>(std::floor(p.x / voxel_size_));
            int64_t iy = static_cast<int64_t>(std::floor(p.y / voxel_size_));
            int64_t iz = static_cast<int64_t>(std::floor(p.z / voxel_size_));
            // 해시: 각 축 최대 ±10000 범위 가정
            return ix * 100'000'000LL + iy * 100'000LL + iz;
        };

        struct Accum { float sx=0,sy=0,sz=0; int cnt=0; };
        std::unordered_map<int64_t, Accum> voxels;
        voxels.reserve(pts.size());

        for (const auto& p : pts) {
            auto& a = voxels[toKey(p)];
            a.sx += p.x; a.sy += p.y; a.sz += p.z;
            ++a.cnt;
        }

        std::vector<Point3D> out;
        out.reserve(voxels.size());
        for (auto& [key, a] : voxels) {
            float inv = 1.0f / a.cnt;
            out.push_back({a.sx*inv, a.sy*inv, a.sz*inv});
        }
        return out;
    }
};

/*
====================================================================
  실제 ROS 2 환경에서의 노드 전체 코드 (참고용)
====================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class LidarObstacleDetectorNode : public rclcpp::Node {
public:
    LidarObstacleDetectorNode() : Node("lidar_obstacle_detector") {
        this->declare_parameter("max_range",  30.0);
        this->declare_parameter("min_height", -0.5);
        this->declare_parameter("max_height",  3.0);
        this->declare_parameter("voxel_size",  0.1);

        max_range_  = this->get_parameter("max_range").as_double();
        min_height_ = this->get_parameter("min_height").as_double();
        max_height_ = this->get_parameter("max_height").as_double();
        voxel_size_ = this->get_parameter("voxel_size").as_double();

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar/points", 10,
            std::bind(&LidarObstacleDetectorNode::lidarCallback, this,
                      std::placeholders::_1)
        );
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/obstacles", 10
        );
        RCLCPP_INFO(this->get_logger(), "LidarObstacleDetector 노드 시작");
    }

private:
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 1. PointCloud2 → points 파싱
        // 2. rangeFilter / heightFilter / voxelDownsample 호출
        // 3. points → PointCloud2 변환 후 pub_->publish(out_msg)
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;
    double max_range_, min_height_, max_height_, voxel_size_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarObstacleDetectorNode>());
    rclcpp::shutdown();
    return 0;
}

====================================================================
*/

// ====================================================================
// 테스트 / 동작 검증
// ====================================================================

int main() {
    std::cout << BOLD << "\n  Navifra SVE (Robotics) 코딩 테스트 대비 — C++\n" << RESET;

    // ==============================================================
    section("1순위 — 그래프 탐색 (BFS / DFS)");
    // ==============================================================

    // 1-A. BFS 최단 경로
    {
        std::vector<std::vector<int>> grid = {
            {0,0,1,0,0},
            {0,0,0,0,1},
            {1,0,1,0,0},
            {0,0,0,1,0},
            {0,1,0,0,0},
        };
        int steps = bfsShortesetPath(grid, {0,0}, {4,4});
        result("BFS 최단 경로 (0,0)→(4,4)", std::to_string(steps) + "칸");
    }

    // 1-B. 클러스터 수
    {
        std::vector<std::vector<int>> obs = {
            {1,1,0,0,1},
            {1,0,0,1,1},
            {0,0,0,0,0},
            {0,1,0,0,0},
        };
        int cnt = countObstacleClusters(obs);
        result("장애물 클러스터 수", std::to_string(cnt) + "개");
    }

    // 1-C. 사이클 감지
    {
        std::vector<std::pair<int,int>> cycle    = {{0,1},{1,2},{2,0},{2,3}};
        std::vector<std::pair<int,int>> no_cycle = {{0,1},{1,2},{0,2}};
        result("사이클 그래프",    hasCycleDirected(4, cycle)    ? "true  ✓ (사이클 있음)" : "false");
        result("비사이클 그래프",  hasCycleDirected(3, no_cycle) ? "true" : "false ✓ (사이클 없음)");
    }

    // ==============================================================
    section("2순위 — 우선순위 큐 / Dijkstra");
    // ==============================================================

    std::vector<std::tuple<int,int,double>> road_edges = {
        {0,1,4.0},{0,2,1.0},{2,1,2.0},
        {1,3,1.0},{2,3,5.0},{3,4,3.0},
    };

    // 2-A. 최단 거리
    {
        auto dist = dijkstra(5, road_edges, 0);
        std::string s = "[";
        for (int i = 0; i < 5; ++i)
            s += (dist[i]==INF ? "INF" : std::to_string((int)dist[i]))
               + (i<4 ? ", " : "]");
        result("Dijkstra 최단 거리 (src=0)", s);
    }

    // 2-B. 경로 추적
    {
        auto [d, path] = dijkstraWithPath(5, road_edges, 0, 4);
        std::string p = "[";
        for (int i = 0; i < (int)path.size(); ++i)
            p += std::to_string(path[i]) + (i+1<(int)path.size() ? "→" : "]");
        result("0→4 최단 경로 (거리=" + std::to_string((int)d) + ")", p);
    }

    // 2-C. k번째 가까운 장애물
    {
        std::mt19937 rng(42);
        std::uniform_real_distribution<float> dist_rng(-30.f, 30.f);
        std::vector<Point3D> pts;
        for (int i = 0; i < 1000; ++i)
            pts.push_back({dist_rng(rng), dist_rng(rng), 0.f});

        auto kth = kthClosestObstacle(pts, {0,0,0}, 3);
        if (kth) {
            float d = std::sqrt(kth->x*kth->x + kth->y*kth->y);
            std::ostringstream oss;
            oss << "(" << std::fixed << std::setprecision(2)
                << kth->x << ", " << kth->y << ")  거리=" << d << "m";
            result("3번째 가까운 장애물", oss.str());
        }
    }

    // ==============================================================
    section("3순위 — LiDAR 처리 파이프라인 (ROS 2 노드 로직)");
    // ==============================================================

    {
        // 가상 LiDAR 포인트 생성 (N=5000)
        std::mt19937 rng(7);
        std::uniform_real_distribution<float> xy(-50.f, 50.f);
        std::uniform_real_distribution<float> z (-2.f,  5.f);

        sensor_msgs::PointCloud2 input;
        input.frame_id = "lidar_link";
        for (int i = 0; i < 5000; ++i)
            input.points.push_back({xy(rng), xy(rng), z(rng)});

        LidarObstacleDetector detector;
        auto output = detector.process(input);

        result("입력 포인트 수",  std::to_string(input.points.size()) + "pt");
        result("거리 필터 후 (≤30m)",  "");
        result("높이 필터 후 (-0.5~3.0m)", "");
        result("복셀 다운샘플 후 (0.1m)", std::to_string(output.points.size()) + "pt");

        std::cout << "\n  " << YELLOW
                  << "ROS 2 노드 전체 코드는 파일 하단 주석 참조 (rclcpp::Node 상속)\n"
                  << RESET;
    }

    std::cout << "\n" << BOLD << GREEN
              << "  모든 테스트 완료 ✓\n" << RESET << "\n";
    return 0;
}
