//============================================================================
// path_generator.h — 基于 C++11 的线性插值路径生成器 接口头文件
//============================================================================
// 编译: g++ -std=c++11 -o path_generator path_generator.cpp -lm
// 库:   g++ -std=c++11 -DPATH_GENERATOR_NO_MAIN -c path_generator.cpp -lm
//============================================================================

#ifndef PATH_GENERATOR_H
#define PATH_GENERATOR_H

#include <vector>
#include <cstdint>
#include <cmath>
#include <utility>

//============================================================================
// 【用户可修改参数区域】— 修改以下参数即可调整路径生成行为
//============================================================================

// ────────────── 地图边界 (mm) ──────────────
#define RECT_X_MIN  2600.0f    // 下边界 x
#define RECT_X_MAX  8600.0f    // 上边界 x
#define RECT_Y_MIN  600.0f     // 右边界 y
#define RECT_Y_MAX  5400.0f    // 左边界 y

// ────────────── 总起点 & 总终点 ──────────────
// 方向约定：x 向前，y 向左
#define FIXED_START_X  2600.0f
#define FIXED_START_Y  5400.0f
#define FIXED_START_YAW 0.000f

#define FIXED_END_X    8600.0f
#define FIXED_END_Y    600.0f
#define FIXED_END_YAW  0.000f

// ────────────── 路径生成参数 ──────────────
#define PATH_POINT_INTERVAL   150.0f    // 线性插值点间隔 (mm)

// ────────────── 速度配置 ──────────────
#define DEFAULT_MAX_VEL   2.0f    // 最大速度 (m/s)
#define DEFAULT_ACCEL     2.0f    // 最大加速度 (m/s²)
#define DEFAULT_DECEL     1.5f    // 最大减速度 (m/s²)

// ────────────── 中间点偏移 ──────────────
#define WAYPOINT_OFFSET_DISTANCE  298.940f   // 默认 298.940mm
// 偏移规则（以车的左右方向为基准）：
//   WP1 的投影点 + 目标点 → 同时沿车右方向偏移
//   WP2 的投影点 + 目标点 → 同时沿车左方向偏移
// 偏移向量：(sinf(yaw), -cosf(yaw)) * offset

// ────────────── 红蓝区镜像 ──────────────
#define ZONE  -1    // -1=蓝区(最终输出y取反), 0=空(不输出), 1=红区
// 镜像规则（仅作用于最终输出，内部计算用原始坐标）：
//   蓝区: y→-y, vy→-vy（旋转标志位和路径起点终点标志位除外）, yaw→-yaw

// ────────────── 示例输入 ──────────────
#define EXAMPLE_WP_NUM1  1
#define EXAMPLE_WP_NUM2  7

//============================================================================
// 【内部常量】— 由用户参数派生或内部使用，用户无需修改
//============================================================================

// 地图派生尺寸
#define RECT_WIDTH  (RECT_X_MAX - RECT_X_MIN)   // x方向长度 6000mm
#define RECT_HEIGHT (RECT_Y_MAX - RECT_Y_MIN)   // y方向长度 4800mm
// 矩形周长 = 2 * (6000 + 4800) = 21600mm

// 旋转标志速度值 (mm/s)
#define ROTATION_MARKER_VEL  0.001f   // Vx=Vy=0.001

//============================================================================
// 【数据结构定义】
//============================================================================

// 二维点
struct Point {
    float x;
    float y;
};

// 路径配置
struct PathConfig {
    float max_vel;   // 最大速度 (m/s)
    float accel;     // 最大加速度 (m/s²)
    float decel;     // 最大减速度 (m/s²)

    PathConfig()
        : max_vel(DEFAULT_MAX_VEL),
          accel(DEFAULT_ACCEL),
          decel(DEFAULT_DECEL) {}
};

// 路径点 — 输出格式: Vx(mm/s), Vy(mm/s), X(mm), Y(mm), yaw(rad)
struct PathPoint {
    float vx;
    float vy;
    float x;
    float y;
    float yaw;
};

// 中间点数据（投影点 & 目标点共用此结构）
struct WaypointData {
    float x;
    float y;
    float yaw;
};

// 三段路径结果
struct ThreeSegmentResult {
    std::vector<PathPoint> seg1;
    std::vector<PathPoint> seg2;
    std::vector<PathPoint> seg3;
    int selected_wp1_num;   // 选择后的 WP1 编号
    int selected_wp2_num;   // 选择后的 WP2 编号
};

//============================================================================
// 【坐标点定义】
//============================================================================

// ────────────── 投影点（中间过渡点，10个，在矩形边缘上）──────────────
// 编号 1~12，其中 5 和 8 未使用
// 使用数组大小 13，索引即编号，0/5/8 号为无效占位
static const WaypointData PROJECTION_POINTS[13] = {
    /* [0]  无效 */ {0, 0, 0},
    /* [1]  下边 */ {2600.0f, 1800.0f,  0.000f},   // 下边 (x=2600)
    /* [2]  下边 */ {2600.0f, 3000.0f,  0.000f},   // 下边 (x=2600)
    /* [3]  下边 */ {2600.0f, 4200.0f,  0.000f},   // 下边 (x=2600)
    /* [4]  右边 */ {5000.0f,  600.0f,  1.571f},   // 右边 (y=600)
    /* [5]  无效 */ {0, 0, 0},
    /* [6]  左边 */ {5000.0f, 5400.0f, -1.571f},   // 左边 (y=5400)
    /* [7]  右边 */ {6200.0f,  600.0f,  1.571f},   // 右边 (y=600)
    /* [8]  无效 */ {0, 0, 0},
    /* [9]  左边 */ {6200.0f, 5400.0f, -1.571f},   // 左边 (y=5400)
    /* [10] 右边 */ {7400.0f,  600.0f,  1.571f},   // 右边 (y=600)
    /* [11] 上边 */ {8600.0f, 3000.0f,  3.141f},   // 上边 (x=8600)
    /* [12] 左边 */ {7400.0f, 5400.0f, -1.571f},   // 左边 (y=5400)
};

// ────────────── 目标点（中间点，10个，偏移后的坐标）──────────────
// 编号与投影点一一对应
static const WaypointData TARGET_POINTS[13] = {
    /* [0]  无效 */ {0, 0, 0},
    /* [1]  下边 */ {2750.0f, 1800.0f,  0.000f},   // 下边
    /* [2]  下边 */ {2750.0f, 3000.0f,  0.000f},   // 下边
    /* [3]  下边 */ {2750.0f, 4200.0f,  0.000f},   // 下边
    /* [4]  右边 */ {5000.0f,  650.0f,  1.571f},   // 右边
    /* [5]  无效 */ {0, 0, 0},
    /* [6]  左边 */ {5000.0f, 5350.0f, -1.571f},   // 左边
    /* [7]  右边 */ {6200.0f,  650.0f,  1.571f},   // 右边
    /* [8]  无效 */ {0, 0, 0},
    /* [9]  左边 */ {6200.0f, 5350.0f, -1.571f},   // 左边
    /* [10] 右边 */ {7400.0f,  650.0f,  1.571f},   // 右边
    /* [11] 上边 */ {8550.0f, 3000.0f,  3.141f},   // 上边
    /* [12] 左边 */ {7400.0f, 5350.0f, -1.571f},   // 左边
};

// ────────────── 矩形四角（用于旋转逻辑）──────────────
// 顺时针方向: C0(左下) → C1(左上) → C2(右上) → C3(右下) → C0
static const Point CORNERS[4] = {
    {2600.0f, 5400.0f},   // C[0] 左下角（≈ FIXED_START）
    {8600.0f, 5400.0f},   // C[1] 左上角
    {8600.0f,  600.0f},   // C[2] 右上角（≈ FIXED_END）
    {2600.0f,  600.0f},   // C[3] 右下角
};

// ────────────── 固定起点 & 终点（WaypointData 格式）──────────────
static const WaypointData FIXED_START = {
    FIXED_START_X, FIXED_START_Y, FIXED_START_YAW
};
static const WaypointData FIXED_END = {
    FIXED_END_X, FIXED_END_Y, FIXED_END_YAW
};


//============================================================================
// 【新增数据结构】— 路径生成中间结果
//============================================================================

// 边缘路径结果（含拐角索引标记）
struct EdgePathResult {
    std::vector<Point> points;          // 插值后的位置点
    std::vector<int> corner_indices;    // points中拐角点的索引（供后续yaw分配使用）
};

// 三段位置路径结果（仅位置，无速度和yaw）
struct ThreeSegmentPositionPath {
    std::vector<Point> seg1;
    std::vector<Point> seg2;
    std::vector<Point> seg3;
};

// 偏移后的投影点和目标点
struct ShiftedWaypoints {
    Point shifted_projection;   // 偏移后的投影点（仍在矩形边缘上）
    Point shifted_target;       // 偏移后的目标点
};

// ────────────── 速度规划相关数据结构 ──────────────

// 路径点类型标记
enum PointType {
    PT_NORMAL,       // 普通路径点
    PT_PROJECTION,   // 投影点（中间过渡点，直角拐弯处）
    PT_TARGET,       // 目标点（停车点）
    PT_CORNER        // 拐角点（矩形四角）
};

// 子段类型
enum SubSegType {
    SUBSEG_EDGE,     // 沿矩形边缘的子段
    SUBSEG_PERP      // 垂直方向子段（投影点↔目标点）
};

// 子段信息
struct SubSegment {
    int start_idx;       // 子段起始点索引（包含）
    int end_idx;         // 子段结束点索引（包含）
    SubSegType type;     // 子段类型
};

// 单段路径的完整元数据（含位置、点类型、子段信息）
struct SegmentPathData {
    std::vector<Point> points;               // 位置点
    std::vector<PointType> point_types;       // 每个点的类型
    std::vector<SubSegment> sub_segments;     // 子段划分
};

// 三段路径的完整元数据
struct ThreeSegmentPathData {
    SegmentPathData seg1;
    SegmentPathData seg2;
    SegmentPathData seg3;
};

// 拐角速度限制 (mm/s) — 投影点和不旋转角落点的速度上限
#define CORNER_SPEED_LIMIT  10.0f

//============================================================================
// 【辅助函数声明】— 距离计算与中间点排序
//============================================================================

// 判断点(x,y)在矩形哪条边上
// 返回值: 0=下边, 1=右边, 2=上边, 3=左边, 不在边上返回-1
int get_edge_index(float x, float y);

// 判断点(x,y)是否为矩形拐角
// 返回值: 0=C0(左下), 1=C1(左上), 2=C2(右上), 3=C3(右下), 不是拐角返回-1
int get_corner_index(float x, float y);

// 将矩形边缘上的点转换为顺时针周长位置
// 以C0(2600,5400)为起点(pos=0)，顺时针方向递增
float calculate_perimeter_position(float x, float y);

// 沿矩形边缘从(x1,y1)顺时针走到(x2,y2)的距离
float calculate_clockwise_distance(float x1, float y1, float x2, float y2);

// 两点间沿矩形边缘的最短距离 = min(顺时针距离, 逆时针距离)
float calculate_edge_distance(float x1, float y1, float x2, float y2);

// 验证中间点编号是否有效
// 有效条件: 1-12范围内 且 PROJECTION_POINTS[wp_num]非占位(x!=0)
bool validate_wp_number(int wp_num);

// 根据最短路径选择中间点顺序
// 计算两条路线沿矩形边缘的总距离:
//   路线A: START → 投影点A → 投影点B → END
//   路线B: START → 投影点B → 投影点A → END
// 选择总距离较短的路线，先到的为WP1，后到的为WP2
// 返回: pair<WP1编号, WP2编号>
std::pair<int, int> select_waypoint_order(int wp_num1, int wp_num2);

//============================================================================
// 【路径位置插值函数声明】— 第三步
//============================================================================

// 将顺时针周长位置反算为矩形边缘上的坐标
// 以C0(2600,5400)为起点(pos=0)，顺时针方向
// pos范围 [0, 周长)，超出范围会取模
Point perimeter_position_to_point(float pos);

// 计算偏移后的投影点和目标点
// WP1: 沿车右方向偏移 (sin(yaw), -cos(yaw)) * WAYPOINT_OFFSET_DISTANCE
// WP2: 沿车左方向偏移 -(sin(yaw), -cos(yaw)) * WAYPOINT_OFFSET_DISTANCE
// 投影点和目标点同时偏移相同向量
ShiftedWaypoints calculate_shifted_points(int wp_num, bool is_wp1);

// 沿矩形边缘最短路径进行线性插值
// 选择顺时针/逆时针中较短的方向
// 按PATH_POINT_INTERVAL间距插值，精确包含经过的拐角点
// 返回EdgePathResult，包含位置点和拐角索引
EdgePathResult generate_edge_path_shortest(float x1, float y1, float x2, float y2);

// 垂直方向直线插值（投影点↔目标点之间）
// 按PATH_POINT_INTERVAL间距生成位置点
// 起终点精确包含
std::vector<Point> generate_perp_path(float from_x, float from_y, float to_x, float to_y);

// 生成三段位置路径（仅位置点，无速度和yaw）
// 根据排序后的WP1/WP2编号，按7个子段拼接三段路径:
//   Seg1: edge(总起点→WP1偏移投影点) + perp(WP1偏移投影点→WP1偏移目标点)
//   Seg2: perp(WP1偏移目标点→WP1偏移投影点) + edge(WP1偏移投影点→WP2偏移投影点)
//         + perp(WP2偏移投影点→WP2偏移目标点)
//   Seg3: perp(WP2偏移目标点→WP2偏移投影点) + edge(WP2偏移投影点→总终点)
// 拼接时去除重复的连接点
ThreeSegmentPositionPath generate_position_path(int wp1_num, int wp2_num);

//============================================================================
// 【速度规划函数声明】— 第四步
//============================================================================

// 生成带元数据的三段位置路径（位置+点类型+子段信息）
// 在 generate_position_path 基础上追踪每个点的类型和子段归属
ThreeSegmentPathData generate_position_path_with_meta(int wp1_num, int wp2_num);

// 对单段路径进行前向-后向梯形速度规划，生成 PathPoint 列表
// 边缘段使用 config 原始参数，垂直段使用 config/2
// 特殊点约束：目标点 v=0，投影点/角落点 v <= CORNER_SPEED_LIMIT
// 段起终点 v=0
std::vector<PathPoint> plan_segment_velocities(const SegmentPathData& seg_data,
                                                const PathConfig& config);

// 对三段路径进行速度规划，返回带速度的 PathPoint 三段结果
void apply_velocity_planning(const ThreeSegmentPathData& path_data,
                              const PathConfig& config,
                              std::vector<PathPoint>& out_seg1,
                              std::vector<PathPoint>& out_seg2,
                              std::vector<PathPoint>& out_seg3);


//============================================================================
// 【yaw 分配与旋转标志插入函数声明】— 第五步
//============================================================================

// 对单段路径分配 yaw 并插入旋转标志
// 分支A: |end_yaw - start_yaw| < 0.01 → 全段 start_yaw，不插旋转标志
// 分支B: yaw 不同 → 在拐角处插入旋转标志
//   B1: 路径中间有拐角 → 拐角点前一行插入旋转标志
//   B2: 无拐角 + 起点为总起点 → 起点后插入旋转标志
//   B3: 无拐角 + 终点为总终点 → 终点前一行插入旋转标志
// 旋转标志格式: Vx=Vy=ROTATION_MARKER_VEL, x/y=拐角(或起终点)坐标, yaw=end_yaw
// 旋转标志前的点 yaw=start_yaw，旋转标志及之后的点 yaw=end_yaw
void apply_yaw_to_segment(std::vector<PathPoint>& path,
                           const SegmentPathData& seg_data,
                           float start_yaw, float end_yaw,
                           bool is_first_seg, bool is_last_seg);

// 对三段路径分别分配 yaw 并插入旋转标志
// Seg1: start_yaw=FIXED_START.yaw, end_yaw=WP1.yaw
// Seg2: start_yaw=WP1.yaw, end_yaw=WP2.yaw
// Seg3: start_yaw=WP2.yaw, end_yaw=FIXED_END.yaw
void apply_yaw_to_all_segments(std::vector<PathPoint>& seg1,
                                std::vector<PathPoint>& seg2,
                                std::vector<PathPoint>& seg3,
                                const ThreeSegmentPathData& path_data,
                                int wp1_num, int wp2_num);

//============================================================================
// 【全局变量声明】— 供外部代码直接引用
//============================================================================

extern std::vector<PathPoint> g_seg1;   // 第一段路径（全局副本）
extern std::vector<PathPoint> g_seg2;   // 第二段路径（全局副本）
extern std::vector<PathPoint> g_seg3;   // 第三段路径（全局副本）

//============================================================================
// 【蓝区镜像函数声明】— 第六步
//============================================================================

// 对单段路径进行蓝区镜像处理（ZONE == -1 时调用）
// 镜像规则（仅 y 轴方向数据取反）：
//   y → -y（全部取反）
//   yaw → -yaw（全部取反）
//   vy → -vy（旋转标志位和起点终点标志位除外）
//   vx, x → 不变
// 例外：旋转标志位（vx≈vy≈ROTATION_MARKER_VEL）和起点终点标志位（首尾点）的 vy 不取反
void apply_zone_mirror_to_path(std::vector<PathPoint>& path);

//============================================================================
// 【核心 API 函数声明】— 主入口 & 文件输出
//============================================================================

// 主入口函数：验证编号 → 选择最短顺序 → 生成位置路径 → 速度规划
//           → yaw分配+旋转标志 → 确保首尾Vx=Vy=0 → 蓝区镜像
// 结果同时存入全局数组 g_seg1/g_seg2/g_seg3
// 参数:
//   wp_num1 - 第一个中间点编号 (1-12, 5/8无效)
//   wp_num2 - 第二个中间点编号
//   config  - 速度配置 (max_vel/accel/decel)
//   zone    - 红蓝区: 1=红区, -1=蓝区, 0=空(不输出)
// 返回: ThreeSegmentResult (三段路径 + 排序后的WP1/WP2编号)
ThreeSegmentResult generate_path_from_waypoint_numbers(
    int wp_num1, int wp_num2, const PathConfig& config, int zone);

// 保存三段路径到文件（C数组格式，含速度信息）
// 输出格式: float path_segment_N[][5] = {{vx,vy,x,y,yaw}, ...};
//           uint16_t path_segment_sizes[3] = {n1, n2, n3};
void save_three_segments(
    const std::vector<PathPoint>& seg1,
    const std::vector<PathPoint>& seg2,
    const std::vector<PathPoint>& seg3,
    const char* filename);

#endif // PATH_GENERATOR_H
