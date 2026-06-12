//============================================================================
// path_generator.cpp — 基于 C++11 的线性插值路径生成器 主程序
//============================================================================

#include "path_generator.h"
#include <cstdio>
#include <algorithm>

/// 矩形周长 (mm)
static const float RECT_PERIMETER = 2.0f * (RECT_WIDTH + RECT_HEIGHT);

//============================================================================
// 【全局变量定义】— 供外部代码直接引用
//============================================================================

std::vector<PathPoint> g_seg1;
std::vector<PathPoint> g_seg2;
std::vector<PathPoint> g_seg3;

//============================================================================
// 【辅助函数】— 地图边界与拐角判断
//============================================================================

/**
 * @brief 判断点(x,y)在矩形哪条边上
 * @param x 点的x坐标 (mm)
 * @param y 点的y坐标 (mm)
 * @return 0=下边, 1=右边, 2=上边, 3=左边, 不在边上返回-1
 */
int get_edge_index(float x, float y) {
    const float EPS = 1.0f;
    if (fabsf(x - RECT_X_MIN) < EPS && y >= RECT_Y_MIN - EPS && y <= RECT_Y_MAX + EPS) return 0;
    if (fabsf(y - RECT_Y_MIN) < EPS && x >= RECT_X_MIN - EPS && x <= RECT_X_MAX + EPS) return 1;
    if (fabsf(x - RECT_X_MAX) < EPS && y >= RECT_Y_MIN - EPS && y <= RECT_Y_MAX + EPS) return 2;
    if (fabsf(y - RECT_Y_MAX) < EPS && x >= RECT_X_MIN - EPS && x <= RECT_X_MAX + EPS) return 3;
    return -1;
}

/**
 * @brief 判断点(x,y)是否为矩形拐角
 * @param x 点的x坐标 (mm)
 * @param y 点的y坐标 (mm)
 * @return 0=C0(左下), 1=C1(左上), 2=C2(右上), 3=C3(右下), 不是拐角返回-1
 */
int get_corner_index(float x, float y) {
    const float EPS = 1.0f;
    for (int i = 0; i < 4; i++) {
        if (fabsf(x - CORNERS[i].x) < EPS && fabsf(y - CORNERS[i].y) < EPS) return i;
    }
    return -1;
}

//============================================================================
// 【辅助函数】— 周长位置与距离计算
//============================================================================

/**
 * @brief 将矩形边缘上的点转换为顺时针周长位置
 * @param x 点的x坐标 (mm)
 * @param y 点的y坐标 (mm)
 * @return 顺时针周长位置 (mm)，以C0(2600,5400)为起点(pos=0)
 * @note 顺时针方向: C0->C1->C2->C3->C0
 *       C0->C1: 左边(y=Y_MAX), x递增, pos范围 [0, 6000]
 *       C1->C2: 上边(x=X_MAX), y递减, pos范围 [6000, 10800]
 *       C2->C3: 右边(y=Y_MIN), x递减, pos范围 [10800, 16800]
 *       C3->C0: 下边(x=X_MIN), y递增, pos范围 [16800, 21600]
 */
float calculate_perimeter_position(float x, float y) {
    // 优先检查角点，确保角点位置唯一确定
    int ci = get_corner_index(x, y);
    if (ci >= 0) {
        switch (ci) {
            case 0: return 0.0f;
            case 1: return RECT_WIDTH;
            case 2: return RECT_WIDTH + RECT_HEIGHT;
            case 3: return 2.0f * RECT_WIDTH + RECT_HEIGHT;
        }
    }
    // 非角点：根据所在边计算周长位置
    int edge = get_edge_index(x, y);
    float pos = 0.0f;
    switch (edge) {
        case 3: pos = (x - RECT_X_MIN); break;
        case 2: pos = RECT_WIDTH + (RECT_Y_MAX - y); break;
        case 1: pos = RECT_WIDTH + RECT_HEIGHT + (RECT_X_MAX - x); break;
        case 0: pos = 2.0f * RECT_WIDTH + RECT_HEIGHT + (y - RECT_Y_MIN); break;
        default: {
            // 不在边上：找最近的边，计算近似周长位置
            float min_dist = 1e9f;
            float d3 = fabsf(y - RECT_Y_MAX);
            float d2 = fabsf(x - RECT_X_MAX);
            float d1 = fabsf(y - RECT_Y_MIN);
            float d0 = fabsf(x - RECT_X_MIN);
            if (d3 < min_dist) { min_dist = d3; pos = (x - RECT_X_MIN); }
            if (d2 < min_dist) { min_dist = d2; pos = RECT_WIDTH + (RECT_Y_MAX - y); }
            if (d1 < min_dist) { min_dist = d1; pos = RECT_WIDTH + RECT_HEIGHT + (RECT_X_MAX - x); }
            if (d0 < min_dist) { min_dist = d0; pos = 2.0f*RECT_WIDTH + RECT_HEIGHT + (y - RECT_Y_MIN); }
            break;
        }
    }
    // 归一化到 [0, 周长)
    if (pos >= RECT_PERIMETER) pos -= RECT_PERIMETER;
    return pos;
}

/**
 * @brief 沿矩形边缘从(x1,y1)顺时针走到(x2,y2)的距离
 * @param x1 起点x坐标 (mm)
 * @param y1 起点y坐标 (mm)
 * @param x2 终点x坐标 (mm)
 * @param y2 终点y坐标 (mm)
 * @return 顺时针距离 (mm)
 */
float calculate_clockwise_distance(float x1, float y1, float x2, float y2) {
    float pos1 = calculate_perimeter_position(x1, y1);
    float pos2 = calculate_perimeter_position(x2, y2);
    float dist = pos2 - pos1;
    if (dist < 0) dist += RECT_PERIMETER;
    return dist;
}

/**
 * @brief 两点间沿矩形边缘的最短距离
 * @param x1 起点x坐标 (mm)
 * @param y1 起点y坐标 (mm)
 * @param x2 终点x坐标 (mm)
 * @param y2 终点y坐标 (mm)
 * @return 最短边缘距离 (mm) = min(顺时针距离, 逆时针距离)
 */
float calculate_edge_distance(float x1, float y1, float x2, float y2) {
    float cw = calculate_clockwise_distance(x1, y1, x2, y2);
    return std::min(cw, RECT_PERIMETER - cw);
}

//============================================================================
// 【中间点验证与排序】
//============================================================================

/**
 * @brief 验证中间点编号是否有效
 * @param wp_num 中间点编号 (1-12)
 * @return true=有效, false=无效(编号0/5/8为占位)
 */
bool validate_wp_number(int wp_num) {
    if (wp_num < 1 || wp_num > 12) return false;
    if (PROJECTION_POINTS[wp_num].x == 0.0f) return false;
    return true;
}

/**
 * @brief 根据最短路径选择中间点顺序
 * @param wp_num1 第一个中间点编号
 * @param wp_num2 第二个中间点编号
 * @return pair<WP1编号, WP2编号>，先到的为WP1
 * @note 计算两条路线沿边缘的总距离，选较短的:
 *       路线A: START->投影点A->投影点B->END
 *       路线B: START->投影点B->投影点A->END
 */
std::pair<int, int> select_waypoint_order(int wp_num1, int wp_num2) {
    float ax = PROJECTION_POINTS[wp_num1].x, ay = PROJECTION_POINTS[wp_num1].y;
    float bx = PROJECTION_POINTS[wp_num2].x, by = PROJECTION_POINTS[wp_num2].y;
    float sx = FIXED_START.x, sy = FIXED_START.y;
    float ex = FIXED_END.x, ey = FIXED_END.y;

    float dist_A = calculate_edge_distance(sx, sy, ax, ay)
                 + calculate_edge_distance(ax, ay, bx, by)
                 + calculate_edge_distance(bx, by, ex, ey);
    float dist_B = calculate_edge_distance(sx, sy, bx, by)
                 + calculate_edge_distance(bx, by, ax, ay)
                 + calculate_edge_distance(ax, ay, ex, ey);

    return (dist_A <= dist_B) ? std::make_pair(wp_num1, wp_num2)
                              : std::make_pair(wp_num2, wp_num1);
}

//============================================================================
// 【路径位置插值函数】— 第三步
//============================================================================

/**
 * @brief 将顺时针周长位置反算为矩形边缘上的坐标
 * @param pos 顺时针周长位置 (mm)，以C0为起点
 * @return 矩形边缘上的坐标 Point
 * @note 顺时针方向: C0->C1(左边, pos 0~6000)
 *       C1->C2(上边, pos 6000~10800)
 *       C2->C3(右边, pos 10800~16800)
 *       C3->C0(下边, pos 16800~21600)
 */
Point perimeter_position_to_point(float pos) {
    // 归一化到 [0, 周长)
    pos = fmodf(pos, RECT_PERIMETER);
    if (pos < 0) pos += RECT_PERIMETER;

    // C0->C1: 左边，x从X_MIN到X_MAX，y=Y_MAX
    if (pos <= RECT_WIDTH) {
        return {RECT_X_MIN + pos, RECT_Y_MAX};
    }
    // C1->C2: 上边，y从Y_MAX到Y_MIN，x=X_MAX
    if (pos <= RECT_WIDTH + RECT_HEIGHT) {
        float d = pos - RECT_WIDTH;
        return {RECT_X_MAX, RECT_Y_MAX - d};
    }
    // C2->C3: 右边，x从X_MAX到X_MIN，y=Y_MIN
    if (pos <= 2.0f * RECT_WIDTH + RECT_HEIGHT) {
        float d = pos - RECT_WIDTH - RECT_HEIGHT;
        return {RECT_X_MAX - d, RECT_Y_MIN};
    }
    // C3->C0: 下边，y从Y_MIN到Y_MAX，x=X_MIN
    float d = pos - 2.0f * RECT_WIDTH - RECT_HEIGHT;
    return {RECT_X_MIN, RECT_Y_MIN + d};
}

/**
 * @brief 将浮点数四舍五入到指定小数位
 * @param val 待舍入的值
 * @param decimals 小数位数（默认3，即0.001mm精度）
 * @return 舍入后的值
 */
static float round_to(float val, int decimals = 2) {
    float factor = 1.0f;
    for (int i = 0; i < decimals; i++) factor *= 10.0f;
    return roundf(val * factor) / factor;
}

/**
 * @brief 将坐标点对齐到矩形边缘并消除浮点误差
 * @param pt 待对齐的点
 * @return 对齐后的点（坐标四舍五入到0.001mm，且接近边缘值时精确对齐）
 */
static Point snap_to_edge(Point pt) {
    // 先四舍五入到0.001mm精度，消除浮点累积误差
    pt.x = round_to(pt.x);
    pt.y = round_to(pt.y);
    // 如果非常接近边缘值则精确对齐
    const float EPS = 0.5f;
    if (fabsf(pt.x - RECT_X_MIN) < EPS) pt.x = RECT_X_MIN;
    if (fabsf(pt.x - RECT_X_MAX) < EPS) pt.x = RECT_X_MAX;
    if (fabsf(pt.y - RECT_Y_MIN) < EPS) pt.y = RECT_Y_MIN;
    if (fabsf(pt.y - RECT_Y_MAX) < EPS) pt.y = RECT_Y_MAX;
    return pt;
}

/**
 * @brief 计算偏移后的投影点和目标点
 * @param wp_num 中间点编号
 * @param is_wp1 true=WP1(沿车右方向偏移), false=WP2(沿车左方向偏移)
 * @return 偏移后的投影点和目标点
 * @note 使用基于边缘类型的精确偏移计算，避免sin/cos浮点误差
 *       下边(yaw=0):   车右方向=-y → offset=(0, -dist)
 *       右边(yaw=π/2): 车右方向=+x → offset=(+dist, 0)
 *       上边(yaw=π):   车右方向=+y → offset=(0, +dist)
 *       左边(yaw=-π/2):车右方向=-x → offset=(-dist, 0)
 */
ShiftedWaypoints calculate_shifted_points(int wp_num, bool is_wp1) {
    float px = PROJECTION_POINTS[wp_num].x;
    float py = PROJECTION_POINTS[wp_num].y;
    float tx = TARGET_POINTS[wp_num].x;
    float ty = TARGET_POINTS[wp_num].y;

    // 根据投影点所在边缘确定精确偏移方向
    int edge = get_edge_index(px, py);
    float dx = 0.0f, dy = 0.0f;

    switch (edge) {
        case 0: // 下边 (x=X_MIN, yaw=0), 车右方向=-y
            dy = -WAYPOINT_OFFSET_DISTANCE;
            break;
        case 1: // 右边 (y=Y_MIN, yaw=π/2), 车右方向=+x
            dx = WAYPOINT_OFFSET_DISTANCE;
            break;
        case 2: // 上边 (x=X_MAX, yaw=π), 车右方向=+y
            dy = WAYPOINT_OFFSET_DISTANCE;
            break;
        case 3: // 左边 (y=Y_MAX, yaw=-π/2), 车右方向=-x
            dx = -WAYPOINT_OFFSET_DISTANCE;
            break;
        default:
            // 不在边上：使用sin/cos作为后备
            {
                float yaw = PROJECTION_POINTS[wp_num].yaw;
                dx = sinf(yaw) * WAYPOINT_OFFSET_DISTANCE;
                dy = -cosf(yaw) * WAYPOINT_OFFSET_DISTANCE;
            }
            break;
    }

    if (!is_wp1) {
        // WP2: 沿车左方向，取反
        dx = -dx;
        dy = -dy;
    }

    ShiftedWaypoints result;
    result.shifted_projection = snap_to_edge({px + dx, py + dy});
    result.shifted_target = snap_to_edge({tx + dx, ty + dy});
    return result;
}

/**
 * @brief 沿矩形边缘最短路径进行线性插值
 * @param x1,y1 起点坐标
 * @param x2,y2 终点坐标
 * @return EdgePathResult 包含插值点和拐角索引
 * @note 选择顺时针/逆时针中较短的方向
 *       按PATH_POINT_INTERVAL间距插值
 *       精确包含经过的拐角点
 */
EdgePathResult generate_edge_path_shortest(float x1, float y1, float x2, float y2) {
    EdgePathResult result;

    // 如果起终点相同，直接返回起点
    if (fabsf(x1 - x2) < 0.01f && fabsf(y1 - y2) < 0.01f) {
        result.points.push_back({x1, y1});
        return result;
    }

    float pos1 = calculate_perimeter_position(x1, y1);
    float pos2 = calculate_perimeter_position(x2, y2);

    // 计算顺时针和逆时针距离
    float cw_dist = pos2 - pos1;
    if (cw_dist < 0) cw_dist += RECT_PERIMETER;
    float ccw_dist = RECT_PERIMETER - cw_dist;

    // 选择较短方向: clockwise=true 表示顺时针
    bool clockwise = (cw_dist <= ccw_dist);
    float total_dist = clockwise ? cw_dist : ccw_dist;

    // 如果总距离为0，直接返回起点
    if (total_dist < 0.01f) {
        result.points.push_back({x1, y1});
        return result;
    }

    // 计算路径经过的拐角点的周长位置
    // 拐角周长位置: C0=0, C1=6000, C2=10800, C3=16800
    float corner_positions[4] = {
        0.0f,                           // C0
        RECT_WIDTH,                     // C1
        RECT_WIDTH + RECT_HEIGHT,       // C2
        2.0f * RECT_WIDTH + RECT_HEIGHT // C3
    };

    // 收集路径经过的拐角点（按行进顺序排列，包含周长位置）
    struct CornerInfo {
        float perimeter_pos;
        Point coord;
        int corner_idx;
    };
    std::vector<CornerInfo> corners_on_path;

    for (int i = 0; i < 4; i++) {
        float cpos = corner_positions[i];
        bool on_path = false;
        if (clockwise) {
            // 顺时针: 从pos1到pos2，检查cpos是否在路径上
            float rel = cpos - pos1;
            if (rel < 0) rel += RECT_PERIMETER;
            if (rel > 0.01f && rel < cw_dist - 0.01f) on_path = true;
        } else {
            // 逆时针: 从pos1到pos2（反方向），检查cpos是否在路径上
            float rel = pos1 - cpos;
            if (rel < 0) rel += RECT_PERIMETER;
            if (rel > 0.01f && rel < ccw_dist - 0.01f) on_path = true;
        }
        if (on_path) {
            corners_on_path.push_back({cpos, CORNERS[i], i});
        }
    }

    // 按行进顺序排序拐角点
    if (clockwise) {
        std::sort(corners_on_path.begin(), corners_on_path.end(),
            [pos1](const CornerInfo& a, const CornerInfo& b) {
                float ra = a.perimeter_pos - pos1;
                if (ra < 0) ra += RECT_PERIMETER;
                float rb = b.perimeter_pos - pos1;
                if (rb < 0) rb += RECT_PERIMETER;
                return ra < rb;
            });
    } else {
        std::sort(corners_on_path.begin(), corners_on_path.end(),
            [pos1](const CornerInfo& a, const CornerInfo& b) {
                float ra = pos1 - a.perimeter_pos;
                if (ra < 0) ra += RECT_PERIMETER;
                float rb = pos1 - b.perimeter_pos;
                if (rb < 0) rb += RECT_PERIMETER;
                return ra < rb;
            });
    }

    // 构建路径段列表：起点 → 拐角1 → 拐角2 → ... → 终点
    // 每段的起终点周长位置
    struct Segment {
        float start_pos;   // 沿行进方向的累积距离
        float end_pos;
    };
    std::vector<Segment> segments;

    float current_dist = 0.0f;
    for (size_t i = 0; i <= corners_on_path.size(); i++) {
        float next_pos_along; // 下一个关键点沿行进方向的距离
        if (i < corners_on_path.size()) {
            if (clockwise) {
                float rel = corners_on_path[i].perimeter_pos - pos1;
                if (rel < 0) rel += RECT_PERIMETER;
                next_pos_along = rel;
            } else {
                float rel = pos1 - corners_on_path[i].perimeter_pos;
                if (rel < 0) rel += RECT_PERIMETER;
                next_pos_along = rel;
            }
            segments.push_back({current_dist, next_pos_along});
            current_dist = next_pos_along;
        } else {
            // 最后一段到终点
            segments.push_back({current_dist, total_dist});
        }
    }

    // 逐段进行插值
    result.points.push_back({x1, y1});

    // 追踪拐角在result.points中的索引
    // 起始点已占据index 0，下一个点将从index 1开始
    int point_index = 1;

    for (size_t seg_i = 0; seg_i < segments.size(); seg_i++) {
        float seg_start_dist = segments[seg_i].start_pos;
        float seg_end_dist = segments[seg_i].end_pos;
        float seg_len = seg_end_dist - seg_start_dist;

        if (seg_len < 0.01f) continue;

        // 计算当前段起终点对应的周长位置
        float seg_start_perim;
        if (clockwise) {
            seg_start_perim = pos1 + seg_start_dist;
        } else {
            seg_start_perim = pos1 - seg_start_dist;
        }
        if (seg_start_perim < 0) seg_start_perim += RECT_PERIMETER;
        if (seg_start_perim >= RECT_PERIMETER) seg_start_perim -= RECT_PERIMETER;

        // 从当前段的起点开始，按间隔插值
        float dist_from_seg_start = PATH_POINT_INTERVAL;
        // 计算段起点已有多少点，调整第一个间隔
        // 当前段的起点就是上一个点（起点或拐角点）
        // 从段起点后的第一个间隔点开始

        while (dist_from_seg_start < seg_len - 0.01f) {
            float point_dist_along; // 该点沿行进方向的累积距离
            if (clockwise) {
                point_dist_along = seg_start_dist + dist_from_seg_start;
            } else {
                point_dist_along = seg_start_dist + dist_from_seg_start;
            }

            // 计算对应的周长位置
            float point_perim;
            if (clockwise) {
                point_perim = pos1 + point_dist_along;
            } else {
                point_perim = pos1 - point_dist_along;
            }
            if (point_perim < 0) point_perim += RECT_PERIMETER;
            if (point_perim >= RECT_PERIMETER) point_perim -= RECT_PERIMETER;

            Point pt = snap_to_edge(perimeter_position_to_point(point_perim));
            result.points.push_back(pt);
            point_index++;

            dist_from_seg_start += PATH_POINT_INTERVAL;
        }

        // 如果这一段有拐角点（不是最后一段），添加拐角点
        if (seg_i < corners_on_path.size()) {
            Point corner_pt = corners_on_path[seg_i].coord;
            result.points.push_back(corner_pt);
            point_index++;
            result.corner_indices.push_back(point_index - 1); // 记录拐角在points中的索引
        }
    }

    // 确保终点精确
    // 检查最后一个点是否接近终点，如果不接近则添加
    Point last_pt = result.points.back();
    float dist_to_end = sqrtf((last_pt.x - x2) * (last_pt.x - x2) +
                               (last_pt.y - y2) * (last_pt.y - y2));
    if (dist_to_end > 1.0f) {
        result.points.push_back({x2, y2});
    } else {
        // 修正最后一个点为精确终点
        result.points.back() = {x2, y2};
    }

    return result;
}

/**
 * @brief 垂直方向直线插值（投影点↔目标点之间）
 * @param from_x,from_y 起点坐标
 * @param to_x,to_y 终点坐标
 * @return 插值后的位置点列表（包含起终点）
 */
std::vector<Point> generate_perp_path(float from_x, float from_y, float to_x, float to_y) {
    std::vector<Point> result;

    float dx = to_x - from_x;
    float dy = to_y - from_y;
    float total_dist = sqrtf(dx * dx + dy * dy);

    // 起终点相同时只返回起点
    if (total_dist < 0.01f) {
        result.push_back({from_x, from_y});
        return result;
    }

    // 添加起点
    result.push_back({from_x, from_y});

    // 按间隔插值
    float d = PATH_POINT_INTERVAL;
    while (d < total_dist - 0.01f) {
        float ratio = d / total_dist;
        result.push_back({from_x + dx * ratio, from_y + dy * ratio});
        d += PATH_POINT_INTERVAL;
    }

    // 添加终点
    result.push_back({to_x, to_y});

    return result;
}

/**
 * @brief 拼接两个Point向量，去除重复的连接点
 * @param first 第一个向量
 * @param second 第二个向量
 * @return 拼接后的向量（first的终点和second的起点重合时只保留一个）
 */
static std::vector<Point> concat_paths(const std::vector<Point>& first,
                                        const std::vector<Point>& second) {
    std::vector<Point> result = first;
    // 如果first的最后一个点与second的第一个点重合，跳过second的第一个点
    if (!first.empty() && !second.empty()) {
        Point last = first.back();
        Point first2 = second.front();
        if (fabsf(last.x - first2.x) < 1.0f && fabsf(last.y - first2.y) < 1.0f) {
            result.insert(result.end(), second.begin() + 1, second.end());
            return result;
        }
    }
    result.insert(result.end(), second.begin(), second.end());
    return result;
}

/**
 * @brief 生成三段位置路径（仅位置点，无速度和yaw）
 * @param wp1_num WP1编号（已排序）
 * @param wp2_num WP2编号（已排序）
 * @return ThreeSegmentPositionPath 三段位置路径
 */
ThreeSegmentPositionPath generate_position_path(int wp1_num, int wp2_num) {
    ThreeSegmentPositionPath result;

    // 计算偏移后的投影点和目标点
    ShiftedWaypoints wp1_shifted = calculate_shifted_points(wp1_num, true);
    ShiftedWaypoints wp2_shifted = calculate_shifted_points(wp2_num, false);

    float sx = FIXED_START.x, sy = FIXED_START.y;
    float ex = FIXED_END.x, ey = FIXED_END.y;

    // ─── Seg1: 总起点 → WP1偏移投影点（沿边缘）→ WP1偏移目标点（垂直） ───
    EdgePathResult seg1_edge = generate_edge_path_shortest(
        sx, sy, wp1_shifted.shifted_projection.x, wp1_shifted.shifted_projection.y);
    std::vector<Point> seg1_perp = generate_perp_path(
        wp1_shifted.shifted_projection.x, wp1_shifted.shifted_projection.y,
        wp1_shifted.shifted_target.x, wp1_shifted.shifted_target.y);
    result.seg1 = concat_paths(seg1_edge.points, seg1_perp);

    // ─── Seg2: WP1偏移目标点 → WP1偏移投影点（垂直返回）→ WP2偏移投影点（沿边缘）→ WP2偏移目标点（垂直） ───
    std::vector<Point> seg2_perp_back = generate_perp_path(
        wp1_shifted.shifted_target.x, wp1_shifted.shifted_target.y,
        wp1_shifted.shifted_projection.x, wp1_shifted.shifted_projection.y);
    EdgePathResult seg2_edge = generate_edge_path_shortest(
        wp1_shifted.shifted_projection.x, wp1_shifted.shifted_projection.y,
        wp2_shifted.shifted_projection.x, wp2_shifted.shifted_projection.y);
    std::vector<Point> seg2_perp_fwd = generate_perp_path(
        wp2_shifted.shifted_projection.x, wp2_shifted.shifted_projection.y,
        wp2_shifted.shifted_target.x, wp2_shifted.shifted_target.y);

    result.seg2 = concat_paths(seg2_perp_back, seg2_edge.points);
    result.seg2 = concat_paths(result.seg2, seg2_perp_fwd);

    // ─── Seg3: WP2偏移目标点 → WP2偏移投影点（垂直返回）→ 总终点（沿边缘） ───
    std::vector<Point> seg3_perp_back = generate_perp_path(
        wp2_shifted.shifted_target.x, wp2_shifted.shifted_target.y,
        wp2_shifted.shifted_projection.x, wp2_shifted.shifted_projection.y);
    EdgePathResult seg3_edge = generate_edge_path_shortest(
        wp2_shifted.shifted_projection.x, wp2_shifted.shifted_projection.y,
        ex, ey);
    result.seg3 = concat_paths(seg3_perp_back, seg3_edge.points);

    return result;
}

//============================================================================
// 【速度规划函数实现】— 第四步
//============================================================================

/**
 * @brief 拼接两个带点类型的路径向量，去除重复连接点
 * @param first 第一个向量（位置+类型）
 * @param second 第二个向量（位置+类型）
 * @param second_type 第二个向量中普通点的默认类型
 * @return 拼接后的向量
 */
static void append_typed_paths(std::vector<Point>& out_points,
                                std::vector<PointType>& out_types,
                                const std::vector<Point>& append_pts,
                                const std::vector<PointType>& append_types) {
    // 如果out的最后一个点与append的第一个点重合，跳过append的第一个点
    bool skip_first = false;
    if (!out_points.empty() && !append_pts.empty()) {
        Point last = out_points.back();
        Point first2 = append_pts.front();
        if (fabsf(last.x - first2.x) < 1.0f && fabsf(last.y - first2.y) < 1.0f) {
            skip_first = true;
        }
    }
    if (skip_first) {
        out_points.insert(out_points.end(), append_pts.begin() + 1, append_pts.end());
        out_types.insert(out_types.end(), append_types.begin() + 1, append_types.end());
    } else {
        out_points.insert(out_points.end(), append_pts.begin(), append_pts.end());
        out_types.insert(out_types.end(), append_types.begin(), append_types.end());
    }
}

/**
 * @brief 为边缘路径的所有点设置类型为 PT_NORMAL（角落点除外，标记为 PT_CORNER）
 * @param edge 边缘路径结果
 * @param corner_indices 边缘路径中的拐角索引
 * @return 点类型向量
 */
static std::vector<PointType> classify_edge_points(const EdgePathResult& edge) {
    std::vector<PointType> types(edge.points.size(), PT_NORMAL);
    for (size_t i = 0; i < edge.corner_indices.size(); i++) {
        int ci = edge.corner_indices[i];
        if (ci >= 0 && ci < (int)types.size()) {
            types[ci] = PT_CORNER;
        }
    }
    return types;
}

/**
 * @brief 为垂直路径的所有点设置类型（首尾除外设为 PT_NORMAL）
 * @param perp 垂直路径点
 * @return 点类型向量
 */
static std::vector<PointType> classify_perp_points(const std::vector<Point>& perp) {
    std::vector<PointType> types(perp.size(), PT_NORMAL);
    return types;
}

/**
 * @brief 生成带元数据的三段位置路径
 * @param wp1_num WP1编号（已排序）
 * @param wp2_num WP2编号（已排序）
 * @return ThreeSegmentPathData 含位置、点类型、子段信息
 */
ThreeSegmentPathData generate_position_path_with_meta(int wp1_num, int wp2_num) {
    ThreeSegmentPathData result;

    // 计算偏移后的投影点和目标点
    ShiftedWaypoints wp1_shifted = calculate_shifted_points(wp1_num, true);
    ShiftedWaypoints wp2_shifted = calculate_shifted_points(wp2_num, false);

    float sx = FIXED_START.x, sy = FIXED_START.y;
    float ex = FIXED_END.x, ey = FIXED_END.y;

    // ─── Seg1: 总起点 → WP1偏移投影点（沿边缘）→ WP1偏移目标点（垂直） ───
    {
        SegmentPathData& seg = result.seg1;

        // 子段1: edge(总起点 → WP1偏移投影点)
        EdgePathResult seg1_edge = generate_edge_path_shortest(
            sx, sy, wp1_shifted.shifted_projection.x, wp1_shifted.shifted_projection.y);
        std::vector<PointType> seg1_edge_types = classify_edge_points(seg1_edge);

        // 子段2: perp(WP1偏移投影点 → WP1偏移目标点)
        std::vector<Point> seg1_perp = generate_perp_path(
            wp1_shifted.shifted_projection.x, wp1_shifted.shifted_projection.y,
            wp1_shifted.shifted_target.x, wp1_shifted.shifted_target.y);
        std::vector<PointType> seg1_perp_types = classify_perp_points(seg1_perp);

        // 标记投影点（edge的最后一个点就是投影点，连接perp的起点）
        // 注意：如果edge最后一个点和perp第一个点重合，拼接时perp的第一个会被跳过
        // 所以投影点就是edge的最后一个点
        if (!seg1_edge.points.empty()) {
            seg1_edge_types.back() = PT_PROJECTION;
        }
        // perp的最后一个点是目标点
        if (!seg1_perp.empty()) {
            seg1_perp_types.back() = PT_TARGET;
        }

        // 拼接
        seg.points = seg1_edge.points;
        seg.point_types = seg1_edge_types;
        append_typed_paths(seg.points, seg.point_types, seg1_perp, seg1_perp_types);

        // 记录子段
        int edge_end = (int)seg1_edge.points.size() - 1;
        seg.sub_segments.push_back({0, edge_end, SUBSEG_EDGE});
        // perp的起始索引需要考虑去重
        int perp_start = edge_end; // 重合点
        int perp_end = (int)seg.points.size() - 1;
        seg.sub_segments.push_back({perp_start, perp_end, SUBSEG_PERP});
    }

    // ─── Seg2: WP1偏移目标点 → WP1偏移投影点（垂直返回）→ WP2偏移投影点（沿边缘）→ WP2偏移目标点（垂直） ───
    {
        SegmentPathData& seg = result.seg2;

        // 子段1: perp(WP1偏移目标点 → WP1偏移投影点)
        std::vector<Point> seg2_perp_back = generate_perp_path(
            wp1_shifted.shifted_target.x, wp1_shifted.shifted_target.y,
            wp1_shifted.shifted_projection.x, wp1_shifted.shifted_projection.y);
        std::vector<PointType> seg2_perp_back_types = classify_perp_points(seg2_perp_back);

        // 子段2: edge(WP1偏移投影点 → WP2偏移投影点)
        EdgePathResult seg2_edge = generate_edge_path_shortest(
            wp1_shifted.shifted_projection.x, wp1_shifted.shifted_projection.y,
            wp2_shifted.shifted_projection.x, wp2_shifted.shifted_projection.y);
        std::vector<PointType> seg2_edge_types = classify_edge_points(seg2_edge);

        // 子段3: perp(WP2偏移投影点 → WP2偏移目标点)
        std::vector<Point> seg2_perp_fwd = generate_perp_path(
            wp2_shifted.shifted_projection.x, wp2_shifted.shifted_projection.y,
            wp2_shifted.shifted_target.x, wp2_shifted.shifted_target.y);
        std::vector<PointType> seg2_perp_fwd_types = classify_perp_points(seg2_perp_fwd);

        // 标记特殊点
        // perp_back的最后一个点是投影点
        if (!seg2_perp_back.empty()) {
            seg2_perp_back_types.back() = PT_PROJECTION;
        }
        // edge的最后一个点是投影点
        if (!seg2_edge.points.empty()) {
            seg2_edge_types.back() = PT_PROJECTION;
        }
        // perp_fwd的最后一个点是目标点
        if (!seg2_perp_fwd.empty()) {
            seg2_perp_fwd_types.back() = PT_TARGET;
        }

        // 拼接
        seg.points = seg2_perp_back;
        seg.point_types = seg2_perp_back_types;

        int perp1_end = (int)seg.points.size() - 1;
        append_typed_paths(seg.points, seg.point_types, seg2_edge.points, seg2_edge_types);
        int edge_end = (int)seg.points.size() - 1;
        append_typed_paths(seg.points, seg.point_types, seg2_perp_fwd, seg2_perp_fwd_types);
        int perp2_end = (int)seg.points.size() - 1;

        // 记录子段
        seg.sub_segments.push_back({0, perp1_end, SUBSEG_PERP});
        seg.sub_segments.push_back({perp1_end, edge_end, SUBSEG_EDGE});
        seg.sub_segments.push_back({edge_end, perp2_end, SUBSEG_PERP});
    }

    // ─── Seg3: WP2偏移目标点 → WP2偏移投影点（垂直返回）→ 总终点（沿边缘） ───
    {
        SegmentPathData& seg = result.seg3;

        // 子段1: perp(WP2偏移目标点 → WP2偏移投影点)
        std::vector<Point> seg3_perp_back = generate_perp_path(
            wp2_shifted.shifted_target.x, wp2_shifted.shifted_target.y,
            wp2_shifted.shifted_projection.x, wp2_shifted.shifted_projection.y);
        std::vector<PointType> seg3_perp_back_types = classify_perp_points(seg3_perp_back);

        // 子段2: edge(WP2偏移投影点 → 总终点)
        EdgePathResult seg3_edge = generate_edge_path_shortest(
            wp2_shifted.shifted_projection.x, wp2_shifted.shifted_projection.y,
            ex, ey);
        std::vector<PointType> seg3_edge_types = classify_edge_points(seg3_edge);

        // 标记特殊点
        if (!seg3_perp_back.empty()) {
            seg3_perp_back_types.back() = PT_PROJECTION;
        }

        // 拼接
        seg.points = seg3_perp_back;
        seg.point_types = seg3_perp_back_types;

        int perp_end = (int)seg.points.size() - 1;
        append_typed_paths(seg.points, seg.point_types, seg3_edge.points, seg3_edge_types);
        int edge_end = (int)seg.points.size() - 1;

        // 记录子段
        seg.sub_segments.push_back({0, perp_end, SUBSEG_PERP});
        seg.sub_segments.push_back({perp_end, edge_end, SUBSEG_EDGE});
    }

    return result;
}

/**
 * @brief 获取点所属子段的类型
 * @param idx 点索引
 * @param sub_segments 子段列表
 * @return 子段类型
 */
static SubSegType get_point_subseg_type(int idx, const std::vector<SubSegment>& sub_segments) {
    for (size_t i = 0; i < sub_segments.size(); i++) {
        if (idx >= sub_segments[i].start_idx && idx <= sub_segments[i].end_idx) {
            return sub_segments[i].type;
        }
    }
    return SUBSEG_EDGE; // 默认
}

/**
 * @brief 对单段路径进行前向-后向梯形速度规划
 * @param seg_data 单段路径元数据
 * @param config 速度配置
 * @return PathPoint 列表（含Vx,Vy,位置，yaw暂为0）
 */
std::vector<PathPoint> plan_segment_velocities(const SegmentPathData& seg_data,
                                                const PathConfig& config) {
    const std::vector<Point>& points = seg_data.points;
    const std::vector<PointType>& point_types = seg_data.point_types;
    const std::vector<SubSegment>& sub_segments = seg_data.sub_segments;

    int n = (int)points.size();
    std::vector<PathPoint> result(n);

    if (n == 0) return result;

    // 边缘段参数（mm/s, mm/s²）
    float edge_max_vel = config.max_vel * 1000.0f;   // m/s → mm/s
    float edge_accel   = config.accel * 1000.0f;      // m/s² → mm/s²
    float edge_decel   = config.decel * 1000.0f;      // m/s² → mm/s²

    // 垂直段参数（/2）
    float perp_max_vel = edge_max_vel / 2.0f;
    float perp_accel   = edge_accel / 2.0f;
    float perp_decel   = edge_decel / 2.0f;

    // 计算相邻点间距
    std::vector<float> dist(n, 0.0f);
    for (int i = 0; i < n - 1; i++) {
        float dx = points[i + 1].x - points[i].x;
        float dy = points[i + 1].y - points[i].y;
        dist[i] = sqrtf(dx * dx + dy * dy);
    }

    // 为每个点设置速度上限和加速度/减速度
    std::vector<float> max_vel_at(n, 0.0f);
    std::vector<float> accel_at(n, 0.0f);
    std::vector<float> decel_at(n, 0.0f);

    for (int i = 0; i < n; i++) {
        SubSegType stype = get_point_subseg_type(i, sub_segments);
        float local_max_vel, local_accel, local_decel;

        if (stype == SUBSEG_PERP) {
            local_max_vel = perp_max_vel;
            local_accel = perp_accel;
            local_decel = perp_decel;
        } else {
            local_max_vel = edge_max_vel;
            local_accel = edge_accel;
            local_decel = edge_decel;
        }

        // 根据点类型设置速度约束
        switch (point_types[i]) {
            case PT_TARGET:
                // 目标点：强制 v=0（停车）
                max_vel_at[i] = 0.0f;
                break;
            case PT_PROJECTION:
                // 投影点：速度限制 <= CORNER_SPEED_LIMIT
                max_vel_at[i] = std::min(local_max_vel, CORNER_SPEED_LIMIT);
                break;
            case PT_CORNER:
                // 角落点：速度限制 <= CORNER_SPEED_LIMIT
                max_vel_at[i] = std::min(local_max_vel, CORNER_SPEED_LIMIT);
                break;
            default:
                max_vel_at[i] = local_max_vel;
                break;
        }

        // 段起点和终点 v=0
        if (i == 0 || i == n - 1) {
            max_vel_at[i] = 0.0f;
        }

        // 加速度/减速度：取该点与相邻点之间的过渡参数
        // 对于子段边界处的点，使用前一个子段的减速度和后一个子段的加速度
        accel_at[i] = local_accel;
        decel_at[i] = local_decel;
    }

    // 对于区间加速度/减速度，使用两个端点中较小的值
    // 这样可以确保在子段边界处平滑过渡
    // 区间 [i, i+1] 的加速度 = min(accel_at[i], accel_at[i+1])
    // 区间 [i, i+1] 的减速度 = min(decel_at[i], decel_at[i+1])

    // ─── 前向遍历：从起点到终点 ───
    std::vector<float> v_forward(n, 0.0f);
    v_forward[0] = 0.0f;  // 起点速度为0
    for (int i = 1; i < n; i++) {
        float a = std::min(accel_at[i - 1], accel_at[i]);
        float v_from_prev = sqrtf(v_forward[i - 1] * v_forward[i - 1] + 2.0f * a * dist[i - 1]);
        v_forward[i] = std::min(v_from_prev, max_vel_at[i]);
    }

    // ─── 后向遍历：从终点到起点 ───
    std::vector<float> v_backward(n, 0.0f);
    v_backward[n - 1] = 0.0f;  // 终点速度为0
    for (int i = n - 2; i >= 0; i--) {
        float d = std::min(decel_at[i], decel_at[i + 1]);
        float v_from_next = sqrtf(v_backward[i + 1] * v_backward[i + 1] + 2.0f * d * dist[i]);
        v_backward[i] = std::min(v_from_next, max_vel_at[i]);
    }

    // ─── 最终速度：取前向和后向的最小值 ───
    std::vector<float> velocity(n, 0.0f);
    for (int i = 0; i < n; i++) {
        velocity[i] = std::min(v_forward[i], v_backward[i]);
    }

    // 确保起终点速度为0
    velocity[0] = 0.0f;
    velocity[n - 1] = 0.0f;

    // 确保目标点速度为0
    for (int i = 0; i < n; i++) {
        if (point_types[i] == PT_TARGET) {
            velocity[i] = 0.0f;
        }
    }

    // ─── 将标量速度分解为 Vx, Vy ───
    for (int i = 0; i < n; i++) {
        result[i].x = points[i].x;
        result[i].y = points[i].y;
        result[i].yaw = 0.0f;  // yaw 将在后续步骤中分配

        if (i == 0 || i == n - 1 || point_types[i] == PT_TARGET) {
            // 起点、终点、目标点：Vx = Vy = 0
            result[i].vx = 0.0f;
            result[i].vy = 0.0f;
            continue;
        }

        // 计算前进方向
        float dir_x, dir_y;
        if (i < n - 1) {
            // 优先使用向下一个点的方向
            float dx = points[i + 1].x - points[i].x;
            float dy = points[i + 1].y - points[i].y;
            float d = sqrtf(dx * dx + dy * dy);
            if (d > 0.01f) {
                dir_x = dx / d;
                dir_y = dy / d;
            } else if (i > 0) {
                // 如果与下一个点重合，使用上一个点的方向
                float dx2 = points[i].x - points[i - 1].x;
                float dy2 = points[i].y - points[i - 1].y;
                float d2 = sqrtf(dx2 * dx2 + dy2 * dy2);
                if (d2 > 0.01f) {
                    dir_x = dx2 / d2;
                    dir_y = dy2 / d2;
                } else {
                    dir_x = 1.0f;
                    dir_y = 0.0f;
                }
            } else {
                dir_x = 1.0f;
                dir_y = 0.0f;
            }
        } else {
            // 最后一个点（已在上面处理，这里为保险）
            float dx = points[i].x - points[i - 1].x;
            float dy = points[i].y - points[i - 1].y;
            float d = sqrtf(dx * dx + dy * dy);
            if (d > 0.01f) {
                dir_x = dx / d;
                dir_y = dy / d;
            } else {
                dir_x = 1.0f;
                dir_y = 0.0f;
            }
        }

        result[i].vx = velocity[i] * dir_x;
        result[i].vy = velocity[i] * dir_y;
    }

    return result;
}

/**
 * @brief 对三段路径进行速度规划
 */
void apply_velocity_planning(const ThreeSegmentPathData& path_data,
                              const PathConfig& config,
                              std::vector<PathPoint>& out_seg1,
                              std::vector<PathPoint>& out_seg2,
                              std::vector<PathPoint>& out_seg3) {
    out_seg1 = plan_segment_velocities(path_data.seg1, config);
    out_seg2 = plan_segment_velocities(path_data.seg2, config);
    out_seg3 = plan_segment_velocities(path_data.seg3, config);
}

//============================================================================
// 【yaw 分配与旋转标志插入函数实现】— 第五步
//============================================================================

/**
 * @brief 对单段路径分配 yaw 并插入旋转标志
 * @param path 带速度的路径（yaw=0），输出带yaw的路径（可能增加旋转标志行）
 * @param seg_data 元数据（点类型、子段信息）
 * @param start_yaw 段起点 yaw
 * @param end_yaw 段终点 yaw
 * @param is_first_seg 是否为第一段（起点=总起点）
 * @param is_last_seg 是否为最后一段（终点=总终点）
 */
void apply_yaw_to_segment(std::vector<PathPoint>& path,
                           const SegmentPathData& seg_data,
                           float start_yaw, float end_yaw,
                           bool is_first_seg, bool is_last_seg) {
    int n = (int)path.size();
    if (n == 0) return;

    // ─── 分支A: yaw 不变 → 全段使用 start_yaw ───
    if (fabsf(end_yaw - start_yaw) < 0.01f) {
        for (int i = 0; i < n; i++) {
            path[i].yaw = start_yaw;
        }
        return;
    }

    // ─── 分支B: yaw 不同 → 需要插入旋转标志 ───

    // 在路径中找拐角点（PT_CORNER 类型），选离起点最近的
    int first_corner_idx = -1;
    for (int i = 0; i < n; i++) {
        if (i < (int)seg_data.point_types.size() && seg_data.point_types[i] == PT_CORNER) {
            first_corner_idx = i;
            break;
        }
    }

    // 构建旋转标志
    PathPoint rotation_marker;
    rotation_marker.vx = ROTATION_MARKER_VEL;
    rotation_marker.vy = ROTATION_MARKER_VEL;

    if (first_corner_idx >= 0) {
        // ─── B1: 路径中间有拐角 → 拐角点前一行插入旋转标志 ───
        // 旋转标志的 x,y = 拐角坐标
        rotation_marker.x = path[first_corner_idx].x;
        rotation_marker.y = path[first_corner_idx].y;
        rotation_marker.yaw = end_yaw;

        // 先给所有点分配 yaw
        for (int i = 0; i < first_corner_idx; i++) {
            path[i].yaw = start_yaw;
        }
        // 拐角点及之后使用 end_yaw
        for (int i = first_corner_idx; i < n; i++) {
            path[i].yaw = end_yaw;
        }

        // 在拐角点前一行插入旋转标志
        path.insert(path.begin() + first_corner_idx, rotation_marker);

    } else if (is_first_seg) {
        // ─── B2: 无拐角 + 起点为总起点 → 起点后插入旋转标志 ───
        rotation_marker.x = FIXED_START.x;
        rotation_marker.y = FIXED_START.y;
        rotation_marker.yaw = end_yaw;

        // 第0个点（起点）yaw = start_yaw，之后 yaw = end_yaw
        path[0].yaw = start_yaw;
        for (int i = 1; i < n; i++) {
            path[i].yaw = end_yaw;
        }

        // 在起点后（索引1）插入旋转标志
        path.insert(path.begin() + 1, rotation_marker);

    } else if (is_last_seg) {
        // ─── B3: 无拐角 + 终点为总终点 → 终点前一行插入旋转标志 ───
        rotation_marker.x = FIXED_END.x;
        rotation_marker.y = FIXED_END.y;
        rotation_marker.yaw = end_yaw;

        // 终点之前的点 yaw = start_yaw，终点 yaw = end_yaw
        for (int i = 0; i < n - 1; i++) {
            path[i].yaw = start_yaw;
        }
        path[n - 1].yaw = end_yaw;

        // 在终点前一行（索引 n-1）插入旋转标志
        path.insert(path.begin() + (n - 1), rotation_marker);

    } else {
        // 不应该出现的情况：yaw不同，无拐角，既不是第一段也不是最后一段
        // 安全处理：全段使用 start_yaw
        for (int i = 0; i < n; i++) {
            path[i].yaw = start_yaw;
        }
    }
}

/**
 * @brief 对三段路径分别分配 yaw 并插入旋转标志
 * @param seg1,seg2,seg3 三段带速度的路径（yaw=0）
 * @param path_data 三段路径的元数据
 * @param wp1_num WP1编号
 * @param wp2_num WP2编号
 */
void apply_yaw_to_all_segments(std::vector<PathPoint>& seg1,
                                std::vector<PathPoint>& seg2,
                                std::vector<PathPoint>& seg3,
                                const ThreeSegmentPathData& path_data,
                                int wp1_num, int wp2_num) {
    // Seg1: start_yaw = FIXED_START.yaw, end_yaw = WP1.yaw
    float seg1_start_yaw = FIXED_START.yaw;
    float seg1_end_yaw = PROJECTION_POINTS[wp1_num].yaw;
    apply_yaw_to_segment(seg1, path_data.seg1, seg1_start_yaw, seg1_end_yaw, true, false);

    // Seg2: start_yaw = WP1.yaw, end_yaw = WP2.yaw
    float seg2_start_yaw = PROJECTION_POINTS[wp1_num].yaw;
    float seg2_end_yaw = PROJECTION_POINTS[wp2_num].yaw;
    apply_yaw_to_segment(seg2, path_data.seg2, seg2_start_yaw, seg2_end_yaw, false, false);

    // Seg3: start_yaw = WP2.yaw, end_yaw = FIXED_END.yaw
    float seg3_start_yaw = PROJECTION_POINTS[wp2_num].yaw;
    float seg3_end_yaw = FIXED_END.yaw;
    apply_yaw_to_segment(seg3, path_data.seg3, seg3_start_yaw, seg3_end_yaw, false, true);
}

//============================================================================
// 【蓝区镜像函数实现】— 第六步
//============================================================================

/**
 * @brief 对单段路径进行蓝区镜像处理（ZONE == -1 时调用）
 * @param path 待镜像的路径（原地修改）
 * @note 镜像规则（仅 y 轴方向数据取反）：
 *   y → -y（全部取反）
 *   yaw → -yaw（全部取反）
 *   vy → -vy（旋转标志位和起点终点标志位除外）
 *   vx, x → 不变
 * 例外：旋转标志位（vx≈vy≈ROTATION_MARKER_VEL）和起点终点标志位（首尾点）的 vy 不取反
 */
void apply_zone_mirror_to_path(std::vector<PathPoint>& path) {
    int n = (int)path.size();
    if (n == 0) return;

    for (int i = 0; i < n; i++) {
        // y 取反
        path[i].y = -path[i].y;

        // yaw 取反（非零时取反，为零时不变，避免 -0.000）
        if (path[i].yaw != 0.0f) {
            path[i].yaw = -path[i].yaw;
        }

        // 判断是否为旋转标志位：vx ≈ ROTATION_MARKER_VEL 且 vy ≈ ROTATION_MARKER_VEL
        bool is_rotation_marker = (fabsf(path[i].vx - ROTATION_MARKER_VEL) < 0.0005f &&
                                   fabsf(path[i].vy - ROTATION_MARKER_VEL) < 0.0005f);

        // 判断是否为起点/终点标志位：首尾点
        bool is_start_or_end = (i == 0 || i == n - 1);

        // vy 取反（旋转标志位和起点终点标志位除外，非零时取反，为零时不变，避免 -0.000）
        if (!is_rotation_marker && !is_start_or_end) {
            if (path[i].vy != 0.0f) {
                path[i].vy = -path[i].vy;
            }
        }
    }
}

/**
 * @brief 保存三段路径到文件（C数组格式，含速度信息）
 * @param seg1,seg2,seg3 三段路径
 * @param filename 输出文件名
 */
void save_three_segments(const std::vector<PathPoint>& seg1,
                          const std::vector<PathPoint>& seg2,
                          const std::vector<PathPoint>& seg3,
                          const char* filename) {
    FILE* fp = fopen(filename, "w");
    if (!fp) {
        printf("Error: Cannot open file %s for writing\n", filename);
        return;
    }

    fprintf(fp, "/* Auto-generated path data - Three Segments (With Velocity) */\n");
    fprintf(fp, "/* Vx(mm/s), Vy(mm/s), X(mm), Y(mm), yaw(rad) */\n\n");

    const char* seg_names[3] = {"path_segment_1", "path_segment_2", "path_segment_3"};
    const std::vector<PathPoint>* segs[3] = {&seg1, &seg2, &seg3};

    for (int s = 0; s < 3; s++) {
        fprintf(fp, "// ========== Segment %d (%zu points) ==========\n", s + 1, segs[s]->size());
        fprintf(fp, "const float %s[][5] = {\n", seg_names[s]);
        for (size_t i = 0; i < segs[s]->size(); i++) {
            const PathPoint& pt = segs[s]->at(i);
            fprintf(fp, "{%11.3ff, %11.3ff, %11.3ff, %11.3ff, %8.3ff}",
                    pt.vx, pt.vy, pt.x, pt.y, roundf(pt.yaw * 1000.0f) / 1000.0f);
            if (i + 1 < segs[s]->size()) fprintf(fp, ",");
            fprintf(fp, "\n");
        }
        fprintf(fp, "};\n\n");
    }

    fprintf(fp, "// Path segment sizes\n");
    fprintf(fp, "uint16_t path_segment_sizes[3] = {%zu, %zu, %zu};\n",
            seg1.size(), seg2.size(), seg3.size());

    fclose(fp);
    printf("Velocity path saved to %s\n", filename);
}

//============================================================================
// 【核心 API 函数实现】— 主入口
//============================================================================

/**
 * @brief 主入口函数：完整路径生成流水线
 * @param wp_num1 第一个中间点编号 (1-12, 5/8无效)
 * @param wp_num2 第二个中间点编号
 * @param config  速度配置 (max_vel/accel/decel)
 * @param zone    红蓝区: 1=红区, -1=蓝区, 0=空(不输出)
 * @return ThreeSegmentResult 三段路径 + 排序后的WP1/WP2编号
 *
 * 流水线步骤:
 *   Step 0: 验证编号 + 选择最短顺序
 *   Step 1: 生成带元数据的位置路径
 *   Step 2: 速度规划（梯形加减速）
 *   Step 3: yaw 分配 + 旋转标志插入
 *   Step 4: 确保每段首尾 Vx=Vy=0
 *   Step 5: 蓝区镜像（zone==-1 时）
 *   Step 6: 存入全局数组 g_seg1/g_seg2/g_seg3
 */
ThreeSegmentResult generate_path_from_waypoint_numbers(
    int wp_num1, int wp_num2, const PathConfig& config, int zone) {

    ThreeSegmentResult result;

    // ─── Step 0: 验证编号 ───
    if (!validate_wp_number(wp_num1) || !validate_wp_number(wp_num2)) {
        printf("[PathGenerator] Error: Invalid waypoint number %d or %d\n", wp_num1, wp_num2);
        return result;  // 返回空结果
    }

    // ─── Step 0: 选择最短顺序 ───
    std::pair<int, int> order = select_waypoint_order(wp_num1, wp_num2);
    result.selected_wp1_num = order.first;
    result.selected_wp2_num = order.second;
    printf("[PathGenerator] WP1=%d, WP2=%d\n", order.first, order.second);

    // ─── Step 1: 生成带元数据的位置路径 ───
    ThreeSegmentPathData path_data = generate_position_path_with_meta(order.first, order.second);

    // ─── Step 2: 速度规划 ───
    std::vector<PathPoint> vel_seg1, vel_seg2, vel_seg3;
    apply_velocity_planning(path_data, config, vel_seg1, vel_seg2, vel_seg3);

    // ─── Step 3: yaw 分配与旋转标志插入 ───
    apply_yaw_to_all_segments(vel_seg1, vel_seg2, vel_seg3,
                              path_data, order.first, order.second);

    // ─── Step 4: 确保每段首尾 Vx=Vy=0 ───
    if (!vel_seg1.empty()) { vel_seg1.front().vx=0; vel_seg1.front().vy=0; vel_seg1.back().vx=0; vel_seg1.back().vy=0; }
    if (!vel_seg2.empty()) { vel_seg2.front().vx=0; vel_seg2.front().vy=0; vel_seg2.back().vx=0; vel_seg2.back().vy=0; }
    if (!vel_seg3.empty()) { vel_seg3.front().vx=0; vel_seg3.front().vy=0; vel_seg3.back().vx=0; vel_seg3.back().vy=0; }

    // ─── Step 5: 蓝区镜像 ───
    if (zone == -1) {
        apply_zone_mirror_to_path(vel_seg1);
        apply_zone_mirror_to_path(vel_seg2);
        apply_zone_mirror_to_path(vel_seg3);
        printf("[PathGenerator] Blue zone mirror applied\n");
    }

    // ─── Step 6: 存入结果和全局数组 ───
    result.seg1 = vel_seg1;
    result.seg2 = vel_seg2;
    result.seg3 = vel_seg3;

    g_seg1 = vel_seg1;
    g_seg2 = vel_seg2;
    g_seg3 = vel_seg3;

    printf("[PathGenerator] Path generated successfully: seg1=%zu, seg2=%zu, seg3=%zu points\n",
           vel_seg1.size(), vel_seg2.size(), vel_seg3.size());

    return result;
}

//============================================================================
// 【main入口】— 可通过 PATH_GENERATOR_NO_MAIN 宏禁用
//============================================================================
#ifndef PATH_GENERATOR_NO_MAIN

int main() {
    PathConfig config;
    ThreeSegmentResult result = generate_path_from_waypoint_numbers(
        EXAMPLE_WP_NUM1, EXAMPLE_WP_NUM2, config, ZONE);

    if (result.seg1.empty() && result.seg2.empty() && result.seg3.empty()) {
        printf("[PathGenerator] Failed to generate path\n");
        return 1;
    }

    save_three_segments(result.seg1, result.seg2, result.seg3,
                        "generated_path_three_segments.txt");

    return 0;
}

#endif
