==============================================================================
                        AI_README.md — 路径生成器项目总览
==============================================================================
                       基于 C++11 的线性插值路径生成器

本文件供 AI 快速了解项目结构、代码逻辑和关键参数，以便高效修改和维护。

==============================================================================
一、项目简介
==============================================================================
  语言：C++11（g++ -std=c++11 编译，需 -lm 链接数学库）
  功能：沿矩形边缘生成最短路径，包含垂直往返段、梯形速度规划、
        yaw 旋转标志插入、红蓝区镜像等功能。
  输出：generated_path_three_segments.txt（C 数组格式，三组 float[][5]）
  编译：
    独立 exe：    g++ -std=c++11 -o path_generator path_generator.cpp -lm
    库（无main）：g++ -std=c++11 -DPATH_GENERATOR_NO_MAIN -c path_generator.cpp -lm

  文件清单：
    path_generator.cpp    — 主程序（含 main 入口，可通过宏禁用）
    path_generator.h      — 接口头文件（数据结构、API 声明）
    README.md             — 使用说明
    AI_README.md          — 本文件

==============================================================================
二、地图与坐标定义
==============================================================================

  【地图边界】（单位：mm）
    RECT_X_MIN = 2600        RECT_X_MAX = 8600
    RECT_Y_MIN = 600         RECT_Y_MAX = 5400
    矩形周长 = 21600mm = 2*(6000+4800)
    方向约定：x 向前，y 向左
    四条边索引：0=下边(x=2600), 1=右边(y=600), 2=上边(x=8600), 3=左边(y=5400)
    除了中间过渡点与中间点，其他路径段插值需要沿着我的四条边进行规划
    中间过渡点与中间点之间

  【总起点 & 总终点】（x, y, yaw）
    FIXED_START = (2600, 5400, 0)       —— 下边左下角
    FIXED_END   = (8600, 600, 0)        —— 下边右下角

  【PROJECTION_POINTS】中间过渡点（10个，在矩形边缘上，用于路径计算）
    编号  (x,     y)      yaw      所在边
     1    (2600, 1800)    0.000     下边(x=2600)
     2    (2600, 3000)    0.000     下边
     3    (2600, 4200)    0.000     下边
     4    (5000,  600)    1.571     右边(y=600)
     7    (6200,  600)    1.571     右边
    10    (7400,  600)    1.571     右边
     6    (5000, 5400)   -1.571     左边(y=5400)
     9    (6200, 5400)   -1.571     左边
    12    (7400, 5400)   -1.571     左边
    11    (8600, 3000)    3.141     上边(x=8600)

  【TARGET_POINTS】中间点（10个，中间点坐标）
    编号  (x,     y)      yaw      所在边
     1    (2750, 1800)    0.000     下边
     2    (2750, 3000)    0.000     下边
     3    (2750, 4200)    0.000     下边
     4    (5000,  650)    1.571     右边
     7    (6200,  650)    1.571     右边
    10    (7400,  650)    1.571     右边
     6    (5000, 5350)   -1.571     左边
     9    (6200, 5350)   -1.571     左边
    12    (7400, 5350)   -1.571     左边
    11    (8550, 3000)    3.141     上边

  【CORNERS】矩形四角（用于旋转逻辑）
    CORNERS[0] = (2600, 5400)   左下角（= FIXED_START）
    CORNERS[1] = (8600, 5400)   左上角
    CORNERS[2] = (8600,  600)   右上角（= FIXED_END）
    CORNERS[3] = (2600,  600)   右下角

==============================================================================
三、关键配置参数
==============================================================================

  【路径生成】
    PATH_POINT_INTERVAL = 150.0        // 线性插值点间隔 (mm)

  【速度配置】
    DEFAULT_MAX_VEL  = 2.0             // 最大速度 (m/s)
    DEFAULT_ACCEL    = 2.0             // 最大加速度 (m/s²)
    DEFAULT_DECEL    = 1.5             // 最大减速度 (m/s²)
    速度规划公式：v = sqrt(v0² + 2*a*d) （梯形加减速）

  【旋转标志】
    ROTATION_MARKER_VEL = 0.001        // 旋转标志速度值 (mm/s)，Vx=Vy=0.001

  【拐角速度限制】
    CORNER_SPEED_LIMIT = 10.0          // 投影点和不旋转角落点的速度上限 (mm/s)

  【中间点偏移】
    WAYPOINT_OFFSET_DISTANCE = 298.940   // 默认 298.940mm
    偏移规则（以车的左右方向为基准）：
      WP1 的投影点 + 目标点 → 同时沿车右方向偏移
      WP2 的投影点 + 目标点 → 同时沿车左方向偏移
    偏移方式：基于边缘类型的精确偏移（避免 sin/cos 浮点误差），效果与 (sinf(yaw), -cosf(yaw)) * offset 等价：
      下边(yaw=0):     车右方向=-y → offset=(0, -dist)
      右边(yaw=π/2):   车右方向=+x → offset=(+dist, 0)
      上边(yaw=π):     车右方向=+y → offset=(0, +dist)
      左边(yaw=-π/2):  车右方向=-x → offset=(-dist, 0)

  【红蓝区镜像】
    ZONE = -1                         // -1=蓝区(最终输出y取反), 0=空(不输出), 1=红区
    镜像规则（仅作用于最终输出，内部计算用原始坐标）：
      蓝区: y→-y, vy→-vy（旋转标志位和路径起点终点标志位除外）, yaw→-yaw

  【起点终点标志位】
    每段路径的起点和终点速度Vx，Vy都为0.000

==============================================================================
四、路径生成逻辑（分段详解）
==============================================================================

  总入口函数：generate_path_from_waypoint_numbers(wp_num1, wp_num2, config, zone)

  【Step 0 — 验证与选择最短顺序】
    1. 验证输入的中间点编号是否有效（1-12 中的合法编号，5/8为无效占位）
    2. 计算 START→WP1→WP2→END 和 START→WP2→WP1→END 两种顺序沿矩形边缘的总距离
    3. 选择总距离较短的顺序，先到的为WP1，后到的为WP2

  【Step 1 — 构建带元数据的位置路径（7个子段→3段）】
    路径中的数据必须包含我的总起点、总终点、拐点、中间过渡点、中间目标点的坐标
    以下 7 段依次拼接成三段完整路径：
      Seg1: edge(总起点→WP1偏移投影点) + perp(WP1偏移投影点→WP1偏移目标点)
      Seg2: perp(WP1偏移目标点→WP1偏移投影点) + edge(WP1偏移投影点→WP2偏移投影点)
            + perp(WP2偏移投影点→WP2偏移目标点)
      Seg3: perp(WP2偏移目标点→WP2偏移投影点) + edge(WP2偏移投影点→总终点)

    偏移后的投影点和目标点由 calculate_shifted_points() 计算。
    拼接时去除重复的连接点（edge的终点与perp的起点重合时只保留一个）。

    关键子函数：
      generate_edge_path_shortest(x1,y1,x2,y2)
        → 返回 EdgePathResult {points, corner_indices}
        → 沿矩形最短边缘方向生成路径（顺时针或逆时针选较短方向）
        → 包含拐角索引标记（用于后续旋转判断）
        → 插值已在此函数内完成（按 PATH_POINT_INTERVAL 间距）
      generate_perp_path(from_x,from_y,to_x,to_y)
        → 生成纯垂直方向位置点（按 PATH_POINT_INTERVAL 插值）
        → 最大速度与最大加速为用户输入值/2
        → 纯直角，Vx 和 Vy 不同时非零

    元数据追踪（generate_position_path_with_meta()）：
      每个点标记类型：PT_NORMAL / PT_PROJECTION / PT_TARGET / PT_CORNER
      每个子段标记类型：SUBSEG_EDGE / SUBSEG_PERP
      输出 ThreeSegmentPathData 供后续速度规划和 yaw 分配使用

  【Step 2 — 速度规划】
    调用 apply_velocity_planning() → 内部调用 plan_segment_velocities()
    → 前向-后向梯形速度规划（起点加速、终点减速）
    → 边缘段使用 config 原始参数（max_vel, accel, decel），垂直段使用 config/2
    → WP1/WP2 目标点（PT_TARGET）强制 v=0（停车）
    → 投影点（PT_PROJECTION）和不旋转的角落点（PT_CORNER）速度上限为 CORNER_SPEED_LIMIT
    → 段起点和终点 v=0
    → Vx,Vy 由前进方向向量×标量速度得出

  【Step 3 — yaw 分配与旋转标志插入】（详细见五）
    调用 apply_yaw_to_all_segments() 对三段分别处理
    yaw角为坐标点中定义的yaw角

    每段路径中判断起点和终点的yaw角是否相同。
    如果相同，那么整段路径保持yaw角不变。
    如果yaw角不同，那么分为三种拐点旋转情况（拐点为四个，有两个拐点就是总起点和总终点）
            1、起点到终点yaw角不同，选择路径中的最近的那个拐角前一行插入旋转标志。
            2、起点到终点yaw角不同，但是起点为总起点且路径中间不经过拐角，那么应该在起点后插入旋转标志。
            3、起点到终点的yaw角不同，但是终点为总终点且路径中间不经过拐角，那么应该在总终点前一行添加旋转标志。
    插入的一行旋转标志格式：
            Vx=Vy=0.001；x，y坐标为拐点坐标，yaw为终点yaw角
            旋转前的yaw保持为起点yaw角不变，旋转后的yaw保持为终点yaw角不变。

  【Step 4 — 确保每段首尾 Vx=Vy=0】
    对 seg1/seg2/seg3 的首尾点设置 vx=vy=0

  【Step 5 — 蓝区镜像（如需要）】
    zone == -1 时对三段分别调用 apply_zone_mirror_to_path()

  【Step 6 — 存入全局数组】
    结果存入 g_seg1/g_seg2/g_seg3 及 ThreeSegmentResult 返回值

==============================================================================
五、旋转标志插入逻辑（apply_yaw_to_segment）
==============================================================================

  功能：对一段路径（已有速度，无 yaw）分配 yaw 并插入旋转标志

  输入：
    path, seg_data (SegmentPathData元数据),
    start_yaw, end_yaw, is_first_seg, is_last_seg

  分支判定：

    【分支A】yaw 不变（|end_yaw - start_yaw| < 0.01）
      → 全段使用 start_yaw，不插旋转标志

    【分支B】yaw 变化 + 段内有拐角（seg_data.point_types 中有 PT_CORNER）
      → B1: 拐角在段中间 → 拐角前一行插入旋转标志
            (Vx=Vy=ROTATION_MARKER_VEL, x=拐角x, y=拐角y, yaw=end_yaw)
      → B2: 无拐角 + 起点为总起点(is_first_seg=true) → 起点后插入旋转标志
      → B3: 无拐角 + 终点为总终点(is_last_seg=true) → 终点前一行插入旋转标志

  旋转标志格式：
    Vx=Vy=ROTATION_MARKER_VEL (=0.001)
    x,y = 拐角坐标（B1）或起终点坐标（B2/B3）
    yaw = 终点 yaw

  注意事项：
    - 垂直段（前往/返回目标点）的 yaw 与所在 WP 的 yaw 相同
    - 旋转标志只在矩形四角处插入，不在中间点/目标点处插入
    - 三个拐点旋转规则对应需求文档第六章

==============================================================================
六、数据结构定义（全部在 path_generator.h）
==============================================================================

    Point { float x, y; }
    PathConfig { float max_vel, accel, decel; }
    PathPoint { float vx, vy, x, y, yaw; }  // v(mm/s), x(mm), yaw(rad)
    WaypointData { float x, y, yaw; }
    ThreeSegmentResult { seg1, seg2, seg3, selected_wp1_num, selected_wp2_num; }

  路径生成中间结果：
    EdgePathResult { vector<Point> points; vector<int> corner_indices; }
    ThreeSegmentPositionPath { vector<Point> seg1, seg2, seg3; }
    ShiftedWaypoints { Point shifted_projection; Point shifted_target; }

  速度规划相关：
    PointType 枚举: PT_NORMAL / PT_PROJECTION / PT_TARGET / PT_CORNER
    SubSegType 枚举: SUBSEG_EDGE / SUBSEG_PERP
    SubSegment { int start_idx, end_idx; SubSegType type; }
    SegmentPathData { vector<Point> points; vector<PointType> point_types;
                       vector<SubSegment> sub_segments; }
    ThreeSegmentPathData { SegmentPathData seg1, seg2, seg3; }

  全局变量（供外部代码直接引用）：
    g_seg1 / g_seg2 / g_seg3 (std::vector<PathPoint>)

==============================================================================
七、核心 API
==============================================================================

    ThreeSegmentResult generate_path_from_waypoint_numbers(
        int wp_num1, int wp_num2, const PathConfig& config, int zone);

    void save_three_segments(
        const std::vector<PathPoint>& seg1,
        const std::vector<PathPoint>& seg2,
        const std::vector<PathPoint>& seg3,
        const char* filename);

==============================================================================
八、输出文件格式
==============================================================================

  generated_path_three_segments.txt 包含三个 C 数组：
    float path_segment_1[][5] = {{vx,vy,x,y,yaw}, ...};
    float path_segment_2[][5] = {...};
    float path_segment_3[][5] = {...};
    uint16_t path_segment_sizes[3] = {n1, n2, n3};

  每行 5 列：Vx(mm/s), Vy(mm/s), X(mm), Y(mm), yaw(rad)
  起点/终点 Vx=Vy=0，目标点 Vx=Vy=0（停车）

==============================================================================
九、关键函数速查表
==============================================================================

  函数名                                  位置         功能
  ────────────────────────────────────────────────────────────────────────
  get_edge_index()                        辅助         判断点在矩形哪条边
  get_corner_index()                      辅助         判断点是否为拐角
  calculate_perimeter_position()          辅助         点→顺时针周长位置
  perimeter_position_to_point()           辅助         周长位置→坐标
  calculate_clockwise_distance()          辅助         顺时针边缘距离
  calculate_edge_distance()               辅助         最短边缘距离
  validate_wp_number()                    辅助         验证中间点编号
  select_waypoint_order()                 辅助         选择最短中间点顺序
  calculate_shifted_points()              辅助         计算偏移后的投影点+目标点
  generate_edge_path_shortest()           辅助         沿边缘最短路径生成+插值
  generate_perp_path()                    辅助         垂直方向直线插值
  generate_position_path()                位置路径     三段位置路径（仅位置）
  generate_position_path_with_meta()      位置路径     三段带元数据位置路径
  plan_segment_velocities()               速度规划     单段前向-后向梯形速度
  apply_velocity_planning()               速度规划     三段速度规划
  apply_yaw_to_segment()                  旋转逻辑     yaw分配+旋转标志（单段）
  apply_yaw_to_all_segments()             旋转逻辑     yaw分配+旋转标志（三段）
  apply_zone_mirror_to_path()             输出         蓝区 y 轴镜像
  generate_path_from_waypoint_numbers()   主入口       验证+选择+生成+镜像
  save_three_segments()                   输出         写文件

十、用户可修改数据

  【调整地图边界】→ RECT_X_MIN/MAX, RECT_Y_MIN/MAX
  【修改起点/终点】→ FIXED_START, FIXED_END
  【添加/修改中间点】→ PROJECTION_POINTS + TARGET_POINTS 同步修改
  【调整路径点密度】→ PATH_POINT_INTERVAL
  【调整速度】→ DEFAULT_MAX_VEL, DEFAULT_ACCEL, DEFAULT_DECEL
  【调整过弯速度】→ CORNER_SPEED_LIMIT
  【调整中间点偏移】→ WAYPOINT_OFFSET_DISTANCE
  【切换红蓝区】→ ZONE
  【修改示例输入】→ EXAMPLE_WP_NUM1, EXAMPLE_WP_NUM2
  【旋转标志速度值】→ ROTATION_MARKER_VEL

                        END OF AI_README.md
