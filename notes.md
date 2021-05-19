# Piecewise_jerk_path_optimizer 
***
## 一、主要任务
对不同任务下的reference line以及对应的可行驶区域（Boundary）通过optimizer求出最优path
## 二、解决方法

## 三、代码学习
PiecewiseJerkPathOptimizer 规划器的核心函数为 Process（）
```
  common::Status Process(const SpeedData& speed_data,
                         const ReferenceLine& reference_line,
                         const common::TrajectoryPoint& init_point,
                         const bool path_reusable,
                         PathData* const path_data) override;
```
* Input：
    * 车速信息
    * 参考线信息
    * 规划开始时的位置信息
    * 路径重复使用标志位
* Output
    * Frenet坐标系下路径点集
***
### 2.1 预处理
1. 将输入参数的坐标转换至Frenet坐标系下
2. 根据任务载入预设权重向量（仅有两种选择：默认 or 换道）
```mermaid
  const auto& config = reference_line_info_->IsChangeLanePath()
                           ? config_.piecewise_jerk_path_optimizer_config()
                                 .lane_change_path_config()
                           : config_.piecewise_jerk_path_optimizer_config()
                                 .default_path_config();

  std::array<double, 5> w = {
      config.l_weight(),
      config.dl_weight() *
          std::fmax(init_frenet_state.first[1] * init_frenet_state.first[1],
                    5.0),
      config.ddl_weight(), config.dddl_weight(), 0.0};
```   
3. 载入所有类型道路边界（可行驶区域），开始针对每一种情况计算最优路径
```mermaid
const auto& path_boundaries =
      reference_line_info_->GetCandidatePathBoundaries();

  std::vector<PathData> candidate_path_data;
  for (const auto& path_boundary : path_boundaries) {
    ...}
```
4. Check regular类型边界是否合理，不合理则跳过
```mermaid
   if (path_boundary.label().find("regular") != std::string::npos &&
        path_boundary_size < 2) {
      continue;
    }
```   
5. 设置规划器最大循环计算次数为4000
  ```mermaid
    int max_iter = 4000;
    // lower max_iter for regular/self/
    if (path_boundary.label().find("self") != std::string::npos) {
      max_iter = 4000;
    }
```
6. 设置终点坐标（0,0,0）/(l,dl,ddl)
```mermaid
std::array<double, 3> end_state = {0.0, 0.0, 0.0};
```
7. 如果是pull over工况则调整终点坐标为pull over位置
```mermaid
    if (!FLAGS_enable_force_pull_over_open_space_parking_test) {
      // pull over scenario
      // set end lateral to be at the desired pull over destination
      const auto& pull_over_status =
          injector_->planning_context()->planning_status().pull_over();
      if (pull_over_status.has_position() &&
          pull_over_status.position().has_x() &&
          pull_over_status.position().has_y() &&
          path_boundary.label().find("pullover") != std::string::npos) {
        common::SLPoint pull_over_sl;
        reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);
        end_state[0] = pull_over_sl.l();
      }
    }
```
8. Check Regular Reference Line有效，并将参考线路径点转换到Frenet坐标系下，
   最终提取L坐标，并更新终点坐标为reference line终点坐标。

```mermaid
if (path_boundary.label().find("regular") != std::string::npos &&
        reference_path_data.is_valid_path_reference()) {
      ADEBUG << "path label is: " << path_boundary.label();
      // when path reference is ready
      for (size_t i = 0; i < path_reference_size; ++i) {
        common::SLPoint path_reference_sl;
        reference_line.XYToSL(
            common::util::PointFactory::ToPointENU(
                reference_path_data.path_reference().at(i).x(),
                reference_path_data.path_reference().at(i).y()),
            &path_reference_sl);
        path_reference_l[i] = path_reference_sl.l();
      }
      end_state[0] = path_reference_l.back();
      path_data.set_is_optimized_towards_trajectory_reference(true);
      is_valid_path_reference = true;
    }
```
9. 读取车辆参数，根据动力学计算横向加速度ddl约束：k = tan（a）/ L； ay = k × vx^2
最终约束为当前点边界线约束+车辆动力学约束
```mermaid
const auto& veh_param =
        common::VehicleConfigHelper::GetConfig().vehicle_param();
    const double lat_acc_bound =
        std::tan(veh_param.max_steer_angle() / veh_param.steer_ratio()) /
        veh_param.wheel_base();
    std::vector<std::pair<double, double>> ddl_bounds;
    for (size_t i = 0; i < path_boundary_size; ++i) {
      double s = static_cast<double>(i) * path_boundary.delta_s() +
                 path_boundary.start_s();
      double kappa = reference_line.GetNearestReferencePoint(s).kappa();
      ddl_bounds.emplace_back(-lat_acc_bound - kappa, lat_acc_bound - kappa);
    }
```
### 2.2 二次规划
1. 通过二次规划对路径进行优化，核心函数为OptimizePath（）
    - Input：
      - SL坐标系下初始L坐标
      - 终点坐标
      - SL坐标系下reference line 路径坐标
      - 路径点数量
      - 路径离散步长
      - 路径有效标志位
      - 路径边界
      - 横向加速度ddl约束
      - 权重向量
      - 最大计算次数
    - Output：
      - SL坐标系下最优路径点坐标（l，dl，ddl）
```mermaid
    bool res_opt = OptimizePath(
        init_frenet_state.second, end_state, std::move(path_reference_l),
        path_reference_size, path_boundary.delta_s(), is_valid_path_reference,
        path_boundary.boundary(), ddl_bounds, w, max_iter, &opt_l, &opt_dl,
        &opt_ddl);
```
