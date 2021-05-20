# Piecewise_jerk_path_optimizer 
***
## 一、主要任务
对不同任务下的reference line以及对应的可行驶区域（Boundary）通过optimizer求出最优path
## 二、解决方法
通过构造并求解二次规划问题获取最优路径  

![二次规划的一般形式](https://i.loli.net/2021/05/20/k4QRlKGSf7mZpOU.png)  

路径规划将SL坐标系下的（l,dl,ddl）作为优化量，构建成本函数：  


- 目标是令l, dl, ddl，dddl最小，即偏离路径距离最小，横向速度最小，横向加速度最小，横向jerk最小  
- dddl通过ddl差分求出，最终Cost Function形式如下：  

![Cost Function](https://i.loli.net/2021/05/20/oetk1dx9ZFRKYwy.png)  

- 构建P矩阵：  
  ![Matrix P](https://i.loli.net/2021/05/20/5dyPaixY6gcQFRb.png)  
  
- 构造二次规划的约束条件：  
  ![constrain](https://i.loli.net/2021/05/20/DCatKIU3j16JAye.png)  
  
  
- 构建A矩阵，以及上下边界矩阵

- 由于成本函数中不含有一次项，因此q为零向量

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
### 3.1 预处理
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
### 3.2 二次规划
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
2. 设置默认终点位置权重，如果为pull over工况则更新为相应权重
```mermaid
  piecewise_jerk_problem.set_end_state_ref({1000.0, 0.0, 0.0}, end_state);
  // pull over scenarios
  // Because path reference might also make the end_state != 0
  // we have to exclude this condition here
  if (end_state[0] != 0 && !is_valid_path_reference) {
    std::vector<double> x_ref(kNumKnots, end_state[0]);
    const auto& pull_over_type = injector_->planning_context()
                                     ->planning_status()
                                     .pull_over()
                                     .pull_over_type();
    const double weight_x_ref =
        pull_over_type == PullOverStatus::EMERGENCY_PULL_OVER ? 200.0 : 10.0;
    piecewise_jerk_problem.set_x_ref(weight_x_ref, std::move(x_ref));
  }
```
3. 构造二次规划结构，构造Cost Function权重以及约束条件
```mermaid
  piecewise_jerk_problem.set_weight_x(w[0]);
  piecewise_jerk_problem.set_weight_dx(w[1]);
  piecewise_jerk_problem.set_weight_ddx(w[2]);
  piecewise_jerk_problem.set_weight_dddx(w[3]);

  piecewise_jerk_problem.set_scale_factor({1.0, 10.0, 100.0});

  auto start_time = std::chrono::system_clock::now();

  piecewise_jerk_problem.set_x_bounds(lat_boundaries);
  piecewise_jerk_problem.set_dx_bounds(-FLAGS_lateral_derivative_bound_default,
                                       FLAGS_lateral_derivative_bound_default);
  piecewise_jerk_problem.set_ddx_bounds(ddl_bounds);

  // Estimate lat_acc and jerk boundary from vehicle_params
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const double axis_distance = veh_param.wheel_base();
  const double max_yaw_rate =
      veh_param.max_steer_angle_rate() / veh_param.steer_ratio() / 2.0;
  const double jerk_bound = EstimateJerkBoundary(std::fmax(init_state[1], 1.0),
                                                 axis_distance, max_yaw_rate);
  piecewise_jerk_problem.set_dddx_bound(jerk_bound);
```
4. 调用Optimize（）函数构造P，A矩阵以及q向量，并调用osqp库解优化问题
```mermaid
bool success = piecewise_jerk_problem.Optimize(max_iter);
```
5. Optimize（）函数内部首先调用FormulateProblem（）函数构造P，A矩阵以及q向量
```mermaid
OSQPData* PiecewiseJerkProblem::FormulateProblem() {
  // calculate kernel
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  CalculateKernel(&P_data, &P_indices, &P_indptr);

  // calculate affine constraints
  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  CalculateAffineConstraint(&A_data, &A_indices, &A_indptr, &lower_bounds,
                            &upper_bounds);

  // calculate offset
  std::vector<c_float> q;
  CalculateOffset(&q);
```
6. P、A矩阵采用csc矩阵构造方式，CalculateKernel（）以及 CalculateAffineConstraint（）分别构造csc矩阵所需的输入：  
    1. 矩阵大小 m×n
    2. 非零元素个数
    3. 非零元素值
    4. 非零元素行索引
    5. 列指针  
```mermaid
data->P = csc_matrix(kernel_dim, kernel_dim, P_data.size(), CopyData(P_data),
                       CopyData(P_indices), CopyData(P_indptr));
data->A = csc_matrix(num_affine_constraint, kernel_dim, A_data.size(),
                 CopyData(A_data), CopyData(A_indices), CopyData(A_indptr));
```
+ CSC矩阵介绍：  
  Compressed Sparse Column（CSC）矩阵构造方式是基于列添加非零元素的位置信息，结合非零元素值来构建矩阵.目的是为了减少矩阵储存空间  
  *Example*：  
  假设有如下矩阵:  
  ```
    1   0   4
    0   3   5
    2   0   6
  ```
  使用csc格式构造上述矩阵：  
  ```
  M = csc_matrix(3, 3, 6, Array(1,2,3,4,5,6), 
                    Array(0,2,1,0,1,2), Array(0,2,3,6))
  ```
  其中3个Array：  
  第一个由非零元素的值组成；  
  第二个代表非零元素所在行的索引号；  
  第三个的首元素固定为0，第二个元素是第一列非零元素的个数，第三个元素是前两列非零元素的个数...
  ```
  Array（1,2,3,4,5,6）
  Array（0,2,1,0,1,2）
  Array（0,2,3,6）
  ```
7. 调用osqp库求解，返回结果
### 3.3 路径生成
1. 调用ToPiecewiseJerkPath（）函数进行SL坐标系下的离散路径点生成  
Input:  
   * 最优路径（l, dl, ddl)
    * 离散边界的步长
    * 边界的起始位置
```
      auto frenet_frame_path =
          ToPiecewiseJerkPath(opt_l, opt_dl, opt_ddl, path_boundary.delta_s(),
                              path_boundary.start_s());
```
2. 取出第一个路径点，调用AppendSegment（）函数生成后续路径点
```
  PiecewiseJerkTrajectory1d piecewise_jerk_traj(x.front(), dx.front(),
                                                ddx.front());

  for (std::size_t i = 1; i < x.size(); ++i) {
    const auto dddl = (ddx[i] - ddx[i - 1]) / delta_s;
    piecewise_jerk_traj.AppendSegment(dddl, delta_s);
  }
```
3. AppendSegment（）函数中将离散路径点起点坐标输入segments_中：
````
void PiecewiseJerkTrajectory1d::AppendSegment(const double jerk,
                                              const double param) {
  CHECK_GT(param, FLAGS_numerical_epsilon);

  param_.push_back(param_.back() + param);

  segments_.emplace_back(last_p_, last_v_, last_a_, jerk, param);

  last_p_ = segments_.back().end_position();

  last_v_ = segments_.back().end_velocity();

  last_a_ = segments_.back().end_acceleration();
}
````
4. 每个segments_初始化时会根据输入的路径坐标调用Evaluate（）函数计算下一个路径点坐标；  
   计算方式是通过3次曲线方程拟合；  
   最终获得离散步长为delta_s（与离散边界步长一致）的离散路径点集
```
ConstantJerkTrajectory1d::ConstantJerkTrajectory1d(const double p0,
                                                   const double v0,
                                                   const double a0,
                                                   const double j,
                                                   const double param)
    : p0_(p0), v0_(v0), a0_(a0), param_(param), jerk_(j) {
  CHECK_GT(param, FLAGS_numerical_epsilon);
  p1_ = Evaluate(0, param_);
  v1_ = Evaluate(1, param_);
  a1_ = Evaluate(2, param_);
}

double ConstantJerkTrajectory1d::Evaluate(const std::uint32_t order,
                                          const double param) const {
  switch (order) {
    case 0: {
      return p0_ + v0_ * param + 0.5 * a0_ * param * param +
             jerk_ * param * param * param / 6.0;
    }
    case 1: {
      return v0_ + a0_ * param + 0.5 * jerk_ * param * param;
    }
    case 2: {
      return a0_ + jerk_ * param;
    }
    case 3: {
      return jerk_;
    }
    default:
      return 0.0;
  }
```
5. 根据预设的离散步长计算最终需要输出的path point
````mermaid
  std::vector<common::FrenetFramePoint> frenet_frame_path;
  double accumulated_s = 0.0;
  while (accumulated_s < piecewise_jerk_traj.ParamLength()) {
    double l = piecewise_jerk_traj.Evaluate(0, accumulated_s);
    double dl = piecewise_jerk_traj.Evaluate(1, accumulated_s);
    double ddl = piecewise_jerk_traj.Evaluate(2, accumulated_s);

    common::FrenetFramePoint frenet_frame_point;
    frenet_frame_point.set_s(accumulated_s + start_s);
    frenet_frame_point.set_l(l);
    frenet_frame_point.set_dl(dl);
    frenet_frame_point.set_ddl(ddl);
    frenet_frame_path.push_back(std::move(frenet_frame_point));

    accumulated_s += FLAGS_trajectory_space_resolution;
  }
````
## Change Log
>*2021.05.20    fanyizhi*
> >add readme file