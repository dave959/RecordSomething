```c++
Status LatticePlanner::PlanOnReferenceLine(
    const TrajectoryPoint& planning_init_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  static size_t num_planning_cycles = 0;
  static size_t num_planning_succeeded_cycles = 0;

  double start_time = Clock::NowInSeconds();
  double current_time = start_time;

  ADEBUG << "Number of planning cycles: " << num_planning_cycles << " "
         << num_planning_succeeded_cycles;
  ++num_planning_cycles;

  reference_line_info->set_is_on_reference_line();
  // 1. 获取参考线并将其转换为路径点格式
  auto ptr_reference_line =
      std::make_shared<std::vector<PathPoint>>(ToDiscretizedReferenceLine(
          reference_line_info->reference_line().reference_points()));

  // 2. 计算基准线上初始规划点的匹配点
  PathPoint matched_point = PathMatcher::MatchToPath(
      *ptr_reference_line, planning_init_point.path_point().x(),
      planning_init_point.path_point().y());

  // 3. 根据匹配点，计算Frenet帧的初始状态
  std::array<double, 3> init_s;
  std::array<double, 3> init_d;
  ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);

  ADEBUG << "ReferenceLine and Frenet Conversion Time = "
         << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();

  auto ptr_prediction_querier = std::make_shared<PredictionQuerier>(
      frame->obstacles(), ptr_reference_line);

  // 4. 解析决策，得到规划目标
  auto ptr_path_time_graph = std::make_shared<PathTimeGraph>(
      ptr_prediction_querier->GetObstacles(), *ptr_reference_line,
      reference_line_info, init_s[0],
      init_s[0] + FLAGS_speed_lon_decision_horizon, 0.0,
      FLAGS_trajectory_time_length, init_d);

  double speed_limit =
      reference_line_info->reference_line().GetSpeedLimitFromS(init_s[0]);
  reference_line_info->SetLatticeCruiseSpeed(speed_limit);

  PlanningTarget planning_target = reference_line_info->planning_target();
  if (planning_target.has_stop_point()) {
    ADEBUG << "Planning target stop s: " << planning_target.stop_point().s()
           << "Current ego s: " << init_s[0];
  }

  ADEBUG << "Decision_Time = "
         << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();

  // 5. 分别生成纵向和横向的一维轨迹束
  Trajectory1dGenerator trajectory1d_generator(
      init_s, init_d, ptr_path_time_graph, ptr_prediction_querier);
  std::vector<std::shared_ptr<Curve1d>> lon_trajectory1d_bundle;
  std::vector<std::shared_ptr<Curve1d>> lat_trajectory1d_bundle;
  trajectory1d_generator.GenerateTrajectoryBundles(
      planning_target, &lon_trajectory1d_bundle, &lat_trajectory1d_bundle);

  ADEBUG << "Trajectory_Generation_Time = "
         << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();

  // 6. 首先，根据动态约束条件对一维轨迹的可行性进行评估；
  //    其次，评估可行的纵向和横向轨迹对，并根据成本进行排序。
  TrajectoryEvaluator trajectory_evaluator(
      init_s, planning_target, lon_trajectory1d_bundle, lat_trajectory1d_bundle,
      ptr_path_time_graph, ptr_reference_line);

  ADEBUG << "Trajectory_Evaluator_Construction_Time = "
         << (Clock::NowInSeconds() - current_time) * 1000;
  current_time = Clock::NowInSeconds();

  ADEBUG << "number of trajectory pairs = "
         << trajectory_evaluator.num_of_trajectory_pairs()
         << "  number_lon_traj = " << lon_trajectory1d_bundle.size()
         << "  number_lat_traj = " << lat_trajectory1d_bundle.size();

  // 获取碰撞检查器和约束检查器的实例
  CollisionChecker collision_checker(frame->obstacles(), init_s[0], init_d[0],
                                     *ptr_reference_line, reference_line_info,
                                     ptr_path_time_graph);

  // 7. 总是得到最佳的轨迹组合；
  //返回第一个无碰撞轨迹。
  size_t constraint_failure_count = 0;
  size_t collision_failure_count = 0;
  size_t combined_constraint_failure_count = 0;

  size_t lon_vel_failure_count = 0;
  size_t lon_acc_failure_count = 0;
  size_t lon_jerk_failure_count = 0;
  size_t curvature_failure_count = 0;
  size_t lat_acc_failure_count = 0;
  size_t lat_jerk_failure_count = 0;

  size_t num_lattice_traj = 0;

  while (trajectory_evaluator.has_more_trajectory_pairs()) {
    double trajectory_pair_cost =
        trajectory_evaluator.top_trajectory_pair_cost();
    auto trajectory_pair = trajectory_evaluator.next_top_trajectory_pair();

    // 将两个一维轨迹合并为一个二维轨迹
    auto combined_trajectory = TrajectoryCombiner::Combine(
        *ptr_reference_line, *trajectory_pair.first, *trajectory_pair.second,
        planning_init_point.relative_time());

    // 检查纵向和横向加速度
    // 考虑轨道曲率
    auto result = ConstraintChecker::ValidTrajectory(combined_trajectory);
    if (result != ConstraintChecker::Result::VALID) {
      ++combined_constraint_failure_count;

      switch (result) {
        case ConstraintChecker::Result::LON_VELOCITY_OUT_OF_BOUND:
          lon_vel_failure_count += 1;
          break;
        case ConstraintChecker::Result::LON_ACCELERATION_OUT_OF_BOUND:
          lon_acc_failure_count += 1;
          break;
        case ConstraintChecker::Result::LON_JERK_OUT_OF_BOUND:
          lon_jerk_failure_count += 1;
          break;
        case ConstraintChecker::Result::CURVATURE_OUT_OF_BOUND:
          curvature_failure_count += 1;
          break;
        case ConstraintChecker::Result::LAT_ACCELERATION_OUT_OF_BOUND:
          lat_acc_failure_count += 1;
          break;
        case ConstraintChecker::Result::LAT_JERK_OUT_OF_BOUND:
          lat_jerk_failure_count += 1;
          break;
        case ConstraintChecker::Result::VALID:
        default:
          // 有意为空
          break;
      }
      continue;
    }

    // 检查与其他障碍物的碰撞
    if (collision_checker.InCollision(combined_trajectory)) {
      ++collision_failure_count;
      continue;
    }

    // 将合并轨迹放入调试数据
    const auto& combined_trajectory_points = combined_trajectory;
    num_lattice_traj += 1;
    reference_line_info->SetTrajectory(combined_trajectory);
    reference_line_info->SetCost(reference_line_info->PriorityCost() +
                                 trajectory_pair_cost);
    reference_line_info->SetDrivable(true);

    // 打印所选的结束条件和开始条件
    ADEBUG << "Starting Lon. State: s = " << init_s[0] << " ds = " << init_s[1]
           << " dds = " << init_s[2];
    // cast投影
    auto lattice_traj_ptr =
        std::dynamic_pointer_cast<LatticeTrajectory1d>(trajectory_pair.first);
    if (!lattice_traj_ptr) {
      ADEBUG << "Dynamically casting trajectory1d ptr. failed.";
    }

    if (lattice_traj_ptr->has_target_position()) {
      ADEBUG << "Ending Lon. State s = " << lattice_traj_ptr->target_position()
             << " ds = " << lattice_traj_ptr->target_velocity()
             << " t = " << lattice_traj_ptr->target_time();
    }

    ADEBUG << "InputPose";
    ADEBUG << "XY: " << planning_init_point.ShortDebugString();
    ADEBUG << "S: (" << init_s[0] << ", " << init_s[1] << "," << init_s[2]
           << ")";
    ADEBUG << "L: (" << init_d[0] << ", " << init_d[1] << "," << init_d[2]
           << ")";

    ADEBUG << "Reference_line_priority_cost = "
           << reference_line_info->PriorityCost();
    ADEBUG << "Total_Trajectory_Cost = " << trajectory_pair_cost;
    ADEBUG << "OutputTrajectory";
    for (uint i = 0; i < 10; ++i) {
      ADEBUG << combined_trajectory_points[i].ShortDebugString();
    }

    break;
    /*
    auto combined_trajectory_path =
        ptr_debug->mutable_planning_data()->add_trajectory_path();
    for (uint i = 0; i < combined_trajectory_points.size(); ++i) {
      combined_trajectory_path->add_trajectory_point()->CopyFrom(
          combined_trajectory_points[i]);
    }
    combined_trajectory_path->set_lattice_trajectory_cost(trajectory_pair_cost);
    */
  }

  ADEBUG << "Trajectory_Evaluation_Time = "
         << (Clock::NowInSeconds() - current_time) * 1000;

  ADEBUG << "Step CombineTrajectory Succeeded";

  ADEBUG << "1d trajectory not valid for constraint ["
         << constraint_failure_count << "] times";
  ADEBUG << "Combined trajectory not valid for ["
         << combined_constraint_failure_count << "] times";
  ADEBUG << "Trajectory not valid for collision [" << collision_failure_count
         << "] times";
  ADEBUG << "Total_Lattice_Planning_Frame_Time = "
         << (Clock::NowInSeconds() - start_time) * 1000;

  if (num_lattice_traj > 0) {
    ADEBUG << "Planning succeeded";
    num_planning_succeeded_cycles += 1;
    reference_line_info->SetDrivable(true);
    return Status::OK();
  } else {
    AERROR << "Planning failed";
    if (FLAGS_enable_backup_trajectory) {
      AERROR << "Use backup trajectory";
      BackupTrajectoryGenerator backup_trajectory_generator(
          init_s, init_d, planning_init_point.relative_time(),
          std::make_shared<CollisionChecker>(collision_checker),
          &trajectory1d_generator);
      DiscretizedTrajectory trajectory =
          backup_trajectory_generator.GenerateTrajectory(*ptr_reference_line);

      reference_line_info->AddCost(FLAGS_backup_trajectory_cost);
      reference_line_info->SetTrajectory(trajectory);
      reference_line_info->SetDrivable(true);
      return Status::OK();

    } else {
      reference_line_info->SetCost(std::numeric_limits<double>::infinity());
    }
    return Status(ErrorCode::PLANNING_ERROR, "No feasible trajectories");
  }
}

} 
```

