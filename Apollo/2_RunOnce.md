RunOnce()是定义在PlanningBase类中的虚函数，需要具体的规划器子类来重写。

车辆状态更新正常--InitFrame()--计算到障碍物的净空距离--判断交通规则--Plan()

```c++
void OnLanePlanning::RunOnce(const LocalView& local_view,
                             ADCTrajectory* const ptr_trajectory_pb) {
  // 重新路由时，参考行可能无法更新。在这种情况下，规划模块在重新启动之前保持未就绪状态。
  static bool failed_to_update_reference_line = false;
  local_view_ = local_view;
  const double start_timestamp = Clock::NowInSeconds();
  const double start_system_timestamp =
      std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();

  // localization
  ADEBUG << "Get localization:"
         << local_view_.localization_estimate->DebugString();

  // chassis
  ADEBUG << "Get chassis:" << local_view_.chassis->DebugString();

  Status status = injector_->vehicle_state()->Update(
      *local_view_.localization_estimate, *local_view_.chassis);

  VehicleState vehicle_state = injector_->vehicle_state()->vehicle_state();
  const double vehicle_state_timestamp = vehicle_state.timestamp();
  DCHECK_GE(start_timestamp, vehicle_state_timestamp)
      << "start_timestamp is behind vehicle_state_timestamp by "
      << start_timestamp - vehicle_state_timestamp << " secs";

  if (!status.ok() || !util::IsVehicleStateValid(vehicle_state)) {
    const std::string msg =
        "Update VehicleStateProvider failed "
        "or the vehicle state is out dated.";
    AERROR << msg;
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_not_ready()
        ->set_reason(msg);
    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    // TODO(all): integrate reverse gear
    ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    GenerateStopTrajectory(ptr_trajectory_pb);
    return;
  }
  // 对齐时间戳
  if (start_timestamp - vehicle_state_timestamp <
      FLAGS_message_latency_threshold) {
    vehicle_state = AlignTimeStamp(vehicle_state, start_timestamp);
  }

  if (util::IsDifferentRouting(last_routing_, *local_view_.routing)) {
    last_routing_ = *local_view_.routing;
    ADEBUG << "last_routing_:" << last_routing_.ShortDebugString();
    injector_->history()->Clear();
    injector_->planning_context()->mutable_planning_status()->Clear();
    reference_line_provider_->UpdateRoutingResponse(*local_view_.routing);
    planner_->Init(config_);
  }

  failed_to_update_reference_line =
      (!reference_line_provider_->UpdatedReferenceLine());

  // 重路由后参考线更新失败时提前返回
  if (failed_to_update_reference_line) {
    const std::string msg = "Failed to update reference line after rerouting.";
    AERROR << msg;
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_not_ready()
        ->set_reason(msg);
    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    GenerateStopTrajectory(ptr_trajectory_pb);
    return;
  }

  // 如有必要，更新 参考线提供程序并靠边停车重新设置
  reference_line_provider_->UpdateVehicleState(vehicle_state);

  // 规划是由预测数据触发的，但我们仍然可以使用估计的周期时间进行缝合
  const double planning_cycle_time =
      1.0 / static_cast<double>(FLAGS_planning_loop_rate);

  // 缝合轨迹，Plan方法的输入参数
  std::string replan_reason;
  std::vector<TrajectoryPoint> stitching_trajectory =
      TrajectoryStitcher::ComputeStitchingTrajectory(
          vehicle_state, start_timestamp, planning_cycle_time,
          FLAGS_trajectory_stitching_preserved_length, true,
          last_publishable_trajectory_.get(), &replan_reason);

  injector_->ego_info()->Update(stitching_trajectory.back(), vehicle_state);
  const uint32_t frame_num = static_cast<uint32_t>(seq_num_++);
  status = InitFrame(frame_num, stitching_trajectory.back(), vehicle_state);

  // 如果InitFrame成功，则计算到障碍物的净空距离
  if (status.ok()) {
    injector_->ego_info()->CalculateFrontObstacleClearDistance(
        frame_->obstacles());
  }

  if (FLAGS_enable_record_debug) {
    frame_->RecordInputDebug(ptr_trajectory_pb->mutable_debug());
  }
  ptr_trajectory_pb->mutable_latency_stats()->set_init_frame_time_ms(
      Clock::NowInSeconds() - start_timestamp);
  
  // 如果InitFrame不成功
  if (!status.ok()) {
    AERROR << status.ToString();
    if (FLAGS_publish_estop) { // 如果发布estop
      // "estop" signal check in function "Control::ProduceControlCommand()"
      // estop_ = estop_ || local_view_.trajectory.estop().is_estop();
      // 我们应该增加更多的信息，以确保ESTOP被触发。
      ADCTrajectory estop_trajectory;
      EStop* estop = estop_trajectory.mutable_estop();
      estop->set_is_estop(true);
      estop->set_reason(status.error_message());
      status.Save(estop_trajectory.mutable_header()->mutable_status());
      ptr_trajectory_pb->CopyFrom(estop_trajectory);
    } else { // 如果不发布estop
      ptr_trajectory_pb->mutable_decision()
          ->mutable_main_decision()
          ->mutable_not_ready()
          ->set_reason(status.ToString());
      status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
      GenerateStopTrajectory(ptr_trajectory_pb);
    }
    // TODO(all): 集成倒档
    ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    frame_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
    const uint32_t n = frame_->SequenceNum();
    injector_->frame_history()->Add(n, std::move(frame_));
    return;
  }
  
  // 至此，InitFrame成功，并已计算得到障碍物的净空距离
  // （接下来进行交通规则判断，如果可行则继续规划，否则停车。比如红灯？还没分析这里）
  for (auto& ref_line_info : *frame_->mutable_reference_line_info()) {
    TrafficDecider traffic_decider;
    traffic_decider.Init(traffic_rule_configs_);
    auto traffic_status =
        traffic_decider.Execute(frame_.get(), &ref_line_info, injector_);
    if (!traffic_status.ok() || !ref_line_info.IsDrivable()) {
      ref_line_info.SetDrivable(false);
      AWARN << "Reference line " << ref_line_info.Lanes().Id()
            << " traffic decider failed";
    }
  }
  
  /*
   * Plan()，重要！重要！重要！
   */
  status = Plan(start_timestamp, stitching_trajectory, ptr_trajectory_pb);
  
  // 输出调试轨迹点
  for (const auto& p : ptr_trajectory_pb->trajectory_point()) {
    ADEBUG << p.DebugString();
  }
  // 获取结束时间点
  const auto end_system_timestamp =
      std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  const auto time_diff_ms =
      (end_system_timestamp - start_system_timestamp) * 1000;
  ADEBUG << "total planning time spend: " << time_diff_ms << " ms.";

  ptr_trajectory_pb->mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  ADEBUG << "Planning latency: "
         << ptr_trajectory_pb->latency_stats().DebugString();
  
  // 如果规划不成功，则输出错误信息，并设置estop
  if (!status.ok()) {
    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
    AERROR << "Planning failed:" << status.ToString();
    if (FLAGS_publish_estop) {
      AERROR << "Planning failed and set estop";
      // "estop" signal check in function "Control::ProduceControlCommand()"
      // estop_ = estop_ || local_view_.trajectory.estop().is_estop();
      // we should add more information to ensure the estop being triggered.
      EStop* estop = ptr_trajectory_pb->mutable_estop();
      estop->set_is_estop(true);
      estop->set_reason(status.error_message());
    }
  }
 
  // 如果规划成功
  // 判断是否需要重新规划，未深入分析
  ptr_trajectory_pb->set_is_replan(stitching_trajectory.size() == 1);
  if (ptr_trajectory_pb->is_replan()) {
    ptr_trajectory_pb->set_replan_reason(replan_reason);
  }

  if (frame_->open_space_info().is_on_open_space_trajectory()) {
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    ADEBUG << "Planning pb:" << ptr_trajectory_pb->header().DebugString();
    frame_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
  } else {
    auto* ref_line_task =
        ptr_trajectory_pb->mutable_latency_stats()->add_task_stats();
    ref_line_task->set_time_ms(reference_line_provider_->LastTimeDelay() *
                               1000.0);
    ref_line_task->set_name("ReferenceLineProvider");
    // TODO(all): integrate reverse gear
    ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
    ADEBUG << "Planning pb:" << ptr_trajectory_pb->header().DebugString();

    frame_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
    if (FLAGS_enable_planning_smoother) {
      planning_smoother_.Smooth(injector_->frame_history(), frame_.get(),
                                ptr_trajectory_pb);
    }
  }

  const uint32_t n = frame_->SequenceNum();
  injector_->frame_history()->Add(n, std::move(frame_));
}
```