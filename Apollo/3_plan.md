```c++
Status OnLanePlanning::Plan(
    const double current_time_stamp,
    const std::vector<TrajectoryPoint>& stitching_trajectory,
    ADCTrajectory* const ptr_trajectory_pb) {
  auto* ptr_debug = ptr_trajectory_pb->mutable_debug();
  if (FLAGS_enable_record_debug) {
    ptr_debug->mutable_planning_data()->mutable_init_point()->CopyFrom(
        stitching_trajectory.back());
    frame_->mutable_open_space_info()->set_debug(ptr_debug);
    frame_->mutable_open_space_info()->sync_debug_instance();
  }

  auto status = planner_->Plan(stitching_trajectory.back(), frame_.get(),
                               ptr_trajectory_pb);

  ptr_debug->mutable_planning_data()->set_front_clear_distance(
      injector_->ego_info()->front_clear_distance());

  if (frame_->open_space_info().is_on_open_space_trajectory()) {
    frame_->mutable_open_space_info()->sync_debug_instance();
    const auto& publishable_trajectory =
        frame_->open_space_info().publishable_trajectory_data().first;
    const auto& publishable_trajectory_gear =
        frame_->open_space_info().publishable_trajectory_data().second;
    publishable_trajectory.PopulateTrajectoryProtobuf(ptr_trajectory_pb);
    ptr_trajectory_pb->set_gear(publishable_trajectory_gear);

    // TODO(QiL): refine engage advice in open space trajectory optimizer.
    auto* engage_advice = ptr_trajectory_pb->mutable_engage_advice();

    // enable start auto from open_space planner.
    if (injector_->vehicle_state()->vehicle_state().driving_mode() !=
        Chassis::DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE) {
      engage_advice->set_advice(EngageAdvice::READY_TO_ENGAGE);
      engage_advice->set_reason(
          "Ready to engage when staring with OPEN_SPACE_PLANNER");
    } else {
      engage_advice->set_advice(EngageAdvice::KEEP_ENGAGED);
      engage_advice->set_reason("Keep engage while in parking");
    }
    // TODO(QiL): refine the export decision in open space info
    ptr_trajectory_pb->mutable_decision()
        ->mutable_main_decision()
        ->mutable_parking()
        ->set_status(MainParking::IN_PARKING);

    if (FLAGS_enable_record_debug) {
      // ptr_debug->MergeFrom(frame_->open_space_info().debug_instance());
      frame_->mutable_open_space_info()->RecordDebug(ptr_debug);
      ADEBUG << "Open space debug information added!";
      // call open space info load debug
      // TODO(Runxin): create a new flag to enable openspace chart
      ExportOpenSpaceChart(ptr_trajectory_pb->debug(), *ptr_trajectory_pb,
                           ptr_debug);
    }
  } else {
    const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();
    const auto* target_ref_info = frame_->FindTargetReferenceLineInfo();
    if (!best_ref_info) {
      const std::string msg = "planner failed to make a driving plan";
      AERROR << msg;
      if (last_publishable_trajectory_) {
        last_publishable_trajectory_->Clear();
      }
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    // Store current frame stitched path for possible speed fallback in next
    // frames
    DiscretizedPath current_frame_planned_path;
    for (const auto& trajectory_point : stitching_trajectory) {
      current_frame_planned_path.push_back(trajectory_point.path_point());
    }
    const auto& best_ref_path = best_ref_info->path_data().discretized_path();
    std::copy(best_ref_path.begin() + 1, best_ref_path.end(),
              std::back_inserter(current_frame_planned_path));
    frame_->set_current_frame_planned_path(current_frame_planned_path);

    ptr_debug->MergeFrom(best_ref_info->debug());
    if (FLAGS_export_chart) {
      ExportOnLaneChart(best_ref_info->debug(), ptr_debug);
    } else {
      ExportReferenceLineDebug(ptr_debug);
      // Export additional ST-chart for failed lane-change speed planning
      const auto* failed_ref_info = frame_->FindFailedReferenceLineInfo();
      if (failed_ref_info) {
        ExportFailedLaneChangeSTChart(failed_ref_info->debug(), ptr_debug);
      }
    }
    ptr_trajectory_pb->mutable_latency_stats()->MergeFrom(
        best_ref_info->latency_stats());
    // set right of way status
    ptr_trajectory_pb->set_right_of_way_status(
        best_ref_info->GetRightOfWayStatus());

    for (const auto& id : best_ref_info->TargetLaneId()) {
      ptr_trajectory_pb->add_lane_id()->CopyFrom(id);
    }

    for (const auto& id : target_ref_info->TargetLaneId()) {
      ptr_trajectory_pb->add_target_lane_id()->CopyFrom(id);
    }

    ptr_trajectory_pb->set_trajectory_type(best_ref_info->trajectory_type());

    if (FLAGS_enable_rss_info) {
      *ptr_trajectory_pb->mutable_rss_info() = best_ref_info->rss_info();
    }

    best_ref_info->ExportDecision(ptr_trajectory_pb->mutable_decision(),
                                  injector_->planning_context());

    // Add debug information.
    if (FLAGS_enable_record_debug) {
      auto* reference_line = ptr_debug->mutable_planning_data()->add_path();
      reference_line->set_name("planning_reference_line");
      const auto& reference_points =
          best_ref_info->reference_line().reference_points();
      double s = 0.0;
      double prev_x = 0.0;
      double prev_y = 0.0;
      bool empty_path = true;
      for (const auto& reference_point : reference_points) {
        auto* path_point = reference_line->add_path_point();
        path_point->set_x(reference_point.x());
        path_point->set_y(reference_point.y());
        path_point->set_theta(reference_point.heading());
        path_point->set_kappa(reference_point.kappa());
        path_point->set_dkappa(reference_point.dkappa());
        if (empty_path) {
          path_point->set_s(0.0);
          empty_path = false;
        } else {
          double dx = reference_point.x() - prev_x;
          double dy = reference_point.y() - prev_y;
          s += std::hypot(dx, dy);
          path_point->set_s(s);
        }
        prev_x = reference_point.x();
        prev_y = reference_point.y();
      }
    }

    last_publishable_trajectory_.reset(new PublishableTrajectory(
        current_time_stamp, best_ref_info->trajectory()));

    ADEBUG << "current_time_stamp: " << current_time_stamp;

    last_publishable_trajectory_->PrependTrajectoryPoints(
        std::vector<TrajectoryPoint>(stitching_trajectory.begin(),
                                     stitching_trajectory.end() - 1));

    last_publishable_trajectory_->PopulateTrajectoryProtobuf(ptr_trajectory_pb);

    best_ref_info->ExportEngageAdvice(
        ptr_trajectory_pb->mutable_engage_advice(),
        injector_->planning_context());
  }

  return status;
}
```

