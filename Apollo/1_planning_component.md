规划模块继承cyber::Component类。需重写Init()和Proc()方法。

1、Init()方法

```c++
bool PlanningComponent::Init() {
  injector_ = std::make_shared<DependencyInjector>();
// 通过FLAGS_use_navigation_mode决定启用哪个planner
  if (FLAGS_use_navigation_mode) {
    planning_base_ = std::make_unique<NaviPlanning>(injector_);
  } else {
    planning_base_ = std::make_unique<OnLanePlanning>(injector_);
  }

  //....
    
  // PlanningBase类的Init()方法
  planning_base_->Init(config_);
    
  /*
   *                      <订阅话题topic>
   * 1、node_在/apollo/cyber/component/component_base.h中定义，是Node类的对象
   * 方法CreateReader<>()创建接收（订阅）节点，该方法被写成模板
   * 规划模块分别接收Routing路由、PlanningLearningData、PadMessage、Stories，
   * 在navi规划中还额外有relative map
   * 2、config_在/apollo/modules/planning/planning_component.h中定义，是
   * PlanningConfig类的对象。PlanningConfig类有proto文件生产，其定义在/apollo/modules
   * /planning/proto/planning_config.proto中
   */
  routing_reader_ = node_->CreateReader<RoutingResponse>(
      config_.topic_config().routing_response_topic(),
      [this](const std::shared_ptr<RoutingResponse>& routing) {
        AINFO << "Received routing data: run routing callback."
              << routing->header().DebugString();
        std::lock_guard<std::mutex> lock(mutex_);
        routing_.CopyFrom(*routing);
      });

  //...
  
  /*
   *                      发布话题topic
   * 方法CreateWriter<>()创建发送（发布）节点，该方法被写成模板
   * 规划模块分别发布ADCTrajectory轨迹、RoutingRequest路由请求、PlanningLearningData
   */
  planning_writer_ = node_->CreateWriter<ADCTrajectory>(
      config_.topic_config().planning_trajectory_topic());

  //...

  return true;
}
```

2、Proc()方法

```c++
/*
 * 输入障碍物的预测轨迹、车辆反馈的状态信息、车辆的定位信息
 */
bool PlanningComponent::Proc(
    const std::shared_ptr<prediction::PredictionObstacles>&
        prediction_obstacles,
    const std::shared_ptr<canbus::Chassis>& chassis,
    const std::shared_ptr<localization::LocalizationEstimate>&
        localization_estimate) {
  ACHECK(prediction_obstacles != nullptr);

  // 检查并处理可能的重路由请求
  // 当车辆在原来导航的路径上无法继续前进时，请求重新路由。比如，当前前方发生车祸
  CheckRerouting();

/*
 * 处理融合输入数据
 * local_view_是结构体LocalView类型，在/apollo/modules/planning/
 * common/local_view.h中定义
 */
  // 从输入获取
  local_view_.prediction_obstacles = prediction_obstacles;
  local_view_.chassis = chassis;
  local_view_.localization_estimate = localization_estimate;
    
  //共享数据，通过多线程的锁机制获取
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!local_view_.routing ||
        hdmap::PncMap::IsNewRouting(*local_view_.routing, routing_)) {
      local_view_.routing =
          std::make_shared<routing::RoutingResponse>(routing_);
    }
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.traffic_light =
        std::make_shared<TrafficLightDetection>(traffic_light_);
    local_view_.relative_map = std::make_shared<MapMsg>(relative_map_);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.pad_msg = std::make_shared<PadMessage>(pad_msg_);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.stories = std::make_shared<Stories>(stories_);
  }

  if (!CheckInput()) {
    AERROR << "Input check failed";
    return false;
  }
    
  // data process for online training
  //...

  // publish learning data frame for RL test
  //...

  /*
   * 生成轨迹
   * ADCTrajectory定义在/apollo/modules/planning/proto/planning.proto
   */
  ADCTrajectory adc_trajectory_pb;
  //**重要！重要！重要！规划模块的主逻辑，由预测数据触发，旧版由定时器触发，目前触发周期为100s，即10Hz**//
  planning_base_->RunOnce(local_view_, &adc_trajectory_pb);
  /*
   * 填充时间
   * FillHeader定义在//apollo/modules/common/util/message_util.h
   * 使用::apollo::cyber::Clock::NowInSeconds()填充当前时间
   * 并令sequence_num加1
   */
  common::util::FillHeader(node_->Name(), &adc_trajectory_pb);

  /*
   * adc_trajectory_pb.header().timestamp_sec()是adc_trajectory_pb这个message
   * 发布的时间，单位s。header是Header类型，定义在/apollo/modules/common/proto/header.proto
   * 
   * adc_trajectory_pb.mutable_trajectory_point()就是ADCTrajectory中的trajectory_point，
   * 即路径数据和速度数据。在临时文件planning.pb.h中被替换为mutable_trajectory_point
   *
   * TrajectoryPoint类定义在/apollo/modules/common/proto/pnc_point.proto
   * p.set_relative_time()指相对于 轨迹开始 的时间relative time from beginning of the 
   * trajectory
   */
  // 由于header中的时间戳变化了，所以要修改轨迹的相对时间
  auto start_time = adc_trajectory_pb.header().timestamp_sec();
  const double dt = start_time - adc_trajectory_pb.header().timestamp_sec();
  for (auto& p : *adc_trajectory_pb.mutable_trajectory_point()) {
    p.set_relative_time(p.relative_time() + dt);
  }
  // 发布轨迹
  planning_writer_->Write(adc_trajectory_pb);

  // record in history
  auto* history = injector_->history();
  history->Add(adc_trajectory_pb);

  return true;
}
```

