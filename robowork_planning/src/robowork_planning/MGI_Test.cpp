#include <robowork_planning/MGI_Test.h>

namespace move_group_interface {

void MGI_Test::initialize()
{
  std::string robot_namespace;
  private_nh_.getParam("robot_namespace", robot_namespace);
  std::string arm_namespace;
  private_nh_.getParam("arm_namespace", arm_namespace);

  tf_listener_ = std::make_unique<tf::TransformListener>();

  // Setup Visualization
  visual_tools_ = std::make_unique<moveit_visual_tools::MoveItVisualTools>(robot_namespace + "/bvr_base_link");
  visual_tools_->deleteAllMarkers();
  //visual_tools->loadRemoteControl();
  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools_->trigger();

  // Setup Interface
  PLANNING_GROUP_ = arm_namespace;  //Assumes arm_namespace:=moveit planning group 
  move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP_);
  planning_scene_interface_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
  joint_model_group_ = move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP_);

  ROS_INFO_NAMED(move_group::NODE_NAME, "Robot Model name: [%s] at frame: [%s]", move_group_->getRobotModel()->getName().c_str(), move_group_->getRobotModel()->getModelFrame().c_str());
  ROS_INFO_NAMED(move_group::NODE_NAME, "ROBOT MODEL-based / SRDF-based Pose Reference frame: [%s]", move_group_->getPoseReferenceFrame().c_str());
  ROS_INFO_NAMED(move_group::NODE_NAME, "ROBOT MODEL-based / SRDF-based Planning frame: [%s]", move_group_->getPlanningFrame().c_str());
  ROS_INFO_NAMED(move_group::NODE_NAME, "SRDF-based End-Effector link: [%s]", move_group_->getEndEffectorLink().c_str());
  ROS_INFO_NAMED(move_group::NODE_NAME, "Available Planning Groups:");
  const std::vector<std::string>& joint_model_group_names = move_group_->getJointModelGroupNames();
  std::copy(joint_model_group_names.begin(), joint_model_group_names.end(), std::ostream_iterator<std::string>(std::cout, " "));

  plan_link_ = move_group_->getPlanningFrame();
  ee_link_ = robot_namespace + "/" + arm_namespace + "/gripper_manipulation_link";

  // Necessary if planning is to take place in frame other than move_group_->getPlanningFrame() - a.k.a. base_link
  //move_group_->setPoseReferenceFrame(robot_namespace + "/bvr_base_link");
  //visual_tools_->setLifetime(30.0);
  // Reduce the speed of the robot arm via a scaling factor of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
  move_group_->setMaxVelocityScalingFactor(0.25);
  // Set a scaling factor for optionally reducing the maximum joint acceleration.
  move_group_->setMaxAccelerationScalingFactor(1.0);
  // Planning with constraints can be slow because every sample must call an inverse kinematics solver (default 5 seconds)
  move_group_->setPlanningTime(5.0); //5.0
  // Number of times the motion plan is to be computed from scratch before the shortest solution is returned. 
  move_group_->setNumPlanningAttempts(100); //10	
  // Set the tolerance that is used for reaching the goal. For joint state goals, this will be distance for each joint, in the configuration space (radians or meters depending on joint type). For pose goals this will be the radius of a sphere where the end-effector must reach.
  move_group_->setGoalTolerance(0.02);
  // Pick one of the available configs - see ee ompl_planning<_SIM>.yaml for a complete list
  move_group_->setPlannerId("RRTConnectkConfigDefault");

  ROS_INFO_STREAM_NAMED(move_group::NODE_NAME, "Available Planning Params for " << move_group_->getPlannerId() << ":");
  std::map<std::string,std::string> planner_params = move_group_->getPlannerParams(move_group_->getPlannerId(), PLANNING_GROUP_);
  for (const auto & param : planner_params) {  std::cout << " - " << param.first << " : " << param.second << std::endl; }
 
  //visual_tools_->prompt("Press 'next' to start");

  // Setup Callbacks
  rc_sub_ = nh_.subscribe("/rviz_visual_tools_gui", 10, &MGI_Test::rc_cb, this);
  endeffector_goal_position_sub_ = nh_.subscribe("/endeffector_goal_position", 10, &MGI_Test::endeffector_goal_position_cb, this);
  endeffector_goal_pose_sub_ = nh_.subscribe("/endeffector_goal_pose", 10, &MGI_Test::endeffector_goal_pose_cb, this);

  ROS_INFO_STREAM_NAMED(move_group::NODE_NAME, "Setup complete!");
}

void MGI_Test::rc_cb(const sensor_msgs::Joy::ConstPtr& msg)
{
  if (msg->buttons.at(1) != 0) {  //'Next'
    plan();
  }
  else if (msg->buttons.at(2) != 0) {  //'Continue'
  }
  else if (msg->buttons.at(3) != 0) {  //'Break'
    if (move_group_) {
      move_group_->stop();
    }
    else {
      ROS_WARN_STREAM("move_group_interface: move_group_ not constructed yet"); 
      throw std::runtime_error("move_group_interface: move_group_ not constructed yet");
    }
  }
  else if (msg->buttons.at(4) != 0) {  //'Stop'
    if (move_group_) {
      move_group_->stop();
    }
    else {
      ROS_WARN_STREAM("move_group_interface: move_group_ not constructed yet"); 
      throw std::runtime_error("move_group_interface: move_group_ not constructed yet");
    }
  }
}

void MGI_Test::endeffector_goal_position_cb(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  if (move_group_) {
    move_group_->stop();
  }
  else {
    ROS_WARN_STREAM("move_group_interface: move_group_ not constructed yet"); 
    throw std::runtime_error("move_group_interface: move_group_ not constructed yet");
  }

  ROS_INFO_STREAM("move_group_interface: New PointStamped goal in frame [" << msg->header.frame_id << "] received!");
  geometry_msgs::PointStamped target_point_received = *msg;

  // Ensure that end-effector reference is in world frame 
  tf::Stamped<tf::Point> t_EE_ref_W;
  tf::pointStampedMsgToTF(target_point_received, t_EE_ref_W);
  T_EE_ref_W_ = tf::Stamped<tf::Pose>(tf::Pose(/*tf::Quaternion(0,0,0,1)*/T_EE_W_.getRotation(), static_cast<tf::Point>(t_EE_ref_W)), t_EE_ref_W.stamp_, t_EE_ref_W.frame_id_);
  if (T_EE_ref_W_.frame_id_ != world_link_) {
    ROS_WARN_STREAM("move_group_interface: Goal in frame [" << T_EE_ref_W_.frame_id_ << "], have to transform into [" << world_link_ << "]"); 
    //TODO
    throw std::runtime_error("move_group_interface: Wrong frame in received msg");
  }

  // Execute planning
  //plan();
}

void MGI_Test::endeffector_goal_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if (move_group_) {
    move_group_->stop();
  }
  else {
    ROS_WARN_STREAM("move_group_interface: move_group_ not constructed yet"); 
    throw std::runtime_error("move_group_interface: move_group_ not constructed yet");
  }

  ROS_INFO_STREAM("move_group_interface: New PoseStamped goal in frame [" << msg->header.frame_id << "] received!");
  geometry_msgs::PoseStamped target_pose_received = *msg;

  // Ensure that end-effector reference is in world frame 
  tf::Stamped<tf::Pose> t_EE_ref_W;
  tf::poseStampedMsgToTF(target_pose_received, t_EE_ref_W);
  T_EE_ref_W_ = tf::Stamped<tf::Pose>(static_cast<tf::Pose>(t_EE_ref_W), t_EE_ref_W.stamp_, t_EE_ref_W.frame_id_);
  if (T_EE_ref_W_.frame_id_ != world_link_) {
    ROS_WARN_STREAM("move_group_interface: Goal in frame [" << T_EE_ref_W_.frame_id_ << "], have to transform into [" << world_link_ << "]"); 
    //TODO
    throw std::runtime_error("move_group_interface: Wrong frame in received msg");
  }

  // Execute planning
  //plan();
}

void MGI_Test::plan() {
    if (T_EE_ref_W_.stamp_ == ros::Time(0)) {
      ROS_WARN_STREAM("move_group_interface: plan() invoked with invalid T_EE_ref_W_.stamp_, aborting..."); 
      return;
    }

    ros::Time time_plan;
    tf::StampedTransform T_P_W;
    tf::StampedTransform T_EE_P;
    try{      
      ros::Time now = ros::Time::now();
      tf_listener_->waitForTransform(world_link_, plan_link_, now, ros::Duration(0.2));
      tf_listener_->lookupTransform(world_link_, plan_link_, now, T_P_W);
      time_plan = T_P_W.stamp_;
      tf_listener_->waitForTransform(plan_link_, ee_link_, time_plan, ros::Duration(0.2));
      tf_listener_->lookupTransform(plan_link_, ee_link_, time_plan, T_EE_P);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return;
    }

    // Update caches
    T_EE_W_ = tf::StampedTransform(T_EE_P * T_P_W, time_plan, world_link_, ee_link_);

    move_group_->clearPathConstraints();
    visual_tools_->deleteAllMarkers();

    robot_state::RobotState start_state(*move_group_->getCurrentState());
    const robot_state::LinkModel* ee_link_model = start_state.getLinkModel(ee_link_);

    // Transform into planning frame
    tf::Transform T_W_P = T_P_W.inverse();
    tf::Vector3 t_W_P = T_W_P.getOrigin();
    tf::Quaternion q_W_P = T_W_P.getRotation();

    tf::Vector3 t_EE_P = T_EE_P.getOrigin();
    tf::Quaternion q_EE_P = T_EE_P.getRotation();

    tf::Transform T_EE_ref_P = T_W_P * T_EE_ref_W_;
    tf::Vector3 t_EE_ref_P = T_EE_ref_P.getOrigin();
    tf::Quaternion q_EE_ref_P = T_EE_ref_P.getRotation();

    // Assign target pose
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = plan_link_;
    target_pose.header.stamp = time_plan;
    target_pose.pose.position.x = t_EE_ref_P.getX(); 
    target_pose.pose.position.y = t_EE_ref_P.getY(); 
    target_pose.pose.position.z = t_EE_ref_P.getZ(); 
    target_pose.pose.orientation.x = q_EE_ref_P.x(); 
    target_pose.pose.orientation.y = q_EE_ref_P.y(); 
    target_pose.pose.orientation.z = q_EE_ref_P.z(); 
    target_pose.pose.orientation.w = q_EE_ref_P.w(); 


    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = ee_link_;
    ocm.header.frame_id = plan_link_;
    ocm.orientation.x = q_EE_ref_P.x(); 
    ocm.orientation.y = q_EE_ref_P.y(); 
    ocm.orientation.z = q_EE_ref_P.z(); 
    ocm.orientation.w = q_EE_ref_P.w(); 
    ocm.absolute_x_axis_tolerance = 90.0 * 0.0174532925;  //rad
    ocm.absolute_y_axis_tolerance = 90.0 * 0.0174532925;  //rad
    ocm.absolute_z_axis_tolerance = 90.0 * 0.0174532925;  //rad
    ocm.weight = 1.0;  //this is the only constraint

    moveit_msgs::Constraints constraints;
    constraints.orientation_constraints.push_back(ocm);
    move_group_->setPathConstraints(constraints);


    //move_group_->setStartStateToCurrentState();
    geometry_msgs::Pose start_pose = move_group_->getCurrentPose().pose;
    start_state.setFromIK(joint_model_group_, start_pose);
    move_group_->setStartState(start_state);
  
    //move_group_->setOrientationTarget(target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w, ee_link_);
    move_group_->setPoseTarget(target_pose, ee_link_);
  
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan (constraints) %s", success ? "" : "FAILED");
  
    // Visualizing plans
    geometry_msgs::PoseStamped target_pose_VT;
    tf_listener_->transformPose(visual_tools_->getBaseFrame(), target_pose, target_pose_VT);
    visual_tools_->publishAxisLabeled(target_pose_VT.pose, "ee_ref_pose");
    visual_tools_->publishTrajectoryLine(my_plan.trajectory_, joint_model_group_);
    visual_tools_->publishTrajectoryPath(my_plan.trajectory_, move_group_->getCurrentState());
    visual_tools_->trigger();
   
      //move_group_->move(); 
      // see: https://github.com/ros-industrial/industrial_moveit/issues/37 and https://github.com/ros-industrial/industrial_moveit/issues/53
      move_group_->asyncExecute(my_plan);
}

}  // namespace move_group_interface
