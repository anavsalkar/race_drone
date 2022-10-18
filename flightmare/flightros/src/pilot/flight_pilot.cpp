#include "flightros/pilot/flight_pilot.hpp"

namespace flightros {

FlightPilot::FlightPilot(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : nh_(nh),
    pnh_(pnh),
    scene_id_(UnityScene::WAREHOUSE),
    unity_ready_(false),
    unity_render_(false),
    receive_id_(0),
    main_loop_freq_(50.0) {
  // load parameters
  if (!loadParams()) {
    ROS_WARN("[%s] Could not load all parameters.",
             pnh_.getNamespace().c_str());
  } else {
    ROS_INFO("[%s] Loaded all parameters.", pnh_.getNamespace().c_str());
  }

  // quad initialization
  quad_ptr_ = std::make_shared<Quadrotor>();

  // add mono camera
  rgb_camera_ = std::make_shared<RGBCamera>();
  Vector<3> B_r_BC(0.0, 0.0, 0.3);
  Matrix<3, 3> R_BC = Quaternion(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  std::cout << R_BC << std::endl;
  rgb_camera_->setFOV(90);
  rgb_camera_->setWidth(720);
  rgb_camera_->setHeight(480);
  rgb_camera_->setRelPose(B_r_BC, R_BC);
  quad_ptr_->addRGBCamera(rgb_camera_);

  // initialization
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);



  Vector<3> gate_size(0.5,0.5,0.5);
  quad_ptr_->setSize(gate_size);

  // adding custom static gate
  std::string object_id = "unity_gate";
  std::string prefab_id = "rpg_gate";
  std::shared_ptr<StaticObject> gate_1 =
    std::make_shared<StaticObject>(object_id, prefab_id);
  gate_1->setPosition(Eigen::Vector3f(-1, -1.25, 4));
  gate_1->setQuaternion(
    Quaternion(0.707, 0.0, 0.0, 0.707));
  gate_1->setSize(gate_size);

  std::string object_id_2 = "unity_gate_2";
  std::shared_ptr<StaticGate> gate_2 =
    std::make_shared<StaticGate>(object_id_2);
  gate_2->setPosition(Eigen::Vector3f(9, 6.25, 2));
  gate_2->setQuaternion(
    Quaternion(0.819, 0.0, 0.0, 0.574));
  gate_2->setSize(gate_size);

  std::string object_id_3 = "unity_gate_3";
  std::shared_ptr<StaticGate> gate_3 =
    std::make_shared<StaticGate>(object_id_3);
  gate_3->setPosition(Eigen::Vector3f(9, -3.5, 2));
  gate_3->setQuaternion(
    Quaternion(0.924, 0.0, 0.0, -0.383));
  gate_3->setSize(gate_size);

  std::string object_id_4 = "unity_gate_4";
  std::shared_ptr<StaticGate> gate_4 =
    std::make_shared<StaticGate>(object_id_4);
  gate_4->setPosition(Eigen::Vector3f(-3.5, -6, 4));
  gate_4->setQuaternion(
    Quaternion(0.707, 0.0, 0.0, -0.707));
  gate_4->setSize(gate_size);

  std::string object_id_5 = "unity_gate_5";
  std::shared_ptr<StaticGate> gate_5 =
    std::make_shared<StaticGate>(object_id_5);
  gate_5->setPosition(Eigen::Vector3f(-3.5, -6, 2));
  gate_5->setQuaternion(
    Quaternion(0.707, 0.0, 0.0, 0.707));
  gate_5->setSize(gate_size);

  std::string object_id_6 = "unity_gate_6";
  std::shared_ptr<StaticGate> gate_6 =
    std::make_shared<StaticGate>(object_id_6);
  gate_6->setPosition(Eigen::Vector3f(4, -1, 2));
  gate_6->setQuaternion(
    Quaternion(0.985, 0.0, 0.0, -0.174));
  gate_6->setSize(gate_size);

  std::string object_id_7 = "unity_gate_7";
  std::shared_ptr<StaticGate> gate_7 =
    std::make_shared<StaticGate>(object_id_7);
  gate_7->setPosition(Eigen::Vector3f(-2.5, 6.5, 4));
  gate_7->setQuaternion(
    Quaternion(0.819, 0.0, 0.0, -0.574));
  gate_7->setSize(gate_size);


  // std::string object_id_3 = "moving_gate";
  // std::shared_ptr<StaticGate> gate_3 =
  //   std::make_shared<StaticGate>(object_id_3);
  // gate_3->setPosition(Eigen::Vector3f(5, 0, 2.5));
  // gate_3->setQuaternion(Quaternion(0.0, 0.0, 0.0, 1.0));






  // initialize subscriber call backs
  sub_state_est_ = nh_.subscribe("flight_pilot/state_estimate", 1,
                                 &FlightPilot::poseCallback, this);

  timer_main_loop_ = nh_.createTimer(ros::Rate(main_loop_freq_),
                                     &FlightPilot::mainLoopCallback, this);


  // wait until the gazebo and unity are loaded
  ros::Duration(5.0).sleep();

  // connect unity
  setUnity(unity_render_);

  /// added gates
  unity_bridge_ptr_->addStaticObject(gate_1);
  unity_bridge_ptr_->addStaticObject(gate_2);
  unity_bridge_ptr_->addStaticObject(gate_3);
  unity_bridge_ptr_->addStaticObject(gate_4);
  unity_bridge_ptr_->addStaticObject(gate_5);
  unity_bridge_ptr_->addStaticObject(gate_6);
  unity_bridge_ptr_->addStaticObject(gate_7);

  connectUnity();
}

FlightPilot::~FlightPilot() {}

void FlightPilot::poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  quad_state_.x[QS::POSX] = (Scalar)msg->pose.pose.position.x;
  quad_state_.x[QS::POSY] = (Scalar)msg->pose.pose.position.y;
  quad_state_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  quad_state_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  quad_state_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.x;
  quad_state_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.y;
  quad_state_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;
  //
  quad_ptr_->setState(quad_state_);

  if (unity_render_ && unity_ready_) {
    unity_bridge_ptr_->getRender(0);
    unity_bridge_ptr_->handleOutput();

    if (quad_ptr_->getCollision()) {
      // collision happened
      // ROS_INFO("COLLISION");
      ;
    }
  }
}

void FlightPilot::mainLoopCallback(const ros::TimerEvent &event) {
  // empty
}

bool FlightPilot::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    ROS_INFO("[%s] Unity Bridge is created.", pnh_.getNamespace().c_str());
  }
  return true;
}

bool FlightPilot::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

bool FlightPilot::loadParams(void) {
  // load parameters
  quadrotor_common::getParam("main_loop_freq", main_loop_freq_, pnh_);
  quadrotor_common::getParam("unity_render", unity_render_, pnh_);

  return true;
}

}  // namespace flightros