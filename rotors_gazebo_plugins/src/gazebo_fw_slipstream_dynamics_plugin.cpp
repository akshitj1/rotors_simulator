//
// Author: Akshit Jain
//

#include "rotors_gazebo_plugins/gazebo_fw_slipstream_dynamics_plugin.h"
#include "ConnectRosToGazeboTopic.pb.h"

typedef ignition::math::Vector3d Vector3;

namespace gazebo
{
GazeboFwSlipstreamDynamicsPlugin::GazeboFwSlipstreamDynamicsPlugin()
    : ModelPlugin(),
      node_handle_(0) {}

GazeboFwSlipstreamDynamicsPlugin::~GazeboFwSlipstreamDynamicsPlugin() {}

void GazeboFwSlipstreamDynamicsPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  if (kPrintOnPluginLoad)
    gzdbg << __FUNCTION__ << "() called." << std::endl;

  gzdbg << "_model = " << _model->GetName() << std::endl;

  model_ = _model;
  world_ = model_->GetWorld();

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "Please specify robotNamespace\n";

  node_handle_ = transport::NodePtr(new transport::Node());

  node_handle_->Init();
  std::string link_name;
  getSdfParam<std::string>(_sdf, "linkName", link_name, "", true);

  link_ = model_->GetLink(link_name);
  if (link_ == NULL)
    gzthrow("couldn't find specified link \"" << link_name << ".");

  getSdfParam<std::string>(_sdf, "actuatorsSubTopic",
                           actuators_sub_topic_,
                           mav_msgs::default_topics::COMMAND_ACTUATORS);
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboFwSlipstreamDynamicsPlugin::OnUpdate, this, _1));
}

void GazeboFwSlipstreamDynamicsPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // todo: why not create pubs and subs in Load?
  if (!pubs_and_subs_created_)
  {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  UpdateForcesAndMoments();
}

void GazeboFwSlipstreamDynamicsPlugin::UpdateForcesAndMoments()
{
  // propellers are spinning right -> CCW, left -> CW
  // right is in -x_B , left is in +x_B

  if (kPrintOnUpdates)
    gzdbg << __FUNCTION__ << "() called." << std::endl;

  // Express the air speed and angular velocity in the body frame.
  // B denotes body frame and W world frame ... e.g., W_rot_W_B denotes
  // rotation of B wrt. W expressed in W.
  ignition::math::Quaterniond W_rot_W_B = link_->WorldPose().Rot();
  // vel of B w.r.t W
  Vector3 vel_B_W = link_->WorldLinearVel();
  Vector3 angular_vel_B_W = link_->RelativeAngularVel();

  // eqn. 5
  double surface_speed = sqrt(pow(vel_B_W.Y(), 2) + pow(vel_B_W.Z(), 2));
  double attack_angle = atan2(vel_B_W.Y(), vel_B_W.Z());

  // forces
  // seperation from COM
  const double kPropDiameter = 0.13, kPropSeperation = 0.14;
  // torque constant is thrust to torque ratio
  const double kMotorTorqueConstant = 0.00872;
  // throttle_(left/right) in newtons so unit forceConstant
  const double kMotorForceConstant = 1;
  double f_prop_l = kMotorForceConstant * throttle_left;
  double f_prop_r = kMotorForceConstant * throttle_right;
  double f_prop_avg = (f_prop_l + f_prop_r) / 2;
  Vector3 f_prop = Vector3(0, 0, f_prop_l + f_prop_r);

  // pw75 airfoil
  // https://colab.research.google.com/drive/1sOBJVBV01y0fO1oZhXUMovB-0IZWwE8v
  const double kL1 = 3.5, kL2 = 0, kL3 = 0, kD1 = 3.0, kD2 = 0, kD3 = 0;
  const double kWingSpan=0.6, kWingChrod=0.2, kWingArea = 0.12;
  const double dynamic_pressure = 1/2*kAirDensity*kWingArea*pow(surface_speed, 2);
  double f_lift = dynamic_pressure * (kL1 * sin(attack_angle) * pow(cos(attack_angle), 2) + kL2 * pow(sin(attack_angle), 3))  + kL3 * f_prop_avg;
  double f_drag = dynamic_pressure * (kD1 * pow(sin(attack_angle), 2) * cos(attack_angle) + kD2 * cos(attack_angle)) + kD3 * f_prop_avg;
  Vector3 f_aero = Vector3(0, f_lift, -f_drag);

  Vector3 f_B = f_prop + f_aero;

  // moments
  const double kP1 = -0.3;
  double m_pitch = dynamic_pressure * kWingChrod * kP1 * sin(attack_angle);

  Vector3 m_prop = Vector3(0, kPropSeperation * (f_prop_r - f_prop_l), kMotorTorqueConstant * (f_prop_r - f_prop_l));

  double v_downwash_l = sqrt((2 * f_prop_l) / (kAirDensity * kPropDiameter) + pow(std::max(0.0, vel_B_W.Z()), 2));
  double elevon_air_speed_l = sqrt(pow(vel_B_W.Y(), 2) + pow(v_downwash_l, 2));
  double v_downwash_r = sqrt((2 * f_prop_r) / (kAirDensity * kPropDiameter) + pow(std::max(0.0, vel_B_W.Z()), 2));
  double elevon_air_speed_r = sqrt(pow(vel_B_W.Y(), 2) + pow(v_downwash_r, 2));

  const double kb_x = 7.46e-6, kc_x = 2.18e-4, kb_y = 4.12e-6, kb_z = 3.19e-6, kc_z = 3.18e-4;
  double m_aero_x = (kb_x + kc_x * delta_elevon_left_) * pow(elevon_air_speed_l, 2) + (kb_x + kc_x * delta_elevon_right_) * pow(elevon_air_speed_r, 2) + m_pitch;
  double m_aero_y = kb_y * (elevon_air_speed_l - elevon_air_speed_r);
  double m_aero_z = (kb_z + kc_z * delta_elevon_left_) * pow(elevon_air_speed_l, 2) - (kb_z + kc_z * delta_elevon_right_) * pow(elevon_air_speed_r, 2);
  Vector3 m_aero = Vector3(m_aero_x,m_aero_y,m_aero_z);

  Vector3 m_B = m_prop + m_aero;

  // Apply the calculated forced and moments to the main body link.
  link_->AddRelativeForce(f_B);
  link_->AddRelativeTorque(m_B);
}

void GazeboFwSlipstreamDynamicsPlugin::CreatePubsAndSubs()
{
  gzdbg << __PRETTY_FUNCTION__ << " called." << std::endl;

  // Create temporary "ConnectRosToGazeboTopic" publisher and message
  gazebo::transport::PublisherPtr gz_connect_ros_to_gazebo_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectRosToGazeboTopic>(
          "~/" + kConnectRosToGazeboSubtopic, 1);
  gz_std_msgs::ConnectRosToGazeboTopic connect_ros_to_gazebo_topic_msg;

  // ============================================ //
  // ===== ACTUATORS MSG SETUP (ROS->GAZEBO) ==== //
  // ============================================ //

  actuators_sub_ =
      node_handle_->Subscribe("~/" + namespace_ + "/" + actuators_sub_topic_,
                              &GazeboFwSlipstreamDynamicsPlugin::ActuatorsCallback,
                              this);

  connect_ros_to_gazebo_topic_msg.set_ros_topic(namespace_ + "/" +
                                                actuators_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   actuators_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_msgtype(
      gz_std_msgs::ConnectRosToGazeboTopic::ACTUATORS);

  gz_connect_ros_to_gazebo_topic_pub->Publish(connect_ros_to_gazebo_topic_msg,
                                              true);
}

// read and cap values
double extractCapped(GzActuatorsMsgPtr &msg, const int channel, bool isZeroBase = false)
{
  return std::max(isZeroBase ? 0.0 : -1.0, std::min(1.0, msg->normalized(channel)));
}

double DeNormalize(
    const double valNormalized,
    const double flatValMin,
    const double flatValMax,
    const bool isZeroBase = false)
{
  if (isZeroBase)
    return valNormalized * (flatValMax - flatValMin) + flatValMin;
  else
    return (valNormalized + 1) * (flatValMax - flatValMin) / 2 + flatValMin;
}

double toRads(const double degrees){
  return degrees/180*M_PI;
}

void GazeboFwSlipstreamDynamicsPlugin::ActuatorsCallback(GzActuatorsMsgPtr &actuators_msg)
{
  if (kPrintOnMsgCallback)
    gzdbg << __FUNCTION__ << "() called." << std::endl;

  // todo: take limits from control surface
  const double kElevonAlphaL = toRads(-30), kElevonAlphaU = -kElevonAlphaL;
  // in newtons
  const double kThrottleL = 0, kThrottleU = 1.2;
  enum ActuatorChannel
  {
    ElevonLeft,
    ElevonRight,
    ThrottleLeft,
    ThrottleRight
  };

  // todo: cap to min and max values
  delta_elevon_left_ = DeNormalize(extractCapped(actuators_msg, ElevonLeft), kElevonAlphaL, kElevonAlphaU);
  delta_elevon_right_ = DeNormalize(extractCapped(actuators_msg, ElevonRight), kElevonAlphaL, kElevonAlphaU);

  throttle_left = DeNormalize(extractCapped(actuators_msg, ThrottleLeft, true), kThrottleL, kThrottleU, true);
  throttle_right = DeNormalize(extractCapped(actuators_msg, ThrottleRight, true), kThrottleL, kThrottleU, true);
  gzdbg <<"Elevon(rads) L:"<<delta_elevon_left_<<", R: "<<delta_elevon_right_<< "\nThrottles set to: " << throttle_left << " N, " << throttle_right << " N" << std::endl;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboFwSlipstreamDynamicsPlugin);
} // namespace gazebo