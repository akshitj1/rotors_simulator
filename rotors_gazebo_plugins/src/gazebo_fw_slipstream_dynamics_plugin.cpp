//
// Author: Akshit Jain
//

#include "rotors_gazebo_plugins/gazebo_fw_slipstream_dynamics_plugin.h"
#include "ConnectRosToGazeboTopic.pb.h"

namespace gazebo
{

GazeboFwSlipstreamDynamicsPlugin::GazeboFwSlipstreamDynamicsPlugin()
    : ModelPlugin(),
      node_handle_(0),
      prev_time(0) {}

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

  double sampling_time = (_info.simTime - prev_time).Double();
  prev_time = _info.simTime;

  calcActuatorsControl(sampling_time);
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
  double surface_speed = getSurfaceSpeed(vel_B_W);
  double attack_angle = getAttackAngle(vel_B_W);

  double f_prop_l = getPropThrust(throttle_left);
  double f_prop_r = getPropThrust(throttle_right);
  double f_prop_avg = (f_prop_l + f_prop_r) / 2;
  Vector3 f_prop = Vector3(0, 0, f_prop_l + f_prop_r);

  const double dynamic_pressure = getDynamicPressure(surface_speed);
  double f_lift = getLiftForce(surface_speed, attack_angle, f_prop_avg);
  double f_drag = getDragForce(surface_speed, attack_angle, f_prop_avg);
  Vector3 f_aero = Vector3(0, f_lift, -f_drag);

  Vector3 f_B = f_prop + f_aero;

  // moments
  double m_pitch = getPitchingMoment(surface_speed, attack_angle);

  Vector3 m_prop = Vector3(0, kPropSeperation * (f_prop_r - f_prop_l), kMotorTorqueConstant * (f_prop_r - f_prop_l));

  double elevon_air_speed_l = getAirspeedOverElevon(f_prop_l, vel_B_W);
  double elevon_air_speed_r = getAirspeedOverElevon(f_prop_r, vel_B_W);

  double m_aero_x = (kb_x + kc_x * delta_elevon_left_) * pow(elevon_air_speed_l, 2) + (kb_x + kc_x * delta_elevon_right_) * pow(elevon_air_speed_r, 2) + m_pitch;
  double m_aero_y = kb_y * (elevon_air_speed_l - elevon_air_speed_r);
  double m_aero_z = (kb_z + kc_z * delta_elevon_left_) * pow(elevon_air_speed_l, 2) - (kb_z + kc_z * delta_elevon_right_) * pow(elevon_air_speed_r, 2);
  Vector3 m_aero = Vector3(m_aero_x, m_aero_y, m_aero_z);

  Vector3 m_B = m_prop + m_aero;

  // Apply the calculated forced and moments to the main body link.
  link_->AddRelativeForce(f_B);
  link_->AddRelativeTorque(m_B);
}

double GazeboFwSlipstreamDynamicsPlugin::getPitchingMoment(const double &surface_speed, const double &attack_angle)
{
  return getDynamicPressure(surface_speed) * kWingChord * kP1 * sin(attack_angle);
}

double GazeboFwSlipstreamDynamicsPlugin::getDownwashAirSpeed(const double &f_prop, const Vector3 &vel)
{
  return sqrt((2 * f_prop) / (kAirDensity * kPropDiameter) + pow(std::max(0.0, vel.Z()), 2));
}

double GazeboFwSlipstreamDynamicsPlugin::getAirspeedOverElevon(const double &f_prop, const Vector3 &vel)
{
  return sqrt(pow(vel.Y(), 2) + pow(getDownwashAirSpeed(f_prop, vel), 2));
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

double toRads(const double degrees)
{
  return degrees / 180 * M_PI;
}

void GazeboFwSlipstreamDynamicsPlugin::ActuatorsCallback(GzActuatorsMsgPtr &actuators_msg)
{
  if (kPrintOnMsgCallback)
    gzdbg << __FUNCTION__ << "() called." << std::endl;

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
  gzdbg << "Elevon(rads) L:" << delta_elevon_left_ << ", R: " << delta_elevon_right_ << "\nThrottles set to: " << throttle_left << " N, " << throttle_right << " N" << std::endl;
}

double GazeboFwSlipstreamDynamicsPlugin::getPropThrust(const double &throttle)
{
  return kMotorForceConstant * throttle;
}

double GazeboFwSlipstreamDynamicsPlugin::getSurfaceSpeed(const Vector3 &vel)
{
  return sqrt(pow(vel.Y(), 2) + pow(vel.Z(), 2));
}

double GazeboFwSlipstreamDynamicsPlugin::getAttackAngle(const Vector3 &vel)
{
  return atan2(vel.Y(), vel.Z());
}

double GazeboFwSlipstreamDynamicsPlugin::getDynamicPressure(const double surface_speed)
{
  return 1.0 / 2 * kAirDensity * kWingArea * pow(surface_speed, 2);
}

double GazeboFwSlipstreamDynamicsPlugin::getLiftForce(const double &surface_speed, const double &attack_angle, const double &f_prop_avg)
{
  return getDynamicPressure(surface_speed) * (kL1 * sin(attack_angle) * pow(cos(attack_angle), 2) + kL2 * pow(sin(attack_angle), 3)) + kL3 * f_prop_avg;
}

double GazeboFwSlipstreamDynamicsPlugin::getDragForce(const double &surface_speed, const double &attack_angle, const double &f_prop_avg)
{
  return getDynamicPressure(surface_speed) * (kD1 * pow(sin(attack_angle), 2) * cos(attack_angle) + kD2 * cos(attack_angle)) + kD3 * f_prop_avg;
}

void getTrajPoint(const double &sim_time, Vector3 &pos_des, Vector3 &vel_des)
{
  // track circular trajectory
  const double kTrajRadius = 5 * GazeboFwSlipstreamDynamicsPlugin::kWingSpan;
  const double kTrajHeigth = 5 * GazeboFwSlipstreamDynamicsPlugin::kWingChord;
  const double kTimePeriod = 3;
  const double kAngularVel = 2 * M_PI / kTimePeriod;

  double theta = (sim_time / kTimePeriod - int(sim_time / kTimePeriod)) * 2 * M_PI;
  pos_des = Vector3(kTrajRadius * cos(theta), kTrajRadius * sin(theta), kTrajHeigth);
  vel_des = kTrajRadius * kAngularVel * Vector3(-sin(theta), cos(theta), 0.0);
}

void GazeboFwSlipstreamDynamicsPlugin::calcActuatorsControl(const double &sim_sampling_time)
{
  if (kPrintOnControlUpdates)
    gzdbg << __FUNCTION__ << "() called." << std::endl;

  Pose3d pose_W_B = link_->WorldPose();

  Vector3d pos_est = pose_W_B.Pos();
  Vector3 vel_est = link_->WorldLinearVel();
  Vector3 acc_est = link_->WorldAngularAccel();

  Quaterniond rot_est = pose_W_B.Rot();
  Vector3 angular_vel_est = link_->RelativeAngularVel();
  Vector3 angular_acc_est = link_->RelativeAngularAccel();

  Vector3 pos_des;
  Vector3 vel_des;
  getTrajPoint(this->prev_time.Double(), pos_des, vel_des);

  // A. position control
  const double kPosControlTimePeriod = sim_sampling_time; // 0.6;
  const double kPosDampingFactor = 1.0;

  Vector3 acc_des = acc_est + 1 / pow(kPosControlTimePeriod, 2) * (pos_des - pos_est) + 2 * kPosDampingFactor / kPosControlTimePeriod * (vel_des - vel_est);
  Vector3 f_des = kMassBody * (acc_des - Vector3(0, 0, kGravity));

  // B. coordinated flight
  // rotations are defined as composition of angle axis rotations  R_c*R_b*R_a
  // a-> sync heading direction as we have most thrust authority in this direction
  // b-> roll to have coordinated flight (no x component) as we donot have any control authority in this dir.
  // c-> find pitch so that combination of thrust and aero forces can achive these within limits
  double rot_angle_a = acos(vel_est.Z() / vel_est.Length());
  Vector3 rot_axis_a = Vector3(-vel_est.Y(), vel_est.X(), 0.0).Normalize();
  Quaterniond rot_a = Quaterniond(rot_axis_a, rot_angle_a);

  Vector3 f_des_a = rot_a * f_des;
  double rot_angle_b = acos(f_des_a.Y() / sqrt(pow(f_des_a.X(), 2) + pow(f_des_a.Y(), 2)));
  Vector3 rot_axis_b = Vector3(0, 0, -sgn(f_des_a.X()));
  Quaterniond rot_b = Quaterniond(rot_axis_b, rot_angle_b);

  Vector3 f_des_ab = rot_b * f_des_a;
  // find pitch angle with nelder-mead

  PitchOptimizationData opt_data;
  opt_data.f_des_ab = f_des_ab;
  double f_prop_avg = (getPropThrust(throttle_right) + getPropThrust(throttle_left)) / 2;
  opt_data.f_lift = getLiftForce(getSurfaceSpeed(vel_est), getAttackAngle(vel_est), f_prop_avg);
  opt_data.f_drag = getDragForce(getSurfaceSpeed(vel_est), getAttackAngle(vel_est), f_prop_avg);
  // pitch angle, avg_thrust
  arma::vec2 optimum_vals = arma::zeros<arma::vec>(2);
  optim::algo_settings_t options;
  options.lower_bounds(0) = -M_PI;
  options.lower_bounds(1) = kThrottleL;

  options.upper_bounds(0) = M_PI;
  options.upper_bounds(1) = kThrottleU;
  options.opt_iter = 50;

  bool success = optim::nm(optimum_vals, coordinated_pitch_residual, &opt_data, options);
  assert(success);
  double rot_angle_c = optimum_vals(0);
  double f_prop_avg_des = optimum_vals(0);

  Vector3 rot_axis_c = Vector3(-1, 0, 0);
  Quaterniond rot_c = Quaterniond(rot_axis_c, rot_angle_c);

  Quaterniond rot_des = (rot_a * rot_b) * rot_c;

  // C. attitude control
  // valid for small errors only. todo: replace with globally valid optimum
  const double kAttControlTimePeriod = sim_sampling_time; // 0.25;
  Quaterniond rot_err = rot_est * rot_des.Inverse();
  Vector3 rot_err_axis;
  double rot_err_angle;
  rot_err.ToAxis(rot_err_axis, rot_err_angle);
  Vector3 angular_vel_des = -1 / kAttControlTimePeriod * rot_err_axis * rot_err_angle;

  // D. body rate control
  const double kAngularVelTimePeriod = sim_sampling_time; // 0.05;
  Vector3 body_inertia_diag = Vector3(0.462, 2.32, 1.87) * 1e-3;
  Matrix3d body_intertia = Matrix3d(
      body_inertia_diag.X(), 0.0, 0.0,
      0.0, body_inertia_diag.Y(), 0.0,
      0.0, 0.0, body_inertia_diag.Z());
  Vector3 torque_des = 1 / kAngularVelTimePeriod * body_intertia * (angular_vel_des - angular_vel_est) +
                       angular_vel_est.Cross(body_intertia * angular_vel_des);

  double f_prop_del = (torque_des.Y() * kAirDensity * kPropDiameter) / (2 * kAirDensity * kPropDiameter * kPropSeperation - 4 * kb_y);
  double f_prop_l_des = f_prop_avg - f_prop_del;
  double f_prop_r_des = f_prop_avg + f_prop_del;

  // todo: can we use symbolic library for programmatic inversion
  double elevon_air_speed_l = getAirspeedOverElevon(f_prop_l_des, vel_des);
  double elevon_air_speed_r = getAirspeedOverElevon(f_prop_r_des, vel_des);

  double defl_term_f = (kb_z * (pow(elevon_air_speed_r, 2) - pow(elevon_air_speed_l, 2)) +
                        kMotorTorqueConstant * (f_prop_l_des - f_prop_r_des) +
                        torque_des.Z()) /
                       (2 * kc_z);
  double defl_term_l = (kb_x * (pow(elevon_air_speed_r, 2) + pow(elevon_air_speed_l, 2)) +
                        getPitchingMoment(getSurfaceSpeed(vel_des), getAttackAngle(vel_des)) -
                        torque_des.X()) /
                       (2 * kc_x);

  // todo: check and prioritize if actuator out of limits
  double elevon_defl_l_des = 1 / pow(elevon_air_speed_l, 2) * (defl_term_f - defl_term_l);
  double elevon_defl_r_des = 1 / pow(elevon_air_speed_r, 2) * (-defl_term_f - defl_term_l);

  this->throttle_left = cap(f_prop_l_des, kThrottleL, kThrottleU);
  this->throttle_right = cap(f_prop_r_des, kThrottleL, kThrottleU);
  this->delta_elevon_left_ = cap(elevon_defl_l_des, kElevonAlphaL, kElevonAlphaU);
  this->delta_elevon_right_ = cap(elevon_defl_r_des, kElevonAlphaL, kElevonAlphaU);
}

// todo: replace with filter utility
const double cap(const double &inp_val, const double &valL, const double &valU)
{
  return std::min(valU, std::max(valL, inp_val));
}

double
coordinated_pitch_residual(const arma::vec &vals_inp, arma::vec *grad_out, void *opt_data)
{
  double pitch_angle = vals_inp(0);
  double thrust_props_avg = vals_inp(1);
  PitchOptimizationData *data = reinterpret_cast<PitchOptimizationData *>(opt_data);
  double del_fy = data->f_lift - cos(pitch_angle) * data->f_des_ab.Y() - sin(pitch_angle) * data->f_des_ab.Z();
  double del_fz = 2 * thrust_props_avg - data->f_drag - cos(pitch_angle) * data->f_des_ab.Z() - sin(pitch_angle) * data->f_des_ab.Y();

  double residual = pow(del_fy, 2) + pow(del_fz, 2);
  return residual;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboFwSlipstreamDynamicsPlugin);
} // namespace gazebo