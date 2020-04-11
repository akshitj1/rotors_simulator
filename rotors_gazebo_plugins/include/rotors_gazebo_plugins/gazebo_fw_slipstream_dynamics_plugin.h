/*
 * Copyright 2017 Pavel Vechersky, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROTORS_GAZEBO_FW_SLIPSTREAM_DYNAMICS_PLUGIN_H
#define ROTORS_GAZEBO_FW_SLIPSTREAM_DYNAMICS_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <mav_msgs/default_topics.h>
#include <optim.hpp>
#include <yaml-cpp/yaml.h>
#include <iomanip>

#include "Actuators.pb.h"

#include "rotors_gazebo_plugins/common.h"

namespace gazebo
{
using namespace ignition::math;
typedef Vector3d Vector3;

typedef const boost::shared_ptr<const gz_sensor_msgs::Actuators>
    GzActuatorsMsgPtr;

template <typename T>
inline void YAMLReadParam(const YAML::Node &node,
                          const std::string &name,
                          T &value);

#define READ_PARAM(node, item) YAMLReadParam(node, #item, item);

struct TsControlParams;

struct TsControlParams
{
  double pos_time_const = 0.6;
  double att_time_const = 0.25;
  double angular_vel_time_const = 0.05;

  TsControlParams() {}

  void LoadVehicleParamsYAML(const std::string &yaml_path)
  {
    const YAML::Node node = YAML::LoadFile(yaml_path);

    READ_PARAM(node, pos_time_const);
    READ_PARAM(node, att_time_const);
    READ_PARAM(node, angular_vel_time_const);
  }
};

// Constants.
static constexpr double kAirDensity = 1.18;
static constexpr double kGravity = 9.81;

class GazeboFwSlipstreamDynamicsPlugin : public ModelPlugin
{
public:
  /// \brief    Constructor.
  GazeboFwSlipstreamDynamicsPlugin();

  /// \brief    Destructor.
  virtual ~GazeboFwSlipstreamDynamicsPlugin();

public:
  /// \brief    Called when the plugin is first created, and after the world
  ///           has been loaded. This function should not be blocking.
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief  	This gets called by the world update start event.
  void OnUpdate(const common::UpdateInfo &);

  /// \brief	Calculates the forces and moments to be applied to the
  ///           tailsitter body.
  void UpdateForcesAndMoments();

private:
  /// \brief    Flag that is set to true once CreatePubsAndSubs() is called,
  ///           used to prevent CreatePubsAndSubs() from be called on every
  ///           OnUpdate().
  bool pubs_and_subs_created_ = false;

  /// \brief    Creates all required publishers and subscribers, incl. routing
  ///           of messages to/from ROS if required.
  /// \details  Call this once the first time OnUpdate() is called (can't be
  ///           called from Load() because there is no guarantee
  ///           GazeboRosInterfacePlugin has loaded and listening to
  ///           ConnectGazeboToRosTopic and ConnectRosToGazeboTopic messages).
  void CreatePubsAndSubs();

  /// \brief    Transport namespace.
  std::string namespace_;
  /// \brief    Topic name for actuator commands.
  std::string actuators_sub_topic_;

  /// \brief    Handle for the Gazebo node.
  transport::NodePtr node_handle_;

  /// \brief    Subscriber for receiving actuator commands.
  gazebo::transport::SubscriberPtr actuators_sub_;

  /// \brief    Pointer to the world.
  physics::WorldPtr world_;
  /// \brief    Pointer to the model.
  physics::ModelPtr model_;
  /// \brief    Pointer to the link.
  physics::LinkPtr link_;
  /// \brief    Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  TsControlParams ts_control_params;
  common::Time prev_time = 0;
  common::Time latest_debug_time = 0;
  const common::Time kDebugInterval = 0.05;
  double sampling_time = 0;

  const bool kPrintOnControlUpdates = false;

  /// \brief    Left elevon deflection [rad].
  double delta_elevon_left_ = 0;
  /// \brief    Right elevon deflection [rad].
  double delta_elevon_right_ = 0;
  /// \brief    Left Throttle input range: [0-1].
  double throttle_left = 0;
  /// \brief    Right Throttle range [0-1].
  double throttle_right = 0;

  // forces
  const double kMassBody = 0.15;
  // seperation from COM
  const double kPropDiameter = 0.13, kPropSeperation = 0.14;
  // torque constant is thrust to torque ratio
  const double kMotorTorqueConstant = 0.00872;
  // throttle_(left/right) in newtons so unit forceConstant
  const double kMotorForceConstant = 1;

  // todo: take limits from control surface
  const double kElevonAlphaL = toRads(-30), kElevonAlphaU = -kElevonAlphaL;
  // in newtons
  const double kThrottleL = 0, kThrottleU = 1.2;

  // pw75 airfoil
  // https://colab.research.google.com/drive/1sOBJVBV01y0fO1oZhXUMovB-0IZWwE8v
  static constexpr double kL1 = 3.5, kL2 = 0, kL3 = 0, kD1 = 3.0, kD2 = 0, kD3 = 0;
  static constexpr double kP1 = 0.3;
  static constexpr double kb_x = -7.46e-6, kc_x = -2.18e-4, kb_y = 4.12e-6, kb_z = 3.19e-6, kc_z = 3.18e-4;

public:
  static constexpr double kWingSpan = 0.6, kWingChord = 0.2, kWingArea = 0.12;
  static constexpr bool kHover = false;

private:
  /// \brief    Processes the actuator commands.
  /// \details  Converts control surface actuator inputs into physical angles
  ///           before storing them and throttle values for later use in
  ///           calculation of forces and moments.
  void ActuatorsCallback(GzActuatorsMsgPtr &actuators_msg);

  void calcActuatorsControl(const double &sim_sampling_time);

  void updateSimFreq(const common::Time &cur_time);

  double toRads(const double degrees);

public:
  static double getSurfaceSpeed(const Vector3 &vel);
  static double getAttackAngle(const Vector3 &vel);
  static double getLiftForce(const double &surface_speed, const double &attack_angle, const double &f_prop_avg);
  static double getDragForce(const double &surface_speed, const double &attack_angle, const double &f_prop_avg);

private:
  double getPropThrust(const double &throttle);
  double getDownwashAirSpeed(const double &f_prop, const Vector3 &vel);
  double getAirspeedOverElevon(const double &f_prop, const Vector3 &vel);
  double getPitchingMoment(const double &surface_speed, const double &attack_angle);
  static double getDynamicPressure(const double surface_speed);
};

struct PitchOptimizationData
{
  Vector3 f_des_ab;
  double v_ref;
};

double coordinated_pitch_residual(const arma::vec &vals_inp, arma::vec *grad_out, void *opt_data);
const double cap(const double &inp_val, const double &valL, const double &valU);
void getTrajPoint(const double &sim_time, Vector3 &pos_des, Vector3 &vel_des);

template <typename T>
inline void YAMLReadParam(const YAML::Node &node,
                          const std::string &name,
                          T &value)
{
  value = node[name].as<T>();
}

} // namespace gazebo

#endif // ROTORS_GAZEBO_FW_SLIPSTREAM_DYNAMICS_PLUGIN_H
