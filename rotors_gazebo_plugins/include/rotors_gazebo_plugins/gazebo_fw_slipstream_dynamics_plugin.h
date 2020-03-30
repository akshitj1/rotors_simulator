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

#include "Actuators.pb.h"

#include "rotors_gazebo_plugins/common.h"

namespace gazebo {

typedef const boost::shared_ptr<const gz_sensor_msgs::Actuators>
	GzActuatorsMsgPtr;

// Constants.
static constexpr double kAirDensity = 1.18;
static constexpr double kGravity = 9.81;

class GazeboFwSlipstreamDynamicsPlugin : public ModelPlugin {
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
  void OnUpdate(const common::UpdateInfo&);

  /// \brief	Calculates the forces and moments to be applied to the
  ///           tailsitter body.
  void UpdateForcesAndMoments();

 private:
  /// \brief    Flag that is set to true once CreatePubsAndSubs() is called,
  ///           used to prevent CreatePubsAndSubs() from be called on every
  ///           OnUpdate().
  bool pubs_and_subs_created_=false;

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

  /// \brief    Left elevon deflection [rad].
  double delta_elevon_left_;
  /// \brief    Right elevon deflection [rad].
  double delta_elevon_right_;
  /// \brief    Left Throttle input range: [0-1].
  double throttle_left;
  /// \brief    Right Throttle range [0-1].
  double throttle_right;

  /// \brief    Processes the actuator commands.
  /// \details  Converts control surface actuator inputs into physical angles
  ///           before storing them and throttle values for later use in
  ///           calculation of forces and moments.
  void ActuatorsCallback(GzActuatorsMsgPtr& actuators_msg);
};

}  // namespace gazebo

#endif // ROTORS_GAZEBO_FW_SLIPSTREAM_DYNAMICS_PLUGIN_H
