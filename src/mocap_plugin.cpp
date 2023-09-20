#include "mocap_plugin.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

namespace gazebo {


// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(GazeboMocapPlugin);

GazeboMocapPlugin::GazeboMocapPlugin(): ModelPlugin()
{
}

GazeboMocapPlugin::~GazeboMocapPlugin() {
  updateConnection_->~Connection();
}

void GazeboMocapPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    
    // Store the pointer to the model
    this->model_ = _model;
    this->world_ = model_->GetWorld();

    // default params
    namespace_.clear();

    // read sdf
    if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzerr << "[gazebo_mocap_plugin] Please specify a robotNamespace.\n";
    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init(namespace_);

    if (_sdf->HasElement("linkName"))
        link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
    else
        gzerr << "[gazebo_mocap_plugin] Please specify a linkName.\n";
    // Get the pointer to the link
    link_ = model_->GetLink(link_name_);
    if (link_ == NULL)
        gzthrow("[gazebo_mocap_plugin] Couldn't find specified link \"" << link_name_ << "\".");

    #if GAZEBO_MAJOR_VERSION >= 9
    last_time_ = world_->SimTime();
    #else
    last_time_ = world_->GetSimTime();
    #endif

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection_ =
        event::Events::ConnectWorldUpdateBegin(
            boost::bind(&GazeboImuPlugin::OnUpdate, this, _1));


}

void GazeboMocapPlugin::OnUpdate(const common::UpdateInfo& _info) {
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time  = world_->SimTime();
#else
  common::Time current_time  = world_->GetSimTime();
#endif
  double dt = (current_time - last_time_).Double();
  last_time_ = current_time;
  double t = current_time.Double();
}


#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d T_W_I = link_->WorldPose(); //TODO(burrimi): Check tf.
#else
  ignition::math::Pose3d T_W_I = ignitionFromGazeboMath(link_->GetWorldPose()); //TODO(burrimi): Check tf.
#endif

  ignition::math::Quaterniond C_W_I = T_W_I.Rot();

  // Copy ignition::math::Quaterniond to gazebo::msgs::Quaternion
  gazebo::msgs::Quaternion* orientation = new gazebo::msgs::Quaternion();
  orientation->set_x(C_W_I.X());
  orientation->set_y(C_W_I.Y());
  orientation->set_z(C_W_I.Z());
  orientation->set_w(C_W_I.W());




}