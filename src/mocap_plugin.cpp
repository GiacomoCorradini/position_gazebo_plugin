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
}

void GazeboMocapPlugin::addNoise() {

}

void GazeboMocapPlugin::OnUpdate() {

}







}