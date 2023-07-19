#ifndef _MOCAP_PLUGIN_HH_
#define _MOCAP_PLUGIN_HH_

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>

namespace gazebo
{

class GazeboMocapPlugin : public ModelPlugin {
    public:

        GazeboMocapPlugin();
        ~GazeboMocapPlugin();
        
    protected:
        
        // Documentation inherited
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        // Add noise
        void addNoise();

        // Update each time step
        void OnUpdate(const common::UpdateInfo&);

    private:
        std::string namespace_;

        // Pointer to the world
        physics::WorldPtr world_;
        // Pointer to the model
        physics::ModelPtr model_;
        // Pointer to the link
        physics::LinkPtr link_;
        // Pointer to the update event connection
        event::ConnectionPtr updateConnection_;


};

} // namespace gazebo

#endif // _MOCAP_PLUGIN_HH_