#ifndef _POSITION_GAZEBO_PLUGIN_HH_
#define _POSITION_GAZEBO_PLUGIN_HH_

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{

class PositionGazeboPluginPrivate;

class PositionGazeboPlugin : public gazebo::ModelPlugin {
    public:

        // Constructor
        PositionGazeboPlugin();

        // Destructor
        virtual ~PositionGazeboPlugin();
             
        // Documentation inherited
        void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

    private:

        // Private data pointer
        std::unique_ptr<PositionGazeboPluginPrivate> impl_;

};

} // gazebo_plugins gazebo

#endif // _POSITION_GAZEBO_PLUGIN_HH_