# Get position plugin

Plugin to obtain the position of a base link in a gazebo environment.

## Build the plugin

```bash
colcon build
```

## Include the plugin in gazebo

From terminal:

```bash
export GAZEBO_PLUGIN_PATH=$HOME:~/position_gazebo_plugin/build/position_gazebo_plugin:$GAZEBO_PLUGIN_PATH
```

You can also add the command in your bashrc
