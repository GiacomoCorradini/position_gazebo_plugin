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

## Add the plugin to your SDF file

```xml
<plugin name='libposition_plugin' filename='libposition_plugin.so'>
    <body_name>base_link</body_name>        <!-- base-link name -->
    <update_rate>50</update_rate>           <!-- update frequency -->
    <xyz_offset>0 0 0</xyz_offset>          <!-- position offset -->
    <rpy_offset>0 0 0</rpy_offset>          <!-- rotation offset -->
    <gaussian_noise>0.01</gaussian_noise>   <!-- gaussian noise -->
</plugin>
```
