# auto_driving_vehicle

## Gazebo Sim

Run the world with the helper script:

```bash
./scripts/run_sim.sh
```

Or run directly from the command line:

```bash
GZ_SIM_RESOURCE_PATH=$PWD/sim/gazebo/models \
GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/sim/gazebo/plugins \
PYTHONPATH=$PWD/sim/gazebo/plugins \
gz sim -r sim/gazebo/worlds/car_world.sdf
```

## truck_gz_driver plugin

The world loads a Python system plugin that applies a constant 10 Nm torque to the front wheels.
If you run Gazebo without `./scripts/run_sim.sh`, export `GZ_SIM_SYSTEM_PLUGIN_PATH` and `PYTHONPATH` so the plugin can be found.

Note: The helper script falls back to the system /usr/bin/gz if the ROS Rolling install does not include the Python system loader plugin.
