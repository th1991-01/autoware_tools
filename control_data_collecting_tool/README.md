# Control data collecting tools

This package provides a tool for collecting data by pure pursuit control within a specified rectangular area.

<img src="resource/demo.gif" width="900">

## Purpose

## How to use

1. Launch Autoware.

```bash
source ~/autoware/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml rviz_config:=$(ros2 pkg prefix control_data_collecting_tool)/share/control_data_collecting_tool/rviz/autoware.rviz map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

2. Set an initial pose, see [here](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/planning-simulation/#2-set-an-initial-pose-for-the-ego-vehicle)

3. Launch control_data_collecting_tool.

```bash
source ~/autoware/install/setup.bash
ros2 launch control_data_collecting_tool control_data_collecting_tool.launch.py
```

4. Select the data collecting area using `DataCollectingAreaSelectionTool` plugin, see gif.

> [!NOTE]
> You cannot change the data collecting area while driving.

5. Start data collecting by clicking the `LOCAL` button on `OperationMode` in `AutowareStatePanel`.

## Parameter
