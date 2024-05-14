# Control data collecting tools

This package provides a tool for collecting data by pure pursuit control within a specified rectangular area.

## 使い方（開発中バージョン）

1 launch autoware

```bash
ros2 launch autoware_launch planning_simulator.launch.xml rviz_config:=$(ros2 pkg prefix control_data_collecting_tool)/share/control_data_collecting_tool/rviz/autoware.rviz map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

2 launch control_data_collecting_tool
```bash
ros2 launch control_data_collecting_tool control_data_collecting_tool.launch.py
```
3 `DataCollectingAreaSelectionTool`を使ってデータ収集領域を選択

- 走行中にはデータ収集領域を変更できない

4 `2D Pose Estimate`を使って initial poseを置く

5 `LOCAL`を押して走行を開始する

## TODO
- ドキュメント整備
- 不要な変数、パッケージ、ヘッダーの削除
