# Control data collecting tools

This package provides a tool for collecting data by pure pursuit control within a specified rectangular area.

## 使い方（開発中バージョン）

0 public版のautowareと、このパッケージを含むリポジトリをgit clone

1 colcon build

2 autoware立ち上げ

```bash
ros2 launch control_data_collecting_tool planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit trajectory_follower_mode:=control_data_collecting_tool
```

3 `DataCollectingAreaSelectionTool`を使ってデータ収集領域を選択

- 走行中にはデータ収集領域を変更できない

4 `2D Pose Estimate`を使って initial poseを置く

5 `LOCAL`を押して走行を開始する（現在は目標軌道は適当に仮置きしている）

## TODO

- ドキュメント整備
- 不要な変数、パッケージ、ヘッダーの削除
