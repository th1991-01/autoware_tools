# Control data collecting tools

This package provides a tool for collecting data by pure pursuit control within a specified rectangular area.

## 使い方（開発中バージョン）

0. public版のautowareと、このパッケージを含むリポジトリをgit clone

1. colcon build

2. `tmp/control.launch.py` を上書きコピー（※アドホックすぎるので改善したい）

```
cp tmp/control.launch.py ~/autoware/src/universe/autoware.universe/launch/tier4_control_launch/launch/control.launch.py
```

3. autoware立ち上げ

```
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit trajectory_follower_mode:=control_data_collecting_tool
```

4. プラグイン追加とトピック描画追加（※アドホックすぎるので改善したい）

- プラグイン追加：`pure_pursuit_control_data_collection_tool` -> `DataCollectingAreaSelectionTool`
- トピック描画追加：
  - データ収集領域：`Display`->`add`->`rviz_default_plugins` -> `Polygon`、トピック名は`/data_collecting_area`
  - 目標軌道：`Display`->`add`->`rviz_default_plugins` -> `MarkerArray`、トピック名は`data_collecting_trajectory_marker_array`（※もしTrajectoryをそのまま描画できるのならばそれに変更したい）

5. 追加したプラグイン`DataCollectingAreaSelectionTool`を使ってデータ収集領域を選択

- 走行中には選択できない

6. initial poseを置く

7. （※この操作をなしでできるように改善したい）

8. 自動走行する（現在は目標軌道は適当に仮置きしている）

## TODO

- 「アドホックすぎるので改善したい」などをなんとかする
- planningを無効化したい
- 不要な変数、パッケージ、ヘッダーの削除
