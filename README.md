# nakalab_realsense
　`rules` ファイルのコピー
```bash
sudo cp 99-realsense.tules /lib/udev/rules.d/
```
rules の反映
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```
　依存関係のインストール
```bash
rosdep install -y -i --from-path src/nakalab_realsense
```
　ワークスペースをビルドする
```bash
colcon build --symlink-install

# このパッケージのみビルドしたい場合
colcon build --symlink-install --packages-up-to nakalab_realsense
```
　実行
```bash
ros2 launch nakalab_realsense camera_launch.py
```

## LaunchArguments
```bash
ros2 launch nakalab_realsense camera_launch.py --show-args
```
|Argument|Default value|Description|
|:---:|:---:|:---|
|params_file|[/params/d435.yaml](/params/d435.yaml)|ROS2 パラメータ YAML ファイルパス|
|camera_name|`d435`|使用するカメラ名。子名前空間、リンク、ノード名に対応します。|
|namespace|`''`|親名前空間|
