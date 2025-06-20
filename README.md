# nakalab_realsense

ネイティブサポートカメラ
- D435
- D415
- L515（Docker）

> [!WARNING]
> Realsense L515 を使用する場合は Docker が必要となります。

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
# d435 カメラとして起動
ros2 launch nakalab_realsense camera_launch.py
```
```bash
# d435 カメラとして起動
ros2 launch nakalab_realsense d435_launch.py
```
```bash
# d415 カメラとして起動
ros2 launch nakalab_realsense d415_launch.py
```
```bash
# l515 カメラとして起動
docker compose up l515
```

## LaunchArguments
```bash
ros2 launch nakalab_realsense camera_launch.py --show-args
```
|Argument|Default value|Description|
|:---:|:---:|:---|
|params_file|[/params/d435.yaml](/params/d435.yaml)|ROS2 パラメータ YAML ファイルパス|
|xacro_file|[/urdf/d435.urdf.xacro](/urdf/d435.urdf.xacro)|デモ用カメラの Xacro モデルファイルの絶対パス|
|camera_name|`d435`|使用するカメラ名。子名前空間、リンク、ノード名に対応します。|
|namespace|`''`|親名前空間|
|use_description|`False`|デモ用のカメラモデルを使用し、Rviz2 を起動する|
