# traj_follow_measurement

油圧ショベルの軌道追従精度を測定・解析・可視化するためのROS 2パッケージ群です。

## 構成パッケージ

- **traj_follow_plotter**: 軌道の記録、追従誤差の統計解析、およびRVizを用いた可視化動画の生成を行います。
- **traj_recorder_msgs**: 軌道記録に使用するカスタムAction等のメッセージを定義しています。

## インストール方法

### 1. リポジトリのクローン
ROS 2ワークスペース（例: `ros2-tms-for-construction_ws`）の`src`ディレクトリにクローンしてください。

```bash
cd ~/ros2-tms-for-construction_ws/src
git clone https://github.com/TsutsumiAkinosuke/traj_follow_measurement.git
```

### 2. 依存関係のインストール
以下のシステムパッケージおよびPythonライブラリが必要です。

```bash
sudo apt update
sudo apt install -y ffmpeg xvfb
pip3 install matplotlib scipy numpy
```

### 3. ビルド
ワークスペースのルートでビルドを行います。

```bash
cd ~/ros2-tms-for-construction_ws
colcon build --symlink-install --packages-up-to traj_follow_plotter traj_recorder_msgs
source install/setup.bash
```

## 使用方法

### MP4アニメーション生成
仮想ディスプレイ（Xvfb）を使用してRViz上で再生し、その様子をffmpegでmp4として保存します。
デフォルトでは`animaton.mp4`が指定したディレクトリに作成されます。

```bash
# 通常の視点（対角視点）での録画
ros2 launch traj_follow_plotter video_generation.launch.py data_dir:=path/to/run_dir

# 以前の視点からの録画（camera_view引数にdiagonalを指定）
ros2 launch traj_follow_plotter video_generation.launch.py data_dir:=path/to/run_dir camera_view:=diagonal

# 撮影距離を指定して録画（デフォルトは20.0）
ros2 launch traj_follow_plotter video_generation.launch.py data_dir:=path/to/run_dir camera_distance:=25.0
```

### RViz表示（MP4生成なし・確認のみ）
現在のディスプレイを使用してRVizを表示します。デバッグや録画なしでの確認に便利です。

```bash
ros2 launch traj_follow_plotter video_generation.launch.py record:=false data_dir:=path/to/run_dir
```

### 追従誤差の可視化
記録されたCSVデータに基づき、目標軌道と実測軌道の乖離やリンクのパディング解析結果を出力します。

```bash
ros2 launch traj_follow_plotter visualize_correspondence.launch.py csv_dir:=path/to/run_dir
```

### 軌道の記録

シミュレータ（OperaSim-PhysX）で動作させた油圧ショベルの軌跡を記録することができます。

1. ros2_tms_for_constructionとtms_if_for_operaを`feature/primitive`ブランチに切り替える。

2. `tms_ts/tms_ts_launch/tms_ts_construction.launch.py`の`primitive_excavator_change_pose_execute_from_plan_retime`のコメントアウトを外す。（新たな別のlaunchファイルとしてコピーしてから修正することを推奨します）
```xml
Node(
    package='tms_ts_primitive',
    executable='primitive_excavator_change_pose_execute_from_plan_retime',
    output='screen',
    parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    namespace = 'zx200'),
```

3. tms_ts/tms_ts_primitiveの`CMakeLists.txt`の以下のコメントアウトを外す。
```xml
find_package(traj_recorder_msgs REQUIRED)

add_executable(primitive_excavator_change_pose_execute_from_plan_retime src/Excavator/

set(TARGETS
    ...
    primitive_excavator_change_pose_execute_from_plan_retime.cpp)
    ...
)

ament_target_dependencies(primitive_excavator_change_pose_execute_from_plan_retime
  srdfdom
  moveit_core
  moveit_ros_planning
  traj_recorder_msgs
)
```

4. ワークスペースをビルドする。

```bash
colcon build --symlink-install --packages-up-to traj_follow_plotter traj_recorder_msgs
source install/setup.bash
```

5. GUIのボタンを押して油圧ショベルのタスクを起動する前に軌道の記録を開始する。現状では`primitive_excavator_change_pose_execute_from_plan_retime`を実行する前に以下のコマンドを実行する必要がある。

```bash
ros2 launch traj_follow_plotter traj_follow_record.launch.py
```
