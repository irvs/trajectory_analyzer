# trajectory_analyzer

A collection of ROS 2 packages for measuring, analyzing, and visualizing the trajectory following accuracy of hydraulic excavators.

## Package List

- **traj_follow_plotter**: Records trajectories, performs statistical analysis of following errors, and generates visualization videos using RViz.
- **traj_recorder_msgs**: Defines custom actions and messages used for trajectory recording.

## Installation

### 1. Clone Repository
Clone into the `src` directory of your ROS 2 workspace (e.g., `ros2-tms-for-construction_ws`).

```bash
cd ~/ros2-tms-for-construction_ws/src
git clone https://github.com/irvs/trajectory_analyzer.git
```

### 2. Install Dependencies
The following system packages and Python libraries are required.

```bash
sudo apt update
sudo apt install -y ffmpeg xvfb
pip3 install matplotlib scipy numpy
```

Install `kdl_parser` from source and apply a patch to fix joint parsing:

```bash
cd ~/ros2-tms-for-construction_ws/src/trajectory_analyzer
git clone https://github.com/jvytee/kdl_parser.git
pip install ./kdl_parser --user

# Apply patch to fix joint parsing in kdl_parser
sed -i "53,56c\    fixed = lambda j,F: kdl.Joint(\n        j.name,\n        getattr(kdl.Joint, 'None', getattr(kdl.Joint, 'NoneJoint', getattr(kdl.Joint, 'Fixed', 0)))\n    )" $(python3 -c "import kdl_parser; print(kdl_parser.__path__[0])")/urdf.py
```

### 3. Build
Build the workspace from the root directory.

```bash
cd ~/ros2-tms-for-construction_ws
colcon build --symlink-install --packages-up-to traj_follow_plotter traj_recorder_msgs
source install/setup.bash
```

## Usage

### MP4 Animation Generation
Uses a virtual display (Xvfb) to playback on RViz and saves the output as an mp4 file using ffmpeg.
By default, `animation.mp4` will be created in the specified directory.

```bash
# Record with normal view (diagonal view)
ros2 launch traj_follow_plotter video_generation.launch.py data_dir:=path/to/run_dir

# Record with a specific view (diagonal specified in camera_view)
ros2 launch traj_follow_plotter video_generation.launch.py data_dir:=path/to/run_dir camera_view:=diagonal

# Record with a custom camera distance (default is 20.0)
ros2 launch traj_follow_plotter video_generation.launch.py data_dir:=path/to/run_dir camera_distance:=25.0
```

If the actual trajectory is not displayed, the `data.csv` in the run directory might not contain end-effector coordinates. Run the following command to add them. The original file will be saved as `data_backup.csv`.

```bash
cd ~/ros2-tms-for-construction_ws
source install/setup.bash
python3 src/trajectory_analyzer/traj_follow_plotter/scripts/add_ee_to_csv.py /path/to/data.csv
```

### RViz Display (Verification only, no MP4 generation)
Displays RViz on the current screen. Used for debugging or verification without recording.
By default, the playback loops when the trajectory ends or after 30 seconds have passed.

```bash
ros2 launch traj_follow_plotter video_generation.launch.py record:=false data_dir:=path/to/run_dir
```

### Visualization of Following Error
Outputs the discrepancy between the target and measured trajectories and link padding analysis results based on recorded CSV data.

```bash
ros2 launch traj_follow_plotter visualize_correspondence.launch.py csv_dir:=path/to/run_dir
```

### Recording Trajectories

You can record the trajectory of a hydraulic excavator operated in a simulator (OperaSim-PhysX).

1. Switch `ros2_tms_for_construction` and `tms_if_for_opera` to the `feature/primitive` branch.

2. Build the workspace.

```bash
cd ~/ros2-tms-for-construction_ws
source install/setup.bash
colcon build
source install/setup.bash
```

3. Create and register a task using Groot. To analyze the motion, you need to operate the excavator using `primitive_excavator_change_pose_execute_from_plan_retime`.

```bash
# Path to Groot might vary
./build/groot/Groot
```

```bash
colcon build
ros2 run tms_ts_manager task_generator.py --ros-args -p bt_tree_xml_file_name:=<task_name> # without ".xml"
```

4. Play the simulator and run the following commands to record the trajectory of the excavation operation.

```bash
# Terminal 1
ros2 launch ros_tcp_endpoint endpoint.py
```

```bash
# Terminal 2
ros2 launch tms_if_for_opera tms_if_for_opera.launch.py
```

```bash
# Terminal 3
ros2 launch zx200_bringup vehicle.launch.py use_rviz:=true command_interface_name:=velocity
```

```bash
# Terminal 4
ros2 launch tms_ts_launch tms_ts_construction.launch.py task_id:=[registered_task_id]
```

```bash
# Terminal 5 (Execute before pressing the green start button for the task)
ros2 launch traj_follow_plotter traj_follow_record.launch.py
```

---

# trajectory_analyzer (Japanese)

油圧ショベルの軌道追従精度を測定・解析・可視化するためのROS 2パッケージ群です。

## 構成パッケージ

- **traj_follow_plotter**: 軌道の記録、追従誤差の統計解析、およびRVizを用いた可視化動画の生成を行います。
- **traj_recorder_msgs**: 軌道記録に使用するカスタムAction等のメッセージを定義しています。

## インストール方法

### 1. リポジトリのクローン
ROS 2ワークスペース（例: `ros2-tms-for-construction_ws`）の`src`ディレクトリにクローンしてください。

```bash
cd ~/ros2-tms-for-construction_ws/src
git clone https://github.com/irvs/trajectory_analyzer.git
```

### 2. 依存関係のインストール
以下のシステムパッケージおよびPythonライブラリが必要です。

```bash
sudo apt update
sudo apt install -y ffmpeg xvfb
pip3 install matplotlib scipy numpy
```

ソースから `kdl_parser` をインストールし、ジョイントのパースを修正するパッチを適用します。

```bash
cd ~/ros2-tms-for-construction_ws/src/trajectory_analyzer
git clone https://github.com/jvytee/kdl_parser.git
pip install ./kdl_parser --user

# kdl_parserのジョイントパース修正パッチを適用
sed -i "53,56c\    fixed = lambda j,F: kdl.Joint(\n        j.name,\n        getattr(kdl.Joint, 'None', getattr(kdl.Joint, 'NoneJoint', getattr(kdl.Joint, 'Fixed', 0)))\n    )" $(python3 -c "import kdl_parser; print(kdl_parser.__path__[0])")/urdf.py
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

実機の軌道が表示されない場合は、そのrunディレクトリ内にあるdata.csvにエンドエフェクタ座標が含まれていない可能性があるため、以下のコマンドを実行してください。元のcsvファイルをdata_backup.csvとして保存し、data.csvにエンドエフェクタ座標を追加します。

```bash
cd ~/ros2-tms-for-construction_ws
source install/setup.bash
python3 src/trajectory_analyzer/traj_follow_plotter/scripts/add_ee_to_csv.py /path/to/data.csv
```

### RViz表示（MP4生成なし・確認のみ）
現在のディスプレイを使用してRVizを表示します。デバッグや録画なしでの確認に使用します。
デフォルトでは軌道が終わるか再生から30秒経過で再生がループします。

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

2. ワークスペースをビルドする。

```bash
cd ~/ros2-tms-for-construction_ws
source install/setup.bash
colcon build
source install/setup.bash
```

3. Grootでタスクを作成して登録する。動作を解析するには`primitive_excavator_change_pose_execute_from_plan_retime`で油圧ショベルを動かす必要があります。

```bash
./build/groot/Groot
```

```bash
colcon build
ros2 run tms_ts_manager task_generator.py --ros-args -p bt_tree_xml_file_name:=<task_name> # without ".xml"
```

4. シミュレータを再生し、以下のコマンドを実行して掘削動作の軌道を記録する。

```bash
# Terminal 1
ros2 launch ros_tcp_endpoint endpoint.py
```

```bash
# Terminal 2
ros2 launch tms_if_for_opera tms_if_for_opera.launch.py
```

```bash
# Terminal 3
ros2 launch zx200_bringup vehicle.launch.py use_rviz:=true command_interface_name:=velocity
```

```bash
# Terminal 4
ros2 launch tms_ts_launch tms_ts_construction.launch.py task_id:=[登録したtask_id]
```

```bash
# Terminal 5 （タスクを開始する緑色のボタンを押す前に実行すること）
ros2 launch traj_follow_plotter traj_follow_record.launch.py
```

