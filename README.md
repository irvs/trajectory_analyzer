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

実機の軌道が表示されない場合は、そのrunディレクトリ内にあるdata.csvにエンドエフェクタ座標が含まれていない可能性があるため、以下のコマンドを実行してください。元のcsvファイルをdata_backup.csvとして保存し、data.csvにエンドエフェクタ座標を追加します。

```bash
cd ~/ros2-tms-for-construction_ws
source install/setup.bash
python3 src/traj_follow_measurement/traj_follow_plotter/scripts/add_ee_to_csv.py [data.csvのパス]
```

### RViz表示（MP4生成なし・確認のみ）
現在のディスプレイを使用してRVizを表示します。デバッグや録画なしでの確認に使用します。

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

1. ros2_tms_for_constructionとtms_if_for_operaを`feature/subtask_for_excavator`ブランチに切り替える。

2. ワークスペースをビルドする。

```bash
cd ~/ros2-tms-for-construction_ws
source install/setup.bash
colcon build
source install/setup.bash
```

3. タスクを登録する。

```bash
ros2 run tms_ts_manager task_generator.py --ros-args -p bt_tree_xml_file_name:=ExcavateRelease_combined_1_23.xml
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
