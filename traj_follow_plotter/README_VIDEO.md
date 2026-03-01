# 動画生成機能（シンプル版）

記録データからURDFモデルを使ってRvizで理想(reference)と実際(feedback)の軌跡を比較するアニメーション動画を生成します。

## 構成（シンプル）

```
traj_follow_plotter/
├── launch/
│   └── video_generation.launch.py  ← これ1つで全部完結
├── config/
│   └── video.rviz                  ← Rviz設定
└── traj_follow_plotter/
    ├── traj_follow_plotter_node.py  ← 記録用ノード
    └── video_player_node.py         ← CSV再生ノード
```

## 使い方（超簡単）

### 1コマンドで動画生成

```bash
cd /home/common/3_SIP/tms_ws
source install/setup.bash

# これだけ！
ros2 launch traj_follow_plotter video_generation.launch.py \
  data_dir:=/home/common/3_SIP/tms_ws/src/traj_follow_measurement/data/run_20260301_200351
```

これで以下が全自動で実行されます：
1. Xvfb起動（仮想ディスプレイ）
2. robot_state_publisher x2（reference/feedback）
3. video_player_node（CSV再生）
4. RViz起動
5. ffmpeg録画開始
6. データ再生完了後、動画保存

**生成される動画**: `data/run_YYYYMMDD_HHMMSS/animation.mp4`

## アーキテクチャ

### 起動順序

```
1. Xvfb (:99)
   ↓
2. robot_state_publisher (ref_/fb_)
   ↓
3. video_player_node
   ↓ (3秒待機)
4. RViz
   ↓ (5秒待機)
5. ffmpeg録画開始
   ↓ (データ再生)
6. 30秒後に自動終了
```

### トピック構成

- `/video_gen/joint_states_ref` → robot_state_publisher_ref → TF: ref_*
- `/video_gen/joint_states_fb` → robot_state_publisher_fb → TF: fb_*

## トラブルシューティング

### ロボットモデルが表示されない

```bash
# 別ターミナルでTFを確認
ros2 topic list | grep joint_states
ros2 topic echo /video_gen/joint_states_ref --once
```

### 動画が真っ黒

- Rvizの起動に時間がかかる場合、launchファイルの`period=3.0`を`5.0`に変更

### データ再生時間が長い

- `playback_speed: 2.0`に変更して2倍速再生

## 今後の拡張

- [ ] データ再生完了を検知して自動終了
- [ ] 軌跡の可視化（Pathメッセージ）
