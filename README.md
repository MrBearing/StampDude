# StampDude

<img src=img_readme/stamp_dude_logo.png width=150px/>

StampDudeは、ROS 2でUnstampedメッセージをStampedメッセージに変換するためのパッケージです。

## 概要

このパッケージは、タイムスタンプを持たないメッセージ（Unstamped）を、現在時刻のタイムスタンプを付与したメッセージ（Stamped）に変換します。テンプレートベースの汎用的な設計により、様々なメッセージタイプに対応しています。

## サポートされているメッセージタイプ

現在、以下のメッセージタイプの変換をサポートしています：

- `geometry_msgs/msg/Twist` → `geometry_msgs/msg/TwistStamped`
- `geometry_msgs/msg/Point` → `geometry_msgs/msg/PointStamped`
- `geometry_msgs/msg/Pose` → `geometry_msgs/msg/PoseStamped`
- `geometry_msgs/msg/Quaternion` → `geometry_msgs/msg/QuaternionStamped`

## アーキテクチャ

### 基底クラス: Stamper

`Stamper<UnstampedMsg, StampedMsg>`は、テンプレートベースの汎用的な基底クラスです。このクラスは：

- 入力トピックからUnstampedメッセージを受信
- 現在時刻のタイムスタンプを付与
- オプションでframe_idを設定
- 出力トピックにStampedメッセージを発行

### 具体的な実装クラス

各メッセージタイプに対して、Stamperクラスを継承した具体的な実装クラスが提供されています：

- `TwistToTwistStamped`
- `PointToPointStamped`
- `PoseToPoseStamped`
- `QuaternionToQuaternionStamped`

## 使用方法

### ビルド

```bash
cd /path/to/your/ros2_workspace
colcon build
source install/setup.bash
```

### 起動

#### コンポーネントノードとしてlaunch ファイルで起動（推奨）

```bash
# 全てのスタンパーを起動（デフォルトではTwistのみ有効）
ros2 launch stamp_dude_bringup stamp_dude.launch.py

# 特定のスタンパーを有効にして起動
ros2 launch stamp_dude_bringup stamp_dude.launch.py enable_twist:=true enable_point:=true

# 全てのスタンパーを有効
ros2 launch stamp_dude_bringup stamp_dude.launch.py enable_twist:=true enable_point:=true enable_pose:=true enable_quaternion:=true
```

#### 個別のノードを起動

```bash
# Twistスタンパー
ros2 run stamp_dude twist_to_twist_stamped_exec

# Pointスタンパー
ros2 run stamp_dude point_to_point_stamped_exec

# Poseスタンパー
ros2 run stamp_dude pose_to_pose_stamped_exec

# Quaternionスタンパー
ros2 run stamp_dude quaternion_to_quaternion_stamped_exec
```

#### コンポーネントノードとして手動で読み込み

```bash
# コンテナを起動
ros2 run rclcpp_components component_container_mt

# 別のターミナルでコンポーネントを読み込み
ros2 component load /ComponentManager stamp_dude stamp_dude::TwistToTwistStamped
ros2 component load /ComponentManager stamp_dude stamp_dude::PointToPointStamped
ros2 component load /ComponentManager stamp_dude stamp_dude::PoseToPoseStamped
ros2 component load /ComponentManager stamp_dude stamp_dude::QuaternionToQuaternionStamped

# 利用可能なコンポーネントタイプを確認
ros2 component types | grep stamp_dude
```

### パラメータ

各ノードは以下のパラメータをサポートしています：

- `frame_id` (string, default: ""): 出力メッセージのheader.frame_idに設定される値

例：
```bash
ros2 run stamp_dude twist_to_twist_stamped_exec --ros-args -p frame_id:=base_link
```

### トピック

各スタンパーは以下のトピックを使用します：

| スタンパー | 入力トピック | 出力トピック |
|-----------|-------------|-------------|
| TwistToTwistStamped | `/twist` | `/twist_stamped` |
| PointToPointStamped | `/point` | `/point_stamped` |
| PoseToPoseStamped | `/pose` | `/pose_stamped` |
| QuaternionToQuaternionStamped | `/quaternion` | `/quaternion_stamped` |

## 新しいメッセージタイプの追加

新しいメッセージタイプのサポートを追加するには：

1. 新しいヘッダーファイルを作成（例：`include/stamp_dude/vector3_to_vector3_stamped.hpp`）
2. Stamperクラスを継承した新しいクラスを実装
3. 対応するソースファイルを作成
4. CMakeLists.txtに新しいターゲットを追加
5. launch ファイルに新しいノードを追加

## ライセンス

Apache License 2.0

## メンテナー

Takumi Okamoto (takumi1988okamoto@gmail.com)
