# robstride_ros2

RobStrideアクチュエータをprivate CAN protocolで制御する、非公式・コミュニティ管理のROS 2 `ros2_control` Hardware Componentです。
CAN frameの送受信には[`ros2_socketcan`](https://github.com/autowarefoundation/ros2_socketcan)が提供する`can_msgs/msg/Frame` topicを使用します。

本プロジェクトはRobStride社とは提携しておらず、同社による承認を受けたものではありません。protocolと型番別profileは、RobStride公式[`Product_Information`](https://github.com/RobStride/Product_Information) repositoryの英語版manualと照合しています。英語版READMEは[`README.md`](README.md)を参照してください。

## 主な機能

- 1つのCAN bus上にある複数のアクチュエータを、1つの`ros2_control` SystemInterfaceで管理
- 各jointで位置・速度・effort command interfaceを使用可能
- モーターごとに異なるcommand modeを同時に使用可能
- RS00、RS01、RS02、RS03、RS04、RS05、RS06、EL05の型番別profile
- 位置、速度、トルク、温度、faultのfeedback
- 起動時のparameter読み戻しとモーター有効化確認
- feedback timeoutと終了時の停止指令再送
- activateのたびにmotor側CAN watchdogを非ゼロ値へ設定

## パッケージ構成

repositoryには4つのROS 2 packageが含まれます。

| package | 役割 |
|---|---|
| `robstride_driver` | CAN通信、モーター指令、feedback、復旧処理 |
| `robstride_ros2_control` | `ros2_control` Hardware Component |
| `robstride_examples` | 型番別Xacro、controller設定、example launch |
| `robstride_ros2` | 上記packageをまとめてインストールするaggregate package |

既存のrobot descriptionとの互換性を保つため、plugin IDは`robstride_ros2/RobStrideSystem`を使用します。

## 対応ROS 2ディストリビューション

同じsource branchで以下のROS 2ディストリビューションに対応し、それぞれのTier 1 Ubuntu環境でCIを実行します。

| ROS 2 | Ubuntu |
|---|---|
| Humble | 22.04（Jammy） |
| Jazzy | 24.04（Noble） |
| Kilted | 24.04（Noble） |
| Lyrical | 26.04（Resolute） |
| Rolling | 26.04（Resolute） |

ディストリビューションごとのsource branchは必要ありません。Rollingは開発版であり、将来の安定版リリース前に互換性のない変更が入る場合があります。Rolling CIは、テスト対象revisionにおける互換性の確認結果です。

## protocolとfirmwareの前提

本packageはRobStride公式manualに記載された29-bit extended frameのprivate protocolを使用します。モーターをCANopenや11-bit MIT protocolではなく、private protocolに設定してください。

以下はCAN値のencode・decodeに使う範囲であり、推奨する機械的な動作範囲ではありません。必要に応じてrobot description側に、より狭いjoint・速度・effort制限を設定してください。

| 型番 | 位置 | 速度 | トルク | Kp | Kd |
|---|---:|---:|---:|---:|---:|
| RS00 | ±4π rad | ±33 rad/s | ±14 Nm | 0..500 | 0..5 |
| RS01 | ±4π rad | ±44 rad/s | ±17 Nm | 0..500 | 0..5 |
| RS02 | ±4π rad | ±44 rad/s | ±17 Nm | 0..500 | 0..5 |
| RS03 | ±4π rad | ±20 rad/s | ±60 Nm | 0..5000 | 0..100 |
| RS04 | ±4π rad | ±15 rad/s | ±120 Nm | 0..5000 | 0..100 |
| RS05 | ±4π rad | ±50 rad/s | ±5.5 Nm | 0..500 | 0..5 |
| RS06 | ±4π rad | ±50 rad/s | ±36 Nm | 0..5000 | 0..100 |
| EL05（EduLite-05） | ±4π rad | ±50 rad/s | ±6 Nm | 0..500 | 0..5 |

## インストール

対応するROS 2 workspaceの`src`内へ配置し、依存packageを解決してbuildします。

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to robstride_ros2 --symlink-install
source install/setup.bash
```

exampleを起動する前にSocketCAN interfaceを準備します。RobStride private protocolは1 Mbit/s CANとextended frameを使用します。

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
```

## exampleの起動

exampleはEL05 profile、CAN ID 1のモーターを使用し、position controllerを起動します。

```bash
ros2 launch robstride_examples robstride_example.launch.py interface:=can0
```

読み込まれたHardwareとfeedbackを確認します。

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states
```

位置をrad単位で指定します。

```bash
ros2 topic pub --rate 20 \
  /robstride_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.2]}"
```

## Hardware parameters

`ros2_control` descriptionの`<hardware>`内に記述します。

| parameter | default | 説明 |
|---|---:|---|
| `host_can_id` | `253` | host CAN ID。範囲は`0..255` |
| `can_tx_topic` | `to_can_bus` | `ros2_socketcan`へ送るframeのtopic |
| `can_rx_topic` | `from_can_bus` | `ros2_socketcan`から受け取るframeのtopic |
| `can_rx_qos_depth` | `32` | reliable・volatileなfeedback QoS depth。多数のモーターを使う場合は増加を検討 |
| `feedback_timeout_ms` | `3000` | feedbackを受信できない状態でERRORを返すまでの時間 |
| `fail_on_feedback_timeout` | `true` | feedback timeout時にHardwareを停止 |
| `run_mode_recovery_timeout_ms` | `500` | active中のモーターがRunへ復帰するまで待つ時間 |
| `run_mode_recovery_retry_interval_ms` | `100` | 自動有効化を再試行する最小間隔 |
| `clear_faults_on_activate` | `true` | activate時にモーターのfaultをclear |
| `set_zero_on_activate` | `false` | activate時の現在位置を機械ゼロに設定 |
| `shutdown_stop_repetitions` | `3` | 終了時にゼロ指令と停止指令を送る回数 |
| `shutdown_stop_interval_ms` | `20` | ゼロ指令と停止指令の送信セット間隔 |
| `shutdown_confirmation_timeout_ms` | `300` | Reset modeのfeedbackを待つ時間。`0`で確認を無効化 |
| `startup_connection_timeout_ms` | `3000` | `ros2_socketcan`のtopic endpointを待つ時間 |
| `startup_confirmation_timeout_ms` | `500` | parameterと有効化確認の1試行あたりのtimeout |
| `startup_retries` | `3` | 起動時のparameter設定と有効化の最大試行回数 |

Hardwareがactiveの間は、各モーターの動作状態を監視します。モーターが意図せずRun以外へ移行するとWARNを出力し、自動的に再度有効化します。Runへの復帰を確認すると指令送信を再開します。`run_mode_recovery_timeout_ms`以内に復帰しない場合はHardwareがERRORを返し、すべてのモーターを停止します。

既定値を使う最小構成は次のとおりです。

```xml
<hardware>
  <plugin>robstride_ros2/RobStrideSystem</plugin>
</hardware>
```

## Joint parameters

各`<joint>`にモーター設定を記述します。

| parameter | default | 説明 |
|---|---:|---|
| `can_id` | 必須 | 一意なmotor CAN ID。範囲は`1..255` |
| `can_timeout_ticks` | 必須 | motor側CAN watchdog。非ゼロ必須。20,000 ticksで1秒 |
| `position_min/max` | 必須 | CAN positionのencode・decode範囲 `[rad]` |
| `velocity_min/max` | 必須 | CAN velocityのencode・decode範囲 `[rad/s]` |
| `effort_min/max` | 必須 | ROS effort指令のclamp範囲 `[Nm]` |
| `effort_wire_min/max` | effort制限と同じ | CAN effortのencode・decode範囲 `[Nm]` |
| `kp_max` / `kd_max` | 必須 | gainのencode上限 |
| `kp` / `kd` | 必須 | position・velocity command interfaceで使用するgain |
| `direction` | `1` | jointの方向。`1`または`-1` |
| `gear_ratio` | `1.0` | ROS joint回転に対する追加のprotocol側回転比 |
| `position_offset` | `0.0` | ROS joint位置offset `[rad]` |

`gear_ratio`は、robot側に追加されたtransmissionの変換比です。アクチュエータ内蔵減速機の減速比ではありません。private protocolの角度がROS jointとして使用する出力軸角度を表す場合は、`1.0`のまま使用してください。

各jointは3つのcommand interfaceと、position・velocity・effortのstate interfaceをすべてexportする必要があります。`temperature`と`fault`は省略できます。

```xml
<command_interface name="position"/>
<command_interface name="velocity"/>
<command_interface name="effort"/>

<state_interface name="position"/>
<state_interface name="velocity"/>
<state_interface name="effort"/>
<state_interface name="temperature"/>
<state_interface name="fault"/>
```

## 型番別Xacro macro

定義は[`robstride_examples/description/robstride_motor_profiles.xacro`](robstride_examples/description/robstride_motor_profiles.xacro)にあります。

| 型番 | macro | default watchdog ticks |
|---|---|---:|
| RS00 | `robstride_rs00_params` | `4000` |
| RS01 | `robstride_rs01_params` | `4000` |
| RS02 | `robstride_rs02_params` | `4000` |
| RS03 | `robstride_rs03_params` | `4000` |
| RS04 | `robstride_rs04_params` | `4000` |
| RS05 | `robstride_rs05_params` | `4000` |
| RS06 | `robstride_rs06_params` | `4000` |
| EL05 | `robstride_edulite05_params` | `4000` |

使用例：

```xml
<xacro:include filename="$(find robstride_examples)/description/robstride_motor_profiles.xacro"/>

<joint name="wheel_joint_1">
  <xacro:robstride_edulite05_params
    can_id="1" direction="1" kp="20.0" kd="0.8"/>
  <xacro:robstride_joint_interfaces/>
</joint>
```

`4000`は本packageの安全上の既定値であり、モーターのfactory defaultではありません。公式manualでは`0`がfactory defaultでwatchdog無効、`20000` ticksが1秒です。本packageはhostからの指令が失われた場合にモーターをResetへ移行させるため、`0`を受け付けません。

## Controllerとcommand mode

| command interface | controller type | 単位 | モーター指令 |
|---|---|---|---|
| position | `position_controllers/JointGroupPositionController` | rad | position、Kp、Kd |
| velocity | `velocity_controllers/JointGroupVelocityController` | rad/s | velocity、Kd。Kpは0 |
| effort | `effort_controllers/JointGroupEffortController` | Nm | torque feed-forward。KpとKdは0 |

HardwareはactiveなROS command interfaceをモーター指令へ変換します。1つのjointで複数のcommand interfaceを同時に使用することはできませんが、別々のjointでは異なるmodeを同時に使用できます。

position controllerからvelocity controllerへ切り替える例：

```bash
ros2 run controller_manager spawner robstride_velocity_controller \
  --inactive --controller-manager /controller_manager

ros2 control switch_controllers \
  --deactivate robstride_position_controller \
  --activate robstride_velocity_controller \
  --controller-manager /controller_manager

ros2 topic pub --rate 20 \
  /robstride_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [1.0]}"
```

effort指令では同様に`robstride_effort_controller`を使用します。

## 複数モーター

モーターごとに一意なjoint名とCAN IDを設定し、controller設定の`joints`でグループ分けします。例えばCAN ID 1～4を速度制御、5～6を位置制御にできます。

```yaml
wheel_velocity_controller:
  ros__parameters:
    joints: [joint_1, joint_2, joint_3, joint_4]

steering_position_controller:
  ros__parameters:
    joints: [joint_5, joint_6]
```

```bash
ros2 topic pub --rate 20 \
  /wheel_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [1.0, 1.0, 1.0, 1.0]}"

ros2 topic pub --rate 20 \
  /steering_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.2, -0.2]}"
```

## Timeoutと停止動作

| 設定 | 動作 |
|---|---|
| `can_timeout_ticks` | hostからの指令が途絶えるとモーターがReset modeへ移行 |
| `feedback_timeout_ms` | モーターからのfeedbackが途絶えるとHardwareがERRORを返す |
| `shutdown_stop_repetitions` | ゼロ指令と停止指令を繰り返し送信 |
| `shutdown_confirmation_timeout_ms` | Reset modeのfeedbackを待機 |

deactivate、shutdown、error、またはactive中のdestructionでは、すべてのモーターへゼロ指令に続いて停止指令を送ります。ROS transportが指令を配送できなかった場合は、設定済みのmotor側CAN watchdogが最終的な停止手段になります。

## License

[MIT](LICENSE)

## 参考資料

- [RobStride Product Information revision `6ad12f5`（2026年7月14日）](https://github.com/RobStride/Product_Information/tree/6ad12f50006273b7ea4eea88980f927d97c22f0d)
- [`ros2_socketcan`](https://github.com/autowarefoundation/ros2_socketcan)
