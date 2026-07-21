# robstride_ros2

RobStride Private Protocol用のROS 2 `ros2_control` Hardware Componentです。Humble、Jazzy、Kilted、Lyrical、Rollingに対応し、CAN通信には`ros2_socketcan`の`can_msgs/msg/Frame`トピックを使用します。

## 内部構成

| component | 責務 |
|---|---|
| `RobStrideSystem` | `ros2_control` lifecycle、joint状態、command interface、起動・停止 |
| `CanTransport` | `ros2_socketcan` topic、QoS、executor、最新Type 1指令と復帰要求の送信 |
| `protocol` | RobStride拡張CAN IDとpayloadのencode/decode |
| `command_mode` | jointごとのposition、velocity、effort claim検証 |

jointの実行時情報は、公開state、feedback、command、claim状態、feedback状態、parameter応答状態に分けて管理します。state mutexを保持したままDDS publishは行いません。


## インストール

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select robstride_ros2 --symlink-install
source install/setup.bash
```

## 起動

```bash
ros2 launch robstride_ros2 robstride_example.launch.py interface:=can0
```

確認：

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states
```

位置指定：

```bash
ros2 topic pub --rate 20 \
  /robstride_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.2]}"
```

## Hardware parameters

`<ros2_control><hardware>`内に記述します。

| parameter | default | 仕様 |
|---|---:|---|
| `host_can_id` | `253` | host CAN ID。`0..255` |
| `can_tx_topic` | `to_can_bus` | `ros2_socketcan`への送信topic |
| `can_rx_topic` | `from_can_bus` | `ros2_socketcan`からの受信topic |
| `can_rx_qos_depth` | `32` | 複数モーターのfeedback受信用QoSのdepth |
| `feedback_timeout_ms` | `3000` | Type 2が途絶えてからHardwareをERRORにするまでの時間 |
| `fail_on_feedback_timeout` | `true` | feedback timeout時にHardwareをERRORへ遷移 |
| `run_mode_recovery_timeout_ms` | `500` | active中にRunから外れたモーターの自動復帰を待つ時間 |
| `run_mode_recovery_retry_interval_ms` | `100` | Type 3 enableを自動再送する最小間隔 |
| `clear_faults_on_activate` | `true` | activate時にfault clearを送信 |
| `set_zero_on_activate` | `false` | activate時に現在位置を機械ゼロへ設定 |
| `shutdown_stop_repetitions` | `3` | 終了時のゼロ指令＋Type 4送信回数 |
| `shutdown_stop_interval_ms` | `20` | Type 4再送間隔 |
| `shutdown_confirmation_timeout_ms` | `300` | Type 2 Reset応答の待機時間。`0`で確認無効 |
| `startup_connection_timeout_ms` | `3000` | `ros2_socketcan` publisher/subscriber接続待ち時間 |
| `startup_confirmation_timeout_ms` | `500` | parameter読み戻し・Run応答の1回あたり待機時間 |
| `startup_retries` | `3` | 起動parameter設定・enableの再試行回数 |

旧名の`can_qos_depth`も`can_rx_qos_depth`の非推奨aliasとして読み込みます。
新しい設定では`can_rx_qos_depth`を使用してください。

全送信frameは単一transport workerと単一DDS DataWriterを通るため、transactionと
運転指令は一つのpublish順序を持ちます。DDSへ渡す前はモーターごとに未送信の最新Type 1を
1件だけ保持します。DDSへ渡した後のbufferingはRMWと`ros2_socketcan`の管理対象であり、
そのqueue内でモーターごとに最新1件となることまでは保証しません。

Hardwareがactiveの間は、Type 2応答のmodeも監視します。モーターがResetなどの
Run以外へ予期せず移行した場合はWARNを表示し、update loop内でDDS publishを行わずに
Type 3 enableを再送します。Run応答を受信すれば運転を継続し、
`run_mode_recovery_timeout_ms`以内にRunへ戻らなければ`read()`がERRORを返します。
その後はcontroller managerがHardwareの`on_error()`を呼び、全モーターを停止します。
復帰用Type 3もtransport threadから送るため、`read()`はDDS publishを行いません。
復帰中はそのモーターのType 1を保留し、Run応答後に最新指令を再開します。
deactivate時はactive世代番号を更新して未送信frameを破棄するため、workerが取り出し済みの
古いframeも次回activate後に送信されません。transport終了時はworkerを止める前に
local queue内のlifecycle transactionを送出し、未送信の速度0・Type 4を破棄しません。

設定例：

```xml
<hardware>
  <plugin>robstride_ros2/RobStrideSystem</plugin>
  <param name="host_can_id">253</param>
  <param name="can_tx_topic">to_can_bus</param>
  <param name="can_rx_topic">from_can_bus</param>
  <param name="feedback_timeout_ms">3000</param>
  <param name="fail_on_feedback_timeout">true</param>
  <param name="run_mode_recovery_timeout_ms">500</param>
  <param name="run_mode_recovery_retry_interval_ms">100</param>
  <param name="clear_faults_on_activate">true</param>
  <param name="set_zero_on_activate">false</param>
  <param name="shutdown_stop_repetitions">3</param>
  <param name="shutdown_stop_interval_ms">20</param>
  <param name="shutdown_confirmation_timeout_ms">300</param>
  <param name="startup_connection_timeout_ms">3000</param>
  <param name="startup_confirmation_timeout_ms">500</param>
  <param name="startup_retries">3</param>
</hardware>
```

## Joint parameters

各`<joint>`に設定します。

| parameter | default | 仕様 |
|---|---:|---|
| `can_id` | 必須 | motor CAN ID。`1..255`、重複不可 |
| `can_timeout_ticks` | 必須 | motor側CAN watchdog。非ゼロ必須 |
| `position_min/max` | 必須 | Type 1/2のmotor-axis position wire範囲 `[rad]` |
| `velocity_min/max` | 必須 | Type 1/2のmotor-axis velocity wire範囲 `[rad/s]` |
| `effort_min/max` | 必須 | effort指令のclamp範囲 `[Nm]` |
| `effort_wire_min/max` | `effort_min/max` | Type 1/2のeffort wire範囲 `[Nm]` |
| `kp_max` / `kd_max` | 必須 | Type 1のgain wire上限 |
| `kp` / `kd` | 必須 | position/velocity制御に使用するgain |
| `direction` | `1` | `1`または`-1` |
| `gear_ratio` | `1.0` | motor回転数 / joint回転数。正数 |
| `position_offset` | `0.0` | ROS joint位置オフセット `[rad]` |



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

`temperature`と`fault`のみ省略可能です。

## 型番別xacro macro

[`description/robstride_motor_profiles.xacro`](description/robstride_motor_profiles.xacro)に定義されています。

| 型番 | macro | gear ratio | watchdog ticks |
|---|---|---:|---:|---|
| RS00 | `robstride_rs00_params` | `1.0` | `4000` |
| RS02 | `robstride_rs02_params` | `1.0` | `4000` |
| RS03 | `robstride_rs03_params` | `1.0` | `1200` |
| RS04 | `robstride_rs04_params` | `1.0` | `2400` |
| RS05 | `robstride_rs05_params` | `1.0` | `4000` |
| RS06 | `robstride_rs06_params` | `1.0` | `4000` |
| EduLite05 | `robstride_edulite05_params` | `1.0` | `4000` |

使用例：

```xml
<xacro:include filename="$(find robstride_ros2)/description/robstride_motor_profiles.xacro"/>

<joint name="wheel_joint_1">
  <xacro:robstride_edulite05_params
    can_id="1" direction="1" kp="20.0" kd="0.8"/>
  <xacro:robstride_joint_interfaces/>
</joint>
```


## Controller

| 制御 | controller type | command単位 | Type 1設定 |
|---|---|---|---|
| position | `position_controllers/JointGroupPositionController` | rad | position + Kp + Kd |
| velocity | `velocity_controllers/JointGroupVelocityController` | rad/s | velocity + Kd、Kp=0 |
| effort | `effort_controllers/JointGroupEffortController` | Nm | effort feed-forward、Kp=Kd=0 |

同じjointで複数のcontrollerを同時にactiveにすることはできません。

速度controllerへ切り替える例：

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

effort controllerへ切り替える場合は、同様に`robstride_effort_controller`をload・activateします。

## 複数モーター

モーターごとに個別jointとCAN IDを定義し、controllerの`joints`でグループ分けします。

CAN ID 1～4を速度、5～6を位置制御するcontroller設定例：

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
| `can_timeout_ticks` | CAN指令が途絶えるとモーター自身がResetへ移行 |
| `feedback_timeout_ms` | Type 2が途絶えるとHardwareをERRORへ遷移し停止処理を実行 |
| `shutdown_stop_repetitions` | 終了時にゼロ指令＋Type 4を再送 |
| `shutdown_confirmation_timeout_ms` | Type 2のReset応答を待機 |

`20000 ticks = 1秒`です。既定の`4000`は約200msです。`can_timeout_ticks`はactivate時に毎回モーターへ設定されます。

## License

MIT License。詳細は[`LICENSE`](LICENSE)を参照してください。
