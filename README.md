# robstride_ros2

RobStrideのprivate CAN protocolを`ros2_control`のSystem Hardware Componentとして扱うROS 2 Humbleパッケージです。CANデバイスは直接開かず、`ros2_socketcan`のClassic CAN topicを利用します。

```
controller -> ros2_control -> RobStrideSystem -> to_can_bus -> ros2_socketcan -> can0
controller <- ros2_control <- RobStrideSystem <- from_can_bus <- ros2_socketcan <- can0
```

## 対応内容

- 1つのSystem pluginで複数モーターを制御
- RobStride Type 1（MIT/運控モード）によるposition・velocity・effort指令
- Type 2 feedbackからposition・velocity・effort・temperature・faultを復号
- lifecycle activate時にfault clear、run mode 0、enableを送信
- deactivate時にゼロトルク指令とType 4 stopを送信
- feedback watchdog、NaN防止、指令clamp、CAN ID重複検査
- `direction`と`position_offset`によるjoint座標変換

## 重要な前提

モーターはRobStride private protocol、CAN 2.0、29-bit拡張frame、通常1 Mbpsに設定してください。`ros2_socketcan`はLinux SocketCAN用なので、このmacOSワークスペースではprotocol/testの検証のみ可能で、実CAN試験はLinuxで行います。

RobStrideは機種ごとにwire scalingが異なります。安全上、各jointに以下を明示します。

- `position_min`, `position_max` [rad]
- `velocity_min`, `velocity_max` [rad/s]
- `effort_min`, `effort_max` [Nm]
- `kp_max`, `kd_max`, および実際に使う`kp`, `kd`

同梱xacroはRS05向けの例です。RS02なら公式manualの代表値はposition ±4π、velocity ±44 rad/s、effort ±17 Nm、Kp 0..500、Kd 0..5です。実機の型番とfirmwareのmanualを必ず優先してください。

## hardware parameters

| parameter | default | 内容 |
|---|---:|---|
| `host_can_id` | `253` | host側CAN ID |
| `can_tx_topic` | `to_can_bus` | ros2_socketcan送信topic |
| `can_rx_topic` | `from_can_bus` | ros2_socketcan受信topic |
| `can_qos_depth` | `500` | ros2_socketcan Humbleと一致するReliable/Volatile depth |
| `feedback_timeout_ms` | `500` | feedback watchdog |
| `fail_on_feedback_timeout` | `true` | timeout時にreadをERRORにする |
| `clear_faults_on_activate` | `true` | activate時にType 4 `data[0]=1`を送る |
| `set_zero_on_activate` | `false` | activate時に現在位置を機械ゼロにする（危険なので既定off） |

各jointは`can_id`、上記scaling/gain、任意の`direction`（`1`または`-1`）と`position_offset`を持ちます。command interfaceはposition/velocity/effortを3つとも宣言してください。controllerがclaimしたinterfaceだけが有効になり、未claimの速度・effortは0、positionは現在feedbackを使います。

## buildと起動

```bash
pixi run colcon build --packages-select can_msgs robstride_ros2 --symlink-install
source install/setup.bash
ros2 launch robstride_ros2 robstride_example.launch.py interface:=can0
```

`ros2_socketcan` 1.2系で独自topic名を使う場合はbridge側をROS remapし、xacro側の`can_tx_topic`/`can_rx_topic`も同じ名前にします。1.3系ではbridge launchの`to_can_bus_topic`/`from_can_bus_topic`引数も利用できます。

position controllerの例:

```bash
# position controllerはlaunch時に起動済み。単位はrad。
ros2 topic pub --rate 20 /robstride_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.5]}"
# 終了はCtrl-C。position指令は最後の値を保持する。
```

## 起動後の速度・トルク制御

`robstride_example.launch.py`はposition controllerを起動します。速度またはトルクへ切り替える場合は、対象controllerをspawnしてから切り替えます。同じjointのcommand interfaceを複数controllerで同時claimしないでください。

```bash
# 起動確認
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states

# 速度制御 [rad/s]
ros2 run controller_manager spawner robstride_velocity_controller \
  --controller-manager /controller_manager
ros2 control switch_controllers \
  --deactivate robstride_position_controller \
  --activate robstride_velocity_controller \
  --controller-manager /controller_manager
ros2 topic pub --rate 20 /robstride_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [1.0]}"
```

速度指令を止めるときは、まず速度0を送ってからcontrollerを停止します。

```bash
ros2 topic pub --once /robstride_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0]}"
ros2 control switch_controllers \
  --deactivate robstride_velocity_controller \
  --activate robstride_position_controller \
  --controller-manager /controller_manager
```

トルク制御はType 1 MIT/運控モードのトルクfeed-forwardです。単位は[Nm]です。

```bash
ros2 run controller_manager spawner robstride_effort_controller \
  --controller-manager /controller_manager
ros2 control switch_controllers \
  --deactivate robstride_position_controller \
  --activate robstride_effort_controller \
  --controller-manager /controller_manager
ros2 topic pub --rate 20 /robstride_effort_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.2]}"
```

トルクを停止する場合:

```bash
ros2 topic pub --once /robstride_effort_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0]}"
ros2 control switch_controllers \
  --deactivate robstride_effort_controller \
  --activate robstride_position_controller \
  --controller-manager /controller_manager
```

`ros2 topic pub --rate`はCtrl-Cで終了してください。最後にcontroller managerを終了すると、Hardware Componentがゼロ指令とType 4 stopを送ります。

## 安全上の注意

最初はモーターを無負荷・低電圧・非常停止可能な状態に置き、低いKp/Kdから確認してください。Type 2の通常positionは±4πのwire範囲であり、多回転絶対位置を保証しません。`set_zero_on_activate=true`は起動のたびに機械ゼロを変更するため、用途を理解した場合だけ有効にしてください。
