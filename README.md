
## DynamicController

### DynamicControllerの起動方法

```
ros2 launch ethercat_easycat easycat_dynamic.launch.py 
```

### DynamicControllerの使い方

- 初期化
```
ros2 topic pub /dynamic_controller/controller_state std_msgs/msg/Int8 "data: 0" -t 1
```

- ゼロ位置に移動（Homing）
```
ros2 topic pub /dynamic_controller/controller_state std_msgs/msg/Int8 "data: 3" -t 1
```

- Sin波デモ動作
```
ros2 topic pub /dynamic_controller/controller_state std_msgs/msg/Int8 "data: 4" -t 1
```

- 動作停止
```
ros2 topic pub /dynamic_controller/controller_state std_msgs/msg/Int8 "data: 6" -t 1
```


- 制御モードを電流制御に変更
```
ros2 topic pub /dynamic_controller/control_mode std_msgs/msg/Int8 "data: 1" -t 1
```
- 制御モードを速度制御に変更
```
ros2 topic pub /dynamic_controller/control_mode std_msgs/msg/Int8 "data: 2" -t 1
```
- 制御モードを位置制御に変更
```
ros2 topic pub /dynamic_controller/control_mode std_msgs/msg/Int8 "data: 3" -t 1
```

### 操作例

- 位置制御でデモ動作をする場合
1. EtherCAT-CANFD変換モジュールの電源ON
2. モーター電源ON
3. DynamicControllerを起動
4. コンソールにIDLINGが出力されるまでまつ
5. Sin波デモ動作（デフォルトは位置制御）
6. 動作停止
7. ゼロ位置に移動（Homing）

- 速度制御でデモ動作をする場合
1. EtherCAT-CANFD変換モジュールの電源ON
2. モーター電源ON
3. DynamicControllerを起動
4. コンソールにIDLINGが出力されるまでまつ
5. 制御モードを*速度制御*に変更
6. コンソールにIDLINGが出力されるまでまつ
7. Sin波デモ動作
8. 動作停止
9. ゼロ位置に移動（Homing）

- 電流制御でデモ動作をする場合
1. EtherCAT-CANFD変換モジュールの電源ON
2. モーター電源ON
3. DynamicControllerを起動
4. コンソールにIDLINGが出力されるまでまつ
5. 制御モードを*電流制御*に変更
6. コンソールにIDLINGが出力されるまでまつ
7. Sin波デモ動作
8. 動作停止
9. ゼロ位置に移動（Homing）



