# single_motor_ctrl

基于仓库内 `linux_sdk` 的单电机 CLI 调试控制工程。

## 项目结构

```text
.
├── CMakeLists.txt
├── configs/
│   └── motor_cli.conf
├── include/
│   ├── app/
│   ├── common/
│   ├── control/
│   ├── device/
│   ├── domain/
│   └── driver/
├── src/
│   ├── app/
│   ├── common/
│   ├── control/
│   ├── device/
│   ├── domain/
│   └── driver/
└── linux_sdk/
```

## 构建

```bash
cmake -S . -B build
cmake --build build -j
```

## CLI 用法

```bash
./build/motor_cli --config configs/motor_cli.conf info
./build/motor_cli --config configs/motor_cli.conf enable
./build/motor_cli --config configs/motor_cli.conf mode position
./build/motor_cli --config configs/motor_cli.conf set-pos 1.0
./build/motor_cli --config configs/motor_cli.conf set-vel 1.5
./build/motor_cli --config configs/motor_cli.conf set-cur 0.8
./build/motor_cli --config configs/motor_cli.conf set-pd-target 1.0 0.5 0.2
./build/motor_cli --config configs/motor_cli.conf monitor 20 100 fast
./build/motor_cli --config configs/motor_cli.conf shell
```

## 支持的指令

- `info`：查询电机型号、电机固件、主板固件。
- `enable` / `disable`：直接下发使能/失能控制字。
- `clear-fault`：清故障。
- `zero`：位置置零。
- `calibrate`：编码器校准。
- `mode <...>`：切换控制模式。
- `set-pos <rad>`：位置模式下发位置指令。
- `set-vel <rad_s>`：速度模式下发转速指令。
- `set-cur <amp>`：电流模式下发电流指令。
- `set-pd-target <pos> <vel> <cur>`：PD 模式下发组合指令。
- `strong-drag <source> <voltage>`：强拖调试。
- `monitor [count] [period_ms] [fast]`：周期读取遥测。
- `shell`：进入交互式调试终端，持续连接主板并逐条下发指令。
