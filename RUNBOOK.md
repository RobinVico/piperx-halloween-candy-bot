# SO-101 → Piper X 遥操作 · 使用手册

> 最后更新：2026-10-31。SO-101（主臂，COM3）通过 candleLight USB-CAN 控制 AgileX Piper X（固件 S-V1.8-9），Windows 11。

## 快速开始（双击文件夹里的 .bat）

| 文件 | 作用 | 真机会动吗 |
|---|---|---|
| `1_check_connections.bat` | 检查 CAN、Piper 状态、SO-101 | 不会 |
| `2_sim_teleop.bat` | SO-101 控制**仿真** Piper X | 不会 |
| `3_dry_run_panel.bat` | 面板 + 仿真，连接真机但**不使能** | 不会 |
| `4_LIVE_teleop_panel.bat` | **真机遥操作**（面板里点"使能"并确认后才动） | **会** |
| `5_mirror_real_arm.bat` | 仿真里显示真机实际姿态 | 不会 |

## 每次开始前
1. Piper X 接 24 V 电源、CAN 线插到机械臂；SO-101 USB 直插电脑（COM3）。
2. **退出 Duet Display**（托盘图标右键 → Quit），它可能占用 CAN 线。
3. 机械臂处于**折叠休息姿势**（夹爪朝上，J4 ≈ -90°）—— "回休息位" 会回到程序启动时的姿势。
4. 周围清空，手放在 Piper 电源开关旁。

## 真机操作流程（`4_LIVE_teleop_panel.bat`）
1. 弹出两个窗口：**紧急控制面板**（置顶）和 **MuJoCo 仿真**。
2. **对齐**：按面板"对齐差"调 SO-101，所有关节 < 20°。要点：**SO-101 手腕往上折，让夹爪朝上**（和 Piper 休息姿势一样）。仿真窗口此时显示 SO-101 对应的目标姿态。
   - 底座朝向不一致：把两台底座摆成同一朝向 → 点紫色 **"校准底座 (J1)"**（会保存，下次自动用）。
3. 状态条变绿 → 点 **"使能并开始跟随"** → 确认。开头 2 秒很慢，之后最高 45°/s。
4. 结束：SO-101 折回 → Piper 回到折叠姿势附近 → 点 **"回休息位并失能"**（或按 H）。

| 面板按钮 / 按键 | 作用 |
|---|---|
| 红色 **急停** / `Esc` | 立即急停（Piper 可能缓慢下沉，托住它；恢复需断电重启） |
| 暂停/继续 / `空格` | 原地保持 / 继续跟随 |
| 回休息位并失能 / `H` | 20°/s 回到启动姿势后失能（**推荐的结束方式**） |
| 停止跟随（保持） | 停止跟随，机械臂保持当前位置（仍使能） |
| 速度滑块 | 实时调整速度上限 5–45°/s |

## 自动保护
- 默认 dry run；`--live` + 面板确认才使能
- 必须先对齐；使能后从真机当前位置起步，无跳变；2 秒内速度从 10% 爬到 100%
- 限速 45°/s、限加速度 200°/s²、Piper 内部速度 50%
- 关节限位 = 官方 URDF 收 5°（休息姿势的硬限位处例外）
- SO-101 读数异常跳变自动丢弃；SO-101 断线 0.2 s → 原地保持
- 指令与实际相差 > 25° 持续 0.5 s（被挡/碰撞）→ 自动急停
- 程序出错 → 自动急停

## 关节对应（已在仿真和真机核对过方向）
| SO-101 | Piper X |
|---|---|
| shoulder_pan（底座，方向取反） | J1 底座 |
| shoulder_lift | J2 肩 |
| elbow_flex | J3 肘 |
| wrist_flex（手腕上下） | **J4（Piper X 的 J4 是上下俯仰）** |
| — | J5 左右，固定不动 |
| wrist_roll | J6 自转 |
| gripper 10–100% | 夹爪 0–70 mm |

映射参数保存在 `teleop/mapping_config.json`（底座零点 -19.2 已校准）。

## 常见问题
| 现象 | 处理 |
|---|---|
| CAN "access denied" / "adapter not accessible" | 退出 Duet Display；**拔插 CAN 的 USB 线**；确认没有别的 python 窗口还开着 |
| `Could not connect on port 'COM3'` | 等几秒再试（程序会自动重试 4 次）；拔插 SO-101 USB；确认 SO-101 舵机电源接着 |
| CAN 只看到大量 0x2A1 | 用了 listen-only 模式；`can_probe.py` 已改为普通模式 |
| 程序退出时卡住 | 已修（SDK 断开 bug），若再出现关窗口即可 |
| SO-101 校准丢失 | 备份在 `config/lerobot_calibration/`，复制回 `%USERPROFILE%\.cache\huggingface\lerobot\calibration\teleoperators\so_leader\my_leader.json` |

## 重装环境（换电脑或 .venv 损坏时）
```
py -3.12 -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements-lock.txt
git clone --depth 1 https://github.com/agilexrobotics/agx_arm_urdf.git third_party/agx_arm_urdf   # 只在要重建仿真模型时需要
```

## ⚠️ 不要做
- 不要调用 `ResetPiper()` / `EmergencyStop(0x02)`：机械臂会**立刻断电掉落**。
- 不要在机械臂抬起时失能（会软掉下落）；用 "回休息位并失能"。
- 不要同时开两个程序（CAN 线和 COM3 只能被一个程序占用）。

## 文件说明
- `teleop/teleop_real.py` 真机主程序 + 紧急控制面板
- `teleop/mapping.py` 关节映射 + 安全/平滑滤波（仿真和真机共用）
- `teleop/piper_driver.py` Piper X 驱动（默认只读，`allow_motion=True` 才能动）
- `teleop/so101_reader.py` SO-101 只读读取 + 后台线程
- `teleop/can_setup.py` CAN 设置（candle 接口，1 Mbit/s）
- `teleop/timing.py` Windows 高精度定时
- `sim/` 仿真模型（`build_model.py` 从官方 URDF 生成）、仿真遥操作、真机镜像
- `tools/` 检查工具
