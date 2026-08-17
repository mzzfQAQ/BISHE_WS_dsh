# MuJoCo 仿真：fishbot + 6-DOF 机械臂

将 ROS 2 仿真中的 fishbot 巡检机器人（含 6-DOF 机械臂）移植到 [MuJoCo](https://mujoco.org/) 物理引擎。

## 文件说明

| 文件 | 说明 |
| --- | --- |
| `fishbot_arm.xml` | MJCF 模型：差速底盘（2 轮 + 2 万向轮）+ 6R 机械臂（基座旋转/肩/肘/腕滚转/腕俯仰/末端旋转）+ 传感器 |
| `viewer_fishbot.py` | 交互查看器（自动演示模式）：机械臂循环 home→up→front，小车慢速前进，鼠标自由视角 |
| `demo_fishbot.py` | 离屏渲染演示：小车前进 + 机械臂摆动，保存 mp4 视频 |

## 模型对应关系（与 ROS 2 URDF 一致）

- 底盘：`base_link` 圆柱 r=0.14 h=0.16，质量 2.5 kg
- 轮距 0.28 m，轮子 r=0.032（MJCF 用扭矩控制，摩擦 3.0）
- 机械臂 6 关节限位与 URDF 一致（±180°/肩 ±115°/肘 -143°~86° 等）
- 关节控制：轮子 = `motor`（扭矩），机械臂 = `position`（kp=40 + 阻尼 1.0）

## 使用

```bash
# 交互查看（GUI）
python3 viewer_fishbot.py

# 生成演示视频
python3 demo_fishbot.py fishbot_arm_demo.mp4
```

## 依赖

```bash
pip3 install mujoco imageio imageio[ffmpeg]
```

## 已知调整（相对 URDF）

- `down` 姿态在 MuJoCo 中会撞地（基座高仅 0.19 m），演示改为 `front`（水平前伸）姿态
- 轮子扭矩 1.0~2.0 Nm 可稳定行驶；阻尼仅加在机械臂关节（避免拖慢轮子）
