# MuJoCo 仿真：fishbot + 6-DOF 机械臂

将 ROS 2 仿真中的 fishbot 巡检机器人（四轮长方体底盘 + Franka Panda 七自由度机械臂）移植到 [MuJoCo](https://mujoco.org/) 物理引擎。

## 文件说明

| 文件 | 说明 |
| --- | --- |
| `fishbot_arm.xml` | MJCF 模型：四轮长方体底盘（0.70×0.50×0.22 m）+ Franka Panda 7-DOF 机械臂（官方 mesh + 惯量） |
| `viewer_fishbot.py` | 交互查看器（自动演示模式）：Panda 循环 ready→up→wave，小车慢速前进，鼠标自由视角 |
| `demo_fishbot.py` | 离屏渲染演示：小车前进 + 机械臂摆动，保存 mp4 视频 |

## 模型对应关系（与 ROS 2 URDF 一致）

- 底盘：`base_link` 长方体 0.70×0.50×0.22 m，质量 8 kg
- 四轮驱动（轮距 0.48，轴距 0.40，轮 r=0.05），扭矩控制 + 摩擦 3.0
- 机械臂：Franka Panda 7 关节，限位/惯量取自官方 panda.urdf.xacro
- 关节控制：轮子 = `motor`（扭矩），Panda = `position`（kp=40 + 阻尼 1.0）

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

- 惯量用 `diaginertia`（主轴项），MuJoCo 3.11 对 URDF 的完整非对角惯量存在解析兼容问题
- 演示姿态：ready / up / wave（避开 Panda 自碰撞的折叠零位）
- 四轮扭矩 1.0~2.0 Nm 可稳定行驶；阻尼仅加在机械臂关节（避免拖慢轮子）
