#!/usr/bin/env python3
"""MuJoCo 交互查看器：fishbot（四轮底盘）+ Franka Panda 机械臂（自动演示模式）

- Panda 自动循环执行 ready -> up -> wave 姿态
- 小车自动慢速前进
- 鼠标操作：左键旋转视角 | 滚轮缩放 | 右键平移
- 关闭窗口即退出
"""
import time
import numpy as np
import mujoco
from mujoco import viewer

XML = "fishbot_arm.xml"

# Panda 预设姿态（对应 SRDF：ready / up / wave）
ARM_POSES = {
    "ready": np.array([0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]),
    "up":    np.array([0.0, -1.4, 0.0, -2.2, 0.0, 1.2, 0.0]),
    "wave":  np.array([0.0, -1.2, -0.5, -1.8, 0.6, 1.0, 1.5]),
}
POSE_SEQ = ["ready", "up", "wave", "ready"]
POSE_DUR = 3.0        # 每个姿态持续时间（秒）
TRANSITION = 1.0      # 姿态过渡时间（秒）
WHEEL_TORQUE = 1.5    # 四轮扭矩（Nm）
N_WHEEL = 4           # 轮子执行器数
N_ARM = 7             # Panda 关节数


def main():
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)

    # Panda 初始 ready 姿态（franka 官方初始，避免折叠零位自碰撞）
    d.qpos[11:18] = np.array([0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854])
    mujoco.mj_forward(m, d)

    v = viewer.launch_passive(m, d, show_left_ui=True, show_right_ui=False)

    print("=" * 50)
    print("  MuJoCo fishbot（四轮底盘）+ Franka Panda（自动演示）")
    print("  Panda 循环: ready -> up -> wave")
    print("  鼠标: 左键旋转 | 滚轮缩放 | 右键平移")
    print("  关闭窗口退出")
    print("=" * 50)

    idx = 0
    elapsed = 0.0
    prev = ARM_POSES[POSE_SEQ[0]].copy()
    cur = ARM_POSES[POSE_SEQ[0]].copy()

    try:
        while v.is_running():
            if elapsed >= POSE_DUR:
                prev = ARM_POSES[POSE_SEQ[idx]].copy()
                idx = (idx + 1) % len(POSE_SEQ)
                cur = ARM_POSES[POSE_SEQ[idx]].copy()
                elapsed = 0.0

            # 平滑过渡
            if elapsed < TRANSITION:
                alpha = elapsed / TRANSITION
                arm_target = prev * (1 - alpha) + cur * alpha
            else:
                arm_target = cur

            d.ctrl[:N_WHEEL] = WHEEL_TORQUE
            d.ctrl[N_WHEEL:N_WHEEL + N_ARM] = arm_target

            mujoco.mj_step(m, d)
            v.sync()
            elapsed += m.opt.timestep
            time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    finally:
        v.close()
        print("已退出。")


if __name__ == "__main__":
    main()
