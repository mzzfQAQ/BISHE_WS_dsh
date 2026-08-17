#!/usr/bin/env python3
"""MuJoCo 交互查看器：fishbot + 6-DOF 机械臂（自动演示模式）

- 机械臂自动循环执行 home -> up -> front 姿态
- 小车自动慢速前进
- 鼠标操作：左键旋转视角 | 滚轮缩放 | 右键平移
- 关闭窗口即退出
"""
import time
import numpy as np
import mujoco
from mujoco import viewer

XML = "fishbot_arm.xml"

# 机械臂预设姿态（对应 SRDF：home / up / front）
ARM_POSES = {
    "home":  np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    "up":    np.array([0.0, -1.57, 0.0, 0.0, 0.0, 0.0]),
    "front": np.array([0.0, -0.8, -0.6, 0.0, -0.3, 0.0]),
}
POSE_SEQ = ["home", "up", "front", "home"]
POSE_DUR = 3.0        # 每个姿态持续时间（秒）
TRANSITION = 1.0      # 姿态过渡时间（秒）
WHEEL_TORQUE = 1.0    # 轮子扭矩（Nm）


def main():
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)

    v = viewer.launch_passive(m, d, show_left_ui=True, show_right_ui=False)

    print("=" * 50)
    print("  MuJoCo fishbot + 6-DOF 机械臂（自动演示）")
    print("  机械臂循环: home -> up -> front")
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

            d.ctrl[0] = WHEEL_TORQUE
            d.ctrl[1] = WHEEL_TORQUE
            d.ctrl[2:8] = arm_target

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
