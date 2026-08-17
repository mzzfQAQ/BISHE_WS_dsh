#!/usr/bin/env python3
"""MuJoCo 自动演示：小车前进 + 机械臂 home/up/down 摆动，渲染保存 mp4

用法：python3 demo_fishbot.py [输出.mp4]
"""
import sys
import numpy as np
import mujoco

XML = "fishbot_arm.xml"
OUT = sys.argv[1] if len(sys.argv) > 1 else "fishbot_arm_demo.mp4"

ARM_POSES = {
    "home": np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    "up":   np.array([0.0, -1.57, 0.0, 0.0, 0.0, 0.0]),
    "front": np.array([0.0, -0.8, -0.6, 0.0, -0.3, 0.0]),
}


def main():
    try:
        import imageio
    except ImportError:
        print("需要 imageio: pip3 install imageio")
        return 1

    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    renderer = mujoco.Renderer(m, height=480, width=640)

    frames = []
    sim_time = 0.0
    dt = m.opt.timestep
    step_count = 0
    total = 8.0          # 总时长 8 秒
    wheel = 2.0          # 驱动扭矩 Nm（前进）
    pose_seq = ["home", "up", "front", "home"]
    pose_dur = [2.0, 2.0, 2.0, 2.0]
    transition = 1.0     # 姿态过渡时间（秒）

    idx = 0
    elapsed = 0.0
    prev_target = ARM_POSES[pose_seq[0]].copy()
    current_target = ARM_POSES[pose_seq[0]].copy()
    while sim_time < total:
        # 姿态切换（记录前后目标，用于平滑插值）
        if elapsed >= pose_dur[idx]:
            prev_target = ARM_POSES[pose_seq[idx]].copy()
            idx = (idx + 1) % len(pose_seq)
            current_target = ARM_POSES[pose_seq[idx]].copy()
            elapsed = 0.0

        # 平滑过渡：切换后的 transition 秒内线性插值
        if elapsed < transition:
            alpha = elapsed / transition
            arm_target = prev_target * (1 - alpha) + current_target * alpha
        else:
            arm_target = current_target

        # 控制：前进 + 机械臂目标姿态
        d.ctrl[0] = wheel
        d.ctrl[1] = wheel
        d.ctrl[2:8] = arm_target

        mujoco.mj_step(m, d)
        sim_time += dt
        elapsed += dt
        step_count += 1

        # 渲染（每 2 步 1 帧，约 30fps 输出）
        if step_count % 2 == 0:
            renderer.update_scene(d, camera="camera_follow")
            frames.append(renderer.render())

    imageio.mimsave(OUT, frames, fps=30)
    print(f"✅ 演示完成，已保存: {OUT}（{len(frames)} 帧）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
