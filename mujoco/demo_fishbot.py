#!/usr/bin/env python3
"""MuJoCo 自动演示：四轮小车前进 + Franka Panda 机械臂姿态切换，渲染保存 mp4

用法：python3 demo_fishbot.py [输出.mp4]
"""
import sys
import numpy as np
import mujoco

XML = "fishbot_arm.xml"
OUT = sys.argv[1] if len(sys.argv) > 1 else "fishbot_panda_demo.mp4"

# Panda 预设姿态（对应 SRDF：ready / up / wave）
ARM_POSES = {
    "ready": np.array([0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]),
    "up":    np.array([0.0, -1.4, 0.0, -2.2, 0.0, 1.2, 0.0]),
    "wave":  np.array([0.0, -1.2, -0.5, -1.8, 0.6, 1.0, 1.5]),
}
N_ARM = 7          # Panda 7 关节
N_WHEEL = 4        # 4 个轮子


def main():
    try:
        import imageio
    except ImportError:
        print("需要 imageio: pip3 install imageio imageio[ffmpeg]")
        return 1

    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)

    # Panda 初始 ready 姿态（franka 官方初始，避免折叠零位自碰撞）
    d.qpos[11:18] = np.array([0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854])
    mujoco.mj_forward(m, d)

    renderer = mujoco.Renderer(m, height=480, width=640)

    frames = []
    sim_time = 0.0
    dt = m.opt.timestep
    step_count = 0
    total = 9.0          # 总时长 9 秒
    wheel = 2.0          # 驱动扭矩 Nm（前进）
    pose_seq = ["ready", "up", "wave", "ready"]
    pose_dur = [2.0, 2.5, 2.5, 2.0]
    transition = 1.0     # 姿态过渡时间（秒）

    idx = 0
    elapsed = 0.0
    prev_target = ARM_POSES[pose_seq[0]].copy()
    current_target = ARM_POSES[pose_seq[0]].copy()
    while sim_time < total:
        # 姿态切换（平滑插值）
        if elapsed >= pose_dur[idx]:
            prev_target = ARM_POSES[pose_seq[idx]].copy()
            idx = (idx + 1) % len(pose_seq)
            current_target = ARM_POSES[pose_seq[idx]].copy()
            elapsed = 0.0
        if elapsed < transition:
            alpha = elapsed / transition
            arm_target = prev_target * (1 - alpha) + current_target * alpha
        else:
            arm_target = current_target

        # 控制：四轮前进 + Panda 目标姿态
        d.ctrl[:N_WHEEL] = wheel
        d.ctrl[N_WHEEL:N_WHEEL + N_ARM] = arm_target

        mujoco.mj_step(m, d)
        sim_time += dt
        elapsed += dt
        step_count += 1

        # 渲染（每 2 步 1 帧）
        if step_count % 2 == 0:
            renderer.update_scene(d, camera="camera_follow")
            frames.append(renderer.render())

    imageio.mimsave(OUT, frames, fps=30)
    print(f"✅ 演示完成，已保存: {OUT}（{len(frames)} 帧）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
