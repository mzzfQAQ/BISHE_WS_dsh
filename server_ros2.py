#!/usr/bin/env python3
import os
import cv2
import threading

# ROS 2 相关导入
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge, CvBridgeError

# FastAPI 相关导入 (⚠️ 这里新增了 Response)
from fastapi import FastAPI, Response
from fastapi.responses import StreamingResponse, FileResponse
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

app = FastAPI()

# 允许跨域请求（防止手机 App 端访问时因安全策略被拦截）
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 【自定义配置】修改为你想传给手机的照片文件夹绝对路径
IMAGE_DIR = os.path.expanduser("~/BISHE_WS") 
if not os.path.exists(IMAGE_DIR):
    os.makedirs(IMAGE_DIR)

# 全局变量与锁，用于安全地在 ROS 线程和 FastAPI 线程间共享图像
latest_frame = None
frame_lock = threading.Lock()
bridge = CvBridge()

# --- ROS 2 节点定义 ---
class CameraSubscriberNode(Node):
    def __init__(self):
        super().__init__('mobile_app_server_node')
        
        # 订阅小车摄像头的真实 ROS 2 话题
        self.subscription = self.create_subscription(
            RosImage,
            '/camera_sensor/image_raw',  # 已匹配你的小车相机话题
            self.image_callback,
            10                           # QoS 历史深度
        )
        self.get_logger().info("ROS 2 摄像头图像订阅节点已启动！")

    def image_callback(self, msg):
        global latest_frame
        try:
            # 将 ROS 2 图像消息转换为 OpenCV 的 BGR 图像
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with frame_lock:
                latest_frame = cv_image
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge 转换失败: {e}")

# --- FastAPI 视频流生成器 (保留备用或给网页端使用) ---
def gen_video_frames():
    global latest_frame
    # 检查 rclpy 是否还在运行
    while rclpy.ok():
        with frame_lock:
            if latest_frame is None:
                continue
            # 浅拷贝一份画面，避免读取时被 ROS 写入线程冲突覆盖
            frame = latest_frame.copy()
            
        # 将 OpenCV 图像压缩为 JPEG 格式
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue
        frame_bytes = buffer.tobytes()
        
        # 组装成 MJPEG 格式数据流
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# --- FastAPI 路由接口 ---
@app.get('/video_feed')
def video_feed():
    """流媒体接口"""
    return StreamingResponse(
        gen_video_frames(), 
        media_type='multipart/x-mixed-replace; boundary=frame'
    )

# 📢 【新增】专门给手机 App 定时轮询用的单帧接口
@app.get('/single_frame')
def single_frame():
    global latest_frame
    with frame_lock:
        if latest_frame is None:
            return {"error": "No frame available"}, 404
        frame = latest_frame.copy()
    
    # 压缩为 JPG
    _, buffer = cv2.imencode('.jpg', frame)
    return Response(content=buffer.tobytes(), media_type="image/jpeg")

@app.get('/images')
def list_images():
    """获取工作空间下的图片列表"""
    if not os.path.exists(IMAGE_DIR): 
        return []
    extensions = ('.png', '.jpg', '.jpeg', '.bmp')
    files = [f for f in os.listdir(IMAGE_DIR) if f.lower().endswith(extensions)]
    # 按时间新旧排序（最新拍的照片排在最前面）
    files.sort(key=lambda x: os.path.getmtime(os.path.join(IMAGE_DIR, x)), reverse=True)
    return files

@app.get('/images/{filename}')
def get_image(filename: str):
    """下载/查看单张照片"""
    file_path = os.path.join(IMAGE_DIR, filename)
    if os.path.exists(file_path):
        return FileResponse(file_path)
    return {"error": "File not found"}, 404

# --- 后台运行 ROS 2 的线程 ---
def ros2_thread_entry():
    rclpy.init()
    node = CameraSubscriberNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"ROS 2 Spin 异常退出: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    # 1. 在后台线程中启动 ROS 2 节点
    ros_thread = threading.Thread(target=ros2_thread_entry, daemon=True)
    ros_thread.start()
    
    # 2. 在主线程中启动 FastAPI Web 服务器 (📢 host 修改为了 0.0.0.0)
    uvicorn.run(app, host='10.181.8.38', port=8000)