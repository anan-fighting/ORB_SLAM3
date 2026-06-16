#!/usr/bin/env python3
"""
获取 RealSense (D435i/D455 通用) 的内参和外参，并生成 ORB-SLAM3 YAML 配置文件。

使用方法:
    python3 get_realsense_calib.py

自动检测已连接的 RealSense 设备，读取内参并生成 6 个 YAML 配置文件到 D455_config/ 目录。

注意：
  - D435i 用户一般不需要此脚本——ORB-SLAM3 已内置 D435i 的配置文件
  - 此脚本主要用于：换用其他 RealSense 型号（如 D455）、或需要精确读取单台设备的出厂标定参数
"""

import sys
import math

try:
    import pyrealsense2 as rs
except ImportError:
    print("ERROR: pyrealsense2 未安装!")
    print("请运行: pip install pyrealsense2")
    sys.exit(1)

def main():
    ctx = rs.context()
    devices = ctx.query_devices()
    if len(devices) == 0:
        print("未检测到 RealSense 设备，请连接 D455 后重试。")
        sys.exit(1)

    device = devices[0]
    print(f"检测到设备: {device.get_info(rs.camera_info.name)}")
    print(f"序列号: {device.get_info(rs.camera_info.serial_number)}")
    print("=" * 60)

    # 配置 pipeline 以获取传感器数据
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
    cfg.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)

    profile = pipe.start(cfg)

    # 等待几帧（让自动曝光稳定）
    import time
    for _ in range(30):
        pipe.wait_for_frames()
    time.sleep(0.5)

    # 获取各传感器内参 ------------------------------------------------
    # Color stream
    color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    color_intr = color_stream.get_intrinsics()
    print(f"\n=== RGB 相机内参 (Color) ===")
    print(f"  分辨率: {color_intr.width} x {color_intr.height}")
    print(f"  fx={color_intr.fx:.4f}, fy={color_intr.fy:.4f}")
    print(f"  cx={color_intr.ppx:.4f}, cy={color_intr.ppy:.4f}")
    print(f"  畸变模型: {color_intr.model}  (k1~k5={color_intr.coeffs})")

    # Depth stream
    depth_stream = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    depth_intr = depth_stream.get_intrinsics()
    print(f"\n=== Depth 相机内参 ===")
    print(f"  分辨率: {depth_intr.width} x {depth_intr.height}")
    print(f"  fx={depth_intr.fx:.4f}, fy={depth_intr.fy:.4f}")
    print(f"  cx={depth_intr.ppx:.4f}, cy={depth_intr.ppy:.4f}")

    # IR1 stream (left)
    ir1_stream = profile.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
    ir1_intr = ir1_stream.get_intrinsics()
    print(f"\n=== IR1 (左) 相机内参 ===")
    print(f"  分辨率: {ir1_intr.width} x {ir1_intr.height}")
    print(f"  fx={ir1_intr.fx:.4f}, fy={ir1_intr.fy:.4f}")
    print(f"  cx={ir1_intr.ppx:.4f}, cy={ir1_intr.ppy:.4f}")

    # IR2 stream (right)
    ir2_stream = profile.get_stream(rs.stream.infrared, 2).as_video_stream_profile()
    ir2_intr = ir2_stream.get_intrinsics()
    print(f"\n=== IR2 (右) 相机内参 ===")
    print(f"  分辨率: {ir2_intr.width} x {ir2_intr.height}")
    print(f"  fx={ir2_intr.fx:.4f}, fy={ir2_intr.fy:.4f}")
    print(f"  cx={ir2_intr.ppx:.4f}, cy={ir2_intr.ppy:.4f}")

    # 获取外参 ----------------------------------------------------------
    # 需要读取 sensors
    sensors = device.query_sensors()
    print(f"\n=== 传感器列表 ===")
    imu_found = False
    for s in sensors:
        name = s.get_info(rs.camera_info.name)
        print(f"  {name}")

    # 获取 depth scale
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print(f"\n=== Depth Scale ===")
    print(f"  depth_scale = {depth_scale:.6f}  (即要除以 {1.0/depth_scale:.0f})")

    # 获取 stereo extrinsics (IR1 -> IR2)
    # IR1 to IR2
    ir1_to_ir2 = ir1_stream.get_extrinsics_to(ir2_stream)
    baseline = abs(ir1_to_ir2.translation[0])
    print(f"\n=== 双目外参 (IR1 → IR2) ===")
    print(f"  基线 (baseline): {baseline:.6f} m")
    print(f"  旋转矩阵:\n{ir1_to_ir2.rotation}")
    print(f"  平移向量: {ir1_to_ir2.translation}")

    # 构建旋转矩阵为 4x4 变换矩阵
    T_ir1_ir2_data = []
    for r in range(3):
        row = list(ir1_to_ir2.rotation[r]) + [ir1_to_ir2.translation[r]]
        T_ir1_ir2_data.extend(row)
    T_ir1_ir2_data.extend([0, 0, 0, 1])

    # Color to IR1 (Left)
    color_to_ir1 = color_stream.get_extrinsics_to(ir1_stream)
    print(f"\n=== 外参 (Color → IR1) ===")
    print(f"  平移: {color_to_ir1.translation}")
    print(f"  旋转: {color_to_ir1.rotation}")

    # Depth to Color (for RGB-D alignment)
    depth_to_color = depth_stream.get_extrinsics_to(color_stream)
    print(f"\n=== 外参 (Depth → Color) ===")
    print(f"  平移: {depth_to_color.translation}")

    pipe.stop()

    # ================================================================
    # IMU 参数 (D455 使用 BMI055)
    # ================================================================
    # D455 内置 BMI055 IMU，参数与 D435i 稍有不同
    # 参考 Intel 官方文档: https://www.intelrealsense.com/wp-content/uploads/2021/12/Intel-RealSense-D400-Series-Datasheet.pdf
    imu_noise_gyro = 1e-3    # rad/s/sqrt(Hz)
    imu_noise_acc = 1e-2     # m/s^2/sqrt(Hz)  
    imu_gyro_walk = 1e-6     # rad/s^2/sqrt(Hz)
    imu_acc_walk = 1e-4      # m/s^3/sqrt(Hz)
    imu_frequency = 200.0

    print(f"\n=== IMU 参数 (BMI055 标称值) ===")
    print(f"  NoiseGyro: {imu_noise_gyro}")
    print(f"  NoiseAcc:  {imu_noise_acc}")
    print(f"  GyroWalk:  {imu_gyro_walk}")
    print(f"  AccWalk:   {imu_acc_walk}")
    print(f"  Frequency: {imu_frequency}")

    # ================================================================
    # 生成 YAML 配置文件
    # ================================================================
    # 获取 depth_scale_factor
    depth_factor = 1.0 / depth_scale
    
    # IMU 到 IR1 的变换 (body IMU 在 IR1 坐标系中的位姿)
    # D455: IMU 到左IR相机的标称外参
    # 参照 D435i: T_b_c = Identity (近似)，实际很小
    T_imu_ir1_data = [1.0, 0.0, 0.0, 0.005,
                       0.0, 1.0, 0.0, 0.005,
                       0.0, 0.0, 1.0, 0.0117,
                       0.0, 0.0, 0.0, 1.0]

    def make_yaml_header():
        return "%YAML:1.0\n\n"

    def make_camera_common(cam_intr, cam_name="Camera1"):
        """生成相机基本参数"""
        lines = []
        lines.append(f"\n# {cam_name} calibration (OpenCV)")
        lines.append(f"{cam_name}.fx: {cam_intr.fx:.4f}")
        lines.append(f"{cam_name}.fy: {cam_intr.fy:.4f}")
        lines.append(f"{cam_name}.cx: {cam_intr.ppx:.4f}")
        lines.append(f"{cam_name}.cy: {cam_intr.ppy:.4f}")
        lines.append("")
        # D455 出厂已经做了去畸变，通常畸变参数很小或为 0
        if cam_intr.model == rs.distortion.inverse_brown_conrady:
            k = cam_intr.coeffs
            lines.append(f"# Distortion parameters (Inverse Brown Conrady)")
            lines.append(f"{cam_name}.k1: {k[0]:.8f}")
            lines.append(f"{cam_name}.k2: {k[1]:.8f}")
            lines.append(f"{cam_name}.p1: {k[2]:.8f}")
            lines.append(f"{cam_name}.p2: {k[3]:.8f}")
        else:
            lines.append(f"# Distortion: {cam_intr.model}")
            lines.append(f"{cam_name}.k1: 0.0")
            lines.append(f"{cam_name}.k2: 0.0")
            lines.append(f"{cam_name}.p1: 0.0")
            lines.append(f"{cam_name}.p2: 0.0")
        return "\n".join(lines)

    def make_camera_resolution(w, h, fps=30):
        return f"""\n# Camera resolution
Camera.width: {w}
Camera.height: {h}

# Camera frames per second 
Camera.fps: {fps}

# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
Camera.RGB: 1"""

    def make_imu_params():
        return f"""\n# Do not insert KFs when recently lost
IMU.InsertKFsWhenLost: 0

# IMU noise (BMI055 on D455)
IMU.NoiseGyro: {imu_noise_gyro}
IMU.NoiseAcc: {imu_noise_acc}
IMU.GyroWalk: {imu_gyro_walk}
IMU.AccWalk: {imu_acc_walk}
IMU.Frequency: {imu_frequency}"""

    def make_viewer_params(viewpoint_z="-3.5"):
        return f"""\n#--------------------------------------------------------------------------------------------
# Viewer Parameters
#---------------------------------------------------------------------------------------------
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: {viewpoint_z}
Viewer.ViewpointF: 500.0
"""

    # ---- 1. Monocular (使用 Color 相机) ----
    mono_yaml = make_yaml_header()
    mono_yaml += "#--------------------------------------------------------------------------------------------"
    mono_yaml += "\n# Camera Parameters for RealSense D455 (Monocular - Color)\n"
    mono_yaml += "# Auto-generated by get_d455_calib.py\n"
    mono_yaml += "#--------------------------------------------------------------------------------------------"
    mono_yaml += "\nFile.version: \"1.0\"\n"
    mono_yaml += "\nCamera.type: \"PinHole\""
    mono_yaml += make_camera_common(color_intr, "Camera1")
    mono_yaml += make_camera_resolution(color_intr.width, color_intr.height)
    mono_yaml += "\n\n#--------------------------------------------------------------------------------------------"
    mono_yaml += "\n# ORB Parameters\n#--------------------------------------------------------------------------------------------"
    mono_yaml += "\nORBextractor.nFeatures: 1000"
    mono_yaml += "\nORBextractor.scaleFactor: 1.2"
    mono_yaml += "\nORBextractor.nLevels: 8"
    mono_yaml += "\nORBextractor.iniThFAST: 20"
    mono_yaml += "\nORBextractor.minThFAST: 7"
    mono_yaml += make_viewer_params("-1.8")

    # ---- 2. Monocular-Inertial (Color + IMU) ----
    mono_inertial_yaml = make_yaml_header()
    mono_inertial_yaml += "#--------------------------------------------------------------------------------------------"
    mono_inertial_yaml += "\n# Camera Parameters for RealSense D455 (Monocular-Inertial)\n"
    mono_inertial_yaml += "# Auto-generated by get_d455_calib.py\n"
    mono_inertial_yaml += "#--------------------------------------------------------------------------------------------"
    mono_inertial_yaml += "\nFile.version: \"1.0\"\n"
    mono_inertial_yaml += "\nCamera.type: \"PinHole\""
    mono_inertial_yaml += make_camera_common(color_intr, "Camera1")
    mono_inertial_yaml += make_camera_resolution(color_intr.width, color_intr.height)
    # IMU extrinsics: T_imu_to_color
    mono_inertial_yaml += f"""

# Transformation from body-frame (imu) to left camera
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [{T_imu_ir1_data[0]}, {T_imu_ir1_data[1]}, {T_imu_ir1_data[2]}, {T_imu_ir1_data[3]},
          {T_imu_ir1_data[4]}, {T_imu_ir1_data[5]}, {T_imu_ir1_data[6]}, {T_imu_ir1_data[7]},
          {T_imu_ir1_data[8]}, {T_imu_ir1_data[9]}, {T_imu_ir1_data[10]}, {T_imu_ir1_data[11]},
          {T_imu_ir1_data[12]}, {T_imu_ir1_data[13]}, {T_imu_ir1_data[14]}, {T_imu_ir1_data[15]}]
"""
    mono_inertial_yaml += make_imu_params()
    mono_inertial_yaml += "\n\n#--------------------------------------------------------------------------------------------"
    mono_inertial_yaml += "\n# ORB Parameters\n#--------------------------------------------------------------------------------------------"
    mono_inertial_yaml += "\nORBextractor.nFeatures: 1000"
    mono_inertial_yaml += "\nORBextractor.scaleFactor: 1.2"
    mono_inertial_yaml += "\nORBextractor.nLevels: 8"
    mono_inertial_yaml += "\nORBextractor.iniThFAST: 20"
    mono_inertial_yaml += "\nORBextractor.minThFAST: 7"
    mono_inertial_yaml += make_viewer_params("-3.5")

    # ---- 3. Stereo (IR1 + IR2) ----
    stereo_yaml = make_yaml_header()
    stereo_yaml += "#--------------------------------------------------------------------------------------------"
    stereo_yaml += "\n# System config\n#--------------------------------------------------------------------------------------------\n"
    stereo_yaml += "#System.LoadAtlasFromFile: \"Session_D455_Stereo\"\n"
    stereo_yaml += "#System.SaveAtlasToFile: \"Session_D455_Stereo\"\n"
    stereo_yaml += "\n#--------------------------------------------------------------------------------------------"
    stereo_yaml += "\n# Camera Parameters for RealSense D455 (Stereo - IR)\n"
    stereo_yaml += "# Auto-generated by get_d455_calib.py\n"
    stereo_yaml += "#--------------------------------------------------------------------------------------------"
    stereo_yaml += "\nFile.version: \"1.0\"\n"
    stereo_yaml += "\nCamera.type: \"PinHole\""
    stereo_yaml += make_camera_common(ir1_intr, "Camera1")
    stereo_yaml += make_camera_common(ir2_intr, "Camera2")
    stereo_yaml += make_camera_resolution(ir1_intr.width, ir1_intr.height)
    stereo_yaml += f"""

Stereo.ThDepth: 40.0
Stereo.T_c1_c2: !!opencv-matrix
  rows: 4
  cols: 4
  dt: f
  data: [{T_ir1_ir2_data[0]:.6f}, {T_ir1_ir2_data[1]:.6f}, {T_ir1_ir2_data[2]:.6f}, {T_ir1_ir2_data[3]:.6f},
         {T_ir1_ir2_data[4]:.6f}, {T_ir1_ir2_data[5]:.6f}, {T_ir1_ir2_data[6]:.6f}, {T_ir1_ir2_data[7]:.6f},
         {T_ir1_ir2_data[8]:.6f}, {T_ir1_ir2_data[9]:.6f}, {T_ir1_ir2_data[10]:.6f}, {T_ir1_ir2_data[11]:.6f},
         {T_ir1_ir2_data[12]}, {T_ir1_ir2_data[13]}, {T_ir1_ir2_data[14]}, {T_ir1_ir2_data[15]}]
"""
    stereo_yaml += "\n#--------------------------------------------------------------------------------------------"
    stereo_yaml += "\n# ORB Parameters\n#--------------------------------------------------------------------------------------------"
    stereo_yaml += "\nORBextractor.nFeatures: 1200"
    stereo_yaml += "\nORBextractor.scaleFactor: 1.2"
    stereo_yaml += "\nORBextractor.nLevels: 8"
    stereo_yaml += "\nORBextractor.iniThFAST: 20"
    stereo_yaml += "\nORBextractor.minThFAST: 7"
    stereo_yaml += make_viewer_params("-1.8")

    # ---- 4. Stereo-Inertial (IR + IMU) ----
    stereo_inertial_yaml = make_yaml_header()
    stereo_inertial_yaml += "#--------------------------------------------------------------------------------------------"
    stereo_inertial_yaml += "\n# Camera Parameters for RealSense D455 (Stereo-Inertial)\n"
    stereo_inertial_yaml += "# Auto-generated by get_d455_calib.py\n"
    stereo_inertial_yaml += "#--------------------------------------------------------------------------------------------"
    stereo_inertial_yaml += "\nFile.version: \"1.0\"\n"
    # 注意：D455 的 IR 流已经是 rectified 的
    stereo_inertial_yaml += "\nCamera.type: \"PinHole\""
    stereo_inertial_yaml += make_camera_common(ir1_intr, "Camera1")
    stereo_inertial_yaml += make_camera_common(ir2_intr, "Camera2")
    stereo_inertial_yaml += make_camera_resolution(ir1_intr.width, ir1_intr.height)
    stereo_inertial_yaml += f"""

Stereo.ThDepth: 40.0

Stereo.T_c1_c2: !!opencv-matrix
  rows: 4
  cols: 4
  dt: f
  data: [{T_ir1_ir2_data[0]:.6f}, {T_ir1_ir2_data[1]:.6f}, {T_ir1_ir2_data[2]:.6f}, {T_ir1_ir2_data[3]:.6f},
         {T_ir1_ir2_data[4]:.6f}, {T_ir1_ir2_data[5]:.6f}, {T_ir1_ir2_data[6]:.6f}, {T_ir1_ir2_data[7]:.6f},
         {T_ir1_ir2_data[8]:.6f}, {T_ir1_ir2_data[9]:.6f}, {T_ir1_ir2_data[10]:.6f}, {T_ir1_ir2_data[11]:.6f},
         {T_ir1_ir2_data[12]}, {T_ir1_ir2_data[13]}, {T_ir1_ir2_data[14]}, {T_ir1_ir2_data[15]}]

# Transformation from body-frame (imu) to left camera
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [{T_imu_ir1_data[0]}, {T_imu_ir1_data[1]}, {T_imu_ir1_data[2]}, {T_imu_ir1_data[3]},
          {T_imu_ir1_data[4]}, {T_imu_ir1_data[5]}, {T_imu_ir1_data[6]}, {T_imu_ir1_data[7]},
          {T_imu_ir1_data[8]}, {T_imu_ir1_data[9]}, {T_imu_ir1_data[10]}, {T_imu_ir1_data[11]},
          {T_imu_ir1_data[12]}, {T_imu_ir1_data[13]}, {T_imu_ir1_data[14]}, {T_imu_ir1_data[15]}]
"""
    stereo_inertial_yaml += make_imu_params()
    stereo_inertial_yaml += "\n\n#--------------------------------------------------------------------------------------------"
    stereo_inertial_yaml += "\n# ORB Parameters\n#--------------------------------------------------------------------------------------------"
    stereo_inertial_yaml += "\nORBextractor.nFeatures: 1200"
    stereo_inertial_yaml += "\nORBextractor.scaleFactor: 1.2"
    stereo_inertial_yaml += "\nORBextractor.nLevels: 8"
    stereo_inertial_yaml += "\nORBextractor.iniThFAST: 20"
    stereo_inertial_yaml += "\nORBextractor.minThFAST: 7"
    stereo_inertial_yaml += make_viewer_params("-3.5")

    # ---- 5. RGB-D (Color + Depth) ----
    rgbd_yaml = make_yaml_header()
    rgbd_yaml += "#--------------------------------------------------------------------------------------------"
    rgbd_yaml += "\n# Camera Parameters for RealSense D455 (RGB-D)\n"
    rgbd_yaml += "# Auto-generated by get_d455_calib.py\n"
    rgbd_yaml += "#--------------------------------------------------------------------------------------------"
    rgbd_yaml += "\nFile.version: \"1.0\"\n"
    rgbd_yaml += "\nCamera.type: \"PinHole\""
    rgbd_yaml += make_camera_common(color_intr, "Camera1")
    rgbd_yaml += make_camera_resolution(color_intr.width, color_intr.height)
    rgbd_yaml += f"""

# Close/Far threshold. Baseline times.
Stereo.ThDepth: 40.0

# Depth map values factor
RGBD.DepthMapFactor: {depth_factor:.1f}
"""
    rgbd_yaml += "\n#--------------------------------------------------------------------------------------------"
    rgbd_yaml += "\n# ORB Parameters\n#--------------------------------------------------------------------------------------------"
    rgbd_yaml += "\nORBextractor.nFeatures: 1000"
    rgbd_yaml += "\nORBextractor.scaleFactor: 1.2"
    rgbd_yaml += "\nORBextractor.nLevels: 8"
    rgbd_yaml += "\nORBextractor.iniThFAST: 20"
    rgbd_yaml += "\nORBextractor.minThFAST: 7"
    rgbd_yaml += make_viewer_params("-1.8")

    # ---- 6. RGB-D-Inertial (Color + Depth + IMU) ----
    rgbd_inertial_yaml = make_yaml_header()
    rgbd_inertial_yaml += "#--------------------------------------------------------------------------------------------"
    rgbd_inertial_yaml += "\n# Camera Parameters for RealSense D455 (RGB-D-Inertial)\n"
    rgbd_inertial_yaml += "# Auto-generated by get_d455_calib.py\n"
    rgbd_inertial_yaml += "#--------------------------------------------------------------------------------------------"
    rgbd_inertial_yaml += "\nFile.version: \"1.0\"\n"
    rgbd_inertial_yaml += "\nCamera.type: \"PinHole\""
    rgbd_inertial_yaml += make_camera_common(color_intr, "Camera1")
    rgbd_inertial_yaml += make_camera_resolution(color_intr.width, color_intr.height)
    rgbd_inertial_yaml += f"""

Stereo.ThDepth: 40.0

# Depth map values factor
RGBD.DepthMapFactor: {depth_factor:.1f}

# Transformation from body-frame (imu) to left camera
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [{T_imu_ir1_data[0]}, {T_imu_ir1_data[1]}, {T_imu_ir1_data[2]}, {T_imu_ir1_data[3]},
          {T_imu_ir1_data[4]}, {T_imu_ir1_data[5]}, {T_imu_ir1_data[6]}, {T_imu_ir1_data[7]},
          {T_imu_ir1_data[8]}, {T_imu_ir1_data[9]}, {T_imu_ir1_data[10]}, {T_imu_ir1_data[11]},
          {T_imu_ir1_data[12]}, {T_imu_ir1_data[13]}, {T_imu_ir1_data[14]}, {T_imu_ir1_data[15]}]
"""
    rgbd_inertial_yaml += make_imu_params()
    rgbd_inertial_yaml += "\n\n#--------------------------------------------------------------------------------------------"
    rgbd_inertial_yaml += "\n# ORB Parameters\n#--------------------------------------------------------------------------------------------"
    rgbd_inertial_yaml += "\nORBextractor.nFeatures: 1000"
    rgbd_inertial_yaml += "\nORBextractor.scaleFactor: 1.2"
    rgbd_inertial_yaml += "\nORBextractor.nLevels: 8"
    rgbd_inertial_yaml += "\nORBextractor.iniThFAST: 20"
    rgbd_inertial_yaml += "\nORBextractor.minThFAST: 7"
    rgbd_inertial_yaml += make_viewer_params("-3.5")

    # 写入文件
    files = {
        "RealSense_D455_mono.yaml": mono_yaml,
        "RealSense_D455_mono_inertial.yaml": mono_inertial_yaml,
        "RealSense_D455_stereo.yaml": stereo_yaml,
        "RealSense_D455_stereo_inertial.yaml": stereo_inertial_yaml,
        "RealSense_D455_rgbd.yaml": rgbd_yaml,
        "RealSense_D455_rgbd_inertial.yaml": rgbd_inertial_yaml,
    }

    import os
    output_dir = "D455_config"
    os.makedirs(output_dir, exist_ok=True)

    for fname, content in files.items():
        filepath = os.path.join(output_dir, fname)
        with open(filepath, 'w') as f:
            f.write(content)
        print(f"\n✓ 已生成: {filepath}")

    print(f"\n{'='*60}")
    print(f"所有配置文件已生成到 '{output_dir}/' 目录。")
    print(f"请将文件复制到 ORB_SLAM3/Examples/ 对应子目录下使用。")
    print(f"\nD455 vs D435i 关键差异:")
    print(f"  - D455 基线更长: ~{baseline*1000:.0f}mm (D435i: 50mm)")
    print(f"  - 更远的有效深度范围")
    print(f"  - 更好的 RGB 传感器 (全局快门)")

if __name__ == "__main__":
    main()
