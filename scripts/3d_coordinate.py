import pyrealsense2 as rs
import numpy as np

def get_3d_coordinates(x, y, depth, intrinsics):
    # 카메라 내부 매개변수(Intrinsic Parameters) 설정
    fx, fy = intrinsics.fx, intrinsics.fy
    cx, cy = intrinsics.ppx, intrinsics.ppy

    # 3D 좌표 계산
    x3d = (x - cx) * depth / fx
    y3d = (y - cy) * depth / fy
    z3d = depth

    return x3d, y3d, z3d

# 카메라 설정
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
pipeline.start(config)

# 내부 매개변수(Intrinsic Parameters) 가져오기
profile = pipeline.get_active_profile()
depth_intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()

try:
    while True:
        # 프레임 얻어오기
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        # 중앙 픽셀 좌표 설정
        x = int(depth_frame.get_width() / 2)
        y = int(depth_frame.get_height() / 2)

        # 깊이 값 얻어오기
        depth_value = depth_frame.get_distance(x, y)

        # 3D 좌표 계산
        x3d, y3d, z3d = get_3d_coordinates(x, y, depth_value, depth_intrinsics)

        print(f"3D 좌표 (x, y, z): ({x3d}, {y3d}, {z3d})")

except KeyboardInterrupt:
    pass

# 카메라 종료
pipeline.stop()
