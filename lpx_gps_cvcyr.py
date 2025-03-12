import re
import numpy as np
from pyproj import Proj, transform
import math

# 定义坐标提取和转换函数
def extract_and_convert_coordinates(text, utm_zone=45, utm_letter='T'):
    # 将输入文本按行分割
    text_lines = text.strip().split('\n')
    
    # 提取坐标
    coordinates = []
    pattern = r'DYNAMIC RAW VERTEX: (\d+\.\d+) (\d+\.\d+)'
    for line in text_lines:
        match = re.search(pattern, line)
        if match:
            x = float(match.group(1))
            y = float(match.group(2))
            coordinates.append((x, y))
    
    # 转换为GPS坐标
    p = Proj(proj="utm", zone=utm_zone, ellps="WGS84")
    gps_coordinates = []
    for x, y in coordinates:
        lon, lat = transform(p, Proj(proj='latlong', ellps='WGS84'), x, y)
        gps_coordinates.append((lat, lon))
    
    return coordinates, gps_coordinates

# 定义CVCYR轨迹预测函数
def predict_vehicle_trajectory_cvcyr(base_info_line, current_utm_corners, prediction_time=5, time_step=0.1):
    # 提取基准信息
    pattern = r'\[procPredictionObstacles\]parse obstacle \[\d+\] base info \[id, type, x, y, yaw, yaw_rate, v, vx, vy, size\] = \[(\d+), (\d+), (\d+\.\d+), (\d+\.\d+), (\d+\.\d+), (-?\d+\.\d+), (\d+\.\d+), (-?\d+\.\d+), (\d+\.\d+), (\d+)\]'
    match = re.search(pattern, base_info_line)
    if match:
        id = int(match.group(1))
        type = int(match.group(2))
        x_base = float(match.group(3))
        y_base = float(match.group(4))
        yaw = float(match.group(5))  # 航向角，单位：弧度
        yaw_rate = float(match.group(6))  # 角速度，单位：弧度/秒
        v = float(match.group(7))
        vx = float(match.group(8))
        vy = float(match.group(9))
        size = int(match.group(10))
    

    print(v)
    print(yaw)
    print(yaw_rate)


    # 提取当前角点的UTM坐标
    current_utm_x = [corner[0] for corner in current_utm_corners]
    current_utm_y = [corner[1] for corner in current_utm_corners]
    
    # 计算预测后的角点，采用CVCYR策略
    predicted_utm_corners = []
    for x, y in zip(current_utm_x, current_utm_y):
        # 初始化当前坐标和航向角
        current_x = x
        current_y = y
        # current_yaw = math.degrees(yaw)
        current_yaw = yaw
        
        # 每个时间步长更新位置和航向角
        for t in np.arange(0, prediction_time, time_step):
            # 更新位置和航向角
            current_x += v * np.sin(current_yaw) * time_step
            current_y += v * np.cos(current_yaw) * time_step
            current_yaw += yaw_rate * time_step
        
        predicted_utm_corners.append((current_x, current_y))
    
    # 将预测后的UTM坐标转换为GPS坐标
    p = Proj(proj="utm", zone=45, ellps="WGS84")
    predicted_gps_corners = []
    for x, y in predicted_utm_corners:
        lon, lat = transform(p, Proj(proj='latlong', ellps='WGS84'), x, y)
        predicted_gps_corners.append((lat, lon))
    
    return predicted_utm_corners, predicted_gps_corners

# 定义生成.lpx文件的函数
def generate_lpx_file(current_gps_corners, predicted_gps_corners, output_file='CVCYR.lpx'):
    # 定义模板行
    template_line = "$,259.8396301269531,44.815437771820164,89.25791535283695,370.0408510638298,7.5,0.000000,0,0,0,0,0,*31"
    
    # 生成.lpx文件内容
    with open(output_file, 'w') as lpx_file:
        for current_corner, predicted_corner in zip(current_gps_corners, predicted_gps_corners):
            # 替换模板中的GPS坐标
            current_line = template_line.replace("44.815437771820164", str(current_corner[0])).replace("89.25791535283695", str(current_corner[1]))
            predicted_line = template_line.replace("44.815437771820164", str(predicted_corner[0])).replace("89.25791535283695", str(predicted_corner[1]))
            
            # 写入文件
            lpx_file.write(current_line + '\n')
            lpx_file.write(predicted_line + '\n')
    
    print(f"Results have been saved to {output_file}")

# 示例用法
if __name__ == "__main__":
    # 用户直接粘贴的四行内容
    text = """
    [2025-03-11 04:26:47.661636]<trace>[dynamic_obstacle.cpp:41]:DYNAMIC RAW VERTEX: 678398.572138 4964881.055100
    [2025-03-11 04:26:47.661823]<trace>[dynamic_obstacle.cpp:41]:DYNAMIC RAW VERTEX: 678407.653803 4964886.123017
    [2025-03-11 04:26:47.662010]<trace>[dynamic_obstacle.cpp:41]:DYNAMIC RAW VERTEX: 678410.041560 4964881.844162
    [2025-03-11 04:26:47.662143]<trace>[dynamic_obstacle.cpp:41]:DYNAMIC RAW VERTEX: 678400.959957 4964876.776247
    """
    
    base_info_line = "[2025-03-11 04:26:47.757472]<trace>[motion_prediction_component.cpp:44]:[procPredictionObstacles]parse obstacle [0] base info [id, type, x, y, yaw, yaw_rate, v, vx, vy, size] = [357, 2, 678404.312500, 4964881.500000, 1.061745, -0.004538, 7.856187, 6.860078, 3.828709, 4]"
    
    # 提取并转换当前角点的UTM坐标
    current_utm_corners, current_gps_corners = extract_and_convert_coordinates(text, utm_zone=45, utm_letter='T')
    print("Current UTM Corners:", current_utm_corners)
    print("Current GPS Corners:", current_gps_corners)
    
    # 预测轨迹并获取预测后的角点，采用CVCYR策略
    predicted_utm_corners, predicted_gps_corners = predict_vehicle_trajectory_cvcyr(base_info_line, current_utm_corners)
    print("Predicted UTM Corners after 5 seconds:", predicted_utm_corners)
    print("Predicted GPS Corners after 5 seconds:", predicted_gps_corners)
    
    # 生成.lpx文件
    generate_lpx_file(current_gps_corners, predicted_gps_corners)