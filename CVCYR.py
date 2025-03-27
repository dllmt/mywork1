import re
import numpy as np
from pyproj import Proj, transform

# 定义坐标提取和转换函数
def extract_and_convert_coordinates(text, utm_zone=45, utm_letter='T'):
    # 提取坐标
    vertices = []
    center = None
    
    # 匹配 DYNAMIC RAW VERTEX 格式的坐标
    pattern_vertex = r'DYNAMIC RAW VERTEX: (\d+\.\d+) (\d+\.\d+)'
    # 匹配中心点格式的坐标  要加-？
    pattern_center = r'parse obstacle \[\d+\] base info \[id, type, x, y, yaw, yaw_rate, v, vx, vy, size\] = \[\d+, \d+, (\d+\.\d+), (\d+\.\d+), (\d+\.\d+), (-?\d+\.\d+), (\d+\.\d+), (-?\d+\.\d+), (-?\d+\.\d+), \d+\]'
    
    for line in text.strip().split('\n'):
        # 匹配 DYNAMIC RAW VERTEX 格式的坐标
        match_vertex = re.search(pattern_vertex, line)
        if match_vertex:
            x = float(match_vertex.group(1))
            y = float(match_vertex.group(2))
            vertices.append((x, y))
            continue
        
        # 匹配中心点格式的坐标
        match_center = re.search(pattern_center, line)
        if match_center:
            x = float(match_center.group(1))
            y = float(match_center.group(2))
            center = (x, y)
            continue
    
    # 转换为GPS坐标
    p = Proj(proj="utm", zone=utm_zone, ellps="WGS84")
    vertices_gps = []
    for x, y in vertices:
        lon, lat = transform(p, Proj(proj='latlong', ellps='WGS84'), x, y)
        vertices_gps.append((lat, lon))
    
    # 转换中心点为GPS坐标
    center_gps = None
    if center:
        x, y = center
        lon, lat = transform(p, Proj(proj='latlong', ellps='WGS84'), x, y)
        center_gps = (lat, lon)
    
    return vertices, vertices_gps, center, center_gps

# 定义CVCYR轨迹预测函数
def predict_center_point_cvcyr(base_info_line, center_point_utm, prediction_time=5, time_step=0.1):
    # 提取基准信息
    pattern = r'parse obstacle \[\d+\] base info \[id, type, x, y, yaw, yaw_rate, v, vx, vy, size\] = \[\d+, \d+, \d+\.\d+, \d+\.\d+, (\d+\.\d+), (-?\d+\.\d+), (\d+\.\d+), (\d+\.\d+), (\d+\.\d+), \d+\]'
    match = re.search(pattern, base_info_line)
    if match:
        yaw = float(match.group(1))  # 航向角，单位：弧度
        yaw_rate = float(match.group(2))  # 角速度，单位：弧度/秒
        v = float(match.group(3))
        vx = float(match.group(4))
        vy = float(match.group(5))
    
    # 初始化当前坐标和航向角
    current_x = center_point_utm[0]
    current_y = center_point_utm[1]
    current_yaw = yaw
    
    # 记录yaw的变化量
    yaw_change = 0.0
    
    # 每个时间步长更新位置和航向角
    for t in np.arange(0, prediction_time, time_step):
        # 更新位置和航向角
        current_x += v * np.sin(current_yaw) * time_step
        current_y += v * np.cos(current_yaw) * time_step
        current_yaw += yaw_rate * time_step
        yaw_change += yaw_rate * time_step
    
    return (current_x, current_y), yaw_change

# 定义计算预测后角点位置的函数
def calculate_predicted_corners_dynamic(current_vertices_utm, predicted_center_utm, yaw_change):
    # 计算当前中心点
    current_center_x = np.mean([vertex[0] for vertex in current_vertices_utm])
    current_center_y = np.mean([vertex[1] for vertex in current_vertices_utm])
    
    # 计算每个角点相对于中心点的偏移量（极坐标形式）
    offsets_polar = []
    for vertex in current_vertices_utm:
        dx = vertex[0] - current_center_x
        dy = vertex[1] - current_center_y
        distance = np.sqrt(dx**2 + dy**2)
        angle = np.arctan2(dy, dx)
        offsets_polar.append((distance, angle))
    
    # 计算预测后的角点位置
    predicted_vertices_utm = []
    for distance, angle in offsets_polar:
        # 计算新的角度，考虑车辆的旋转
        new_angle = angle - yaw_change
        predicted_dx = distance * np.cos(new_angle)
        predicted_dy = distance * np.sin(new_angle)
        predicted_x = predicted_center_utm[0] + predicted_dx
        predicted_y = predicted_center_utm[1] + predicted_dy
        predicted_vertices_utm.append((predicted_x, predicted_y))
    
    return predicted_vertices_utm

# 定义生成.lpx文件的函数
def generate_lpx_file(vertices_gps, center_gps, predicted_vertices_gps, predicted_center_gps, output_file):
    # 定义模板行
    template_line = "$,259.8396301269531,44.815437771820164,89.25791535283695,370.0408510638298,7.5,0.000000,0,0,0,0,0,*31"
    
    # 生成.lpx文件内容
    with open(output_file, 'w') as lpx_file:
        # 写入当前角点
        for vertex in vertices_gps:
            line = template_line.replace("44.815437771820164", str(vertex[0])).replace("89.25791535283695", str(vertex[1]))
            lpx_file.write(line + '\n')
        
        # 写入当前中心点
        if center_gps:
            line = template_line.replace("44.815437771820164", str(center_gps[0])).replace("89.25791535283695", str(center_gps[1]))
            lpx_file.write(line + '\n')
        
        # 写入预测后的角点
        for vertex in predicted_vertices_gps:
            line = template_line.replace("44.815437771820164", str(vertex[0])).replace("89.25791535283695", str(vertex[1]))
            lpx_file.write(line + '\n')
        
        # 写入预测后的中心点
        if predicted_center_gps:
            line = template_line.replace("44.815437771820164", str(predicted_center_gps[0])).replace("89.25791535283695", str(predicted_center_gps[1]))
            lpx_file.write(line + '\n')
    
    print(f"Results have been saved to {output_file}")

# 提取日志文本中的时间戳
def extract_timestamp(log_text):
    pattern = r'\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})\.\d+\]'
    match = re.search(pattern, log_text)
    if match:
        # 将冒号替换为下划线
        return match.group(1).replace(':', '_').replace(' ', '_')
    else:
        return "unknown_time"

# 示例用法
if __name__ == "__main__":
    # 用户直接粘贴的日志内容
    log_text = """
    [2025-03-17 11:05:41.633954]<trace>[dynamic_obstacle.cpp:41]:DYNAMIC RAW VERTEX: 678980.659699 4965629.525604
    [2025-03-17 11:05:41.634185]<trace>[dynamic_obstacle.cpp:41]:DYNAMIC RAW VERTEX: 678982.121170 4965639.822419
    [2025-03-17 11:05:41.634346]<trace>[dynamic_obstacle.cpp:41]:DYNAMIC RAW VERTEX: 678986.972549 4965639.133821
    [2025-03-17 11:05:41.634475]<trace>[dynamic_obstacle.cpp:41]:DYNAMIC RAW VERTEX: 678985.511079 4965628.837037
    [2025-03-17 11:05:41.633255]<trace>[motion_prediction_component.cpp:44]:[procPredictionObstacles]parse obstacle [0] base info [id, type, x, y, yaw, yaw_rate, v, vx, vy, size] = [1707, 2, 678983.812500, 4965634.500000, 0.140964, -0.266163, 4.633249, 0.650959, 4.587292, 4]
    """
    
    # 提取时间戳
    timestamp = extract_timestamp(log_text)
    prediction_time = 5  # 预测时间
    output_file = f"{timestamp}_{prediction_time}.lpx"
    
    # 提取并转换当前角点和中心点的UTM坐标
    vertices_utm, vertices_gps, center_utm, center_gps = extract_and_convert_coordinates(log_text)
    print("Current UTM Vertices:", vertices_utm)
    print("Current GPS Vertices:", vertices_gps)
    print("Current UTM Center:", center_utm)
    print("Current GPS Center:", center_gps)
    
    # 预测中心点位置和yaw变化量，采用CVCYR策略
    predicted_center_utm, yaw_change = predict_center_point_cvcyr(log_text, center_utm)
    print("Predicted Center UTM after 5 seconds:", predicted_center_utm)
    print("Yaw change after 5 seconds:", yaw_change)
    
    # 将预测后的中心点转换为GPS坐标
    p = Proj(proj="utm", zone=45, ellps="WGS84")
    lon, lat = transform(p, Proj(proj='latlong', ellps='WGS84'), predicted_center_utm[0], predicted_center_utm[1])
    predicted_center_gps = (lat, lon)
    print("Predicted Center GPS after 5 seconds:", predicted_center_gps)
    
    # 计算预测后的角点位置
    predicted_vertices_utm = calculate_predicted_corners_dynamic(vertices_utm, predicted_center_utm, yaw_change)
    print("Predicted UTM Vertices after 5 seconds:", predicted_vertices_utm)
    
    # 将预测后的角点转换为GPS坐标
    predicted_vertices_gps = []
    for x, y in predicted_vertices_utm:
        lon, lat = transform(p, Proj(proj='latlong', ellps='WGS84'), x, y)
        predicted_vertices_gps.append((lat, lon))
    
    # 生成.lpx文件
    generate_lpx_file(vertices_gps, center_gps, predicted_vertices_gps, predicted_center_gps, output_file)