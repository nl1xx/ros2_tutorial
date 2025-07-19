import open3d as o3d
import numpy as np
import random

# 加载点云文件
pcd = o3d.io.read_point_cloud("./table_scene_lms400.pcd")
print(pcd)
print(np.asarray(pcd.points))

# 可视化原始点云
o3d.visualization.draw_geometries([pcd], window_name="原始点云")

# 体素栅格降采样
print("体素降采样前点数:", len(pcd.points))
voxel_size = 0.02
downsampled_pcd = pcd.voxel_down_sample(voxel_size)
print("体素降采样后点数:", len(downsampled_pcd.points))
o3d.visualization.draw_geometries([downsampled_pcd], window_name="降采样后的点云")

# 统计离群点移除
print("进行统计离群点移除...")
nb_neighbors = 20   # 邻域点的数量
std_ratio = 2.0     # 标准差乘数阈值
cl, ind = downsampled_pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)

# 可视化结果，将离群点染成红色
inlier_cloud = downsampled_pcd.select_by_index(ind)
outlier_cloud = downsampled_pcd.select_by_index(ind, invert=True)
outlier_cloud.paint_uniform_color([1, 0, 0])  # 红色
inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])  # 灰色
o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud], window_name="离群点移除结果")

# RANSAC 参数
distance_threshold = 0.01  # 内点到平面的最大距离
ransac_n = 3  # 拟合平面所需的点数
num_iterations = 1000  # 迭代次数

# 初始化剩余点云
remaining_cloud = inlier_cloud
planes = []  # 用于存储检测到的平面

# 循环检测平面，直到剩余点云的点数小于某个阈值
while len(remaining_cloud.points) > 1000:  # 假设剩余点云的点数小于1000时停止
    plane_model, inliers = remaining_cloud.segment_plane(distance_threshold, ransac_n, num_iterations)
    [a, b, c, d] = plane_model
    print(f"检测到平面: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    # 提取平面内的点
    plane_cloud = remaining_cloud.select_by_index(inliers)
    plane_cloud.paint_uniform_color([random.randint(0, 1), random.randint(0, 1), random.randint(0, 1)])
    planes.append(plane_cloud)

    # 移除平面内的点，得到剩余点云
    remaining_cloud = remaining_cloud.select_by_index(inliers, invert=True)

# 可视化所有检测到的平面和剩余点云
o3d.visualization.draw_geometries(planes + [remaining_cloud], window_name="所有检测到的平面和剩余点云")
