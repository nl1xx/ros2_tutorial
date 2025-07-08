import open3d as o3d
import numpy as np
from matplotlib import pyplot as plt

# 加载点云文件
pcd = o3d.io.read_point_cloud("./table_scene_lms400.pcd")
print(pcd)
print(np.asarray(pcd.points))

# 可视化
o3d.visualization.draw_geometries([pcd], window_name="原始点云")

# 体素栅格降采样
print("体素降采样前点数:", len(pcd.points))
# 定义体素大小为0.01米
voxel_size = 0.02
downsampled_pcd = pcd.voxel_down_sample(voxel_size)
print("体素降采样后点数:", len(downsampled_pcd.points))
o3d.visualization.draw_geometries([downsampled_pcd], window_name="降采样后的点云")

# 统计离群点移除
print("进行统计离群点移除...")
# nb_neighbors: 指定邻域点的数量
# std_ratio: 标准差乘数阈值
cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

# 可视化结果，将离群点染成红色
inlier_cloud = pcd.select_by_index(ind)
outlier_cloud = pcd.select_by_index(ind, invert=True)
outlier_cloud.paint_uniform_color([1, 0, 0])  # 红色
inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])  # 灰色
o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud], window_name="离群点移除结果")

# RANSAC 参数
distance_threshold = 1     # 内点到平面的最大距离
ransac_n = 3               # 拟合平面所需的点数
num_iterations = 1000      # 迭代次数

# 平面分割
plane_model, inliers = inlier_cloud.segment_plane(distance_threshold, ransac_n, num_iterations)
[a, b, c, d] = plane_model
print(f"平面方程: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

# 将平面内的点染成绿色，平面外的点保留原色
plane_cloud = inlier_cloud.select_by_index(inliers)
plane_cloud.paint_uniform_color([0, 1, 0])  # 绿色
objects_cloud = inlier_cloud.select_by_index(inliers, invert=True)
o3d.visualization.draw_geometries([plane_cloud, objects_cloud], window_name="平面分割结果")

# 欧式聚类提取
# 使用DBSCAN进行聚类
eps = 0.1            # 同一簇中两点之间的最大距离
min_points = 100     # 形成一个簇所需的最小点数
labels = np.array(plane_cloud.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))

max_label = labels.max()
print(f"检测到 {max_label + 1} 个聚类。")

# 为不同的聚类赋予不同的颜色
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))[:, :3]  # 取RGB通道
colors[labels < 0] = 0  # 噪声点为黑色
plane_cloud.colors = o3d.utility.Vector3dVector(colors)
o3d.visualization.draw_geometries([plane_cloud], window_name="欧式聚类结果")
