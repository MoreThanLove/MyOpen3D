import open3d as o3d
import numpy as np
import copy

# 常见的点云处理方法(一)
# 一、去噪算法
"""
 1.Statistical Outlier Removal(SOR:统计滤波)
    步骤：对于点云中的点D(i)以及K，可以获取距离D(i)最近的K个点，计算这K个点到D(i)的平均距离dis(i)(1<=i<=N)，
其中N为点云的个数，则认为随机变量dis(i)服从Guess(u,t)，其中u为dis(i)的均值，t为dis(i)的标准差。对于某个点S，
若dis(s)>u+ct(其中c为标准差倍数)，则认为点S为噪声点，将其移除。
"""
pcd1 = o3d.io.read_point_cloud('file/knotmesh.ply')
# # print(len(pcd1.points))  1440
# pcd1_noise_removed, pcd1_remained_idx = pcd1.remove_statistical_outlier(10, 1)  # 移除噪声点后的点云对象，剩余点的索引
# # print(len(pcd1_remained_idx))  1173
# o3d.visualization.draw_geometries([pcd1], width=800, height=800)
# o3d.visualization.draw_geometries([pcd1_noise_removed], width=800, height=800)

"""
 2.Radius Outlier Removal(ROR:半径滤波)
    步骤：通常认为噪声点周围的点是稀疏的，于是遍历每一个点Di，对于给定的半径R以及最少点阈值MinPts，如果
以Di为中心，R为半径的球体中的点的数量小于MinPts，则认为Di是噪声点。
"""
pcd2 = copy.deepcopy(pcd1)
# 如果球心为Di，半径为20的球体内包含的点的个数小于10，则认为Di是噪声点
# pcd2_noise_removed, pcd2_remained_idx = pcd2.remove_radius_outlier(10, 20)
# print(len(pcd2.points))
# print(len(pcd2_remained_idx))
# o3d.visualization.draw_geometries([pcd2], width=800, height=800)
# o3d.visualization.draw_geometries([pcd2_noise_removed], width=800, height=800)


# 二、下采样算法
"""
  1.随机采样(Random Sample)
"""
pcd3 = o3d.io.read_point_cloud('file/bun_zipper.ply')
pcd3_down_sample = pcd3.random_down_sample(0.2)  # 下采样并保留20%的点
o3d.visualization.draw_geometries([pcd3_down_sample], width=800, height=800)

"""
  2.均匀采样(Uniform Sample)
"""
pcd4 = copy.deepcopy(pcd3)
pcd4_down_sample = pcd4.uniform_down_sample(10)  # 每隔20个点采样一次
o3d.visualization.draw_geometries([pcd4_down_sample], width=800, height=800)

"""
  3.体素采样(Voxel Sample)
"""
pcd5 = copy.deepcopy(pcd4)
pcd5_down_sample = pcd5.voxel_down_sample(0.01)
o3d.visualization.draw_geometries([pcd5_down_sample], width=800, height=800)

"""
  4.曲率采样
"""


# def vector_angle(x, y):
#     Lx = np.sqrt(x.dot(x))
#     Ly = (np.sum(y ** 2, axis=1)) ** (0.5)
#     cos_angle = np.sum(x * y, axis=1) / (Lx * Ly)
#     angle = np.arccos(cos_angle)
#     angle2 = angle * 360 / 2 / np.pi
#     return angle2
#
#
# knn_num = 10  # 自定义参数值(邻域点数)
# angle_thre = 30  # 自定义参数值(角度值)
# N = 5  # 自定义参数值(每N个点采样一次)
# C = 10  # 自定义参数值(采样均匀性>N)
#
# pcd6 = copy.deepcopy(pcd3)
# point = np.asarray(pcd6.points)
# point_size = point.shape[0]
# tree = o3d.geometry.KDTreeFlann(pcd6)
# o3d.geometry.pointcloud.estimate_normals(
#     pcd6, search_param=o3d.geometry.KDTreeSearchParamKNN(knn=knn_num))
# normal = np.asarray(pcd6.normals)
# normal_angle = np.zeros((point_size))
# for i in range(point_size):
#     [_, idx, dis] = tree.search_knn_vector_3d(point[i], knn_num + 1)
#     current_normal = normal[i]
#     knn_normal = normal[idx[1:]]
#     normal_angle[i] = np.mean(vector_angle(current_normal, knn_normal))
#
# point_high = point[np.where(normal_angle >= angle_thre)]
# point_low = point[np.where(normal_angle < angle_thre)]
# pcd_high = o3d.geometry.pointcloud()
# pcd_high.points = o3d.utility.Vector3dVector(point_high)
# pcd_low = o3d.geometry.pointcloud()
# pcd_low.points = o3d.utility.Vector3dVector(point_low)
# pcd_high_down = o3d.geometry.pointcloud.uniform_down_sample(pcd_high, N)
# pcd_low_down = o3d.geometry.pointcloud.uniform_down_sample(pcd_low, C)
# pcd_finl = o3d.geometry.pointcloud()
# pcd_finl.points = o3d.utility.Vector3dVector(np.concatenate((np.asarray(pcd_high_down.points),
#                                                              np.asarray(pcd_low_down.points))))
# o3d.visualization.draw_geometries([pcd_finl], width=800, height=800)

"""
  5.最远点采样：首先随机选择一个点，其次，在剩下点中寻找最远的点，再去在剩下点中找到同时离这两个点最远的点……
以此类推，知道满足采样点的个数。
"""

"""
  6.曲面均匀采样
"""
pcd7 = o3d.io.read_triangle_mesh('file/knotmesh.ply')
pcd7_down_sample = pcd7.sample_points_uniformly(300)  # 保留300个网格
o3d.visualization.draw_geometries([pcd7_down_sample], width=800, height=800)

"""
  7.泊松磁盘采样
"""
pcd8 = o3d.geometry.TriangleMesh(o3d.io.read_triangle_mesh('file/knotmesh.ply'))
pcd8_down_sample = pcd8.sample_points_poisson_disk(1000)
o3d.visualization.draw_geometries([pcd7_down_sample], width=800, height=800)