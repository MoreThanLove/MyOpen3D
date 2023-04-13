# env      : pytorch
# coding: utf8
# @Time    : 2023/2/10 1:09
# @Author  : WangMiao
# @Email   : 416115905@qq.com
# @Software: PyCharm
# A Little Every Day
# 点云分割
import open3d as o3d
import numpy as np
from matplotlib import pyplot as plt

"""
1.平面分割/平面拟合(基于RANSAC)
    Random Sample Consensus:随机采样一致性
    算法步骤：
        1.首先确定拟合数学模型
        2.确定拟合模型所必要的点个数n
        3.从点云数据中随机选取n个点计算数学模型
        4.计算inliner数量：计算所有点与3中的数学模型中的距离di，若di<d(人为设定)，则认为i点为inliner点，否则为outliner点
        5.重复3、4步，保留inliner点个数最多的那个模型
"""
# pcd1 = o3d.geometry.pointcloud(o3d.io.read_point_cloud('file/bun_zipper.ply'))
# pcd1 = pcd1.uniform_down_sample(10)
# plan_parameters, inliers = pcd1.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
# a, b, c, d = plan_parameters
# print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
# inlier_cloud = pcd1.select_by_index(inliers)
# inlier_cloud.paint_uniform_color([1, 0, 0])
# outlier_cloud = pcd1.select_by_index(inliers, invert=True)
# outlier_cloud.paint_uniform_color([0, 1, 0])
# o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

"""
2.聚类算法(DBSCAN):Density-based spatial clusering of applications，基于密度的聚类算法，根据周围点的密度，将点进行聚类划分
    具体情况可以参考周志华的《机器学习》中对应的部分。
"""
pcd2 = o3d.geometry.PointCloud(o3d.io.read_point_cloud('file/bun_zipper.ply'))
o3d.visualization.draw_geometries([pcd2], width=800, height=800)
with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(pcd2.cluster_dbscan(eps=0.05, min_points=10))
    print(labels)
min_label = labels.min()
max_label = labels.max()
pcList = []
colors = plt.get_cmap('tab20')(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
pcd2.colors = o3d.utility.Vector3dVector(colors[:, :3])
for i in range(min_label, max_label+1):
    label_index = np.where(labels == i)
    temp_pc = pcd2.select_by_index(np.array(label_index)[0])
    pcList.append(temp_pc)
    o3d.visualization.draw_geometries([temp_pc])
o3d.visualization.draw_geometries([pcd2])
