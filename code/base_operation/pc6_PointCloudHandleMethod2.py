# 常见的点云处理方法(二)
import open3d as o3d
import numpy as np
import copy


# pcd1 = o3d.geometry.pointcloud()
# pcd2 = o3d.geometry.pointcloud()
# pcd1.points = o3d.utility.Vector3dVector(np.random.randint(-5, 5, size=(100, 3)))
# pcd1.paint_uniform_color([1, 0, 0])
# pcd2.points = o3d.utility.Vector3dVector(np.random.randint(3, 10, size=(100, 3)))
# pcd2.paint_uniform_color([0, 1, 0])


# o3d.visualization.draw_geometries([pcd1, pcd2], width=800, height=800)


# 1.计算点云2中的点云到点云1中的点云的最近距离
# dist = pcd2.compute_point_cloud_distance(pcd1)
# pstidx = [i for i, distance in enumerate(dist) if dist[i] <= 1.5]  # 如果dist[i]小于等于1.5则认为pcd2中的第i个点云属于pcd1
# same_part = pcd2.select_by_index(pstidx)  # pcd2中属于pcd1中的点云
# same_part.paint_uniform_color([1, 0, 0])
# diff_part = pcd2.select_by_index(pstidx, invert=True)  # pcd2中不属于pcd1中的点云
# diff_part.paint_uniform_color([0, 1, 0])
# o3d.visualization.draw_geometries([same_part, diff_part], width=800, height=800)


# 2.点云切片(求给定两个平面之间的点云集合)
def get_plane(point1, point2, point3):
    """根据三个点获取对应的平面方程的参数"""
    vector1 = point2 - point1
    vector2 = point3 - point1
    A = vector1[1] * vector2[2] - vector2[1] * vector1[2]
    B = vector1[2] * vector2[0] - vector1[0] * vector2[2]
    C = vector1[0] * vector2[1] - vector2[0] * vector1[1]
    D = -(A * point1[0] + B * point1[1] + C * point1[2])
    return A, B, C, D


pcd3 = o3d.io.read_point_cloud('file/bun_zipper.ply')
A, B, C, D = get_plane(pcd3.points[341], pcd3.points[3221], pcd3.points[19906])
D1 = D + 0.00002
D2 = D - 0.00015
idx = []
for i, point in enumerate(pcd3.points):
    if (A * point[0] + B * point[1] + C * point[2] + D1) * (A * point[0] + B * point[1] + C * point[2] + D2) < 0:
        """如果在两个平面之间就将点的索引加入到idx中"""
        idx.append(i)
sliced_points = pcd3.select_by_index(idx)
sliced_points.paint_uniform_color([1, 0, 0])
rest_points = pcd3.select_by_index(idx, invert=True)
rest_points.paint_uniform_color([0, 1, 0])
o3d.visualization.draw_geometries([sliced_points], width=800, height=800)


# 3.包围盒
# pcd4 = o3d.geometry.pointcloud(o3d.io.read_point_cloud('file/bun_zipper.ply'))
# # AABB包围盒:分别获取每个轴的最大值和最小值，然后围起来即可
# aabb = pcd4.get_axis_aligned_bounding_box()
# # 包围盒颜色
# aabb.color = (1, 0, 0)
# # 包围盒中心点
# [center_x, center_y, center_z] = aabb.get_center()
# # 包围盒的8个顶点坐标
# vertex = aabb.get_box_points()
# # 包围盒的边
# side = aabb.get_extent()
# # 展示
# o3d.visualization.draw_geometries([aabb, pcd4], width=800, height=800)
#
# # OBB包围盒
# obb = pcd4.get_oriented_bounding_box()
# obb.color = (0, 0, 1)
# # 展示
# o3d.visualization.draw_geometries([obb, pcd4], width=800, height=800)

# 4.去重心化操作
# pcd5 = o3d.geometry.pointcloud(o3d.io.read_point_cloud('file/bun_zipper.ply'))
# o3d.visualization.draw_geometries([pcd5], width=800, height=800)
# center = pcd5.get_center()
# for i, point in enumerate(pcd5.points):
#     pcd5.points[i] = point - center
# o3d.visualization.draw_geometries([pcd5], width=800, height=800)
