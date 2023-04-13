# Open3d中的数据结构
import open3d as o3d
import numpy as np

# 1.八叉树
# pcd1 = o3d.io.read_point_cloud('file/bun_zipper.ply')
# pcd1.paint_uniform_color([1, 0, 0])
# # 建立八叉树，最大深度为10
# octree = o3d.geometry.Octree(max_depth=10)
# # 将点云对象转为八叉树，size_expand指定结点的大小
# octree.convert_from_point_cloud(pcd1, size_expand=0.1)
# # 可视化
# o3d.visualization.draw_geometries([octree], window_name='八叉树', width=1000, height=1000)

# 2.KDTree
# pcd2 = o3d.geometry.pointcloud()  # 创建原始点云对象
# pcd2.points = o3d.utility.Vector3dVector(np.random.normal(-3, 5, size=(500, 3)))
# pcd2.paint_uniform_color([1, 0, 0])
# pcd3 = o3d.geometry.pointcloud()  # 创建异于原始点云的点云对象
# pcd3.points = o3d.utility.Vector3dVector(np.random.normal(3, 5, size=(500, 3)))
# pcd3.paint_uniform_color([0, 1, 0])
# # 根据pcd2生成KDTree
# kdtree = o3d.geometry.KDTreeFlann(pcd2)
# # 存储点云中相同部分点云坐标
# ptsIdx = []
# # KNN搜索用到的参数
# k = 1
# # 距离阈值
# dist_max = 1
# # 获取pcd3中的点云个数
# points = np.array(pcd3.points)
# point_num = points.shape[0]
# # 遍历pcd3中的每一个点
# for i in range(point_num):
#     # k为返回点的个数，idx为索引，dist为距离
#     k, idx, dist = kdtree.search_knn_vector_3d(pcd3.points[i], k)
#     # print(k, idx, dist)
#     # 如果距离小于阈值，说明是相同的点，加入到ptsIdx中
#     if dist[0] < dist_max:
#         ptsIdx.append(i)
# # 可视化操作
# same_part = pcd3.select_by_index(ptsIdx)
# same_part.paint_uniform_color([1, 0, 0])
# diff_part = pcd3.select_by_index(ptsIdx, invert=True)
# diff_part.paint_uniform_color([0, 1, 0])
# o3d.visualization.draw_geometries([same_part, diff_part], width=1080, height=800)
