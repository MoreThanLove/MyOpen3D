# 表面重建
import open3d as o3d
import numpy as np
import copy
import ComputerVision.pointcloud.open3d_tutorial as o3dtut

"""
常见的重建方法：
    
点云表面重建的基本流程：
    输入点云数据——>采样、去除噪声点、平滑处理——>估计法线、获取曲率信息——>使用重建方法进行重建
显式重建和隐式重建：
    显式重建：不改变点云的位置以及个数，只添加对应的拓扑关系。
        1.Delaunay三角剖分
            基本思路：每个三角形的外接圆的内部不能包含其它的任何顶点，从而避免了产生细长的三角形。
            算法流程：
                (1)先将点云通过法线投影到某一二维坐标平面内
                (2)然后对投影得到的点云做平面内的三角化，从而得到各点的拓扑连接关系(基于Delaunay三角剖分)
                (3)最后根据平面内投影点的拓扑连接关系确定各原始三维点间的拓扑连接，所得三角网格即为重建得到的曲面模型
        2.alpha-shape
        3.ball pivoting：使用半径为r的球在表面进行滚动，如果有3个点能够支撑该球，则这3个点就会形成一个三角拓扑关系。
    隐式重建：对于表面F上的每个数据点x，首先定义一个点函数f(x)，根据f(x)可以生成其它点。
        1.径向基函数法
            径向基函数：一般用多个径向基函数U(r)(r=x-c，c为x的中心)来拟合一个复杂的函数F(x)，F(x)=∑W(i)U(r)
            常见的径向基函数：
                高斯函数：U(r)=exp{-r²/a²}
                多二次函数：U(r)=1+a²r²
                逆二次函数：U(r)=(1+a²r²)1/2
        2.移动最小二乘法
        3.泊松重建法
            
"""

# 1.显式建模：alpha-shape算法
# pcd1 = o3d.geometry.pointcloud(o3d.io.read_point_cloud('file/bun_zipper.ply'))
# 使用alpha-shape算法对点云进行重建，变为mesh
# mesh1 = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd1, 0.01)
# o3d.visualization.draw_geometries([mesh1], width=800, height=800)

# 2.显示建模：滚球法(Ball Pivoting)
pcd2 = o3d.geometry.PointCloud(o3d.io.read_point_cloud('file/bun_zipper.ply'))
pcd2.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))
radii = [0.005, 0.01, 0.02, 0.04]
mesh2 = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd2, o3d.utility.DoubleVector(radii))
o3d.visualization.draw_geometries([mesh2], width=800, height=800)

# 3.隐式重建：泊松重建
# pcd3 = o3d.geometry.pointcloud(o3d.io.read_point_cloud('file/bun_zipper.ply'))
# o3d.visualization.draw_geometries([pcd3], width=800, height=800, point_show_normal=True)
# pcd3.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=5))
# with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
#     mesh3, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd3, depth=9)
# mesh3.paint_uniform_color([0, 0, 1])
# o3d.visualization.draw_geometries([mesh3], width=800, height=800)
# # 过滤低密度的点
# vertices_to_remove = densities < np.quantile(densities, 0.01)
# mesh3.remove_vertices_by_mask(vertices_to_remove)
#
# o3d.visualization.draw_geometries([mesh3], width=800, height=800)

# pcd4 = o3d.io.read_point_cloud('file/eagle.ply')
# o3d.visualization.draw_geometries([pcd4], width=800, height=800)
# with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
#     mesh4, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd4, depth=9)
# o3d.visualization.draw_geometries([mesh4], width=800, height=800)
# vertices_to_remove = densities < np.quantile(densities, 0.01)
# mesh4.remove_vertices_by_mask(vertices_to_remove)
# o3d.visualization.draw_geometries([mesh4], width=800, height=800)

# 4.隐式重建：Marching squares算法(MC重建)
