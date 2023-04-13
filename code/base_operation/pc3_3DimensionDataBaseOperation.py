# 三维数据的基本操作
import open3d as o3d
import numpy as np
import copy

# 1.平移:translate
mesh1 = o3d.geometry.TriangleMesh.create_coordinate_frame()
mesh1_tx = copy.deepcopy(mesh1).translate((1, 0, 0))  # 向x轴移动1个单位
mesh1_ty = copy.deepcopy(mesh1).translate((0, 2, 0))  # 向y轴移动2个单位
mesh1_tz = copy.deepcopy(mesh1).translate((0, 0, 4))  # 向z轴移动4个单位
print(f'Center of mesh1:{mesh1.get_center()}')
print(f'Center of mesh1_tx:{mesh1_tx.get_center()}')
print(f'Center of mesh1_ty:{mesh1_ty.get_center()}')
print(f'Center of mesh1_tz:{mesh1_tz.get_center()}')
o3d.visualization.draw_geometries([mesh1, mesh1_tx, mesh1_ty, mesh1_tz], window_name='Translate', height=800, width=800)

# 2.平移到指定位置
mesh2 = o3d.geometry.TriangleMesh.create_coordinate_frame()
mesh2_p = copy.deepcopy(mesh2).translate((3, 3, 3), relative=False)
print(f'Center of mesh2:{mesh2.get_center()}')
print(f'Center of mesh2_p:{mesh2_p.get_center()}')
o3d.visualization.draw_geometries([mesh2, mesh2_p], window_name='MoveToAbsulatePosition', height=800, width=800)

# 3. 欧拉角旋转
mesh3 = o3d.geometry.TriangleMesh.create_coordinate_frame()
mesh3_rotate = copy.deepcopy(mesh3).rotate(mesh3.get_rotation_matrix_from_xyz((np.pi / 6, np.pi / 4, np.pi / 2)),
                                           center=(0, 0, 0)).translate((3, 0, 0)).translate((0, 2, 0))  # center指定旋转中心
o3d.visualization.draw_geometries([mesh3, mesh3_rotate], window_name='Rotate', width=800, height=800)

# 4. 缩放
mesh4 = o3d.geometry.TriangleMesh.create_coordinate_frame()
mesh4_zoom = copy.deepcopy(mesh4).scale(0.5, center=mesh4.get_center()).translate((3, 0, 0))
o3d.visualization.draw_geometries([mesh4, mesh4_zoom], window_name='Zoom',
                                  height=800, width=800)
