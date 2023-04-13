import open3d as o3d
import matplotlib.pyplot as plt
from d2l import torch as d2l

# 三维数据的表达

# 1.open3d中点云的读取和保存
# pcd = o3d.io.read_point_cloud('file/bun090.ply')  # 读取点云文件
# o3d.visualization.draw(pcd)  # 展示
# o3d.io.write_point_cloud('file/copy_bun090.ply', pcd)  # 保存点云文件

# 2.open3d中网格的读取和保存
# mesh_data = o3d.data.KnotMesh()  # 系统提供的网格数据
# mesh = o3d.io.read_triangle_mesh(mesh_data.path)  # 读取网格文件
# o3d.visualization.draw(mesh)  # 展示
# o3d.io.write_triangle_mesh('file/knotmesh.ply', mesh)  # 保存网格文件

# 3.open3d中RGBD(深度图)数据的创建以及点云转换
# color_raw = o3d.io.read_image('rgb_img_path')  # 读取RGB图像
# depth_raw = o3d.io.read_image('depth_img_path')  # 读取Depth Map
# rgbd_img = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw)  # 根据RGB和Depth Map创建深度图
# plt.subplot(1, 2, 1)
# plt.imshow(rgbd_img.color)  # 展示RGB图
# plt.subplot(1, 2, 2)
# plt.imshow(rgbd_img.depth)  # 展示Depth Map
# plt.show()
# 将深度图转为点云
# pcd = o3d.geometry.pointcloud.create_from_rgbd_image(rgbd_img, o3d.camera.PinholeCameraIntrinsic(
#    o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
# pcd.transform([1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1])
# o3d.visualization.draw_geometries([pcd])

