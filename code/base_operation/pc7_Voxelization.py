# env      : pytorch
# coding: utf8
# @Time    : 2023/2/8 21:22
# @Author  : WangMiao
# @Email   : 416115905@qq.com
# @Software: PyCharm
# A Little Every Day
# 体素化
import open3d as o3d
import numpy as np

"""
体素的概念：体素是像素、体积和元素的组合词，相当于三维空间中的像素。
体素的优点：(1)普通点云数据之间是离散独立的，而体素数据中每个体素都与其周围的体素存在关系。
          (2)体素具有降采样的表现形式，能够处理大规模数据。
          (3)二维中的技术可以迁移到体素中。
体素的缺点：
          (1)信息丢失，与分辨率有关。
          (2)稀疏的点云数据会构建出很多空体素。
"""

# 1.从点云构建体素
pcd1 = o3d.geometry.PointCloud(o3d.io.read_point_cloud('file/bun_zipper.ply'))
pcd1.paint_uniform_color([150 / 560, 205 / 560, 205 / 560])
pcd1.estimate_normals()
voxel1 = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd1, voxel_size=0.002)
o3d.visualization.draw_geometries([voxel1], width=800, height=800)

# 2.从网格构建体素
# mesh1 = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd1, radii=o3d.utility.DoubleVector(
#     np.array([0.005, 0.01, 0.02, 0.04])))
# mesh1.paint_uniform_color([150 / 560, 205 / 560, 205 / 560])
# o3d.visualization.draw_geometries([mesh1], width=800, height=800)
# voxel2 = o3d.geometry.VoxelGrid.create_from_triangle_mesh(mesh1, voxel_size=0.002)
# o3d.visualization.draw_geometries([voxel2], width=800, height=800)

# 3.检查测试点是否在体素内
output = voxel1.check_if_included(o3d.utility.Vector3dVector(pcd1.points))
# print(output)

# 4.获取每个体素的信息
voxel1_info = voxel1.get_voxels()


# print(voxel1_info[1])
# Voxel with grid_index: (31, 1, 55), color: (0.267857, 0.366071, 0.366071)，第一个表示该体素在三维坐标中的位置，第二个为颜色

# 5.根据体素构建八叉树
# octree = o3d.geometry.Octree(max_depth=9)
# octree.create_from_voxel_grid(voxel1)
# o3d.visualization.draw_geometries([octree], width=800, height=800)

# 6.体素下采样(点云下采样的一种方式:点被存储到体素中，每个占用的体素通过平均内部所有的点生成一个点)
# downpcd = pcd1.voxel_down_sample(voxel_size=0.005)
# o3d.visualization.draw_geometries([downpcd], width=800, height=800)

# 7.体素雕刻
def xyz_spherical(xyz):
    x = xyz[0]
    y = xyz[1]
    z = xyz[2]
    r = np.sqrt(x * x + y * y + z * z)
    r_x = np.arccos(y / r)
    r_y = np.arctan2(z, x)
    return [r, r_x, r_y]


def get_rotation_matrix(r_x, r_y):
    rot_x = np.asarray([[1, 0, 0], [0, np.cos(r_x), -np.sin(r_x)],
                        [0, np.sin(r_x), np.cos(r_x)]])
    rot_y = np.asarray([[np.cos(r_y), 0, np.sin(r_y)], [0, 1, 0],
                        [-np.sin(r_y), 0, np.cos(r_y)]])
    return rot_y.dot(rot_x)


def get_extrinsic(xyz):
    rvec = xyz_spherical(xyz)
    r = get_rotation_matrix(rvec[1], rvec[2])
    t = np.asarray([0, 0, 2]).transpose()
    trans = np.eye(4)
    trans[:3, :3] = r
    trans[:3, 3] = t
    return trans


def preprocess(model):
    min_bound = model.get_min_bound()
    max_bound = model.get_max_bound()
    center = min_bound + (max_bound - min_bound) / 2.0
    scale = np.linalg.norm(max_bound - min_bound) / 2.0
    vertices = np.asarray(model.vertices)
    vertices -= center
    model.vertices = o3d.utility.Vector3dVector(vertices / scale)
    return model


def voxel_carving(mesh, cubic_size, voxel_resolution, w=300, h=300):
    mesh.compute_vertex_normals()
    camera_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1.0,
                                                            resolution=10)

    # Setup dense voxel grid.
    voxel_carving = o3d.geometry.VoxelGrid.create_dense(
        width=cubic_size,
        height=cubic_size,
        depth=cubic_size,
        voxel_size=cubic_size / voxel_resolution,
        origin=[-cubic_size / 2.0, -cubic_size / 2.0, -cubic_size / 2.0],
        color=[1.0, 0.7, 0.0])

    # Rescale geometry.
    camera_sphere = preprocess(camera_sphere)
    mesh = preprocess(mesh)

    # Setup visualizer to render depthmaps.
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=w, height=h, visible=False)
    vis.add_geometry(mesh)
    vis.get_render_option().mesh_show_back_face = True
    ctr = vis.get_view_control()
    param = ctr.convert_to_pinhole_camera_parameters()

    # Carve voxel grid.
    centers_pts = np.zeros((len(camera_sphere.vertices), 3))
    for cid, xyz in enumerate(camera_sphere.vertices):
        # Get new camera pose.
        trans = get_extrinsic(xyz)
        param.extrinsic = trans
        c = np.linalg.inv(trans).dot(np.asarray([0, 0, 0, 1]).transpose())
        centers_pts[cid, :] = c[:3]
        ctr.convert_from_pinhole_camera_parameters(param)

        # Capture depth image and make a point cloud.
        vis.poll_events()
        vis.update_renderer()
        depth = vis.capture_depth_float_buffer(False)

        # Depth map carving method.
        voxel_carving.carve_depth_map(o3d.geometry.Image(depth), param)
        print("Carve view %03d/%03d" % (cid + 1, len(camera_sphere.vertices)))
        # o3d.visualization.draw_geometries([voxel_carving], width=800, height=800)
    vis.destroy_window()

    return voxel_carving


if __name__ == "__main__":
    mesh = o3d.io.read_triangle_mesh('file/ArmadilloMesh.ply')
    cubic_size = 2.0
    voxel_resolution = 128.0

    carved_voxels = voxel_carving(mesh, cubic_size, voxel_resolution)
    print("Carved voxels ...")
    print(carved_voxels)
    o3d.visualization.draw([carved_voxels])
