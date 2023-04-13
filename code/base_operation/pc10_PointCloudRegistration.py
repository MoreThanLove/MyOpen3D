# env      : pytorch
# coding: utf8
# @Time    : 2023/2/14 16:00
# @Author  : WangMiao
# @Email   : 416115905@qq.com
# @Software: PyCharm
# A Little Every Day
# 点云配准
import open3d as o3d
import numpy as np

"""
    实际场景中，我们很难一次性完整的获得扫描的场景或者物体的全部信息，因为我们需要进行多视角的扫描点云数据，在数据处理中需要将各片点云数据
纳入到一个统一的坐标系中再进行后续的应用，这就是点云的配准/拼接。
    同名点：指在点云中处在真实世界同一位置的点
        当初始状态较好时：一般根据欧式距离来筛选
        当初始状态不好时：可以使用点特征描述符(特征向量)的二范数来筛选，特征向量一般包括(坐标、法线、曲率、颜色、纹理等)
            两大算法：PFH(Point Feature Histogram):点特征直方图
                    FPFP(Fast Point Feature Histogram):快速点特征直方图
                    参考：D:\File\机器学习\机器学习资料\知识图片\PFH&FPFH
    迭代最近点（Iterative Closest Point, 简称ICP）算法流程：
        1.选点：
            确定参与到配准过程中的点集。
        2.匹配确定同名点对：
            ICP中以两片点云中欧式空间距离最小的点对为同名点。
            FPFH中使用SAC—IA算法配准：
                在待配准点云中随机选取N个点，要求N个点中点与点之间的距离大于d，然后N个点中的每个点都根据FPFH从配准点云中获取M个点，
            再从M个点中随机选取一个点作为配准点，因此一次流程最终获得N对同名点，经过W次流程，选取匹配最好的同名点作为最终结果。
        3.非线性优化求解：
            采用SVD或者四元数求解变换参数R和平移参数t，优化函数为∑Wi||(RPi+t)-Qi||²，i∈[1,n]
            详情参照：D:\File\机器学习\机器学习资料\知识图片\ICP_Solution
        4.变换：
            将求解的变换参数应用到待配准点云上。
        5.迭代：
            计算此时的状态参数判断配准是否完成。以前后两次参数迭代变化的差或者RMSE值是否小于给定阈值为迭代终止条件。否则返回(1)
"""


# 1. ICP
# 获取点云数据
# source_cloud = o3d.geometry.pointcloud(o3d.io.read_point_cloud('file/cloud_bin_0.pcd'))
# target_cloud = o3d.geometry.pointcloud(o3d.io.read_point_cloud('file/cloud_bin_1.pcd'))
# source_cloud.paint_uniform_color([1.0, 0.706, 0])
# target_cloud.paint_uniform_color([0, 0.651, 0.929])
# threshold = 0.02  # RMSE残差阈值，小于该残差阈值，迭代终止
#
# # 初始位姿
# trans_init = np.array([
#     [0.862, 0.011, -0.507, 0.5],
#     [-0.139, 0.967, -0.213, 0.7],
#     [0.487, 0.255, 0.835, -1.4],
#     [0.0, 0.0, 0.0, 1.0]
# ])
# # 显示未配准点云
# o3d.visualization.draw_geometries([source_cloud, target_cloud], zoom=0.4459, front=[0.9288, -0.2951, -0.2242],
#                                   lookat=[1.6784, 2.0612, 1.4451], up=[-0.3402, -0.9189, -0.1996], width=1080,
#                                   height=900)
# # 点到点的ICP
# result = o3d.pipelines.registration.registration_icp(source_cloud, target_cloud, threshold, trans_init,
#                                                      o3d.pipelines.registration.TransformationEstimationPointToPoint())
# print(result)
# print(f"Transformation is:{result.transformation}")
# # 显示点到点的配准结果
# source_cloud.transform(result.transformation)
# o3d.visualization.draw_geometries([source_cloud, target_cloud], zoom=0.4459, front=[0.9288, -0.2951, -0.2242],
#                                   lookat=[1.6784, 2.0612, 1.4451], up=[-0.3402, -0.9189, -0.1996], width=1080,
#                                   height=900)
# # 重置点云
# source_cloud = o3d.geometry.pointcloud(o3d.io.read_point_cloud('file/cloud_bin_0.pcd'))
# target_cloud = o3d.geometry.pointcloud(o3d.io.read_point_cloud('file/cloud_bin_1.pcd'))
# source_cloud.paint_uniform_color([1.0, 0.706, 0])
# target_cloud.paint_uniform_color([0, 0.651, 0.929])
# # 点到面的ICP
# result = o3d.pipelines.registration.registration_icp(source_cloud, target_cloud, threshold, trans_init,
#                                                      o3d.pipelines.registration.TransformationEstimationPointToPlane())
# print(result)
# print(result)
# print(f"Transformation is:{result.transformation}")
# # 显示点到面的配准结果
# source_cloud.transform(result.transformation)
# o3d.visualization.draw_geometries([source_cloud, target_cloud], zoom=0.4459, front=[0.9288, -0.2951, -0.2242],
#                                   lookat=[1.6784, 2.0612, 1.4451], up=[-0.3402, -0.9189, -0.1996], width=1080,
#                                   height=900)

# 2.FGR(Fast Global Registration)
def preprocess_point_cloud(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2.0, max_nn=30))
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(pcd_down, o3d.geometry.KDTreeSearchParamHybrid(
        radius=voxel_size * 5.0, max_nn=100))
    return pcd_down, pcd_fpfh


if __name__ == '__main__':
    src_path = 'file/cloud_bin_0.pcd'
    dst_path = 'file/cloud_bin_1.pcd'
    voxel_size = 0.05  # 采样格子大小
    max_iterations = 64  # 配准的最大迭代次数
    max_tuples = 1000  # 最多同名点个数
    distance_threshold = 0.5 * voxel_size  # 同名点之间的距离阈值
    # 读取数据
    src = o3d.io.read_point_cloud(src_path)
    dst = o3d.io.read_point_cloud(dst_path)
    src.paint_uniform_color([1, 0, 0])
    dst.paint_uniform_color([0, 1, 0])
    # 显示配准之前的点云
    o3d.visualization.draw([src, dst])
    # 预处理
    src_down, src_fpfh = preprocess_point_cloud(src, voxel_size)
    dst_down, dst_fpfh = preprocess_point_cloud(dst, voxel_size)
    # FGR
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        src_down, dst_down, src_fpfh,
        dst_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold,
            iteration_number=max_iterations,
            maximum_tuple_count=max_tuples))
    o3d.visualization.draw([src.transform(result.transformation), dst])
