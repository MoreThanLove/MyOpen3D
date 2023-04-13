import open3d as o3d
import numpy as np

if __name__ == '__main__':
    # open3d可视化操作
    pcd1 = o3d.io.read_point_cloud('file/bun_zipper.ply')  # 读取点云文件

    # 1.设置颜色，[R,G,B]归一化后的数值，[0,1]之间
    pcd1.paint_uniform_color([0, 0, 1])  # 蓝色
    color = np.array(pcd1.colors)  # 获取颜色值（numpy形式）
    # print(color.shape)  # (35947, 3)表示共有35947个点，3表示RGB形式
    even_index = [i for i in range(color.shape[0]) if i % 2 == 0]  # 记录偶数点的索引
    color[even_index] = [1, 0, 0]  # 将偶数点的颜色改为红色
    pcd1.colors = o3d.utility.Vector3dVector(color)  # 将numpy形式的color转为Open3D format形式并重新给点云的颜色赋值

    # 2.给点云对象pcd1估计(需要多个点组成平面来估计)法向，search_param表示搜索方法max_nn表示最多多少个点合成一个面，radius表示点的搜索范围
    # 也可以不写参数，默认为KDTreeSearchParamKNN with knn = 30
    pcd1.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
    o3d.visualization.draw_geometries([pcd1], window_name='Open3D', width=1080, height=680, point_show_normal=True)
    # 3.隐藏某个视角(camera_location)radius范围内的点，返回一个列表[pointcloud.geometry.TriangleMesh, List[int]]，第二个参数为剩余点的索引
    _, pt_map = pcd1.hidden_point_removal(camera_location=[0, 0, 0.25], radius=25)
    pcd1 = pcd1.select_by_index(pt_map,
                                invert=True)  # 根据剩余点的坐标重新制作一个点云，invert默认为False表示获取的是剩余点，为True表示获取隐藏的点
    #   参数1为可视化对象列表，window_name为窗口的名字，width和height为窗口的宽和高，point_show_normal表示是否显示法向
    # left为左边距，top为上边距
    o3d.visualization.draw_geometries([pcd1], window_name='Open3D', width=1080, height=680, left=50, top=50,
                                      point_show_normal=True)

    # 4.设置坐标系，背景颜色以及点的大小
    pcd2 = o3d.io.read_point_cloud('file/bun_zipper.ply')  # 获取点云对象
    # 获取可视化句柄
    viewer = o3d.visualization.Visualizer()
    # 创建窗口
    viewer.create_window(window_name='可视化', width=800, height=600)
    # 获取参数管理器
    opt = viewer.get_render_option()
    # 设置背景颜色
    opt.background_color = np.array([0, 1, 1])
    # 设置点的大小
    opt.point_size = 1
    # 添加坐标系
    opt.show_coordinate_frame = True
    # 给点云设置颜色
    pcd2.paint_uniform_color([0, 0, 1])
    # 将点云对象加入到窗口中
    viewer.add_geometry(pcd2)
    # 激活界面循环
    viewer.run()
    # 结束
    viewer.destroy_window()
