import mujoco
import mujoco.viewer
import numpy as np
import open3d as o3d
import time
import os
import xml.etree.ElementTree as ET

# 点云数据转为obj文件的函数
# 点云数据转为obj文件的函数
def point_cloud_to_obj(pcd, obj_file):
    # 转换 Open3D 点云为 Mesh
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha=10.0)
    
    # 使用 write_triangle_mesh 保存为 obj 文件
    o3d.io.write_triangle_mesh(obj_file, mesh)
    print(f"Updated OBJ file at {obj_file}")


# 更新XML文件中mesh引用的obj文件路径
def update_mesh_in_xml(xml_path, obj_file):
    tree = ET.parse(xml_path)
    root = tree.getroot()
    
    # 找到 mesh 元素并更新文件路径
    mesh_elem = root.find(".//asset/mesh[@name='your_mesh']")
    if mesh_elem is not None:
        mesh_elem.set("file", obj_file)
    else:
        print("Mesh element not found in XML.")
        return
    
    # 保存更新后的XML文件
    tree.write(xml_path)
    print(f"Updated XML file at {xml_path}")

# 获取Open3D的点云数据
def get_point_cloud_data():
    # 这里可以用 Open3D 加载或者从传感器中获取点云数据
    # 示例中加载一个点云文件进行测试
    pcd = o3d.io.read_point_cloud("cheff.ply")  # 你的点云数据
    return pcd

# 加载MuJoCo模型和初始化仿真
xml_path = 'xxx.xml'
obj_file = 'point_cloud.obj'

# 获取点云数据
pcd = get_point_cloud_data()

# 转换并保存为.obj文件
point_cloud_to_obj(pcd, obj_file)

# 更新XML文件
update_mesh_in_xml(xml_path, obj_file)

# 加载MuJoCo模型
m = mujoco.MjModel.from_xml_path(xml_path)
d = mujoco.MjData(m)

# 启动MuJoCo仿真和可视化
with mujoco.viewer.launch_passive(m, d) as viewer:
    start_time = time.time()

    while viewer.is_running():
        # 每隔一定时间更新点云
        if time.time() - start_time > 2:  # 每2秒更新一次
            pcd = get_point_cloud_data()  # 重新获取点云数据
            point_cloud_to_obj(pcd, obj_file)  # 更新.obj文件
            update_mesh_in_xml(xml_path, obj_file)  # 更新XML文件

            # 重新加载模型数据
            m = mujoco.MjModel.from_xml_path(xml_path)
            d = mujoco.MjData(m)

            start_time = time.time()  # 重置计时器

        # 运行仿真步骤
        mujoco.mj_step(m, d)
        viewer.sync()

        # 渲染更新
        # viewer.render()

        # 控制仿真更新的时间
        time.sleep(m.opt.timestep)
