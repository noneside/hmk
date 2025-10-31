import mujoco
import mujoco.viewer
import numpy as np
import xml.etree.ElementTree as ET
import time

# 读取XML模型文件
xml_path = 'xxx.xml'
tree = ET.parse(xml_path)
root = tree.getroot()

# 获取 worldbody 元素，确保 body 被放入其中
worldbody = root.find('worldbody')
if worldbody is None:
    worldbody = ET.SubElement(root, 'worldbody')

# 添加 <asset> 标签来定义网格文件
assets = root.find('asset')
if assets is None:
    assets = ET.SubElement(root, 'asset')

# 定义网格文件路径
mesh_name = 'your_mesh'
mesh_file = 'mesh.obj'
mesh = ET.SubElement(assets, 'mesh', name=mesh_name, file=mesh_file)

# 在适当的位置添加 <geom> 标签引用这个网格
body = ET.SubElement(worldbody, 'body', name='body1', pos='0 0 0')
geom = ET.SubElement(body, 'geom', type='mesh', mesh=mesh_name)

# 保存修改后的XML文件
modified_xml_path = 'modified_model.xml'
tree.write(modified_xml_path)

# 加载修改后的XML模型
m = mujoco.MjModel.from_xml_path(modified_xml_path)

# 初始化仿真数据
d = mujoco.MjData(m)

# 设置网格的初始位置
geom_index = 0  # 假设你想修改第一个geom的位置
d.geom_xpos[geom_index] = np.array([1.0, 2.0, 3.0])

# 打印当前geom的位置，确保更新成功
print(f'Updated position: {d.geom_xpos[geom_index]}')

# 启动仿真视图
with mujoco.viewer.launch_passive(m, d) as viewer:
    # Close the viewer automatically after 30 wall-seconds.
    start = time.time()

    while viewer.is_running() and time.time() - start < 30:
        step_start = time.time()

        # Example modification of a viewer option: toggle contact points every two seconds.
        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
