import time
import mujoco
import mujoco.viewer
import os

# 获取当前工作目录
cwd = os.getcwd()
# 构建模型文件的完整路径。假设 force.xml 位于 'mujoco_learning/API-MJCF/' 目录下
model_path = os.path.join(cwd,'mujoco_learning','API-MJCF','force.xml')

# 从 XML 文件路径加载 MuJoCo 模型 (MjModel)
m = mujoco.MjModel.from_xml_path(model_path)
# 初始化 MuJoCo 数据结构 (MjData)，用于存储状态和计算结果
d = mujoco.MjData(m)

# 执行一次步进，用于初始化所有内部数据结构，如传感器数据
mujoco.mj_step(m, d)

'''--------box (盒子) 相关设置--------'''
# 获取名为 "box" 的 body ID
box_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "box")
# 定义要施加的力 (x, y, z)
box_force = [0.0, 0.0, 10]
# 定义要施加的扭矩 (x, y, z)
box_torque = [0.0, 0.0, 0.0]
# 定义施加力的点 (相对于 body 坐标系原点)
box_point = [0.0, 0.0, 0.0]
# 获取名为 "red_point" 的 site ID (用于确定施力点)
red_point = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "red_point")
# 获取 "red_point" site 在世界坐标系中的当前位置
point = d.site_xpos[red_point]
mujoco.mj_applyFT(m, d, box_force, box_torque, box_point, box_id, d.qfrc_applied)
# 注意：mj_applyFT 用于计算并累加广义力 (Generalized Forces) 到 d.qfrc_applied 数组中。
'''--------box (盒子) 相关设置--------'''

'''--------力——加速度 (Sphere) 相关设置--------'''
# 获取名为 "sphere" 的 body ID
sphere_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "sphere")
# 定义施加的力/扭矩（如果使用 mj_applyFT）
sphere_force = [0.0, 0.0, 0.0]
sphere_torque = [0.0, 0.0, 0.0]
sphere_point = [0.0, 0.0, 0.0]
# mujoco.mj_applyFT(m, d, sphere_force, sphere_torque, sphere_point, sphere_id, d.qfrc_applied)
'''--------力——加速度 (Sphere) 相关设置--------'''

'''--------扭矩 (Pointer) 相关设置--------'''
# 获取名为 "pointer" 的 body ID
pointer_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pointer")
# 定义施加的力/扭矩（如果使用 mj_applyFT）
pointer_force = [0.0, 0.0, 0.0]
pointer_torque = [0.0, 0.0, 0.0]
pointer_point = [0.0, 0.0, 0.0]
# mujoco.mj_applyFT(m, d, pointer_force, pointer_torque, pointer_point, pointer_id, d.qfrc_applied)
'''--------扭矩 (Pointer) 相关设置--------'''

# 启动 MuJoCo Viewer，使用被动模式 (passive)
with mujoco.viewer.launch_passive(m, d) as viewer:
  start = time.time()
  # 循环运行，直到 Viewer 关闭或达到 30 秒
  while viewer.is_running() and time.time() - start < 30:
    step_start = time.time()
    
    '''--------box (使用 mj_applyFT 施加力)--------'''
    # # 清除上一时刻施加的广义力
    # d.qfrc_applied[:] = 0
    # # 在指定的世界坐标点 point 施加力 box_force 到 box_id 对应的 body 上
    # # 结果存储在 d.qfrc_applied 中
    # mujoco.mj_applyFT(m, d, box_force, box_torque, point, box_id, d.qfrc_applied)
    # # 执行物理步进
    # mujoco.mj_step(m, d)
    '''--------box (使用 mj_applyFT 施加力)--------'''
    
    '''--------box (使用 xfrc_applied 施加力)--------'''
    # # d.xfrc_applied 是一个 (nbody x 6) 的数组，用于直接施加笛卡尔力/扭矩到 body 的质心
    # # 索引 box_id 对应 body 的力矩向量 [fx, fy, fz, tx, ty, tz]
    # box_xfrc_applied = d.xfrc_applied[box_id]
    # box_xfrc_applied[0] = 0.0 # fx (力 - x方向)
    # box_xfrc_applied[1] = 0.0 # fy
    # box_xfrc_applied[2] = 0.0 # fz
    # box_xfrc_applied[3] = 0.0 # tx (扭矩 - x方向)
    # box_xfrc_applied[4] = 0.0 # ty
    # box_xfrc_applied[5] = 0.0 # tz
    # mujoco.mj_step(m, d)
    '''--------box (使用 xfrc_applied 施加力)--------'''
    
    '''--------力——加速度 (Sphere) 分析--------'''
    # 通过执行器 (actuator) 施加力。假设第一个执行器控制 sphere 的平移自由度。
    d.ctrl[0] = 0.6
    
    # 确保没有通过笛卡尔力/扭矩施加额外的外部力
    sphere_xfrc_applied = d.xfrc_applied[sphere_id]
    sphere_xfrc_applied[0] = 0.0 # fx
    sphere_xfrc_applied[1] = 0.0 # fy
    sphere_xfrc_applied[2] = 0.0 # fz
    sphere_xfrc_applied[3] = 0.0 # tx
    sphere_xfrc_applied[4] = 0.0 # ty
    sphere_xfrc_applied[5] = 0.0 # tz
    
    # 执行物理步进，计算新的状态和力
    mujoco.mj_step(m, d)
    
    # 打印广义力 (Generalized Forces) 的各个分量。
    # 假设 sphere 的平移自由度对应广义坐标 qpos[0] 和 qvel[0]，因此我们查看索引 0 的力。
    print("qfrc_passive:%f  qfrc_actuator:%f  qfrc_applied:%f qfrc_bias:%f  efc_force:%f" % (
    d.qfrc_passive[0],        # 被动力 (如重力、阻尼、弹簧力)
    d.qfrc_actuator[0],       # 执行器力 (来自 d.ctrl)
    d.qfrc_applied[0],        # 应用力 (来自 mj_applyFT 或直接设置)
    d.qfrc_bias[0],           # 偏置力 (科里奥利力、离心力)
    d.efc_force[0]))          # 约束力 (如接触力、关节极限力)
    
    # 打印传感器测量的线性加速度 (假设传感器名为 "lin_acc")
    print("lin_acc:",d.sensor("lin_acc").data[0])
    
    # 根据牛顿第二定律 F = ma，计算理论加速度：a = F_net / m
    # F_net 是所有广义力的总和
    total_force = (d.qfrc_passive[0] + d.qfrc_actuator[0] +
                   d.qfrc_applied[0] + d.qfrc_bias[0] + d.efc_force[0])
    # 假设 sphere 的质量是 m.body_mass[sphere_id]
    # 注意：这个计算是针对广义坐标的，对于简单平移，广义质量就是体质量。
    acc = total_force / m.body_mass[sphere_id]
    print("计算加速度:",acc)
    
    # 打印传感器测量的线速度和位置
    print("lin_vel:",d.sensor("lin_vel").data[0])
    print("lin_pos:",d.sensor("lin_pos").data[0])
    '''--------力——加速度 (Sphere) 分析--------'''
    
    '''--------扭矩 (Pointer) 分析--------'''
    # # 通过第二个执行器施加扭矩
    # d.ctrl[1] = 0.6
    # 
    # # 确保没有通过笛卡尔力/扭矩施加额外的外部力
    # pointer_xfrc_applied = d.xfrc_applied[pointer_id]
    # pointer_xfrc_applied[0] = 0.0 # fx
    # pointer_xfrc_applied[1] = 0.0 # fy
    # pointer_xfrc_applied[2] = 0.0 # fz
    # pointer_xfrc_applied[3] = 0.0 # tx
    # pointer_xfrc_applied[4] = 0.0 # ty
    # pointer_xfrc_applied[5] = 0.0 # tz
    # mujoco.mj_step(m, d)
    # 
    # # 打印广义力分量 (假设旋转自由度对应索引 1)
    # print("qfrc_passive:%f  qfrc_actuator:%f  qfrc_applied:%f qfrc_bias:%f  efc_force:%f" % (
    #   d.qfrc_passive[1], d.qfrc_actuator[1], d.qfrc_applied[1], d.qfrc_bias[1], d.efc_force[1]))
    # 
    # # 计算总扭矩 (广义力总和，对应旋转自由度)
    # tau = d.qfrc_passive[1] + d.qfrc_actuator[1] + d.qfrc_applied[1] + d.qfrc_bias[1] + d.efc_force[1]
    # print("计算扭矩:",tau)
    # print("测量扭矩:",d.sensor("torque").data)
    # print("pivot_pos:",d.sensor("pivot_pos").data)
    # print("pivot_vel:",d.sensor("pivot_vel").data)
    '''--------扭矩 (Pointer) 分析--------'''
    

    # 使用 viewer.lock() 确保在修改 viewer 选项时线程安全
    with viewer.lock():
      # 示例：每两秒切换一次接触点可视化
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

    # 同步 viewer：将物理状态变化、外部扰动和 GUI 选项更新到 viewer
    viewer.sync()

    # 粗略的时间控制，确保模拟步进接近模型的 timestep 设定 (实时模拟)
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)