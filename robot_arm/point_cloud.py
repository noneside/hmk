# point_cloud.py
import open3d as o3d
import numpy as np

# 生成一个简单的点云
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.random.randn(1000, 3))
# 着色
pcd.colors = o3d.utility.Vector3dVector(np.random.rand(1000, 3))

# 可视化（如果在服务器无显示，可能需要使用 headless 渲染方式，或保存为文件再查看）
o3d.visualization.draw_geometries([pcd])
PY