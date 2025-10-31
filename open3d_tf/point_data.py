import numpy as np
import open3d as o3d

# 创建球形点云
radius = 1.0
points = []
for theta in np.linspace(0, np.pi, 100):
    for phi in np.linspace(0, 2 * np.pi, 100):
        x = radius * np.sin(theta) * np.cos(phi)
        y = radius * np.sin(theta) * np.sin(phi)
        z = radius * np.cos(theta)
        points.append([x, y, z])

# 将点云转换为 Open3D 格式
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(np.array(points))

# 计算法线
point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))

# 可视化点云
# o3d.visualization.draw_geometries([point_cloud])

# 使用 Poisson 重建算法生成网格
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(point_cloud, depth=9)

# 可视化生成的网格
o3d.visualization.draw_geometries([mesh])

# 将网格保存为 .obj 文件
o3d.io.write_triangle_mesh("mesh.obj", mesh)
