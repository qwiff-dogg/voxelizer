import open3d as o3d
import sys

if len(sys.argv) == 2:
    pcd = o3d.io.read_point_cloud(sys.argv[1])
    o3d.visualization.draw_geometries([pcd])
    o3d.visualization.draw([pcd], point_size=2)
else:
    print("No PCD file specified")
