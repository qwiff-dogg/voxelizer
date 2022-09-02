import open3d as o3d
import sys
from os.path import exists

if len(sys.argv) == 2:
    filename = sys.argv[1]
    if exists(filename):
        pcd = o3d.io.read_point_cloud(filename)
        o3d.visualization.draw_geometries([pcd])
        o3d.visualization.draw([pcd], point_size=2)
    else:
        print(f"File not found: {filename}")
else:
    print("No PCD file specified")
