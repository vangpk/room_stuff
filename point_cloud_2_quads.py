import sys
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d

#############
args_len = len(sys.argv)

if args_len < 2:
    print("Usage %s <input 3d file woth point cloud> [<subsample ratio>]", sys.argv[0])
    exit(0)

input_file = sys.argv[1]
dist_thres = 0.035 # meters. E.g. 0.035 is 35mm plane thicness threshold
plane_size_thres = 2.5
plane_point_count_thres = 300

d=0.05

pcd = o3d.io.read_point_cloud(input_file)
print("point cloud length: ", len(pcd.points))

meshout = o3d.geometry.TriangleMesh()
meshqout = o3d.geometry.TetraMesh()

#o3d.visualization.draw_geometries([pcd])

#pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=16), fast_normal_computation=True)

max_plane_idx = 15
segment_planes={}
segments={}
bboxes={}
horiz_plane_models=[]
horiz_planes_pts=[]
horiz_plane_bboxes=[]
rest=pcd
big_plane_count = 0

i = 0
for p in pcd.points:
    col = pcd.colors[i]
    #print(p, col)
    p1=p+[-d,0,-d]
    p2=p+[-d,0,d]
    p3=p+[d,0,-d]
    meshout.vertices.append(p1)
    meshout.vertices.append(p2)
    meshout.vertices.append(p3)
    meshout.vertex_colors.append(col)
    meshout.vertex_colors.append(col)
    meshout.vertex_colors.append(col)
    i3 = i*3
    meshout.triangles.append([i3,i3+1,i3+2])
    #print(meshout.vertices[i])
    #print(meshout.triangles[i])
    i += 1

#meshout.compute_vertex_normals()
#meshout.compute_triangle_normals()

axes = o3d.geometry.TriangleMesh.create_coordinate_frame()
o3d.visualization.draw_geometries([meshout]+[axes], mesh_show_back_face=True)

fname_out = input_file.replace(".", "_triangles.")
o3d.io.write_triangle_mesh(fname_out, meshout)

