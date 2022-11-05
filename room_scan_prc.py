import sys
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d

def big_plane_decision(segment, plane_size_thres, plane_point_count_thres):
    is_big_plane = False

    bbox = segment.get_oriented_bounding_box()
    minb = bbox.get_min_bound()
    maxb = bbox.get_max_bound()
    plane_sizes = abs(maxb - minb)
    is_big_plane = False
    big_dimension_count = 0
    for d in plane_sizes:
        if d > plane_size_thres:
            big_dimension_count += 1
    if big_dimension_count >= 2:
        is_big_plane = True
    if len(segment.points) < plane_point_count_thres:
        is_big_plane = False

    return is_big_plane, bbox, abs(maxb - minb)

# plane model: ax + by + cz + d = 0
def is_plane_horizontal(p_model):
    is_horiz = False
    print(p_model)
    b_y = abs(p_model[1])
    if b_y > 0.990:
        is_horiz = True
    return is_horiz

def is_plane_vertical(p_model):
    is_vert = False
    b_y = abs(p_model[1])
    if b_y < 0.025:
        is_vert = True
    return is_vert

def is_plane_contiguous(pcd):
    is_contig = False
    labels = np.array(pcd.cluster_dbscan(eps=0.20, min_points=70))
    cluster_count = labels.max() + 1
    print(labels.shape)
    print("cluster count: %d"%cluster_count)
    
    visualise = True
    if visualise:
        max_label = labels.max()
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame()
        o3d.visualization.draw_geometries([pcd]+[axes])

    if cluster_count > 0 and cluster_count < 5:
        is_contig = True
    if labels.shape[0] < 1000:
        is_contig = False
    return is_contig

#############
args_len = len(sys.argv)

if args_len < 2:
    print("Usage %s <input 3d file woth point cloud> [<subsample ratio>]", sys.argv[0])
    exit(0)

input_file = sys.argv[1]
dist_thres = 0.035 # meters
plane_size_thres = 2.5
plane_point_count_thres = 300

sub_rate = 1
if args_len > 2:
    sub_rate = int(sys.argv[2])

pcd = o3d.io.read_point_cloud(input_file)
print("point cloud length: ", len(pcd.points))

if sub_rate > 1:
    pcd = pcd.uniform_down_sample(sub_rate)
    print("subsampled point cloud length: ", len(pcd.points))
    fname_out = input_file.replace(".", "_subsampled_%d."%sub_rate)
    o3d.io.write_point_cloud(fname_out, pcd)

#o3d.visualization.draw_geometries([pcd])

#pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=16), fast_normal_computation=True)

max_plane_idx = 15
segment_models={}
segments={}
bboxes={}
rest=pcd
big_plane_count = 0
for i in range(max_plane_idx):
    colors = plt.get_cmap("tab20")(i)
    model, inliers = rest.segment_plane(
    distance_threshold=dist_thres,ransac_n=3,num_iterations=1000)
    segment=rest.select_by_index(inliers)

    #check the physical size of the plane and choose only big ones.
    is_big_plane, bbox, dims = big_plane_decision(segment, plane_size_thres, plane_point_count_thres)
    is_horiz = is_plane_horizontal(model)
    is_vert = is_plane_vertical(model)
    is_contiguous = is_plane_contiguous(segment)
    if is_contiguous and (is_horiz or is_vert):
    #if is_big_plane and (is_horiz or is_vert):
        bboxes[big_plane_count] = bbox
        segment_models[big_plane_count] = model
        segments[big_plane_count]=segment
        segments[big_plane_count].paint_uniform_color(list(colors[:3]))
        big_plane_count += 1

    rest = rest.select_by_index(inliers, invert=True)
    print("pass",i,"/",max_plane_idx,"done.")
    print("max - min bounds: ", dims, ", is big plane: ", is_big_plane)

#o3d.visualization.draw_geometries([bboxes[i] for i in range(5)])

#o3d.visualization.draw_geometries([segments[i] for i in range(max_plane_idx)]+[rest])
axes = o3d.geometry.TriangleMesh.create_coordinate_frame()
o3d.visualization.draw_geometries([segments[i] for i in range(big_plane_count)]+[axes])

#labels = np.array(pcd.cluster_dbscan(eps=0.05, min_points=10))

def find_plane(pcd):
    plane_model, inliers = pcd.segment_plane(distance_threshold=dist_thres, ransac_n=3, num_iterations=1000)

    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)

    inlier_cloud.paint_uniform_color([1, 0, 0])
    outlier_cloud.paint_uniform_color([0.6, 0.6, 0.6])

    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
