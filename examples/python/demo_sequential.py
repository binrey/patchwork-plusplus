import os
import sys
import open3d as o3d
import numpy as np
import velodyne_decoder as vd
import time
import imageio


def save_view_point(vis, filename):
    vis.run()  # user changes the view and press "q" to terminate
    param = vis.get_view_control().convert_to_pinhole_camera_parameters()
    o3d.io.write_pinhole_camera_parameters(filename, param)
    vis.destroy_window()
    
    
cur_dir = os.path.dirname(os.path.abspath(__file__))
data_dir = os.path.join(cur_dir, '../../data/')

try:
    patchwork_module_path = os.path.join(cur_dir, "../../build/python_wrapper")
    sys.path.insert(0, patchwork_module_path)
    import pypatchworkpp
except ImportError:
    print("Cannot find pypatchworkpp!")
    exit(1)

def read_bin(bin_path):
    scan = np.fromfile(bin_path, dtype=np.float32)
    scan = scan.reshape((-1, 4))

    return scan

if __name__ == "__main__":

    # Patchwork++ initialization
    params = pypatchworkpp.Parameters()
    params.verbose = False

    PatchworkPLUSPLUS = pypatchworkpp.patchworkpp(params)

    config = vd.Config(model='VLP-32C', rpm=600)
    pcap_file = "./2022-06-10-15-29-36_Velodyne-VLP-32C-Data.pcap"
    
    vis = o3d.visualization.Visualizer()
    vis.create_window(width = 800, height = 600)

    vw = imageio.get_writer("out.mp4")
    
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    ground_o3d = o3d.geometry.PointCloud()
    nonground_o3d = o3d.geometry.PointCloud()
    nframe = 0
    
    for stamp, points in vd.read_pcap(pcap_file, config, as_pcl_structs=False):
        # Load point cloud
        pointcloud = points[:, :4]

        # Estimate Ground
        PatchworkPLUSPLUS.estimateGround(pointcloud)

        # Get Ground and Nonground
        ground      = PatchworkPLUSPLUS.getGround()
        nonground   = PatchworkPLUSPLUS.getNonground()
        time_taken  = PatchworkPLUSPLUS.getTimeTaken()

        # Get centers and normals for patches
        # centers     = PatchworkPLUSPLUS.getCenters()
        # normals     = PatchworkPLUSPLUS.getNormals()

        # print("Origianl Points  #: ", pointcloud.shape[0])
        # print("Ground Points    #: ", ground.shape[0])
        # print("Nonground Points #: ", nonground.shape[0])
        print("Time Taken : ", time_taken / 1000000, "(sec)")
        # print("Press ... \n")
        # print("\t H  : help")
        # print("\t N  : visualize the surface normals")
        # print("\tESC : close the Open3D window")
        

        ground_o3d.points = o3d.utility.Vector3dVector(ground)
        ground_o3d.colors = o3d.utility.Vector3dVector(
            np.array([[0.0, 1.0, 0.0] for _ in range(ground.shape[0])], dtype=float) # RGB
        )

        
        nonground_o3d.points = o3d.utility.Vector3dVector(nonground)
        nonground_o3d.colors = o3d.utility.Vector3dVector(
            np.array([[1.0, 0.0, 0.0] for _ in range(nonground.shape[0])], dtype=float) #RGB
        )

        # centers_o3d = o3d.geometry.PointCloud()
        # centers_o3d.points = o3d.utility.Vector3dVector(centers)
        # centers_o3d.normals = o3d.utility.Vector3dVector(normals)
        # centers_o3d.colors = o3d.utility.Vector3dVector(
        #     np.array([[1.0, 1.0, 0.0] for _ in range(centers.shape[0])], dtype=float) #RGB
        # )

        if nframe == 0:
            vis.add_geometry(mesh)
            vis.add_geometry(ground_o3d)
            vis.add_geometry(nonground_o3d)
            # vis.add_geometry(centers_o3d)
        else:
            vis.update_geometry(ground_o3d)
            vis.update_geometry(nonground_o3d)
            # vis.update_geometry(centers_o3d)            

        vis.poll_events()
        vis.update_renderer()
        nframe += 1
        
        image = vis.capture_screen_float_buffer()
        vw.append_data((np.array(image)*255).astype(np.uint8))
