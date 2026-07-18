from lib.cloud_utils import PointCloudUtils


path_points=[[0.0, 0.0, 0.0], [0.0, 0.2, 2.0], [-0.54, 0.2, 2.75], [1.723, 0.2, 3.687], [2.66, 0.2, 5.95], [1.723, 0.2, 8.213], [-0.54, 0.2, 9.15], [-2.803, 0.2, 8.213], [-3.74, 0.2, 5.95], [-2.803, 0.2, 3.687], [-0.54, 0.2, 2.75], [0.0, -1.0, 0.0]]
print(path_points)
pcu = PointCloudUtils()
pcu.process_point_cloud(
                        input_ply_path="/home/uav/lab/muav/src/langguide/scripts/egovlm/log/point_cloud_sphere.ply",
                        annotation_data=path_points, 
                        modeling_type="path", 
                        output_ply_path="./log/point_cloud_sphere_path.ply", 
                        show_visualization=True, 
                        radius=0.1
                        )