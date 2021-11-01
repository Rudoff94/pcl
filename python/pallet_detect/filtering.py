import numpy as np
import matplotlib.pyplot as plt
import pcl
import time

def main():
    print("load file")
    cloud = pcl.PointCloud_PointXYZRGB()
    #cloud = pcl.PointCloud()
    cloud.from_file(b'raw_data.pcd')

    print("Downsampling")
    #downsampling
    downsample = cloud.make_voxel_grid_filter()
    downsample.set_leaf_size(0.01, 0.01, 0.01)
    cloud = downsample.filter()
    cloud.to_file(b"downsample.pcd")

    print("passThrough filter")
    #passThrough
    pass_through = cloud.make_passthrough_filter()
    pass_through.set_filter_field_name('z')
    pass_through.set_filter_limits(0.1 , 1.0)
    cloud = pass_through.filter()
    cloud.to_file(b"pass_through.pcd")
    
    file = open("pc_rack.txt", 'w')
    np.savetxt(file, cloud.to_array())

main()