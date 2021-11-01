import numpy as np
import pcl

def compare_array(arr, arr_rgb):
    print("compare_array")
    print(arr_rgb.shape)
    for i, point in enumerate(arr):
        index = np.where(arr_rgb[:, :3] == point)
        arr_rgb = np.delete(arr_rgb, index[0][0], 0)
    print(arr_rgb.shape)
    return arr_rgb


def merge_color(cloud, cloud_rgb):
    print("merge_color")
    corrected_array = compare_array(cloud.to_array(), cloud_rgb.to_array())
    cloud_out = pcl.PointCloud_PointXYZRGB()
    cloud_out.from_array(corrected_array)
    print(corrected_array[:5])
    print("-----------------------------------")
    print(cloud_out.to_array()[:5])
    return cloud_out
    

def main():
    print("load file")
    cloud = pcl.PointCloud()
    cloud.from_file(b'pass_through.pcd')
    
    print("stat_outlier")
    stat_outlier = cloud.make_statistical_outlier_filter()
    #tat_outlier.set_negative(True)
    stat_outlier.set_mean_k(50)
    stat_outlier.set_std_dev_mul_thresh(1.0)
    cloud = stat_outlier.filter()

    
    print("save file")
    file = open("pc_rack.txt", 'w')
    np.savetxt(file, cloud.to_array())
    cloud.to_file(b"stat_outlier.pcd")

main()