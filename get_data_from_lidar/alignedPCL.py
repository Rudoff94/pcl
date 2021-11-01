# First import library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
# Import argparse for command-line options
import argparse
# Import os.path for file path manipulation
import os.path

import pcl
import random

def rspointcloud_to_pcl(pc, depth_frame, color_frame):
    
    points = pc.calculate(depth_frame)
    pc.map_to(color_frame)
     
    v, t = points.get_vertices(), points.get_texture_coordinates()
    verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
    texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv

     
    colors_data = np.ndarray((verts.shape[0],1), dtype=np.float32)
    for i in range(colors_data.shape[0]):
        r_value = int(255 * random.random())
        g_value = int(255 * random.random())
        b_value = int(255 * random.random())
        #color_value = r_value << 16 | g_value << 8 | b_value
        color_value
        colors_data[i, 0] = color_value 
    verts = np.hstack((verts, colors_data))
    
    #sp = points.get_profile().as_video_stream_profile()

    pcl_out = pcl.PointCloud_PointXYZRGB()

    pcl_out.resize(verts.shape[0])
    pcl_out.from_array(verts)
    pcl.save(pcl_out, "pcl_out.pcd")
    
    #pcl_out.width = sp.width()
    #pcl_out.height = sp.height()
    #pcl_out.is_dense = false
    #pcl_out.points.resize(points.size())
    
# Create object for parsing command-line options
parser = argparse.ArgumentParser(description="Read recorded bag file and display depth stream in jet colormap.\
                                Remember to change the stream fps and format to match the recorded.")
# Add argument which takes path to a bag file as an input
parser.add_argument("-i", "--input", type=str, help="Path to the bag file")
# Parse the command line arguments to an object
args = parser.parse_args()
# Safety if no parameter have been given
if not args.input:
    print("No input paramater have been given.")
    print("For help type --help")
    exit()
# Check if the given file have bag extension
if os.path.splitext(args.input)[1] != ".bag":
    print("The given file is not of correct file format.")
    print("Only .bag files are accepted")
    exit()
try:
    # Create pipeline
    pipeline = rs.pipeline()

    # Create a config object
    config = rs.config()

    # Tell config that we will use a recorded device from file to be used by the pipeline through playback.
    rs.config.enable_device_from_file(config, args.input)

    # Configure the pipeline to stream the depth stream
    # Change this parameters according to the recorded bag file resolution
    config.enable_stream(rs.stream.depth, rs.format.z16, 30)
    config.enable_stream(rs.stream.color)
    
    #config.enable_stream(rs.stream.depth, rs.format.z16, 30)
    #config.enable_stream(rs.stream.color, rs.format.bgr8, 30)

    # Start streaming from file
    pipeline.start(config)

    # Create opencv window to render image in
    cv2.namedWindow("Depth Stream", cv2.WINDOW_AUTOSIZE)
    
    # Create colorizer object
    colorizer = rs.colorizer()

    pc = rs.pointcloud()

    # Streaming loop
    while True:
        # Get frameset of depth
        frames = pipeline.wait_for_frames()

        # Get depth frame
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # Colorize depth frame to jet colormap
        depth_color_frame = colorizer.colorize(depth_frame)

        # Convert depth_frame to numpy array to render image in opencv
        depth_color_image = np.asanyarray(depth_color_frame.get_data())

        # Render image in opencv window
        cv2.imshow("Depth Stream", depth_color_image)
        key = cv2.waitKey(1)
        # if pressed escape exit program
        if key == 27:
            rspointcloud_to_pcl(pc, depth_frame, color_frame)
            cv2.destroyAllWindows()
            break

finally:
    pass
