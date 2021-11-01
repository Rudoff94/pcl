from ctypes import string_at
import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
import os.path

import pcl
import random

def get_colors(textures, color_frame):
    texcoords = np.asanyarray(textures).view(np.float32).reshape(-1, 2)  # uv
    width  = color_frame.get_width()
    height = color_frame.get_height()
    #img = np.asanyarray(color_frame.get_data()).flatten()
    img = np.asanyarray(color_frame.get_data())
    colors_data = np.ndarray((texcoords.shape[0],1), dtype=np.float32)

    print('colorize')
    for i in range(texcoords.shape[0]):
        x_value = min(max(int(texcoords[i][0] * width  + 0.5), 0), width - 1)
        y_value = min(max(int(texcoords[i][1] * height  + 0.5), 0), height - 1)
        #print(x_value, y_value, img[y_value, x_value, :])
        
        # bytes_v = x_value * color_frame.get_bytes_per_pixel()
        # strides_v = y_value * color_frame.get_stride_in_bytes()
        # texture_index = bytes_v + strides_v
        # r,g,b = img[texture_index], img[texture_index + 1], img[texture_index + 2]
        
        r,g,b = img[y_value, x_value, :]
        
        color = r << 16 | g << 8 | b

        colors_data[i] = color
    # print("get_bytes_per_pixel")
    # print(color_frame.get_bytes_per_pixel())
    # print("get_stride_in_bytes")
    # print(color_frame.get_stride_in_bytes())
    print('end colorize')
    return colors_data

def rspointcloud_to_pcl(pc, depth_frame, color_frame):
    
    pc.map_to(color_frame)
    points = pc.calculate(depth_frame)
    
    v, t = points.get_vertices(), points.get_texture_coordinates()
    verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
    colors_data = get_colors(t, color_frame)
    
    # for i in range(colors_data.shape[0]):
    #     r_value = int(255 * random.random())
    #     g_value = int(255 * random.random())
    #     b_value = int(255 * random.random())
    #     #color_value = r_value << 16 | g_value << 8 | b_value
    #     color_value
    #     colors_data[i, 0] = color_value 
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
    
parser = argparse.ArgumentParser(description="Read recorded bag file and display depth stream in jet colormap.\
                                Remember to change the stream fps and format to match the recorded.")
parser.add_argument("-i", "--input", type=str, help="Path to the bag file")
args = parser.parse_args()
if not args.input:
    print("No input paramater have been given.")
    print("For help type --help")
    exit()
if os.path.splitext(args.input)[1] != ".bag":
    print("The given file is not of correct file format.")
    print("Only .bag files are accepted")
    exit()
try:
    pipeline = rs.pipeline()
    config = rs.config()
    rs.config.enable_device_from_file(config, args.input)

    config.enable_stream(rs.stream.depth) #, rs.format.z16, 30)
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
    align = rs.align(rs.stream.color)

    # Streaming loop
    while True:
        # Get frameset of depth
        frames = pipeline.wait_for_frames()
        
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        # depth_frame = frames.get_depth_frame()
        # color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

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
