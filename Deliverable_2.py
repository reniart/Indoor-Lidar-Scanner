#deliverable 2 Viraj Bane 400399205

import serial
import numpy as np
import open3d as o3d
import math
import time

def main():
    
    f = open("location_data.xyz", "w") #Open text file in write mode
    print("Opening: {}".format(f.name)) 

    s = serial.Serial('COM7', 115200, timeout = 10)
    print("Opening: {}".format(s.name))

    # reset the buffers of the UART port to delete the remaining data in the buffers
    s.reset_output_buffer()
    s.reset_input_buffer()


    # recieve measurements from UART of MCU
    degree = 0
    degreeIncrement = 22.5
    x = 0
    rotations = 3 #number of rotations/scans
    pointsPerScan = 16


    for r in range(rotations):

        # # wait for user's signal to start the program
        # input("Press Enter to start scan {}".format(r+1))
        # # send the character 's' to MCU via UART
        # # This will signal MCU to start the 
        # # transmission
        # s.write('s'.encode())

        # wait for user's signal to start the program
        input("Press Enter to start scanning")
        # send the character 's' to MCU via UART
        # This will signal MCU to start the transmission
        s.write('s'.encode())

        for i in range(pointsPerScan):
            raw = s.readline()
            dist = int(float(raw.decode()))

            rad_degree = math.radians(degree)
            y = int(math.cos(rad_degree)*dist)#split distance measurement
            z = int(math.sin(rad_degree)*dist)

            f.write("{} {} {}\n".format(x, y, z))#we split the distance measurement into y and z bcuz the tof is sideways
            degree += degreeIncrement
            print("Measured Distance: {}, Y-Component: {}, Z-Component: {}".format(dist, y, z))
        x += 1000
        degree = 0
        time.sleep(30)
        
    # the encode() and decode() function are needed to convert string to bytes
    # because pyserial library functions work with type "bytes"


    #close the port and file
    print("Closing: {}".format(s.name))
    s.close()

    print("Closing: {}".format(f.name))
    f.close()

    print("Read in the prisim point cloud data (pcd)")
    pcd = o3d.io.read_point_cloud("location_data.xyz", format="xyz")

    print("The PCD Array:")
    print(np.asarray(pcd.points))

    #print("Visualize PCD")
    #o3d.visualization.draw_geometries([pcd])

    yz_slice_vertex = []
    for x in range(0, pointsPerScan*rotations):
        yz_slice_vertex.append([x])

    #connect all points in a plane

    lines = []
    for x in range(0, pointsPerScan*rotations, pointsPerScan):
        for p in range(0, pointsPerScan - 1):
            lines.append([yz_slice_vertex[x+p], yz_slice_vertex[x + p + 1]])
        lines.append([yz_slice_vertex[x+ (pointsPerScan - 1)], yz_slice_vertex[x]])

    # connect planes

    for x in range(0,(pointsPerScan*(rotations-1)) - 1,pointsPerScan): # x will be exectured equal to 0, 16, 32
        for p in range(0, pointsPerScan):
            lines.append([yz_slice_vertex[x+p], yz_slice_vertex[x+p + pointsPerScan]])

    # This line maps the lines to the 3d coordinate vertices
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)), lines=o3d.utility.Vector2iVector(lines))
    # Lets see what our point cloud data with lines looks like graphically
    o3d.visualization.draw_geometries([line_set])


if __name__ == '__main__':
    main()

