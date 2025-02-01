#This file contains exclusively the visualization portion of deliverable 2, 
#and is used to test and verify visuals without having to run the whole program


import numpy as np
import open3d as o3d

def main():
    print("Read in the prisim point cloud data (pcd)")
    pcd = o3d.io.read_point_cloud("location_f_data.xyz", format="xyz")

    print("The PCD Array:")
    print(np.asarray(pcd.points))

    rotations = 13
    pointsPerScan = 16

    yz_slice_vertex = []
    for x in range(0, pointsPerScan*rotations):
        yz_slice_vertex.append([x])

    #connect all points in a plane

    lines = []
    for x in range(0, 16*rotations, 16):
        for p in range(0, pointsPerScan - 1):
            lines.append([yz_slice_vertex[x+p], yz_slice_vertex[x + p + 1]])
        lines.append([yz_slice_vertex[x+ (pointsPerScan - 1)], yz_slice_vertex[x]])

    # connect planes

    for x in range(0,(16*(rotations-1)) - 1,16): # x will be exectured equal to 0, 16, 32
        for p in range(0, pointsPerScan):
            lines.append([yz_slice_vertex[x+p], yz_slice_vertex[x+p + pointsPerScan]])

    # This line maps the lines to the 3d coordinate vertices
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)), lines=o3d.utility.Vector2iVector(lines))
    # Lets see what our point cloud data with lines looks like graphically
    o3d.visualization.draw_geometries([line_set])

if __name__ == '__main__':
    main()
