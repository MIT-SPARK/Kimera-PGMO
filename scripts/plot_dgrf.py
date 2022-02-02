#!/usr/bin/env python3
import argparse
import numpy as np
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D

def read_node_line(line_str):
    entries = line_str.split()
    return long(entries[1]), [float(entries[2]), float(entries[3]), float(entries[4])]

def read_edge_line(line_str):
    entries = line_str.split()
    return [long(entries[1]), long(entries[2])]

def read_dgrf_file(filename):
    file = open(filename, 'r')
    lines = file.readlines()

    nodes = {}
    betweens = []
    temp_betweens = []
    dedges = []
    temp_dedges = []
    for line in lines:
        if line.startswith("NODE"):
            key, pos = read_node_line(line)
            nodes[key] = pos
        elif line.startswith("BETWEEN_TEMP"):
            temp_betweens.append(read_edge_line(line))
        elif line.startswith("BETWEEN"):
            betweens.append(read_edge_line(line))
        elif line.startswith("DEDGE_TEMP"):
            temp_dedges.append(read_edge_line(line))
        elif line.startswith("DEDGE"):
            dedges.append(read_edge_line(line))
    return nodes, betweens, temp_betweens, dedges, temp_dedges

def generate_segments(edges, nodes):
    source = []
    target = []
    for e in edges: 
        source.append(nodes[e[0]])
        target.append(nodes[e[1]])
    return np.hstack([np.array(source), np.array(target)])

def main():
    parser = argparse.ArgumentParser(
        description="utiltiy to plot a deformation graph."
    )
    parser.add_argument("dgrf", type=str, help="input .dgrf file.")
    args = parser.parse_args()

    nds, btns, tbtns, degs, tdegs = read_dgrf_file(args.dgrf)

    all_pos = np.array(nds.values())
    x_min = np.min(all_pos[:,0])
    x_max = np.max(all_pos[:,0])
    y_min = np.min(all_pos[:,1])
    y_max = np.max(all_pos[:,1])
    z_min = np.min(all_pos[:,2])
    z_max = np.max(all_pos[:,2])

    btn_segs = generate_segments(btns, nds)
    tbtns_segs = generate_segments(tbtns, nds)
    degs_segs = generate_segments(degs, nds)
    tdegs_segs = generate_segments(tdegs, nds)

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    btn_segs = btn_segs.reshape((-1,2,3))
    ax.add_collection(Line3DCollection(btn_segs, linewidths=0.5, colors='b'))
    degs_segs = degs_segs.reshape((-1,2,3))
    ax.add_collection(Line3DCollection(degs_segs, linewidths=0.5, colors='r'))
    tbtns_segs = tbtns_segs.reshape((-1,2,3))
    ax.add_collection(Line3DCollection(tbtns_segs, linewidths=0.5, colors='g'))
    tdegs_segs = tdegs_segs.reshape((-1,2,3))
    ax.add_collection(Line3DCollection(tdegs_segs, linewidths=0.5, colors='m'))


    ax.set_xlim3d(x_min - 1.0, x_max + 1.0)
    ax.set_ylim3d(y_min - 1.0, y_max + 1.0)
    ax.set_zlim3d(z_min - 1.0, z_max + 1.0)

    plt.show()

if __name__ == "__main__":
    main()
