"""
This python script takes as input the timing log produced by the mesh frontend.
Author: Yun Chang
""" 
import argparse
import numpy as np
import csv
import matplotlib.pyplot as plt
import sys

def read_mesh_full_log(filename):
    stats = {"num_vertices": [], "num_faces": [], "spin_time": []}
    with open(filename, mode='r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            stats["num_vertices"].append(int(row["num-vertices-full"]))
            stats["num_faces"].append(int(row["num-triangles-full"]))
            stats["spin_time"].append(float(row["process-time(mu-s)"]) * 1e-6)
    return stats

def read_mesh_graph_log(filename):
    stats = {"num_new_vertices": [], "num_new_edges": [], "spin_time": []}
    with open(filename, mode='r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            stats["num_new_vertices"].append(int(row["num-vertices-new"]))
            stats["num_new_edges"].append(int(row["num-edges-new"]))
            stats["spin_time"].append(float(row["compress-time(mu-s)"]) * 1e-6)
    return stats

def plot_frontend_timing(folder):
	full_log = folder + "/mf_full_log.csv"
	graph_log = folder + "/mf_graph_log.csv"
	full_stats = read_mesh_full_log(full_log)
	graph_stats = read_mesh_graph_log(graph_log)

	fig, axs = plt.subplots(2, 2)
	# Plot all process timing
	axs[0,0].plot(full_stats["spin_time"])
	axs[0,0].set_ylabel("full mesh spin [s]")
	axs[0,1].plot(graph_stats["spin_time"])
	axs[0,1].set_ylabel("mesh graph spin [s]")
	axs[1,0].scatter(full_stats["num_vertices"], full_stats["spin_time"])
	axs[1,0].set_xlabel("# vertices in full mesh")
	axs[1,0].set_ylabel("full mesh spin [s]")
	axs[1,1].scatter(graph_stats["num_new_vertices"], graph_stats["spin_time"])
	axs[1,1].set_xlabel("# vertices in mesh graph")
	axs[1,1].set_ylabel("mesh graph spin [s]")
	plt.show()

def main():
    parser = argparse.ArgumentParser(description="create pgmo timing plots. ")
    parser.add_argument("log", type=str, help="path to mesh frontend log folder.")
    args = parser.parse_args()

    plot_frontend_timing(args.log)

if __name__ == "__main__":
	main() 
