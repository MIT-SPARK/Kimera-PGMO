"""
This python script takes as input the timing log produced by the mesh frontend.
Author: Yun Chang
""" 
import argparse
import numpy as np
import csv
import matplotlib.pyplot as plt
import sys

def read_pgmo_log(filename):
    stats = {"num_vertices": [], "num_graph_vertices": [], "pg_cb": [], \
    		"mg_cb": [], "fm_cb": [], "num_kf": [], "num_lc": []}
    with open(filename, mode='r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            stats["num_vertices"].append(int(row["num-vertices"]))
            stats["num_graph_vertices"].append(int(row["num-vertices-simplified"]))
            stats["num_kf"].append(int(row["num-keyframes"]))
            stats["num_lc"].append(int(row["num-loop-closures"]))
            stats["pg_cb"].append(float(row["pg-cb-time(mu-s)"]) * 1e-6)
            stats["mg_cb"].append(float(row["inc-mesh-cb-time(mu-s)"]) * 1e-6)
            stats["fm_cb"].append(float(row["full-mesh-cb-time(mu-s)"]) * 1e-6)
    return stats

def plot_pgmo_timing(filename):
	stats = read_pgmo_log(filename)
	fig, axs = plt.subplots(1, 2)
	# Plot all process timing
	axs[0].plot(stats["num_kf"], stats["pg_cb"], label="pose-graph")
	axs[0].plot(stats["num_kf"], stats["mg_cb"], label="mesh-graph")
	axs[0].plot(stats["num_kf"], stats["fm_cb"], label="full-mesh")
	axs[0].legend()
	axs[0].set_ylabel("spin time [s]")
	axs[0].set_xlabel("keyframes")
	# Plot number of loop closures
	axs[1].plot(stats["num_kf"], stats["num_lc"])
	axs[1].set_ylabel("# loop closures")
	axs[1].set_xlabel("keyframes")
	plt.show()

def main():
    parser = argparse.ArgumentParser(description="create pgmo timing plots. ")
    parser.add_argument("log_path", type=str, help="path to pgmo log.")
    args = parser.parse_args()

    plot_pgmo_timing(args.log_path)

if __name__ == "__main__":
	main() 
