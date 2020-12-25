"""
This python script takes as input the log produced by pgmo 
and evluate and plots the timing of the callback functions during the run
Author: Yun Chang
""" 

import numpy as np 
import matplotlib.pyplot as plt
import sys

num_factors = []
num_vertices = []
inc_mesh_cb_t = []
full_mesh_cb_t = []
pose_graph_cb_t = []
path_cb_t = []

with open(sys.argv[1], 'r') as log_file:
	log_entries = log_file.readlines()
	# discard first header line 
	log_entries = log_entries[1:]

	for entry in log_entries: 
		entry_as_list = entry.split(",")
		num_factors.append(int(entry_as_list[3]))
		num_vertices.append(int(entry_as_list[4]))
		inc_mesh_cb_t.append(int(entry_as_list[6]))
		full_mesh_cb_t.append(int(entry_as_list[7]))
		pose_graph_cb_t.append(int(entry_as_list[8]))
		path_cb_t.append(int(entry_as_list[9]))

# First print the timing of each stat 
inc_mesh_cb_stats = []
full_mesh_cb_stats = []
pose_graph_cb_stats = []
path_cb_stats = []

inc_mesh_cb_stats.append(np.mean(np.asarray(inc_mesh_cb_t)))
inc_mesh_cb_stats.append(np.min(np.asarray(inc_mesh_cb_t)))
inc_mesh_cb_stats.append(np.max(np.asarray(inc_mesh_cb_t)))

full_mesh_cb_stats.append(np.mean(np.asarray(full_mesh_cb_t)))
full_mesh_cb_stats.append(np.min(np.asarray(full_mesh_cb_t)))
full_mesh_cb_stats.append(np.max(np.asarray(full_mesh_cb_t)))

pose_graph_cb_stats.append(np.mean(np.asarray(pose_graph_cb_t)))
pose_graph_cb_stats.append(np.min(np.asarray(pose_graph_cb_t)))
pose_graph_cb_stats.append(np.max(np.asarray(pose_graph_cb_t)))

path_cb_stats.append(np.mean(np.asarray(path_cb_t)))
path_cb_stats.append(np.min(np.asarray(path_cb_t)))
path_cb_stats.append(np.max(np.asarray(path_cb_t)))

print("Incremental mesh callback timing (microseconds): mean: {} min: {} max: {} ".format(inc_mesh_cb_stats[0], inc_mesh_cb_stats[1], inc_mesh_cb_stats[2]))
print("Full mesh callback timing (microseconds): mean: {} min: {} max: {} ".format(full_mesh_cb_stats[0], full_mesh_cb_stats[1], full_mesh_cb_stats[2]))
print("Incremental pose graph callback timing (microseconds): mean: {} min: {} max: {} ".format(pose_graph_cb_stats[0], pose_graph_cb_stats[1], pose_graph_cb_stats[2]))
print("Path callback timing (microseconds): mean: {} min: {} max: {} ".format(path_cb_stats[0], path_cb_stats[1], path_cb_stats[2]))

fig, axs = plt.subplots(2, 2)
axs[0, 0].scatter(range(len(inc_mesh_cb_t)), inc_mesh_cb_t)
axs[0, 0].hlines(inc_mesh_cb_stats[0], 0, len(inc_mesh_cb_t), color='r')
axs[0, 0].set_title('Incremental Mesh Callback Elapsed')
axs[0, 1].scatter(num_vertices, full_mesh_cb_t)
axs[0, 1].hlines(full_mesh_cb_stats[0], 0, num_vertices[-1], color='r')
axs[0, 1].set_title('Full Mesh Callback Elapsed')
axs[0, 1].set_xlabel('Size of Mesh (number of vertices)')
axs[1, 0].scatter(num_factors, pose_graph_cb_t)
axs[1, 0].hlines(pose_graph_cb_stats[0], 0, num_factors[-1], color='r')
axs[1, 0].set_title('Pose Graph Callback Elapsed')
axs[1, 0].set_xlabel('Size of Factpr Graph (number of factors)')
axs[1, 1].scatter(range(len(path_cb_t)), path_cb_t)
axs[0, 0].hlines(path_cb_stats[0], 0, len(path_cb_t), color='r')
axs[1, 1].set_title('Path Callback Elapsed')

for ax in axs.flat:
    ax.set(ylabel='Elapsed (microseconds')

plt.show()
