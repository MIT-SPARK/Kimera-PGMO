"""
This python script takes as input the log produced by the mesh frontend 
and evluate and plots the timing during the main callback
Author: Yun Chang
""" 

import numpy as np 
import matplotlib.pyplot as plt
import sys

num_vertices = []
num_simplified_vertices = []
num_new_indices = []
num_new_edges = []
mesh_cb_t = []

with open(sys.argv[1], 'r') as log_file:
	log_entries = log_file.readlines()
	# discard first header line 
	log_entries = log_entries[1:]

	for entry in log_entries: 
		entry_as_list = entry.split(",")
		num_vertices.append(int(entry_as_list[0]))
		num_simplified_vertices.append(int(entry_as_list[1]))
		num_new_indices.append(int(entry_as_list[2]))
		num_new_edges.append(int(entry_as_list[3]))
		mesh_cb_t.append(int(entry_as_list[4]))

# First print the timing of each stat 
mesh_cb_stats = []

mesh_cb_stats.append(np.mean(np.asarray(mesh_cb_t)))
mesh_cb_stats.append(np.min(np.asarray(mesh_cb_t)))
mesh_cb_stats.append(np.max(np.asarray(mesh_cb_t)))

print("Callback timing (microseconds): mean: {} min: {} max: {} ".format(mesh_cb_stats[0], mesh_cb_stats[1], mesh_cb_stats[2]))

fig, axs = plt.subplots(2, 2)
axs[0, 0].scatter(num_vertices, mesh_cb_t)
axs[0, 0].hlines(mesh_cb_stats[0], 0, num_vertices[-1], color='r')
axs[0, 0].set_title('Callback Elapsed vs Total Number of Vertices')
axs[0, 0].set_xlabel('Total Number of Vertices')
axs[0, 1].scatter(num_simplified_vertices, mesh_cb_t)
axs[0, 1].hlines(mesh_cb_stats[0], 0, num_simplified_vertices[-1], color='r')
axs[0, 1].set_title('Callback Elapsed vs Number of Simplified Vertices')
axs[0, 1].set_xlabel('Number of Simplified Vertices')
axs[1, 0].scatter(num_new_indices, mesh_cb_t)
axs[1, 0].hlines(mesh_cb_stats[0], 0, np.max(np.asarray(num_new_indices)), color='r')
axs[1, 0].set_title('Callback Elapsed vs Number of Indices to Add')
axs[1, 0].set_xlabel('Number of Indices to Add')
axs[1, 1].scatter(num_new_edges, mesh_cb_t)
axs[1, 1].hlines(mesh_cb_stats[0], 0, np.max(np.asarray(num_new_edges)), color='r')
axs[1, 1].set_title('Callback Elapsed vs Number of Edges to Add')
axs[1, 1].set_xlabel('Number of Edges to Add')

for ax in axs.flat:
    ax.set(ylabel='Elapsed (microseconds')

plt.show()
