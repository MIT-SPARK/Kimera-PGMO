"""
This python script takes as input the timing log produced by the mesh frontend.
Author: Yun Chang
""" 

import numpy as np 
import matplotlib.pyplot as plt
import sys

full_spin_t = []
f_comp_t = []
s_comp_t = []

with open(sys.argv[1], 'r') as log_file:
	log_entries = log_file.readlines()
	# discard first header line 
	log_entries = log_entries[1:]

	for entry in log_entries: 
		entry_as_list = entry.split(",")
		full_spin_t.append(int(entry_as_list[0]))
		f_comp_t.append(int(entry_as_list[1]))
		s_comp_t.append(int(entry_as_list[2]))


plt.plot(full_spin_t)
plt.plot(f_comp_t)
plt.plot(s_comp_t)

plt.legend(["full", "full compression", "graph compression"])
plt.title("Mesh Frontend timing breakdown")

plt.show()
