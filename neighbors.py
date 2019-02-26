## Based heavily on the implementation of Alibi Routing provided by University of Maryland

#
# Copyright (c) 2015, University of Maryland All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation and/or
# other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

import socket
import os
from random import sample, randint
from math import ceil, atan2, degrees
import logging
import util as utils
import AlibiMsg as msg
import constants as c
import json
import numpy as np

class Neighbors:
	def __init__(self):
		self.nodes = []
		self.clients = []
		self.gps = []
		self.bins = []
		self.bin_intervals = []
		self.nn_list = {}
		self.length_max_nodes = 32
		self.length_max_clients = 128
		self.gossip_num = 5
		self.ewma_weight = 0.7

	def add_rtts(self, node_list, random_loss=False):
		up_neighbors = []
		down_neighbors = []
		for node in node_list:
			latency = self.calculate_latency(node[0])
			if random_loss:
				if randint(0, 4) < 1:
					latency = None
			# print node[0], node[1], node[2], latency
			if latency is None:
				down_neighbors.append((node[0],))
			else:
				up_neighbors.append((node[0], node[1], node[2], latency))

		return up_neighbors, down_neighbors

	def update_rtts(self, neighbors, neighbor_info):
		for info in neighbor_info:
			for i in range(len(neighbors)):
				if info[0] == neighbors[i][0]:
					# update using exponentially-weighted moving average
					new_rtt = self.ewma_weight * float(info[3]) + \
							  (1 - self.ewma_weight) * float(neighbors[i][3])
					neighbors[i] = (neighbors[i][0], neighbors[i][1], neighbors[i][2], new_rtt)
					break

	def process_rtt(self, neighbors):
		up_neighbors, down_neighbors = neighbors

		#print(up_neighbors, "$" * 20,down_neighbors)
		#print(self.nodes, self.clients)
		self.nodes[:] = [x for x in self.nodes if not self.is_in_list(x, down_neighbors)]
		self.clients[:] = [x for x in self.clients if not self.is_in_list(x, down_neighbors)]
		#print(self.nodes, "$" * 20,self.clients)

		self.update_rtts(self.nodes, up_neighbors)
		self.update_rtts(self.clients, up_neighbors)

		final_nodes = [x for x in up_neighbors if not self.is_in_list(x, self.clients + self.nodes)]

		#print(final_nodes)

		return final_nodes

	def add_to_known_peers(self, nodes):
		up_neighbors = self.process_rtt(nodes)
		self.clients += up_neighbors

		if len(self.clients) > self.length_max_clients:
			self.clients = self.clients[len(self.clients) - self.length_max_clients]

	def update_neighbors(self):
		neighbors = []

		if len(self.nodes) < self.length_max_nodes:
			neighbors = sample(self.clients,
							   min(self.length_max_nodes - len(self.nodes), len(self.clients)))
			self.clients[:] = [x for x in self.clients if x not in neighbors]
			self.nodes += neighbors

		if len(self.nodes) == 0:
			return

		r_list = []
		iter_num = randint(3, 6)
		for i in range(iter_num):
			if len(self.clients) == 0:
				break

			self.gen_bin_intervals()
			self.fill_bins()

			# choose a neighbor to remove
			node_r = self.choose_one_from_nodes()
			node_a = sample(self.clients, 1)[0]
			r_list.append(node_r)
			self.clients.remove(node_a)
			self.nodes.remove(node_r)
			self.nodes.append(node_a)

			neighbors.append(node_a)
			self.nn_list.pop(node_a[0], None)

		self.clients += r_list
		return neighbors
	
	def choose_one_from_nodes(self):
		max_len = None

		for b in self.bins:
			if max_len is None or (b is not None and max_len <= len(b)):
				max_len = len(b)
				max_bin = b

		target = max_bin[self.calculate_angles(max_bin)]

		return target

	def gen_bin_intervals(self, bin_num=10, strategy=0):
		# uniform
		min_rtt = None
		max_rtt = None

		for (ip, lon, lat, rtt) in self.nodes:
			if min_rtt is None or min_rtt > float(rtt):
				min_rtt = float(rtt)

			if max_rtt is None or max_rtt < float(rtt):
				max_rtt = float(rtt)

		interval = ceil((max_rtt - min_rtt) / bin_num)

		self.bin_intervals = []
		split_point = ceil(bin_num / 3)

		for i in range(bin_num + 1):
			self.bin_intervals.append(min_rtt + i * interval)
			if strategy == 1 and i < split_point:
				self.bin_intervals.append(min_rtt + (i + 0.5) * interval)

		self.bin_intervals[-1] += 0.01

	def fill_bins(self):
		self.bins = [None] * (len(self.bin_intervals) - 1)

		rtt_list = [rtt for (ip, lon, lat, rtt) in self.nodes]
		bin_indices = np.digitize(rtt_list, self.bin_intervals)

		for i in range(len(bin_indices)):
			idx = bin_indices[i] - 1
			if self.bins[idx] is None: self.bins[idx] = []
			self.bins[idx].append(self.nodes[i])

	def calculate_angles(self, node):
		if len(node) < 2:
			return randint(0, len(node) - 1)

		angles = []
		for p in node:
			angle = degrees(atan2(float(p[2]) - float(self.gps[1]), float(p[1]) - float(self.gps[0])))
			if angle < 0:
				angle += 360
			angles.append((angle, p[0]))

		angles.sort()

		for i in range(len(angles)):
			if i == 0:
				angle = angles[1][0] + 360 - angles[-1][0]
			elif i == (len(angles) - 1):
				angle = angles[0][0] + 360 - angles[-2][0]
			else:
				angle = angles[i + 1][0] - angles[i - 1][0]

			angles[i] += (angle,)

		min_node_ip = min(angles, key=lambda a: a[2])[1]

		for i in range(len(node)):
			if node[i][0] == min_node_ip:
				return i

		return None

	@staticmethod
	def calculate_latency(node):
		nonce = randint(0, 65535)
		ping = msg.Ping(nonce)
		(response, latency) = utils.send_msg(node, c.PORT, ping, False, True, True, 1)

		pr_nonce = msg.Msg.fromBytes(response).param["nonce"]

		logging.debug('Checking if nonces are matching (%d == %d)', nonce, pr_nonce)

		return latency if nonce == pr_nonce else None


	def set_gps(self, coordinates):
		self.gps = coordinates

	def remove_node(self, node):
		self.nn_list.pop(node, None)
		self.nodes[:] = [x for x in self.nodes if not self.equal_ips(x, (node,))]

	def remove_item(self, item, items_list):
		return [x for x in items_list if not self.equal_ips(x, (item,))]

	def is_in_list(self, item, t_list):
		for t_item in t_list:
			if t_item[0] == item[0]:
				return True

		return False

	def __contains__(self, item, items_list):
		for items in items_list:
			return items[0] == item[0]

		return False

	@staticmethod
	def equal_ips(ip_addr1, ip_addr2):
		return ip_addr1[0] == ip_addr2[0]
