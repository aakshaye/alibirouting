#!/usr/bin/python3

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

from __future__ import with_statement
from shapely.wkt import dumps, loads
from shapely.geometry import MultiPolygon, Polygon
import gc_distance.gc_distance as gc
import numpy
import sys
import os

# Load precomputed target regions from file, and return a dictionary that stores the target regions. The returned dictionary can be passed as an argument of get_target_region.
def load_precomputed_trs(filename):
	trs={}

	with open(filename, 'r') as f:
		for line in f:
			(src_lon,src_lat,dst_lon,dst_lat,ineq_factor,tr)=line.split('||')
			coords=(src_lon,src_lat,dst_lon,dst_lat)

			if not trs.has_key(coords):
				trs[coords]={}
			
			if tr.rstrip() == 'NoTargetRegion':
				trs[coords][float(ineq_factor)]=None
			else:
				trs[coords][float(ineq_factor)]=loads(tr)

	return trs

# An auxiliary function for dist_point_mpoly_nb.
def s_dumps(mpoly):
	ret_str=''
	if mpoly.geom_type == 'MultiPolygon':
		for poly in mpoly.geoms:
			for point in poly.exterior.coords:
				 ret_str+='%.10f %.10f\n' % (point[0], point[1])
			ret_str+='P\n'
	else:
		for point in mpoly.exterior.coords:
			 ret_str+='%.10f %.10f\n' % (point[0], point[1])
		ret_str+='P\n'

	return ret_str

# Compute distance (in meters) between a point and (multi-)polygon.
def dist_point_to_mpoly(mpoly, lon, lat):
	if isinstance(mpoly, MultiPolygon):
		return gc.dist_to_mpolygon(dumps(mpoly), lon, lat)
	else:
		return gc.dist_to_mpolygon(dumps(MultiPolygon([mpoly])), lon, lat)

# Compute distance (in meters) between a point and (multi-)polygon. This version does not use Boost library unlike dist_point_to_mpoly
def dist_point_to_mpoly_nb(mpoly, lon, lat):
	return gc.dist_to_mpolygon_nb(s_dumps(mpoly), lon, lat)


# Compute the target region for the triple (forbidden_region:fr, source:(src_lon, src_lat), destination:(dst_lon, dst_lat)). This method can compute target regions for multiple deltas. For example, letting ineq_factor := delta+1, if ineq_factor = 1, max_ineq_factor = 2 and ineq_factor_interval = 0.5, target regions for ineq_factor = 1, 1.5, 2.0 will be computed and returned as a list.
def gen_multiple_desirable_regions_by_grid(pre_file_name, src_lon, src_lat, dst_lon, dst_lat, grid_scale=2, ineq_factor=2, tolerance=0.5, strategy=1, max_ineq_factor=None, ineq_factor_interval=None, p_fr=None):
	if p_fr == None:
		print('Forbidden region must be given')
		return None

	if tolerance == 0:
		fr = p_fr
	else:
		fr = p_fr.simplify(tolerance)

	if max_ineq_factor == None:
		max_ineq_factor = ineq_factor
		ineq_factor_interval = 1

	if isinstance(fr, MultiPolygon):
		str_fr = dumps(fr)
	else:
		str_fr = dumps(MultiPolygon([fr]))

	ret_str = gc.gen_desirable_region_by_grid(str_fr, src_lon, src_lat, dst_lon, dst_lat, grid_scale, ineq_factor, strategy, max_ineq_factor, ineq_factor_interval)

	sps = ret_str.split('|')
	ret_list=[]
	for sp in sps:
		if len(sp) < 2:
			continue

		if len(sp) < 20:
			ret_list.append(None)
		else:
			ret_list.append(loads(sp))

	return ret_list

# Get the target region for the triple (forbidden_region:fr, source:src_coord, destination:dst_coord) from precomputed_target_regions if exists. Otherwise, compute it.
def get_target_region(fr, src_coord, dst_coord, ineq_factor=1.05, precomputed_target_regions={}):

	#if precomputed_target_regions not in ((src_coord[0],src_coord[1],dst_coord[0],dst_coord[1])):
	#	if precomputed_target_regions[(src_coord[0],src_coord[1],dst_coord[0],dst_coord[1])] not in (ineq_factor):
	#		return precomputed_target_regions[(src_coord[0],src_coord[1],dst_coord[0],dst_coord[1])][ineq_factor]
	return gen_multiple_desirable_regions_by_grid('', float(src_coord[0]), float(src_coord[1]), float(dst_coord[0]), float(dst_coord[1]), 2, ineq_factor, 0, 1, ineq_factor, 1, fr)[0]



