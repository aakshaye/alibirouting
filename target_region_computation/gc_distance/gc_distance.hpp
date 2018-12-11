/*
 * Copyright (c) 2015, University of Maryland All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define RADIUS 6378137
const double PI = 4.0*atan(1.0);

#ifndef LIGHT_WEIGHT
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/multi/geometries/multi_polygon.hpp>

#include <boost/geometry/io/wkt/wkt.hpp>

#include <boost/foreach.hpp>

typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::linestring<point_type> line_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;
typedef boost::geometry::model::multi_polygon<polygon_type> mpolygon_type;
typedef boost::geometry::ring_type<polygon_type>::type ring_type;
#endif


double dist_cosine(double lon1, double lat1, double lon2, double lat2);

double bearing(double lon1, double lat1, double lon2, double lat2);

double dist_haversine(double lon1, double lat1, double lon2, double lat2);

void dest_point(double lon1, double lat1, double bearing, double dist, double *res_lon, double *res_lat);

int gc_intersect(double lon1, double lat1, double b1, double lon2, double lat2, double b2, double *int_lon, double *int_lat);

double dist_to_gc(double lon1, double lat1, double lon2, double lat2, double lon3, double lat3);

double along_track_distance(double lon1, double lat1, double lon2, double lat2, double lon3, double lat3);

void symmetric_point(double lon1, double lat1, double lon2, double lat2, double lon3, double lat3, double *sym_lon, double *sym_lat);

double min_sum_dist_to_line(double lon1, double lat1, double lon2, double lat2, double lon3, double lat3, double org_lon4, double org_lat4, double sym_lon4, double sym_lat4);

double dist_to_line(double lon1, double lat1, double lon2, double lat2, double lon3, double lat3);

double dist_to_mpolygon(std::string wkt_in_string, double lon3, double lat3);

std::string gen_desirable_region_by_grid(std::string wkt_in_string, double src_lon, double src_lat, double dst_lon, double dst_lat, double grid_scale, double ineq_factor, int strategy, double max_ineq_factor, double ineq_factor_interval);

double dist_to_mpolygon_nb(std::string wkt_in_string, double lon3, double lat3);

