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

#include "gc_distance.hpp"


// Compute the great circle distance between two points using the spherical law of cosines.
double dist_cosine(double lon1, double lat1, double lon2, double lat2) {
	double lon1_rad = lon1*PI/180.0;
	double lat1_rad = lat1*PI/180.0;
	double lon2_rad = lon2*PI/180.0;
	double lat2_rad = lat2*PI/180.0;

	return acos(sin(lat1_rad)*sin(lat2_rad) + cos(lat1_rad)*cos(lat2_rad)*cos(lon2_rad-lon1_rad))*RADIUS;
}

// Compute the initial beraing of the line connecting given two points.
double bearing(double lon1, double lat1, double lon2, double lat2) {
	double lon1_rad = lon1*PI/180.0;
	double lat1_rad = lat1*PI/180.0;
	double lon2_rad = lon2*PI/180.0;
	double lat2_rad = lat2*PI/180.0;

	double ret = atan2(sin(lon2_rad-lon1_rad)*cos(lat2_rad), cos(lat1_rad)*sin(lat2_rad)-sin(lat1_rad)*cos(lat2_rad)*cos(lon2_rad-lon1_rad)) * 180.0/PI;
	
	return fmod(ret+360, 360);
}

// Given an initial bearing, identify a point that is "dist" (= a given distance) away from a given point.
void dest_point(double lon1, double lat1, double o_bearing, double dist, double *res_lon, double *res_lat) {
	double lon1_rad = lon1*PI/180.0;
	double lat1_rad = lat1*PI/180.0;
	double n_dist = dist/RADIUS;
	double bearing = o_bearing*PI/180.0;

	*res_lat = asin(sin(lat1_rad)*cos(n_dist)+cos(lat1_rad)*sin(n_dist)*cos(bearing));
	*res_lon = lon1_rad+atan2(sin(bearing)*sin(n_dist)*cos(lat1_rad), cos(n_dist)-sin(lat1_rad)*sin(*res_lat));

	*res_lat = (*res_lat)*180.0/PI;
	*res_lon = (fmod(*res_lon+PI,2*PI) - PI)*180.0/PI;
}

// Compute the intersection of two points.
int gc_intersect(double lon1, double lat1, double b1, double lon2, double lat2, double b2, double *int_lon, double *int_lat) {
	double lon1_rad = lon1*PI/180.0;
	double lat1_rad = lat1*PI/180.0;
	double lon2_rad = lon2*PI/180.0;
	double lat2_rad = lat2*PI/180.0;
	double b1_rad = b1*PI/180.0;
	double b2_rad = b2*PI/180.0;

	double d_lat = lat2_rad - lat1_rad;
	double d_lon = lon2_rad - lon1_rad;

	double d_12 = 2*asin(sqrt(sin(d_lat/2)*sin(d_lat/2)+cos(lat1_rad)*cos(lat2_rad)*sin(d_lon/2)*sin(d_lon/2)));

	double t1 = acos((sin(lat2_rad)-sin(lat1_rad)*cos(d_12))/(sin(d_12)*cos(lat1_rad)));
	if (isnan(t1)) {
		t1 = 0;
	}
	double t2 = acos((sin(lat1_rad)-sin(lat2_rad)*cos(d_12))/(sin(d_12)*cos(lat2_rad)));

	double b_12, b_21;
	
	if (sin(d_lon)>0) {
		b_12 = t1;
		b_21 = 2*PI-t2;
	}
	else {
		b_12 = 2*PI-t1;
		b_21 = t2;
	}

	double a1 = fmod(b1_rad-b_12+PI,2*PI)-PI;
	double a2 = fmod(b_21-b2_rad+PI,2*PI)-PI;

	if ( (sin(a1) == 0 && sin(a2) == 0) || (sin(a1)*sin(a2) < 0) ) {
		return -1;
	}

	a1 = fabs(a1);
	a2 = fabs(a2);

	double a3 = acos(-cos(a1)*cos(a2)+sin(a1)*sin(a2)*cos(d_12));
	double d_13 = atan2(sin(d_12)*sin(a1)*sin(a2), cos(a2)+cos(a1)*cos(a3));

	*int_lat = asin(sin(lat1_rad)*cos(d_13)+cos(lat1_rad)*sin(d_13)*cos(b1_rad));
	*int_lon = fmod((lon1_rad + (atan2(sin(b1_rad)*sin(d_13)*cos(lat1_rad), cos(d_13)-sin(lat1_rad)*sin(*int_lat)))+PI),2*PI)-PI;

	*int_lat = *int_lat*180.0/PI;
	*int_lon = *int_lon*180.0/PI;

	return 0;
}

// Compute the great circle distance between two points using the haversine formula.
double dist_haversine(double lon1, double lat1, double lon2, double lat2) {
	double lon1_rad = lon1*PI/180.0;
	double lat1_rad = lat1*PI/180.0;
	double lon2_rad = lon2*PI/180.0;
	double lat2_rad = lat2*PI/180.0;

	double ret = sin((lat2_rad-lat1_rad)/2)*sin((lat2_rad-lat1_rad)/2)+cos(lat1_rad)*cos(lat2_rad)*sin((lon2_rad-lon1_rad)/2)*sin((lon2_rad-lon1_rad)/2);

	return 2*atan2(sqrt(ret), sqrt(1-ret))*RADIUS; 
}

// Compute the cross track distance.
double dist_to_gc(double lon1, double lat1, double lon2, double lat2, double lon3, double lat3) {

	double b12 = bearing(lon1, lat1, lon2, lat2)*PI/180;
	double b13 = bearing(lon1, lat1, lon3, lat3)*PI/180;
	double d13 = dist_cosine(lon1, lat1, lon3, lat3)/RADIUS;

	return	asin(sin(d13)*sin(b13-b12))*RADIUS;
}

// Compute the along track distance. 
double along_track_distance(double lon1, double lat1, double lon2, double lat2, double lon3, double lat3) {

	double dxt = dist_to_gc(lon1, lat1, lon2, lat2, lon3, lat3)/RADIUS;
	double d13 = dist_cosine(lon1, lat1, lon3, lat3)/RADIUS;
	return acos(cos(d13)/cos(dxt))*RADIUS;
}

// Compute a symmetric point of a given point. 
void symmetric_point(double lon1, double lat1, double lon2, double lat2, double lon3, double lat3, double *sym_lon, double *sym_lat) {
	double dat = along_track_distance(lon1, lat1, lon2, lat2, lon3, lat3);
	double dat2 = along_track_distance(lon2, lat2, lon1, lat1, lon3, lat3);
	double b = bearing(lon1, lat1, lon2, lat2);

	double int_lon1, int_lat1;
	double int_lon2, int_lat2;
	dest_point(lon1, lat1, b, dat, &int_lon1, &int_lat1);
	dest_point(lon1, lat1, b, 2*PI*RADIUS-dat, &int_lon2, &int_lat2);
	double int_lon=int_lon2, int_lat=int_lat2;

	if (dist_haversine(lon3, lat3, int_lon1, int_lat1) < dist_haversine(lon3, lat3, int_lon2, int_lat2)) {
		int_lon = int_lon1;
		int_lat = int_lat1;
	}

	/*if(lon1==12.602780 && fabs(lat1-47.672775)<0.0000001) {
		printf("MULTIPOINT(%lf %lf, %lf %lf, %lf %lf)\n", int_lon, int_lat, lon3, lat3, int_lon2, int_lat2);
	}*/

	//printf ("intersection - %lf %lf\n", int_lon, int_lat);

	double dxt = fabs(dist_to_gc(lon1, lat1, lon2, lat2, lon3, lat3));
	double b2 = bearing(lon3, lat3, int_lon, int_lat);
	dest_point(lon3, lat3, b2, dxt*2, sym_lon, sym_lat);
}

// Compute the sum of the (minimum) distances from the 3rd point (lon3, lat3) and 4th point ({org, sym}_lon4, {org, sym}_lat4) to the (great circle) line segment connecting the first and second point.
double min_sum_dist_to_line(double lon1, double lat1, double lon2, double lat2, double lon3, double lat3, double org_lon4, double org_lat4, double sym_lon4, double sym_lat4) {
	double dist_to_org = dist_haversine(lon3, lat3, org_lon4, org_lat4);
	double dist_to_sym = dist_haversine(lon3, lat3, sym_lon4, sym_lat4);

	double lon4=org_lon4, lat4=org_lat4, dist_34=dist_to_org;

	if (dist_to_org < dist_to_sym) {
		lon4 = sym_lon4;
		lat4 = sym_lat4;
		dist_34 = dist_to_sym;
	}

	// check if gc segment 12 and 34 intersect
	/*if ( (dist_to_gc(lon1,lat1,lon2,lat2,lon3,lat3)*dist_to_gc(lon1,lat1,lon2,lat2,lon4,lat4) <= 0) && (dist_to_gc(lon3,lat3,lon4,lat4,lon1,lat1)*dist_to_gc(lon3,lat3,lon4,lat4,lon2,lat2) <= 0) ) {
		//printf("intersects!!\n");
		return dist_34;
	}*/

	double b1 = bearing(lon1, lat1, lon2, lat2);
	double b2 = bearing(lon3, lat3, lon4, lat4);

	double int_lon, int_lat, int_lon2, int_lat2;
	int ret = gc_intersect(lon1, lat1, b1, lon3, lat3, b2, &int_lon, &int_lat);

	if (!ret) {
		int i=0;
		double dist_12 = dist_haversine(lon1, lat1, lon2, lat2);
		for (;i<2;i++) {
			double dist_1i2 = dist_haversine(lon1, lat1, int_lon, int_lat) + dist_haversine(lon2, lat2, int_lon, int_lat);
			double dist_3i4 = dist_haversine(lon3, lat3, int_lon, int_lat) + dist_haversine(lon4, lat4, int_lon, int_lat);

			if((fabs(dist_1i2-dist_12) < 0.000001) && (fabs(dist_3i4-dist_34) < 0.000001)) {
				//printf("intersects!!\n");
				return dist_34;
			}

			//antipode
			int_lon = fmod(int_lon+180,360)-180;
			int_lat = -int_lat;
		}
	}

	double dist_314 = dist_haversine(lon1, lat1, lon3, lat3)+dist_haversine(lon1, lat1, lon4, lat4);
	double dist_324 = dist_haversine(lon2, lat2, lon3, lat3)+dist_haversine(lon2, lat2, lon4, lat4);

	if (dist_314 < dist_324) {
		return dist_314;
	}

	return dist_324;
}

// Compute the distance from the third point (lon3, lat3) to the line segment connecting the first and second point.
double dist_to_line(double lon1, double lat1, double lon2, double lat2, double lon3, double lat3) {

	double seg_length = dist_haversine(lon1, lat1, lon2, lat2);

	double dat1 = along_track_distance(lon1, lat1, lon2, lat2, lon3, lat3);
	double dat2 = along_track_distance(lon2, lat2, lon1, lat1, lon3, lat3);

	double min_dat, max_dat;
	if(dat1 < dat2) {
		min_dat = dat1;
		max_dat = dat2;
	}
	else {
		max_dat = dat1;
		min_dat = dat2;
	}

	double d13 = dist_haversine(lon1, lat1, lon3, lat3);
	double d23 = dist_haversine(lon2, lat2, lon3, lat3);
	double min_node_dist;

	if(d13 < d23)
		min_node_dist = d13;
	else
		min_node_dist = d23;

	if(max_dat >= seg_length) {
		return min_node_dist;
	}
	else{
		double dxt = fabs(dist_to_gc(lon1, lat1, lon2, lat2, lon3, lat3));
		if(dxt < min_node_dist)
			return dxt;
		else
			return min_node_dist;
	}
}

#ifndef LIGHT_WEIGHT
// Invoke min_sum_dist_line for all the lines of a given (multi-)polygon, and return the minimum value.
double min_sum_dist(mpolygon_type in_mpoly, double lon3, double lat3, double lon4, double lat4, std::vector<std::pair<double, double> > sym_points) {
	int i = 0, j = 0, idx = 0;
	double min_dist=INFINITY;

	for(i=0;i<boost::size(in_mpoly);i++) {
		for(j=0;j<boost::size(boost::geometry::exterior_ring(in_mpoly[i]))-1;j++) {

			double lon1 = boost::geometry::get<0>(boost::geometry::exterior_ring(in_mpoly[i])[j]);
			double lat1 = boost::geometry::get<1>(boost::geometry::exterior_ring(in_mpoly[i])[j]);
			double lon2 = boost::geometry::get<0>(boost::geometry::exterior_ring(in_mpoly[i])[j+1]);
			double lat2 = boost::geometry::get<1>(boost::geometry::exterior_ring(in_mpoly[i])[j+1]);

			double dist = min_sum_dist_to_line(lon1, lat1, lon2, lat2, lon3, lat3, lon4, lat4, sym_points[idx].first, sym_points[idx].second);

			idx++;

			if (dist < min_dist) {
				min_dist = dist;
			}
		}
	}

	return min_dist;
}

// Compute symmetric points for all the lines of a given (multi-)polyon.
void all_symmetric_points(mpolygon_type in_mpoly, double lon3, double lat3, std::vector<std::pair<double, double> >& sym_points) {
	int i = 0, j = 0;
	for(i=0;i<boost::size(in_mpoly);i++) {
		for(j=0;j<boost::size(boost::geometry::exterior_ring(in_mpoly[i]))-1;j++) {

			double lon1 = boost::geometry::get<0>(boost::geometry::exterior_ring(in_mpoly[i])[j]);
			double lat1 = boost::geometry::get<1>(boost::geometry::exterior_ring(in_mpoly[i])[j]);
			double lon2 = boost::geometry::get<0>(boost::geometry::exterior_ring(in_mpoly[i])[j+1]);
			double lat2 = boost::geometry::get<1>(boost::geometry::exterior_ring(in_mpoly[i])[j+1]);
		
			double sym_lon, sym_lat;
			symmetric_point(lon1, lat1, lon2, lat2, lon3, lat3, &sym_lon, &sym_lat);
			//sym_points.push_back(std::pair<double,double>(sym_lon, sym_lat));
			sym_points.push_back(std::make_pair(sym_lon, sym_lat));
		}
	}
}
#endif

// Compute target regions (based on Snell's law).
std::string gen_desirable_region_by_grid(std::string wkt_in_string, double src_lon, double src_lat, double dst_lon, double dst_lat, double grid_scale, double ineq_factor, int strategy, double max_ineq_factor, double ineq_factor_interval) {
#ifndef LIGHT_WEIGHT
	int lon_num = 360/grid_scale + 1;
	int lat_num = 180/grid_scale + 1;

	mpolygon_type in_mpoly;
	boost::geometry::read_wkt(wkt_in_string, in_mpoly);

	max_ineq_factor+=0.00000001;

	int i,j;

	char **grid = NULL;
	grid = new char*[lon_num];
	for(i=0;i<lon_num;i++) {
		grid[i] = new char[lat_num];
		for(j=0;j<lat_num;j++)
			grid[i][j] = 0;
	}

	std::vector<std::pair<double,double> > sym_s, sym_d;
	std::string ret("");

	if(strategy == 1) {
		all_symmetric_points(in_mpoly, src_lon, src_lat, sym_s);
	}
	all_symmetric_points(in_mpoly, dst_lon, dst_lat, sym_d);


	/* check each of grid points */
	for(i=0;i<lon_num;i++) {
		double tmp_lon = i*grid_scale-180;
		for(j=0;j<lat_num;j++) {
			double tmp_lat = j*grid_scale-90;

			//tmp_lon = -180;
			//tmp_lat = -60;

			double d_sr, d_sfr;
			if(strategy == 1) {
				d_sr = dist_haversine(src_lon, src_lat, tmp_lon, tmp_lat);
				d_sfr = min_sum_dist(in_mpoly, tmp_lon, tmp_lat, src_lon, src_lat, sym_s);

				/* d_sr and d_sfr corresponds to D(s,g) and min[D(s,f) + D(f,g)] respectively
				ineq_factor corresponds to (1+\delta). Thus the following checks the first part of the equation 2. */
				if (ineq_factor*d_sr >= d_sfr)
					continue;
			}

			double d_rd = dist_haversine(dst_lon, dst_lat, tmp_lon, tmp_lat);
			double d_rfd = min_sum_dist(in_mpoly, tmp_lon, tmp_lat, dst_lon, dst_lat, sym_d);

			/* Here we check the second part of the equation 2. */
			if (ineq_factor*d_rd < d_rfd)
				grid[i][j] = 1;

			double t_ineq_factor=ineq_factor+ineq_factor_interval;
			for(;t_ineq_factor<=max_ineq_factor;t_ineq_factor+=ineq_factor_interval) {
				if (strategy==1) {
					if ((t_ineq_factor*d_sr < d_sfr) && (t_ineq_factor*d_rd < d_rfd))
						grid[i][j]++;
				}
				else {
					if (t_ineq_factor*d_rd < d_rfd)
						grid[i][j]++;
				}
			}
		}
	}

	double t_ineq_factor=ineq_factor;
	for(;t_ineq_factor<=max_ineq_factor;t_ineq_factor+=ineq_factor_interval) {
		mpolygon_type out_mpoly;

		for(i=0;i<lon_num-1;i++) {
			for(j=0;j<lat_num-1;j++) {
				polygon_type tmp_poly;
				if (grid[i][j] > 0) { boost::geometry::append(tmp_poly, boost::geometry::make<point_type>(i*grid_scale-180, j*grid_scale-90)); }
				if (grid[i][j+1] > 0) { boost::geometry::append(tmp_poly, boost::geometry::make<point_type>(i*grid_scale-180, (j+1)*grid_scale-90)); }
				if (grid[i+1][j+1] > 0) { boost::geometry::append(tmp_poly, boost::geometry::make<point_type>((i+1)*grid_scale-180, (j+1)*grid_scale-90)); }
				if (grid[i+1][j] > 0) { boost::geometry::append(tmp_poly, boost::geometry::make<point_type>((i+1)*grid_scale-180, j*grid_scale-90)); }

				if (boost::size(boost::geometry::exterior_ring(tmp_poly)) > 2) {
					boost::geometry::append(tmp_poly, boost::geometry::exterior_ring(tmp_poly)[0]);
					mpolygon_type tmp_out_mpoly;
					//std::cout << boost::geometry::wkt(tmp_poly) << std::endl;
					//printf("%d %d\n", lon_num, lat_num);
					boost::geometry::union_(out_mpoly, tmp_poly, tmp_out_mpoly);
					out_mpoly = tmp_out_mpoly;
					//std::cout << boost::geometry::wkt(out_mpoly) << std::endl;
				}

				grid[i][j]--;

			}

			grid[i][j]--;
		}

		for(j=0;j<lat_num;j++) {
			grid[i][j]--;
		}

		std::stringstream t_ret;
		t_ret << std::fixed << std::setprecision(10) << boost::geometry::wkt(out_mpoly);

		//std::cout << boost::geometry::wkt(out_mpoly) << std::endl << std::endl;
		ret=ret+"|"+t_ret.str();
	}


	for(i=0;i<lon_num;i++) {
		delete [] grid[i];
	}
	delete [] grid;

	//return ret.str();
	return ret;
#else
	return std::string("NULL");
#endif
}

// Compute distance (in meters) between a point and (multi-)polygon.
double dist_to_mpolygon(std::string wkt_in_string, double lon3, double lat3) {
#ifndef LIGHT_WEIGHT
	mpolygon_type in_mpoly;

	boost::geometry::read_wkt(wkt_in_string, in_mpoly);

	double min_dist=INFINITY;
	int i = 0, j = 0;
	for(i=0;i<boost::size(in_mpoly);i++) {
		for(j=0;j<boost::size(boost::geometry::exterior_ring(in_mpoly[i]))-1;j++) {
			double lon1 = boost::geometry::get<0>(boost::geometry::exterior_ring(in_mpoly[i])[j]);
			double lat1 = boost::geometry::get<1>(boost::geometry::exterior_ring(in_mpoly[i])[j]);
			double lon2 = boost::geometry::get<0>(boost::geometry::exterior_ring(in_mpoly[i])[j+1]);
			double lat2 = boost::geometry::get<1>(boost::geometry::exterior_ring(in_mpoly[i])[j+1]);
			
			double dist = dist_to_line(lon1, lat1, lon2, lat2, lon3, lat3);

			if( dist < min_dist )
				min_dist = dist;
		}
	}
#else
	double min_dist = -1;
#endif

	return min_dist;
}

// Compute distance (in meters) between a point and (multi-)polygon. This version does not use Boost library unlike dist_point_to_mpoly
double dist_to_mpolygon_nb(std::string wkt_in_string, double lon3, double lat3) {
	std::istringstream f(wkt_in_string);

	std::string line;
	double prev_lon, prev_lat, lon, lat;
	int is_first = 1;

	double min_dist=INFINITY;

	while(std::getline(f, line)) {
		if (line.length() == 1) {
			is_first = 1;
			continue;
		}
		std::istringstream lf(line);
		std::string lon_str, lat_str;
		std::getline(lf, lon_str, ' ');
		std::getline(lf, lat_str, ' ');
		prev_lon = lon;
		prev_lat = lat;
		lon = atof(lon_str.c_str());
		lat = atof(lat_str.c_str());
		if (!is_first) {
			//std::cout << line << std::endl;
			//std::cout <<  std::fixed << std::setprecision(10) <<  prev_lon << ' ' << prev_lat << ' ' << lon << ' ' << lat << std::endl;

			double dist = dist_to_line(prev_lon, prev_lat, lon, lat, lon3, lat3);

			if( dist < min_dist )
				min_dist = dist;
		}
		else {
			is_first = 0;
		}
	}

	return min_dist;
}

