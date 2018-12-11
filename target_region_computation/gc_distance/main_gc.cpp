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

// This is a simple test program for gc_distance.cpp.

int main(int argc, char *argv[]) {

	double lon1 = atof(argv[1]);
	double lat1 = atof(argv[2]);
	double lon2 = atof(argv[3]);
	double lat2 = atof(argv[4]);

	printf("%10.10lf %lf %lf %lf %lf\n", PI, lon1, lat1, lon2, lat2);

	printf ("%lf\n", dist_cosine(lon1, lat1, lon2, lat2));
	printf ("%lf\n", bearing(lon1, lat1, lon2, lat2));
	printf ("%lf\n", dist_haversine(lon1, lat1, lon2, lat2));
	printf ("%lf\n", dist_to_gc(lon1, lat1, lon2, lat2, lon2-1, lat2-1));
	printf ("%lf\n", along_track_distance(lon1, lat1, lon2, lat2, lon2-1, lat2-1));
	printf ("%lf\n", dist_to_line(lon1, lat1, lon2, lat2, lon2-1, lat2-1));

	/*double lon3, lat3;
	double b = bearing(4, 10, 4.063633, 1.022922);
	printf ("%lf\n",b);
	dest_point(4, 10, b, 999348.5*2, &lon3, &lat3);
	printf ("%lf %lf %lf\n", lon3, lat3, dist_cosine(lon3, lat3,4.063633, 1.022922) );*/

	/*double sym_lon, sym_lat;
	symmetric_point(1, 1, 3, 1, 4, 10, &sym_lon, &sym_lat);
	printf ("%lf %lf %lf\n", sym_lon, sym_lat, dist_cosine(sym_lon, sym_lat,4.063633, 1.022922) );*/

	/*
	double int_lon, int_lat;
	double b1=bearing(4,10,sym_lon,sym_lat);
	double b2=bearing(1,1,50,1);
	printf ("b1 - %lf, b2 - %lf\n",b1,b2);
	int ret=gc_intersect(4,10,b1,1,1,b2, &int_lon, &int_lat);
	printf ("%d %lf %lf\n", ret, int_lon, int_lat);
	*/

	/*double res_dist = min_sum_dist_to_line(1, 1, 3, 1, 4, 10, 4, 10, sym_lon, sym_lat);
	printf("res dist - %lf\n",res_dist);*/


	std::string wkt_in_string;
	getline(std::cin, wkt_in_string);

#ifndef LIGHT_WEIGHT
	//printf("min dist - %lf\n", dist_to_mpolygon(wkt_in_string, lon1, lat1));
	
//    std::cout << gen_desirable_region_by_grid(wkt_in_string, lon1, lat1, lon2, lat2,5,2,0) << std::endl;
    /*std::cout << gen_desirable_region_by_grid(wkt_in_string, lon1, lat1, lon2, lat2,5,1.05,1) << std::endl;
    std::cout << gen_desirable_region_by_grid(wkt_in_string, lon1, lat1, lon2, lat2,5,1.10,1) << std::endl;
    std::cout << gen_desirable_region_by_grid(wkt_in_string, lon1, lat1, lon2, lat2,5,1.15,1) << std::endl;
    std::cout << gen_desirable_region_by_grid(wkt_in_string, lon1, lat1, lon2, lat2,5,1.20,1) << std::endl;*/


    std::cout << gen_desirable_region_by_grid(wkt_in_string, lon1, lat1, lon2, lat2, 5, 1.00, 1, 1.00,0.05) << std::endl;

	//dump_all_dist_to_mpolygon(wkt_in_string, "./test.dat", 5);
    //std::cout << load_all_dist_to_mpolygon_and_compute("./test.dat", wkt_in_string, lon1, lat1, lon2, lat2,5,1.05,1,1.05,0.05) << std::endl;
#endif

	return 0;
}


