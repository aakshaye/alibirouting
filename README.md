Prerequisites:
	Python 3.6 is required
	Python packages Shapely and Numpy are required
	The packages SWIG and Boost are also required to be installed

	The project can be built by running
		make -C target_region_computation/gc_distance

Input Data:
	The forbidden region must be defined on a map. An online tool can be used for this (https://clydedacruz.github.io/openstreetmap-wkt-playground/). The resulting WKB binary data should be dumped into a file (we used xxd for this). Our sample file is named forbidden_region_illinois.dat.

	Further, the GPS coordinates of the current server are required, and so are the bootstrap node hostnames.

	We have deployed on a topology of 6 nodes inside of GENI. The main files (excluding client.py) must be copied to all nodes.

	The main thread of execution can be started by running

		./AlibiDaemon.py

	To execute a query, run

		./client.py < input.txt
