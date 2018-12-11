from shapely.geometry import Point
import constants as c
import logging
import socket
import sys
import os
sys.path.append(os.path.abspath('target_region_computation'))

import regions as r



def get_location(mlat, mlon, mpoly):
    return r.dist_point_to_mpoly_nb(mpoly, mlon, mlat) * c.METERS_PER_MILE_CONVERSION


def get_latency(distance):
    return distance * c.SLOPE + c.INTERCEPT


def get_next_hop(query, hostname, slon, slat, dlon, dlat, avoid_zone, relay_zone, m_lon, m_lat, nodes_list, nn_list, p_node=None,
                 strategy=1):
    nodes = []

    avoided_region = get_location(m_lat, m_lon, avoid_zone)
    latency_ar = get_latency(avoided_region)

    relay_region = get_location(m_lat, m_lon, relay_zone)

    start_min_distance = relay_region - avoided_region
    node_id = None

    for neighbor in nodes_list:
        (n_id, n_lat, n_lon, n_rtt) = neighbor

        if p_node is not None and n_id == p_node:
            continue

        if n_rtt < latency_ar:
            avoided_region = get_location(n_lat, n_lon, avoid_zone)
            relay_region = get_location(n_lat, n_lon, relay_zone)

            min_distance = relay_region - avoided_region

            latency_ar = get_latency(avoided_region)

            for neighbor_new in nn_list.get(n_id, []):
                (nn_id, nn_lat, nn_lon, nn_rtt) = neighbor_new

                if nn_rtt < latency_ar:
                    distance = get_location(m_lat, m_lon, avoid_zone) - \
                               get_location(m_lat, m_lon, relay_zone)

                    if distance < min_distance:
                        min_distance = distance

            if start_min_distance < start_min_distance:
                node_id = n_id
                start_min_distance = min_distance

    nodes.append(node_id)

    return nodes

def nodelist_within_region(query_id, name, longitude, latitude, nodes_list, nn_list, avoid_zone, relay_zone):
   avoided_region = get_location(latitude, longitude, avoid_zone)
   latency_ar = get_latency(avoided_region)

   logging.info('Query ID: %d; Checking for neighbors inside target region', query_id)

   for neighbor in nodes_list:
       (n_id, n_lat, n_lon, n_rtt) = neighbor

       if n_rtt < latency_ar:

           if Point(n_lon, n_lat).intersects(relay_zone):
               return n_id

           avoided_region = get_location(n_lat, n_lon, avoid_zone)
           latency_ar = get_latency(avoided_region)

           if n_id not in nn_list:
               logging.info('No neighbors present for %s;', n_id)
               continue

           for neighbor_new in nn_list:
               (nn_id, nn_lat, nn_lon, nn_rtt) = neighbor_new

               if nn_rtt < latency_ar:

                   if Point(nn_lon, nn_lat).intersects(relay_zone):
                       return nn_id

   return None

def node_in_path(target_ip, path):
    m_node = socket.gethostbyaddr(target_ip)

    for nodes in path:
        return m_node == nodes[0]

    return False
