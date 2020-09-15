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

import socket
import constants as c
import util as u
import logging
import threading
import socketserver
import AlibiMsg as M
import queue
import neighbors as nbors
import time
import signal
import forwarding as f
from shapely.wkb import loads as b_loads, dumps as b_dumps
from shapely.geometry import Point
from random import *
import sys

#configuration parameters
my_latlon_map = {}
my_hostname = None
n = nbors.Neighbors()
# using set becuase invariant is that queries don't repeat ever
past_queries = set()
# lock for accessing the set of query ids
qid_lock = threading.Lock()
neighbor_lock = threading.Lock()
active_lock = threading.Lock()
active_queries = set()
queryq_lock = threading.Lock()
daemon_lock = threading.Lock()
n_lock = threading.Lock()
queryq = {}
initialized = False
gossip_timer = None
refresh_neighbors_timer = None
request_neighborlist_timer = None



class ThreadedTCPRequestHandler(socketserver.BaseRequestHandler):
    
    def handle(self):
        length = None
        data = ''
        to_read = 0
        read_more = False
        newdata = self.request.recv(4096)
        if newdata[:3].decode("utf-8") == "+++":
            i = 3
            while newdata[i] != ord('+'):
                to_read *= 10
                to_read += int(chr(newdata[i]))
                i +=1
            newdata = newdata[i + 1:]
        else:
            print("Invalid start of data, exiting")
            return
        
        while True:
            available = len(newdata)
            if to_read > available:
                #print("available " + str(available))
                data += newdata[:].decode("utf-8")
                #print("data", data)
                to_read -= available
                #print("Yet to read " + str(to_read))
                newdata = self.request.recv(4096)
                continue
            else:
                data += newdata[:to_read].decode("utf-8")
                to_read
                break
        
        msg = M.Msg.fromBytes(data)
        mtype = msg.param["mtype"]
        #print(mtype)
        if mtype == M.PING_MSG:
            logging.debug("Received ping")
            self.pingHandler(msg)
        elif mtype == M.QUERY:
            logging.debug("Received query")
            self.query_handler(msg)
        elif mtype == M.QUERY_REQ:
            logging.debug("Received query from client")
            self.query_request_handler(msg)
        elif mtype == M.QUERY_CHK_RQ:
            logging.debug("Received checked query")
            self.query_chk_req_handler(msg)
        elif mtype == M.NLIST_REQ:
            logging.debug("Received request for neighbor list")
            self.neighbor_list_handler(msg)
        elif mtype == M.QUERY_RESP:
            logging.debug("Received response")
            self.query_resp_handler(msg)
        elif mtype == M.GOSSIP:
            logging.debug("Received gossip")
            self.gossipHandler(msg)
        elif mtype == M.JOIN:
            logging.debug('Received join request')
            self.joinPeerHandler(msg)
        else:
            logging.warning("Unsupported data")


    def pingHandler(self, m):
        nonce = int(m.param["nonce"])
        p = M.Msg.asBytes(M.PingResponse(nonce))
        pbytes = p.encode("utf-8")
        self.request.sendall(pbytes)

    def query_handler(self, q):
        global past_queries

        myip, myport = self.request.getsockname()

        query = q.param["query"]
        shost = q.param["shost"]
        slat = q.param["slat"]
        slon = q.param["slon"]
        dhost = q.param["dhost"]
        dlat = q.param["dlat"]
        dlon = q.param["dlon"]
        ttl = q.param["ttl"]
        forbidden_region = q.param["fr"]
        target_region = q.param["tr"]
        path = q.param["path"]

        with qid_lock:
            if query in past_queries:
                # this query has been performed before, disregard it
                return

        # Add self to the path taken by the packet so far
        new_path = path.append((my_hostname, my_latlon_map["lon"], my_latlon_map["lat"]))
        # We are not using query forking, so strategy is fixed
        logging.info('QUERYID %s;SRC %s;DST %s;STRATEGY non-forking ;Hop Counts %d;ttl %d;Query Received', query, shost, dhost, len(path), ttl)

        # Check if the query has expired TTL
        if len(path) > ttl:
            # We are not using query forking, otherwise we would check strategy here
            # not storing query overhead at this point either
            # send a response to requester and exit
            response = M.ResponseQueryMsg(query, False, my_hostname + ": TTL expired", new_path)
            u.send_msg(shost, 23456, response)
            return

        logging.info('QUERYID %s;SRC %s;DST %s;STRATEGY non-forking ;Processing Query', query, shost, dhost)

        # If current host is within the target region, our search for an alibi relay was
        # successful and we can respond accordingly
        if Point(my_latlon_map["lon"], my_latlon_map["lat"]).intersects(target_region):
            logging.info('QUERYID %s;SRC %s;DST %s;%s within relayzone', query, shost, dhost, my_hostname)
            response = M.ResponseQueryMsg(query, True, my_hostname, new_path)
            # would update query overheads at this point
            u.send_msg(shost, 23456, response)
            return

        # Otherwise we should find a viable next hop
        target_node = f.nodelist_within_region(query, my_hostname, my_latlon_map["lon"], my_latlon_map["lat"], n.nodes, n.nn_list, forbidden_region, target_region)
        if target_node != None:
            # found some neighboring node within the target regions
            response = M.ResponseQueryMsg(query, True, target_node, new_path)
            u.send_msg(shost, 23456, response)
            return

        # get the next hop to visit
        previous_node = self.request.getpeername()[0]

        # Now we must choose a safe next hop.
        viable_nodes = f.get_next_hop(query, my_hostname, slon, slat, dlon, dlat, forbidden_region, target_region, my_latlon_map["lon"], my_latlon_map["lat"], n.nodes, n.nn_list, previous_node, path, 1)

        # are there any safe nodes available?
        if len(viable_nodes) == 0:
            # no safe nodes - return with failure
            response = M.ResponseQueryMsg(query, False, my_hostname + ": No safe next hop", new_path)
            # update query overhead here
            u.send_msg(shost, 23456, response)

        # Not using query forking, so only pick best node for next hop
        next_hop = viable_nodes[0]
        
        if not next_hop:
            # no safe nodes - return with failure
            response = M.ResponseQueryMsg(query, False, my_hostname + ": No safe next hop", new_path)
            # update query overhead here
            u.send_msg(shost, 23456, response)

        # attempt to forward the query
        logging.info('QUERYID %s;SRC %s;DST %s;Trying to forward from %s to %s', query, shost, dhost, myip ,next_hop)
        query = M.QueryMsg(query, slat, slon, dlat, dlon, ttl, forbidden_region, target_region, path, shost, dhost)
        # update query overhead here
        if not u.send_msg(next_hop, 23456, query):
            logging.info('QUERYID %s;SRC %s;DST %s;Query forwarding to %s failed', query, shost, dhost, next_hop)
            with n_lock:
                n.remove_item_from_nlist(next_hop)
                n.update_neighbor_list()

            self.handle_query(q)

    
    def query_chk_req_handler(self, req):
        # Checks if the query with the given ID is active or not
        global active_queries
        query = req.param["query"]
        response = M.QueryCheckReplyMsg(query, (1 if (query in active_queries) else 0))
        # update query overhead here
        r = M.asBytes(response)
        rbytes = r.encode("utf-8")
        self.request.sendall(rbytes)

    
    def query_resp_handler(self, msg):
        # when a response is received for a query, log it and add the response to query's
        # activity queue
        query = msg.param["query"]
        succeeded = msg.param["succeeded"]
        result = msg.param["result"]
        path = msg.param["path"]

        logging.info('QUERYID %d;Query Response Msg;%d;%s', query, succeeded, result)

        with queryq_lock:
            queryq[query].put((succeeded, result, path))


    def neighbor_list_handler(self, msg):
        myip, myport = self.request.getsockname()
        senderip, senderport = self.request.getpeername()

        logging.info('%s requested neighor list', senderip)

        slat = msg.param["slat"]
        slon = msg.param["slon"]
        # repurpose lon and lat to represent diverse demands
        # if lon <= -1100 => request neighbors with latency information
        # if lon > -500 => request neighbors and known-active peers without latency information
        # if -1100 < lon <= -500 and lat == 0 => request neighbors without latency information
        # if -1100 < lon <= -500 and lat != 0 => request known-active peers without latency information

        if slon > -500:
            # add the sender to known-active-peers
            node_list = n.add_rtts([(senderip, slon, slat)])
            with n_lock:
                n.add_to_known_peers(node_list)
                n.update_neighbors()
            print()

                # send back the gossip msg (containing neighbors and known-active peers)
            send_buf = list(set(n.clients+n.nodes+[(myip, my_latlon_map['lon'], my_latlon_map['lat'], 0.0)]))
        elif slon > - 1100:
            with n_lock:
                if slat == 0:
                    # send back the gossip msg (containing neighbors)
                    send_buf = n.nodes
                else:
                    # send back the gossip msg (containing known-active peers)
                    send_buf = n.clients
        else:
            with n_lock:
                # send back the gossip msg (containing neighbors with latency information)
                send_buf = n.nodes
    
        gossip_msg = M.GossipMsg(send_buf)
        self.request.sendall(gossip_msg.asBytes().encode("utf-8"))


    def query_request_handler(self, q):
        # client query requests are routed here
        global queryq, past_queries, active_queries

        # requests cannot be handled if node has not been initialized
        while not initialized:
            logging.info("Node is not initialized yet")
            time.sleep(2)

        myip, myport = self.request.getsockname()
        #q = M.Msg.fromBytes(json_data)
        query = q.param["query"]
        shost = q.param["shost"]
        slat = q.param["slat"]
        slon = q.param["slon"]
        dhost = q.param["dhost"]
        dlat = q.param["dlat"]
        dlon = q.param["dlon"]
        ttl = q.param["ttl"]
        forbidden_region = b_loads(bytes.fromhex(q.param["fr"]))
        target_region = b_loads(bytes.fromhex(q.param["tr"]))

        #print(forbidden_region)
        #print(type(target_region))
        with qid_lock:
            if query in past_queries:
                # this query has been performed before, disregard it
                logging.info('QUERYID %s;SRC %s;DST %s;STRATEGY non-forking; Query Discarded', query, shost, dhost)
                return

        if not c.TESTING:
            with daemon_lock:
                past_queries.add(query)

        # at this point the path is just being created, so only add self
        new_path = [(my_hostname, my_latlon_map["lon"], my_latlon_map["lat"])]

        # We are not using query forking, so strategy is fixed
        logging.info('QUERYID %s;SRC %s;DST %s;STRATEGY non-forking ;At source.', query, shost, dhost)

        # If current host is within the target region, our search for an alibi relay was
        # successful and we can respond accordingly
        if Point(my_latlon_map["lon"], my_latlon_map["lat"]).intersects(target_region):
            logging.info('QUERYID %s;SRC %s;DST %s;%s within relayzone', query, shost, dhost, my_hostname)
            response = M.ResponseQueryMsg(query, True, my_hostname, '[]')
            # would update query overheads at this point
            #u.send_msg(shost, 23456, response)
            self.request.sendall(response.asBytes().encode('utf-8'))
            return

        # Otherwise we should find a viable next hop
        target_node = f.nodelist_within_region(query, my_hostname, my_latlon_map["lon"], my_latlon_map["lat"], n.nodes, n.nn_list, forbidden_region, target_region)
        if target_node != None:
            # found some neighboring node within the target regions
            response = M.ResponseQueryMsg(query, True, target_node, '[]')
            #u.send_msg(shost, 23456, response)
            self.request.sendall(response.asBytes().encode('utf-8'))
            return

        # Now we must choose a safe next hop.
        viable_nodes = f.get_next_hop(query, my_hostname, slon, slat, dlon, dlat, forbidden_region, target_region, my_latlon_map["lon"], my_latlon_map["lat"], n.nodes, n.nn_list)

        # are there any safe nodes available?
        if len(viable_nodes) == 0:
            # no safe nodes - return with failure
            response = M.ResponseQueryMsg(query, False, my_hostname + ": No safe next hop", '[]')
            # update query overhead here
            #u.send_msg(shost, 23456, response)
            self.request.sendall(response.asBytes().encode('utf-8'))
            return

        # Not using query forking, so only pick best node for next hop
        next_hop = viable_nodes[0]
        
        if not next_hop:
            # no safe nodes - return with failure
            response = M.ResponseQueryMsg(query, False, my_hostname + ": No safe next hop", new_path)
            # update query overhead here
            #u.send_msg(shost, 23456, response)
            self.request.sendall(response.asBytes().encode('utf-8'))
            return

        syncQ = queue.Queue()
        # save query in queue of query actions for this query ID (similar to producer-consumer)
        # mark the query as active until a response is received
        #print("Entering Query Queue initialized")
        with queryq_lock:
            #print("Query Queue initialized")
            queryq[query] = syncQ # created a new queue for this query
        
        with active_lock:
            active_queries.add(query)

        # attempt to forward the query
        logging.info('QUERYID %s;SRC %s;DST %s;Trying to forward from source %s to %s', query, shost, dhost, myip ,next_hop)
        to_send = M.QueryMsg(query, slat, slon, dlat, dlon, ttl, forbidden_region, target_region, new_path, shost, dhost)
        u.send_msg(next_hop, 23456, to_send)
        # IF THE QUERY COMPLETES SUCCESSFULLY, WE EXPECT THE RESULT TO ARRIVE IN THE QUEUE
        while True:
            try:
                # perform a blocking read with a timeout.
                response = q.get(True, c.TIMEOUT)
            except:
                # the read failed. Inform client that the query timed out
                logging.info('QUERYID %d;SRC %s;DST %s;Query timed out', query, shost, dhost)
                msg = M.ResponseQueryMsg(query, False, "Query timed out at source", '[]')
                break
            
            if response:
                msg = M.ResponseQueryMsg(query, response.params["succeeded"], response.params["result"], new_path)
                if response.params["succeeded"]:
                    logging.info('SUCCESS:: QUERYID %d;SRC %s;DST %s;Got a query response (Relay - %s)|%s', query, my_hostname, dhost, response.params["result"], new_path)
                    break
                else:
                    logging.info('FAILURE:: QUERYID %d;SRC %s;DST %s', query, my_hostname, dhost)
                    break

        # remove the query from the active list and empty out its sync queue
        with active_lock:
            active_queries.remove(query)
        with queryq_lock:
            q.queue.clear()

        # update query overhead here
        # send a response to the client
        client = self.request.getpeername()[0]
        self.request.sendall(msg.asBytes().encode("utf-8"))

    def gossipHandler(self, msg, peer=None, myip=None):
        if not myip:
            (myip,p) = self.request.getsockname()
        if not peer:
            peer = self.request.getpeername()[0]
        abc = M.GossipMsg(None)
        x = msg
        #print("neighbors" in x.param)
        neighbor_list = []
        if "neighbors" in x.param:
            neighbor_list = x.param["neighbors"]
        
        #print (neighbor_list)
        # don't tell me about myself
        neighbor_list = n.remove_item(myip, neighbor_list)
        neighbor_list = n.add_rtts(neighbor_list)

        (up, down) = neighbor_list

        with n_lock:
            n.add_to_known_peers(neighbor_list)
            b = n.update_neighbors()

        # update neighbors of neighbors
        self.get_neighbors_neighbortables(b)

    # Used for connecting to the network of alibi routing nodes
    def joinPeerHandler(self, msg):
        global initialized, gossip_timer, refresh_neighbors_timer, request_neighborlist_timer

        if not initialized:
            mylat = my_latlon_map["lat"]
            mylon = my_latlon_map["lon"]
            # now request list of neighbors from all bootstrapping nodes
            req = M.NeighborReqMsg(mylat, mylon)

            while True:
            # do repeatedly until initialization is complete
            # not all nodes will have enough neighbors in the first step
            # repeated execution ensures that the current node has enough neghbors
                for bootnode in c.bootstrap_nodes:
                    # this node itself might be a bootstrap node.
                    #In that case, we just move on to the next node
                    if bootnode == my_hostname:
                        continue

                    # else request neighbor list from that node
                    nlist_msg = u.send_msg(bootnode, 23456, req, False, True)

                    if nlist_msg and nlist_msg != None:
                        logging.info("Received response from bootstrapping node %s", bootnode)
                        # response received is actually a gossip message
                        x = M.Msg.fromBytes(nlist_msg)
                        self.gossipHandler(x, bootnode, socket.gethostbyname(socket.gethostname()))

                        # if sufficient neighbors have not been obtained, keep repeating
                        if len(n.nodes) < 2:
                            continue

                        gossip_timer = threading.Timer(c.GOSSIP, self.scheduled_gossip)
                        gossip_timer.start()
                        refresh_neighbors_timer = threading.Timer(c.REFRESH_INTERVAL, self.refresh_neighbors_of_neighbors)
                        refresh_neighbors_timer.start()
                        request_neighborlist_timer = threading.Timer(c.NLIST_REQ_INTERVAL, self.refresh_nn_list)
                        request_neighborlist_timer.start()

                        initialized = True
                        break

                if initialized:
                    break
                #pause before retrying
                time.sleep(10)

    # asking neighbors for neighbor tables
    def get_neighbors_neighbortables(self, nlist):
        msg = M.NeighborReqMsg(-2000, 0)
        for nbrs in nlist:
            #print(nbrs)
            response = u.send_msg(nbrs[0], 23456, msg, False, True)
            if response:
                # DEPENDS ON UTILS
                m = M.Msg.fromBytes(response)
                nnlist_a = m.param["neighbors"]
                #print(nnlist_a)
                with n_lock:
                    n.nn_list[nbrs[0]] = nnlist_a

    #
    def refresh_neighbors_of_neighbors(self):
        # This task is only called when a timer expires
        global request_neighborlist_timer

        with n_lock:
            candidate_list = list(n.clients)
            candidate_list.append(list(n.nodes))

        # refresh RTTs 
        refreshed_rtt_list = n.add_rtts(candidate_list)
        with n_lock:
            n.process_rtt(refreshed_rtt_list)
            new_neighbors = n.update_neighbors()
            # authors refresh the neighbor list here but we 
            # do not find it necessary for our test topologies

        self.get_neighbors_neighbortables(new_neighbors)
        # since this is a timer task, it must be re-scheduled here
        refresh_neighbors_timer = threading.Timer(c.REFRESH_INTERVAL, self.refresh_neighbors_of_neighbors)
        refresh_neighbors_timer.start()

    def scheduled_gossip(self):
        global gossip_timer

        to_send = None
        with n_lock:
            nbor = sample(n.nodes, 1)[0]
            to_send = list(n.clients + n.nodes)

        if to_send:
            msg = M.GossipMsg(to_send)
            u.send_msg(nbor[0], 23456, msg)
        gossip_timer = threading.Timer(c.GOSSIP, self.scheduled_gossip)
        gossip_timer.start()


    def refresh_nn_list(self):
        global request_neighborlist_timer

        self.get_neighbors_neighbortables(n.nodes)

        request_neighborlist_timer = threading.Timer(c.NLIST_REQ_INTERVAL, self.refresh_nn_list)
        request_neighborlist_timer.start()


# shutting down gracefully
def sigint_handler(signal, fram):
    gossip_timer.cancel()
    refresh_neighbors_timer.cancel()
    request_neighborlist_timer.cancel()
    server.shutdown()
    sys.exit(0)

signal.signal(signal.SIGINT, sigint_handler)


class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    def server_bind(self):
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(self.server_address)



# set host name and gps coordinates
my_hostname=socket.gethostbyaddr(socket.gethostname())[0]
file_gps_coords=u.get_gps_coords(my_hostname,c.GPS_FILE)

if file_gps_coords == None:
    logging.error('Cannot obtain gps coords - %s', my_hostname)
    sys.exit(1)

my_latlon_map["lon"] = file_gps_coords[0]
my_latlon_map["lat"] = file_gps_coords[1]

n.set_gps(file_gps_coords)

# set logging parameters
#logging_map={'hostname':host_name}

logging.Formatter.converter = time.gmtime
#logging.basicConfig(filename="alibi_daemon.log", format='%(asctime)s|%(levelname)s|%(message)s', level=c.LOGGING_LEVEL)
logging.basicConfig(format='%(levelname)s|%(message)s', level=logging.DEBUG)
HOST, PORT = 'localhost', c.PORT
server = ThreadedTCPServer(('', PORT), ThreadedTCPRequestHandler)

# run server
server_thread = threading.Thread(target=server.serve_forever)
# Exit the server thread when the main thread terminates
server_thread.daemon = True
server_thread.start()

join_msg = M.JoinMsg()

u.send_msg(HOST, PORT, join_msg, False)

server_thread.join()

gossip_timer.cancel()
refresh_neighbors_timer.cancel()
request_neighborlist_timer.cancel()
server.shutdown()
sys.exit(0)
