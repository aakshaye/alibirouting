from shapely.wkb import dumps as b_dumps, loads as b_loads # 'well-known binary' data format for GIS, used by geometry library
from shapely.wkt import dumps as t_dumps, loads as t_loads # 'well-known text' data format for GIS, used by geometry library
import socket
import json
import base64

# Message codes

PING_MSG = "Ping"
PONG_MSG = "Pong"
QUERY = "Query"
QUERY_REQ = "QReqM"
QUERY_RESP = "QRespM"
QUERY_CHK_RQ = "QCReqM"
QUERY_CHK_RP = "QCReplM"
QUERY_OVH_RQ = "QCOverM"
QUERY_OVH_RP = "QCOverM"
GOSSIP = "Gossip"
NLIST_REQ = "ReqNbors"
JOIN = "Join"

class Msg:
	def __init__(self, param):
		self.param = param

	def __repr__(self):
		return "Msg: " + str(self.param)

	def asBytes(self):
		s = json.dumps(self.param)
		return "+++" + str(len(s)) + "+" + s

	def fromBytes(jsoned):
		msg = Msg(json.loads(jsoned))
		return msg


class Ping (Msg):
	def __init__(self, nonce):
		params = {}
		params["mtype"] = PING_MSG
		params["nonce"] = nonce
		Msg.__init__(self, params)

class PingResponse (Msg):
	def __init__(self, nonce):
		params = {}
		params["mtype"] = PONG_MSG
		params["nonce"] = nonce
		Msg.__init__(self, params)

class GossipMsg (Msg):
	def __init__(self, nlist):
		params = {}
		params["mtype"] = GOSSIP
		params["neighbors"] = nlist
		Msg.__init__(self, params)

class RequestQueryMsg (Msg):
	def __init__(self, query, slat, slon, dlat, dlon, ttl, forbidden_region, target_region, shost, dhost):
		params = {}
		params["mtype"] = QUERY_REQ
		params["query"] = query
		params["shost"] = shost
		params["slat"] = slat
		params["slon"] = slon
		params["dhost"] = dhost
		params["dlat"] = dlat
		params["dlon"] = dlon
		params["ttl"] = ttl
		if forbidden_region:
			#print(b_dumps(forbidden_region))
			params["fr"] = b_dumps(forbidden_region).hex()
			#params["fr"] = base64.b64encode(b_dumps(forbidden_region))
		else:
			params["fr"] = None
		if target_region:
			params["tr"] = b_dumps(target_region).hex()
		else:
			params["tr"] = None
		Msg.__init__(self, params)

	def fromBytes(self, buf):
		params = Msg.fromBytes(buf)
		#params["fr"] = b_loads(base64.b64decode(params["fr"]))
		params["fr"] = b_loads(bytes.fromhex(params["fr"]))
		params["tr"] = b_loads(bytes.fromhex(params["tr"]))
		Msg.__init__(self, params)

class ResponseQueryMsg (Msg):
	def __init__(self, query, succeeded, res_msg, path):
		params = {}
		params["mtype"] = QUERY_RESP
		params["query"] = query
		params["succeeded"] = succeeded
		params["result"] = res_msg
		params["path"] = path
		Msg.__init__(self, params)

class QueryCheckRequestMsg (Msg):
	def __init__(self, query):
		params = {}
		params["mtype"] = QUERY_CHK_RQ
		params["query"] = query

		Msg.__init__(self, params)

class QueryCheckReplyMsg (Msg):
	def __init__(self, query, is_dup):
		params = {}
		params["mtype"] = QUERY_CHK_RP
		params["query"] = query
		params["isdup"] = is_dup

		Msg.__init__(self, params)

class QueryMsg (Msg):
	def __init__(self, query, slat, slon, dlat, dlon, ttl, forbidden_region, target_region, path, shost, dhost):
		params = {}
		params["mtype"] = QUERY
		params["query"] = query
		params["shost"] = shost
		params["slat"] = slat
		params["slon"] = slon
		params["dhost"] = dhost
		params["dlat"] = dlat
		params["dlon"] = dlon
		params["ttl"] = ttl
		params["path"] = path
		if forbidden_region:
			params["fr"] = b_dumps(forbidden_region).hex()
			#params["fr"] = base64.b64encode(b_dumps(forbidden_region))
		else:
			params["fr"] = None
		if target_region:
			params["tr"] = b_dumps(target_region).hex()
		else:
			params["tr"] = None
		Msg.__init__(self, params)

	def fromBytes(self, buf):
		params = Msg.fromBytes(buf)
		params["fr"] = b_loads(bytes.fromhex(params["fr"]))
		params["tr"] = b_loads(bytes.fromhex(params["tr"]))
		Msg.__init__(self, params)

class NeighborReqMsg (Msg):
	def __init__(self, lat, lon):
		params = {}
		params["mtype"] = NLIST_REQ
		params["slat"] = lat
		params["slon"] = lon
		Msg.__init__(self, params)

class JoinMsg (Msg):
	def __init__(self):
		params = {}
		params["mtype"] = JOIN
		Msg.__init__(self, params)
