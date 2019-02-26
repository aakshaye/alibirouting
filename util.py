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
import logging
import sys
import time

host_name = socket.gethostbyaddr(socket.gethostname())[0]


# Read GPS coordinate from a file. Format: hostname ip longitude latitude
def get_gps_coords(host_id, filename):
	with open(filename, 'r') as f:
		for line in f:
			if host_id in line:
				sp = line.split()
				return float(sp[2]), float(sp[3])

	return None


# send a TCP message (and return a reponse).
def send_msg(ip, port, message, w_marshalling=False, reply=False, latency=False, timeout=10):
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sock.settimeout(timeout)

	success = True

	rtt = None

	try:
		sock.connect((ip, port))

		if latency:
			time_sent = time.time()
		if w_marshalling:
			sock.sendall(message.marshal())
		else:
			sock.sendall(message.asBytes().encode('utf-8'))

		if reply is True:
			length = None
			data = ''
			to_read = 0
			read_more = False
			newdata = sock.recv(4096)
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
					newdata = sock.recv(4096)
					continue
				else:
					data += newdata[:to_read].decode("utf-8")
					to_read
					break
					
			if latency:
				rtt = (time.time() - time_sent) * 1000

	except socket.error:
		_, e, _ = sys.exc_info()
		logging.error('Exception;%s;%s;%s', e, ip, port)
		data = None
		success = False
	finally:
		sock.close()

	if reply is True:
		if latency:
			return (data, rtt)
		else:
			return data
	else:
		return success
