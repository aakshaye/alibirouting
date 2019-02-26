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

METERS_PER_MILE_CONVERSION = 0.000621371
SLOPE = 0.0161
INTERCEPT = 0.0
PORT = 23456
# gossip msg interval (secs)
GOSSIP = 80

# neighbor sets refreshment interval (secs)
REFRESH_INTERVAL = 120

# neighbor table request interval (secs)
NLIST_REQ_INTERVAL = 60

# timeout for query
QUERY_TIMEOUT = 5

LOGGING_LEVEL = 10

TTL_LIST = [2, 4, 7]

TESTING = False


bootstrap_nodes = ['pcvm3-29.instageni.illinois.edu','pcvm2-13.instageni.rutgers.edu','pcvm2-3.instageni.idre.ucla.edu','pcvm2-6.instageni.research.umich.edu','pcvm2-11.instageni.washington.edu']

GPS_FILE='./data/coordinates_ip_host.dat'