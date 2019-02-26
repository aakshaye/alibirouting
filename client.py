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

from shapely.wkb import loads
import sys
import os
import time
import constants as c
import util as utils
import AlibiMsg as msg

sys.path.append(os.path.abspath('target_region_computation'))
import regions as r

time_to_live = c.TTL_LIST
query_id = 0


def create_and_send_query_msg(s_node, d_node, sn_long, sn_lat, dn_long, dn_lat, target_r, forbidden_r, q_id):
    if target_r is None:
        error = 'TargetRegionNotFound'
        print(error)
        return

    query_request = msg.RequestQueryMsg(q_id, sn_lat, sn_long, dn_lat, dn_long, time_to_live, forbidden_r, target_r,
                                        s_node, d_node)

    response = utils.send_msg(s_node, c.PORT, query_request, w_marshalling=False, reply=True, timeout=7)

    if response is None:
        print('Cannot send to nor receive from %s,%d' % (s_node, q_id))
    else:
        query_response = msg.Msg.fromBytes(response)
        print(query_response)
        if not query_response.param["succeeded"]:
            print('Failure,%s,%s' % (query_response.param["query"], query_response.param["result"]))
        else:
            print('Success,%s,%s' % (query_response.param["query"], query_response.param["result"]))

for line in sys.stdin:
    (s_host, sh_long, sh_lat, d_host, dh_long, dh_lat, forbidden_region, delta) = line.split(',')

    diff_factor = float(delta) + 1.0
    try:
        with open('./data/forbidden_region_%s.bin.dat' % forbidden_region, 'rb') as f:
            forbidden_reg = loads(f.read())
    except FileNotFoundError:
        forbidden_reg = None

    print('Generating target region ...')
    target_region = r.get_target_region(forbidden_reg, (sh_long, sh_lat), (dh_long, dh_lat), float(diff_factor))

    if target_region is None:
        continue

    print('# Query ID - %d,%s' % (query_id, line.strip()))

    create_and_send_query_msg(s_host, d_host, float(sh_long), float(sh_lat), float(dh_long), float(dh_lat),
                              target_region, forbidden_reg, query_id)

    query_id += 1

    time.sleep(1)
