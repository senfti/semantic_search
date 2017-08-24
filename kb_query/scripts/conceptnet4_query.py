#!/usr/bin/env python

import divisi2
import kb_query.srv
import rospy

spread = None

def do_query(req):
    global spread
    similarity = []
    for i in range(len(req.terms1)):
        tmp = spread.entry_named(req.terms1[i], req.terms2[i])
        similarity.append(tmp)
    return kb_query.srv.KbQuerySrvResponse(similarity)

if __name__ == "__main__":
    assoc = divisi2.network.conceptnet_assoc('en')
    U, S, _ = assoc.svd(k=100)
    spread = divisi2.reconstruct_activation(U, S)

    rospy.init_node('kb_query_server')
    s = rospy.Service('kb_query_srv', kb_query.srv.KbQuerySrv, do_query)
    print "Ready to reason"
    rospy.spin()
