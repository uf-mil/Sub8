#!/usr/bin/env python
import numpy as np

import rospy
import sub8_tools
from twisted.internet import defer

from sub8_msgs.srv import Bounds, BoundsResponse


class BoundsServer(object):
    def __init__(self):
        self.bounds = np.array([[5,5], [-5, 5], [-5, -5], [5, -5]])
        self.enforce = True
        rospy.Service('/get_bounds', Bounds, self.got_request)


    def got_request(self, req):
        to_frame = "enu" if req.to_frame == '' else req.to_frame
        
        resp = BoundsResponse(enforce=False)

        bounds = [sub8_tools.numpy_to_point(i) for i in self.bounds]
        resp.enforce = self.enforce
        resp.bounds = bounds

        return resp

if __name__ == "__main__":
    rospy.init_node("bounds_server")
    bs = BoundsServer()
    rospy.spin()