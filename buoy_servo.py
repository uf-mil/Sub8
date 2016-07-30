from twisted.internet import defer
from txros import util, tf
import numpy as np

'''TODO: Add movements to actually hit the buoy'''
@util.cancellableInlineCallbacks
def run(sub):
    # bin_search is a Deferred
    print "We're looking for a buoy"

    print "Executing search pattern"
    #yield sub.move.right(2.0).go()
    #yield sub.move.down(0.3).go()

    print "Now trying to pose it"
    response = yield sub.#get pixel from matt's segmentation
    if not response.found:
        print 'failed to discover bbuoy location'
        return
    else:
        print "Got buoy pose"
        print response.pose
    while True:

        response = yield sub.#get pixel from matt's segmentation
        print "Buoy at", response.pose
        if abs(response.pose.x)>10 or abs(response.pose.y)>10:
            if response.pose.x<0:
                yield sub.move.left(abs(response.pose.x/500)).go()
            if response.pose.x>0:
                yield sub.move.right(abs(response.pose.x/500)).go()
            if response.pose.y>0:
                yield sub.move.backward(abs(response.pose.y/500)).go()
            if response.pose.y<0:
                yield sub.move.backward(abs(response.pose.y/500)).go()
        else:
            break
    yield sub.to_height(1)

    while True:
        
        response = yield sub.bin.#get pixel from matt's segmentation
        print "Buoy at", response.pose
        if abs(response.pose.x)>10 or abs(response.pose.y)>10:
            if response.pose.x<0:
                yield sub.move.left(abs(response.pose.x/500)).go()
            if response.pose.x>0:
                yield sub.move.right(abs(response.pose.x/500)).go()
            if response.pose.y>0:
                yield sub.move.backward(abs(response.pose.y/500)).go()
            if response.pose.y<0:
                yield sub.move.backward(abs(response.pose.y/500)).go()
        else:
            break
    
    yield sub.to_height(0.5)

    

    