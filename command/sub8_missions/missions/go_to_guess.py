from txros import util
import os
import sys
SPEED_LIMIT = .2  # m/s
fp = open("/home/lucas/clicked.txt", 'r')
line = fp.readline()
seq = False
point_num = sys.argv[1]
position = [0, 0, 0]
while line:
    line = fp.readline().strip()
    #print line[:6]
    if line[:6] == ("seq: " + str(point_num)):
        print "SEQ ", point_num
        seq = True
    if seq and line == "position:":
        print "POSITION"
        position[0] = float(fp.readline().rstrip()[7:])
        position[1] = float(fp.readline().rstrip()[7:])
        position[2] = -1
        break
fp.close()
print position
if position[0]!=0:
    print "RUN"
    @util.cancellableInlineCallbacks
    def run(sub):
        yield self.sub.move.look_at_without_pitching(
            np.array(position[0:3])).go(SPEED_LIMIT)
        yield self.sub.move.relative(np.array(position[0:3])).go(SPEED_LIMIT)
