import rospy
import tf
from move import MovePub
import sys, select, termios, tty
from aizuspider_description.srv import (
    Grasp,
    SolveIK,
    )

from geometry_msgs.msg import (
    Pose,
    PointStamped,
    )


class KeyTest:
    def __init__(self):
        self.settings_ = termios.tcgetattr(sys.stdin)
        rospy.init_node('move_sample')
        mv = MovePub(name)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        keys = []
        key = None
        while(True):
            ret = select.select([sys.stdin], [], [], 0)
            ##print(ret)
            # if len(ret[0]) < 1:
            #     if len(keys) >= 1:
            #         break
            #     msg = Joy()
            #     msg.axes = [0]*8
            #     msg.buttons = [0]*11
            #     self.pub_.publish(msg)
            #     continue
            key = sys.stdin.read(1)
            keys.append(key)
            if len(key) >= 1:
                break

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings_)
        #print(keys)
        #''.join(keys)
        return key

    def main(self):
        prev1 = None
        prev2 = None

        try:
            while(1):
                key = self.getKey()
                if key == '\x03':
                    break
                escaped = False
                if prev2 == '\x1b' and prev1 == '[':
                    escaped = True
                    #print('escaped')

                print('%d %s %s'%(len(key), key.encode('hex'), key))

                prev2 = prev1
                prev1 = key

                if escaped:
                    if key == 'A':
                        print('Go Forward') ## forward 0.5 m/sec
                        rospy.Subscriber('%sground_truth_pose'%(name), PoseStamped, mv.callback)
                        mv.wait_first_callback()
                        mv.move(0.5, 0)
                    elif key == 'B':
                        print('Go Backward')## backward
                        rospy.Subscriber('%sground_truth_pose'%(name), PoseStamped, mv.callback)
                        mv.wait_first_callback()
                        mv.move(-0.5, 0)
                    elif key == 'C':
                        print('Turn CW') ## cw (-yaw) 0.6 rad/sec
                        rospy.Subscriber('%sground_truth_pose'%(name), PoseStamped, mv.callback)
                        mv.wait_first_callback()
                        mv.move(0, 0.1)
                    elif key == 'D':
                        print('Turn CCW') ## ccw(+yaw)
                        rospy.Subscriber('%sground_truth_pose'%(name), PoseStamped, mv.callback)
                        mv.wait_first_callback()
                        mv.move(0, -0.1)

                if key == 'q':
                    print('Stop')
                    rospy.Subscriber('%sground_truth_pose'%(name), PoseStamped, mv.callback)
                    mv.wait_first_callback()
                    mv.move(0, 0)

        except Exception as e:
            print(e)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings_)

name = 'AizuSpiderAA'
jp = KeyTest()
jp.main()
