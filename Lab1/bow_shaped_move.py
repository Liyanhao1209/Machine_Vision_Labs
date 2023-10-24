#!/usr/bin/python3
# coding=utf8
import sys
import rospy
from chassis_control.msg import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

print('Bow Shaped Move Lab')

start = True
offset_ticks = 0
cur_i = 0


# before shut down
def stop():
    # similar to external in C
    global start

    start = False
    print('Shutting down...')
    set_velocity.publish(0, 0, 0)  # stop


def activity(linV, directA, yawR):
    # publish a chassis control msg, with linear velocity 60，direction angle 90，yaw rate 0(<0，clockwise)
    set_velocity.publish(linV, directA, yawR)


if __name__ == '__main__':
    # init node
    rospy.init_node('bow_shaped_move', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)  # stop callback function
    # Mc Wheel chassis control
    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)

    # the corresponding offset ticks for the chassis control
    # forward->turn right->forward->turn right->forward->turn left->forward->turn left->forward->turn right->forward
    ticks = [4.0, 1.0, 2.0, 1.0, 4.0, 1.0, 2.0, 1.0, 4.0, 1.0, 2.0]
    args = [
        [60, 90, 0],
        [0, 90, -0.3],
        [60, 90, 0],
        [0, 90, -0.3],
        [60, 90, 0],
        [0, 90, 0.3],
        [60, 90, 0],
        [0, 90, 0.3],
        [60, 90, 0],
        [0, 90, -0.3],
        [60, 90, 0]
    ]

    # without interrupt from the keyboard,do the routes
    route_steps_len = len(ticks)
    while start & cur_i < route_steps_len:
        # get tick
        fin_tick = ticks[cur_i]
        # get args
        arg = args[cur_i]
        # act
        activity(args[0], args[1], args[2])
        # inc offset_ticks
        offset_ticks += 1
        # next route?
        if offset_ticks >= fin_tick:
            cur_i += 1
            offset_ticks = 0
        # actual tick inc
        rospy.sleep(1)

    # spin rotation until interrupt from the keyboard
    while start:
        activity(0, 90, -0.3)
        rospy.sleep(1)

    set_velocity.publish(0, 0, 0)  # stop
    print('Shut down')
