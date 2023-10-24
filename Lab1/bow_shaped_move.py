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


def forward():
    # publish a chassis control msg, with linear velocity 60，direction angle 90，yaw rate 0(<0，clockwise)
    set_velocity.publish(60, 90, 0)


def backward():
    set_velocity.publish(60, -90, 0)

    # TODO: Finish the rightward,the leftward and the turn-around function


def rightward():
    set_velocity.publish()


def leftward():
    set_velocity.publish()


def turnAround():
    set_velocity.publish()


if __name__ == '__main__':
    # init node
    rospy.init_node('bow_shaped_move', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)  # stop callback function
    # Mc Wheel chassis control
    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)

    # the corresponding offset ticks for the chassis control
    # right->forward->left->forward->right->forward->turn around
    ticks = [4.0, 2.0, 4.0, 2.0, 4.0, 2.0]
    route = [rightward, forward, leftward, forward, rightward, forward]

    # without interrupt from the keyboard,do the routes
    route_steps_len = len(ticks)
    while start & cur_i < route_steps_len:
        # get tick
        fin_tick = ticks[cur_i]
        # get route
        action = route[cur_i]
        # act
        action()
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
        turnAround()
        rospy.sleep(1)

    set_velocity.publish(0, 0, 0)  # stop
    print('Shut down')
