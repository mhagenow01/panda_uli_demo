from __future__ import print_function
import rospy
from sensor_msgs.msg import JointState
import time

frequency = 8
dt = 1. / frequency
lastTime = time.time()

def clear():
    print('\n' * 80)

def format_cell(v):
    return '{: >10.3}'.format(v)

def pretty_print_table(row_names, row_values, col_width = 10):
    clear()
    nCols = 1 + max(len(values) for values in row_values)
    table_width = 1 + (col_width + 1) * nCols
    row_sep = '-' * table_width
    print(row_sep)
    for name, values in zip(row_names, row_values):
        print('|{: >10}|'.format(name) + '|'.join(map(format_cell, values)) + '|')
        print(row_sep)

def print_joints(msg):
    global lastTime
    now = time.time()
    if now - lastTime < dt:
        return
    lastTime = now
    names = [
        'position',
        'velocity',
        'effort'
    ]
    values = [
        msg.position,
        msg.velocity,
        msg.effort
    ]

    pretty_print_table(names, values)


rospy.init_node('joint_watcher')
sub = rospy.Subscriber('/franka_ros_interface/custom_franka_state_controller/joint_states', JointState, print_joints)

rospy.spin()