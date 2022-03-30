#! /usr/bin/env python3
import rospy
import codecs
import socket
import struct
from geometry_msgs.msg import WrenchStamped


sock = None

def init_udp(UDP_IP = "192.168.1.1"):
    UDP_PORT = 49152
    # MESSAGE = "1234000200000000".decode("hex")
    MESSAGE = codecs.decode('1234000200000000', 'hex')
    # MESSAGE = "1234000200000000".encode().decode("hex")
    global sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

def get_ftdata():
    data, server = sock.recvfrom(1024)
    data = bytearray(data)
    s = struct.Struct('> I I I i i i i i i')
    unpacked_data = s.unpack(data)
    ft = unpacked_data[3:]
    ft = [ft_/1000000.0 for ft_ in ft]
    return ft

def ft_pub():
    pub = rospy.Publisher("ftmini40",WrenchStamped,queue_size=100)
    rospy.init_node('ftmini40',anonymous=True)
    r = rospy.Rate(1000)
    w = WrenchStamped()
    init_udp()
    while not rospy.is_shutdown():
        ft = get_ftdata()
        w.header.stamp = rospy.Time.now()
        w.header.frame_id = "ftmini"
        w.wrench.force.x = ft[0]
        w.wrench.force.y = ft[1]
        w.wrench.force.z = ft[2]
        w.wrench.torque.x = ft[3]
        w.wrench.torque.y = ft[4]
        w.wrench.torque.z = ft[5]
        pub.publish(w)
        r.sleep()


if __name__ == "__main__":
    try:
        ft_pub()
    except rospy.ROSInterruptException:
        pass
