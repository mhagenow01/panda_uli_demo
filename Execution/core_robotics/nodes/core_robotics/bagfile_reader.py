""" Tools for writing to a bagfile and reading a bagfile

 Last Updated: 02/21/2019
"""

__author__ = "Guru Subramani and Mike Hagenow"

import rosbag
import rospy
import numpy as np
import shutil


def write_to_bagfile(filepath,topicname,msgs,headerstamps,write_permissions,createbackup = True):
    """
        Writes specific message data to a bagfile without ROS invocation

        :param filepath: bagfile location to store
        :param topicname: topic that messaged are written to
        :param msgs: message data to write
        :param headerstamps:
        :param write_permissions:
        :param createbackup:
        :type filepath:
        :type topicname:
        :type msgs:
        :type headerstamps:
        :type write_permissions:
        :type createbackup:
        :returns: none
        """

    # write_permissions -> 'a' -> append to bagfile ; 'w' -> write new bagfile

    if createbackup == True:
        shutil.copy2(filepath,"./" + filepath + "_backup.bag")

    elif type(createbackup) == type("string_type"):
        shutil.copy2(filepath, createbackup)

    with rosbag.Bag(filepath, write_permissions) as bag:
        if type(headerstamps[0]) == type(rospy.Time.from_sec(0)):
            for msg,stamp in zip(msgs,headerstamps):
                bag.write(topicname, msg, stamp)

        else:
            for msg,stamp in zip(msgs,headerstamps):
                bag.write(topicname, msg, rospy.Time.from_sec(stamp))



class bagfile_reader():
    def __init__(self,bagfilepath):
        """
            Instantiates class and loads serialized data into dictionary

            :param bagfilepath: bagfile with data
            :type bagfilepath: string
            :returns: none
        """

        self.filepath = bagfilepath
        try:
            self.bag = rosbag.Bag(bagfilepath)
        except :
            self.bag = None
            print("bag file not found!")
            return
        self.topics = set()

        self.data_dict = {}
        self.time_stamps_dict = {}

        #Go through messages and add to dictionaries
        for topic, msg, t in self.bag.read_messages():
            if topic not in self.topics:
                self.topics.add(topic)
                self.data_dict[topic] = []
                self.time_stamps_dict[topic] = []
            self.data_dict[topic].append(msg)
            self.time_stamps_dict[topic].append(t)
	


    def get_topic_msgs(self,topic_name):
        """
        for a given topic, pull msgs and topics from dictionary

        :param topic_name: string of ROS topic name
        :type topic_name: string
        :returns: msgs, time_stamps
        :rtype: array, array
                """
        if topic_name in self.topics:
            msgs = np.array([element for element in self.data_dict[topic_name]])
            time_stamps = np.array(
                [(element).to_sec() for element in self.time_stamps_dict[topic_name]])  # - time_stamps_dict["/IMUData"][0]

        else:
            msgs = []
            time_stamps = []

        return msgs, time_stamps


    def get_topic_sample_rate(self,topic_name):
        time_stamps = np.array(
            [(element).to_sec() for element in self.time_stamps_dict[topic_name]])  # - time_stamps_dict["/IMUData"][0]

        return np.mean(np.diff(time_stamps))

    def get_possible_topic_values(self,topic_name):
        if topic_name in self.topics:
            return list(set(np.array([element for element in self.data_dict[topic_name]])))
        else:
            return []

    def write_to_bagfile(self,topicname,msgs,headerstamps,createbackup = True):

        if createbackup == True:
            shutil.copy2(self.filepath,"./" + self.filepath + "_backup.bag")

        elif type(createbackup) == type("string_type"):
            shutil.copy2(self.filepath, createbackup)

        with rosbag.Bag(self.filepath, 'a') as bag:
            if type(headerstamps[0]) == type(rospy.Time.from_sec(0)):
                for msg,stamp in zip(msgs,headerstamps):
                    bag.write(topicname, msg, stamp)

            else:
                for msg,stamp in zip(msgs,headerstamps):
                    bag.write(topicname, msg, rospy.Time.from_sec(stamp))


if __name__ == "__main__":
# Example usage
    bf= bagfile_reader("Take1.bag")
    f1,ft1 = bf.get_topic_msgs("/labels")


