#!/usr/bin/env python3

"""
File:   log_write_python.py
Author: psherman-uva
Date:   Jan. 2023

Description: ROS node example to write data to SQLite database
"""

from amrl_logging.srv import LoggingStart, LoggingStop, LoggingBufferCheck
from amrl_logging.msg import LoggingData

import rospy
import math

def setup(table_name, topic_name):
    try:
        service_name = "/sqlite/start_log"
        col_names = ["test_name", "timestamp", "angle", "sine", "cosine"]
        
        rospy.wait_for_service(service_name, 5.0)
        rospy.loginfo("Start logging service is active")
        
        start_log = rospy.ServiceProxy(service_name, LoggingStart)
        resp = start_log(table_name, topic_name, col_names)

        if resp.success:
            rospy.loginfo("Logging from python node enabled.")
            return True
        else:
            rospy.logwarn("Failed start logging from python node")

    except rospy.ROSException:
        rospy.logwarn("Service not active")

    return False

def cleanup(table_name):
    service_name = "/sqlite/log_buffer_check"
    log_check = rospy.ServiceProxy(service_name, LoggingBufferCheck)

    done = False
    while not done:
        resp = log_check(table_name)
        if resp.is_empty and resp.size == 0:
            done = True

    service_name = "/sqlite/stop_log"
    log_stop = rospy.ServiceProxy(service_name, LoggingStop)
    log_stop(table_name)

###                      ###
#   Start of Main Script   #
###                      ###

if __name__ == "__main__":
  db_filename = "/home/patrick/uva/database/ProjectDatabase.db"
  table_name  = "ExampleTable"
  topic_name  = "python_example_data"

  try:
    rospy.init_node("python_log_writer", anonymous=False)

    if setup(table_name, topic_name):
      pub = rospy.Publisher(topic_name, LoggingData, queue_size=10)
      rate = rospy.Rate(10)

      count = 0
      msg = LoggingData()
      msg.test_name = "python_example"
      msg.data = [None] * 4

      while not rospy.is_shutdown() and count < 720:
        msg.header.seq   += 1
        msg.header.stamp  = rospy.Time.now()

        msg.data[0] = rospy.Time.now().to_sec()
        msg.data[1] = count * math.pi / 180.0
        msg.data[2] = math.sin(msg.data[1])
        msg.data[3] = math.cos(msg.data[1])

        pub.publish(msg)

        count += 1
        rate.sleep()
      rospy.loginfo("Finished data publishing")

      cleanup(table_name)

      
  except rospy.ROSInterruptException:
    rospy.loginfo("ROS Exception: Ending program.")
