#-*- coding: UTF-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import serial
import sys

import rospy

from lib_nmea_serial.driver import RosNMEADriver

def main():
    rospy.init_node('nmea_serial_save')

    serial_port = rospy.get_param('~port', '/dev/ttyUSB1')
    serial_baud = rospy.get_param('~baud', 115200)
    frame_id = RosNMEADriver.get_frame_id()
    file_path = rospy.get_param('~save_path')
    file_path2 = rospy.get_param('~save_path2')
    #file_path = '/home/bit_uav/bit_uav/gps_ws/src/nmea_serial_save/data/kml_data.csv' 
    kml_mode = rospy.get_param('~kml_mode')
    #kml_mode = True

    try:
        GPS = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=2)  # 用的串口库, 初始化serial对象GPS

        try:
            # 初始化一个RosBMEADriver()的对象driver
            driver = RosNMEADriver(file_path, file_path2, kml_mode)
            while not rospy.is_shutdown():
                data = GPS.readline().strip()
                try:
                    driver.add_sentence(data, frame_id)
                except ValueError as e:
                    rospy.logwarn(
                        "Value error, likely due to missing fields in the NMEA message. "
                        "Error was: %s. Please report this issue to WangXingbo. " %
                        e)
        # ROS关闭或者串口异常
        except (rospy.ROSInterruptException, serial.serialutil.SerialException):
            GPS.close()  # Close GPS serial port
    except serial.SerialException as ex:
        rospy.logfatal(
            "Could not open serial port: I/O error({0}): {1}".format(ex.errno, ex.strerror))
        file.close()
