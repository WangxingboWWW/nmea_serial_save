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

import math
import rospy

#from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
#from geometry_msgs.msg import TwistStamped, QuaternionStamped
#from tf.transformations import quaternion_from_euler

from lib_nmea_serial.checksum_utils import check_nmea_checksum
import lib_nmea_serial.parser


class RosNMEADriver(object):

    def __init__(self,file_path,file_path2,kml_mode):
        try:
            self.file = open(file_path, 'w') # file is for kml
        except IOError:
            rospy.logerr("can not open the save file: %s." % file_path)
	try:
	    self.file2 = open(file_path2, 'w') # file2 is for enu
	except IOError:
            rospy.logerr("can not open the save file2: %s." % file_path2)    

        #self.time_ref_source = rospy.get_param('~time_ref_source', None)
        #self.valid_fix = False

        self.kml_mode = kml_mode

        # 以下9个参数暂时没有使用
        # epe = estimated position error
        self.default_epe_quality0 = rospy.get_param('~epe_quality0', 1000000)
        self.default_epe_quality1 = rospy.get_param('~epe_quality1', 4.0)
        self.default_epe_quality2 = rospy.get_param('~epe_quality2', 0.1)
        self.default_epe_quality4 = rospy.get_param('~epe_quality4', 0.02)
        self.default_epe_quality5 = rospy.get_param('~epe_quality5', 4.0)
        self.default_epe_quality9 = rospy.get_param('~epe_quality9', 3.0)
        self.using_receiver_epe = False  # 在弄清楚测量方差后, 要用!

        self.lon_std_dev = float("nan")
        self.lat_std_dev = float("nan")
        self.alt_std_dev = float("nan")


    # Returns True if we successfully did something with the passed in
    # nmea_string
    def add_sentence(self, nmea_string, frame_id, timestamp=None):
        if not check_nmea_checksum(nmea_string):
            rospy.logwarn("Received a sentence with an invalid checksum. " +
                          "Sentence was: %s" % repr(nmea_string))
            return False

        parsed_sentence = lib_nmea_serial.parser.parse_nmea_sentence(
            nmea_string)
        if not parsed_sentence:
            rospy.logwarn(
                "Failed to parse NMEA sentence. Sentence was: %s" %
                nmea_string)
            return False

        # 如果是想转成kml格式的话, 仅仅保存位置然后退出
        if self.kml_mode is True:
            if 'GPGGA' in parsed_sentence:
                data = parsed_sentence['GPGGA']

                self.file.write('UAV,')
                
                fix_type = data['fix_type']
		if fix_type == 4:
		    self.file.write('green,')
		elif fix_type == 5:
    		    self.file.write('blue,')
		elif fix_type == 2:
		    self.file.write('yellow,')
		elif fix_type == 1:
		    self.file.write('red,')
		else:
		    self.file.write('black,')
		    
                altitude = data['altitude']
		#straltitude = str(altitude)
		#wtf = "\" straltitude \" " 
		#self.file.write(wtf + ',')

		self.file.write('"' + str(altitude) + '"' + ',')

                self.file.write(str(altitude) + ',')

                latitude = data['latitude']
                if data['latitude_direction'] == 'S':
                    latitude = -latitude
                self.file.write(str(latitude) + ',')

                longitude = data['longitude']
                if data['longitude_direction'] == 'W':
                    longitude = -longitude
                self.file.write(str(longitude))
		self.file.write('\n')

                #return True

	    #elif 'GPHDT' in parsed_sentence:
		#return False

        if 'GPGGA' in parsed_sentence:
            self.file2.write('GPGGA,')
            data = parsed_sentence['GPGGA']

            current_time = rospy.get_rostime()
            self.file2.write(str(current_time) + ',')

            # utm时间
            utc_time = data['utc_time']  # 是一个2-tuple
            self.file2.write(str(utc_time[0]))
            self.file2.write('.')
            self.file2.write(str(utc_time[1]) + ',')

            fix_type = data['fix_type']
            self.file2.write(str(fix_type) + ',')

            num_satellites = data['num_satellites']
            self.file2.write(str(num_satellites) + ',')

            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            self.file2.write(str(latitude) + ',')

            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude
            self.file2.write(str(longitude) + ',')

            altitude = data['altitude'] + data['mean_sea_level']
            self.file2.write(str(altitude))
            self.file2.write('\n')
            # 先忽略不确定度

        elif 'GPZDA' in parsed_sentence:
            self.file2.write("GPZDA,")
            data = parsed_sentence['GPZDA']

            # utm时间
            utc_time = data['utc_time']  # 是一个2-tuple
            self.file2.write(str(utc_time[0]))
            self.file2.write('.')
            self.file2.write(str(utc_time[1]))
            self.file2.write('\n')

        elif 'GPHDT' in parsed_sentence:  # GPHDT(航向信息)
            self.file2.write("GPHDT,")
            data = parsed_sentence['GPHDT']

            #utc_time = data['utc_time']  # 是一个2-tuple
            #self.file2.write(str(utc_time[0]))
            #self.file2.write('.')
            #self.file2.write(str(utc_time[1]))
            if data['heading']:
                heading = data['heading']
                self.file2.write(str(heading))
                self.file2.write('\n')

        else:
            return False

    """Helper method for getting the frame_id with the correct TF prefix"""

    @staticmethod
    def get_frame_id():
        frame_id = rospy.get_param('~frame_id', 'wgs84')
        """Add the TF prefix"""
        prefix = ""
        prefix_param = rospy.search_param('tf_prefix')
        if prefix_param:
            prefix = rospy.get_param(prefix_param)
            return "%s/%s" % (prefix, frame_id)
        else:
            return frame_id
