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

import re
import datetime
import calendar
import math
import logging
logger = logging.getLogger('rosout')


def safe_float(field):
    try:
        return float(field)
    except ValueError:
        return float('NaN')


def safe_int(field):
    try:
        return int(field)
    except ValueError:
        return 0


def convert_latitude(field):
    return safe_float(field[0:2]) + safe_float(field[2:]) / 60.0


def convert_longitude(field):
    return safe_float(field[0:3]) + safe_float(field[3:]) / 60.0


def convert_time(nmea_utc):
    """
    Parameters:
        nmea_utc - nmea sentence

    Returns:
        2-tuple of (unix seconds, nanoseconds),
        or (NaN, NaN) if sentence does not contain valid time
    """
    # If one of the time fields is empty, return NaN seconds
    if not nmea_utc[0:2] or not nmea_utc[2:4] or not nmea_utc[4:6]:
        return (float('NaN'), float('NaN'))

    # Get current time in UTC for date information
    utc_time = datetime.datetime.utcnow()
    hours = int(nmea_utc[0:2])
    minutes = int(nmea_utc[2:4])
    seconds = int(nmea_utc[4:6])
    nanosecs = int(nmea_utc[7:]) * pow(10, 9 - len(nmea_utc[7:]))

    ## resolve the ambiguity of day
    day_offset = int((utc_time.hour - hours)/12.0)
    utc_time += datetime.timedelta(day_offset)
    utc_time.replace(hour=hours, minute=minutes, second=seconds)  # 除了年月日, 要用gps中真实的数值

    unix_secs = calendar.timegm(utc_time.timetuple())
    return (unix_secs, nanosecs)

def convert_time_zda(day_str, month_str, year_str, time_str):
    """
    Parameters:
        nmea_utc - nmea sentence

    Returns:
        2-tuple of (unix seconds, nanoseconds),
        or (NaN, NaN) if sentence does not contain valid time
    """
    # If one of the time fields is empty, return NaN seconds
    if not time_str[0:2] or not time_str[2:4] or not time_str[4:6]:
        return (float('NaN'), float('NaN'))

    pc_year = datetime.date.today().year

    ## resolve the ambiguity of century
    """
    example 1: utc_year = 99, pc_year = 2100
    years = 2100 + int((2100 % 100 - 99) / 50.0) = 2099
    example 2: utc_year = 00, pc_year = 2099
    years = 2099 + int((2099 % 100 - 00) / 50.0) = 2100
    """
    utc_year = int(year_str)
    years = pc_year + int((pc_year % 100 - utc_year) / 50.0)

    months = int(month_str)
    days = int(day_str)

    hours = int(time_str[0:2])
    minutes = int(time_str[2:4])
    seconds = int(time_str[4:6])
    nanosecs = int(time_str[7:]) * pow(10, 9 - len(time_str[7:]))  # 0.22秒, 22乘10的7次幂得到整数纳秒

    unix_secs = calendar.timegm((years, months, days, hours, minutes, seconds))  # 返回从计算机元年到给定tuple时间的秒数
    return (unix_secs, nanosecs)



#字典, 每个key对应一个list, list中有一堆tuple
"""Format for this dictionary is a sentence identifier (e.g. "GGA") as the key, with a
list of tuples where each tuple is a field name, conversion function and index
into the split sentence"""
parse_maps = {
    "GPGGA": [
        ("utc_time", convert_time, 1),
        ("fix_type", int, 6),  # 重要
        ("num_satellites", safe_int, 7),
        ("latitude", convert_latitude, 2),
        ("latitude_direction", str, 3),
        ("longitude", convert_longitude, 4),
        ("longitude_direction", str, 5),
        ("altitude", safe_float, 9),
        ("mean_sea_level", safe_float, 11),  # 椭球分离值
        ("hdop", safe_float, 8),  # HDOP 水平精度因子
    ],
    "GPHDT": [
        ("heading", safe_float, 1),
    ],
    "GPZDA": [
        ("utc_time", safe_float, 1),
        ("day", int, 2),
        ("month", int, 3),
        ("year", int, 4),
    ],
    "GPGST": [
        ("utc_time", convert_time, 1),
        ("ranges_std_dev", safe_float, 2),
        ("semi_major_ellipse_std_dev", safe_float, 3),
        ("semi_minor_ellipse_std_dev", safe_float, 4),
        ("semi_major_orientation", safe_float, 5),
        ("lat_std_dev", safe_float, 6),
        ("lon_std_dev", safe_float, 7),
        ("alt_std_dev", safe_float, 8),
    ],
    "GIMBALLEDPVA": [
        ("latitude", convert_latitude, 3),
        ("longitude", convert_longitude, 4),
        ("altitude", safe_float, 5),
    ],
}


def parse_nmea_sentence(nmea_sentence):
    # Check for a valid nmea sentence

    if not re.match(
            r'(^\$GP|^\$GI|^\$GN|^\$IN).*\*[0-9A-Fa-f]{2}$', nmea_sentence):
        logger.debug(
            "Regex didn't match, sentence not valid NMEA? Sentence was: %s" %
            repr(nmea_sentence)) # repr函数完整的显示内容, 例如可能会包含"\n"
        return False
    # list
    fields = [field.strip(',') for field in nmea_sentence.split(',')]

    # Ignore the $
    sentence_type = fields[0][1:]

    # 根据字典中的key查询
    if sentence_type not in parse_maps:
        logger.debug("Sentence type %s not in parse map, ignoring."
                     % repr(sentence_type))
        return False

    parse_map = parse_maps[sentence_type]

    # 空字典
    parsed_sentence = {}
    # 遍历list中的每个tuple
    # 以GPGGALONG为例, 第一个entry是GPS状态, 所以找到的是第<6>个语句
    for entry in parse_map:
        parsed_sentence[entry[0]] = entry[1](fields[entry[2]]) #类型转换, 转换为parse_maps中定义好的数据类型

    if sentence_type == "GPZDA":  # 将日期加到utc上去
        parsed_sentence["utc_time"] = convert_time_zda(fields[3], fields[4], fields[5], fields[1])

    # key是sentence_type, 返回的是大字典?
    return {sentence_type: parsed_sentence}
