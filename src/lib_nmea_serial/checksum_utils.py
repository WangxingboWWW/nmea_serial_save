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


# Check the NMEA sentence checksum. Return True if passes and False if failed
def check_nmea_checksum(nmea_sentence):
    split_sentence = nmea_sentence.split('*')
    # 每个语句中只有一个*号
    # 保留*之前的字符串
    if len(split_sentence) != 2:
        # No checksum bytes were found... improperly formatted/incomplete NMEA
        # data?
        return False
    # 用于移除字符串头尾指定的字符（默认为空格或换行符）或字符序列, 因为校验数据后跟着的是结束符, 回车符号
    transmitted_checksum = split_sentence[1].strip()

    # Remove the $ at the front
    # BestXYZ不适用这个
    data_to_checksum = split_sentence[0][1:]
    checksum = 0
    # 对每一位c转换为其ASCII对应的数值
    for c in data_to_checksum:
        # ord()函数返回的该ASCII码对应的是十进制数, 按位异或
        checksum ^= ord(c)
    # upper()将小写转换为大写后的字符串
    return ("%02X" % checksum) == transmitted_checksum.upper()
