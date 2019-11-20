# 功能
# 1) 将NMEA语句转换为椭球坐标系(纬度, 经度, 大地高)下的数值, 并保存文件
# 2) 将NMEA语句直接保留正高(海拔), 维度, 经度, 为例kml文件的转换

# 出现的WARNING
# 1) add_sentence中, 以#开头的语句无法通过checksum
# 2) add_sentence中, 例如GPRMCM没有写进parser中, 无法解析

# change the save_path/save_path2 in launch file.
