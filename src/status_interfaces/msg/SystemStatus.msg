# 消息接口定义文件，文件名首字母大写
# 定义完数据接口后，需要再CMakeList.txt中对该文件进行注册，声明为消息接口文件
builtin_interfaces/Time stamp # 记录时间戳，builtin_interfaces为ROS 2的内置功能包
string host_name
float32 cpu_percent
float32 memory_percent
float32 memory_total
float32 memory_available
float64 net_sent
float64 net_recv

# 还有的内置其它类型为：
# bool, 
# byte, char, 
# float32, float64, 
# int8, uint8, int16, uint16, int32, uint32, int64, uint64
# string
