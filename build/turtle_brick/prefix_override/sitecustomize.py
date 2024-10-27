import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zhengxiao-han/homework/me495/hw2/ws/src/homework-2-0nhc/install/turtle_brick'
