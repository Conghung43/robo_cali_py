import struct
import numpy as np
from ast import literal_eval

data = b'1245.494751,-26.025839,1390.145386,-180.000000,0.000000,180.000000\r\n'
data = data.replace(b'\r\n',b'')
data = [float(v) for v in data.decode("utf-8").split(',')]
loop_num = int(len(data)/2)
for index in range(loop_num):
    index = index*2
    current_data = data[index:index+2]
    print(int.from_bytes(current_data, byteorder='big'))
    # print(struct.unpack("e", current_data)[0])
    # float_str = "-0b101010101"
    print(struct.unpack("<e", current_data)[0])
    print(struct.unpack(">e", current_data)[0])
    print(struct.unpack("!e", current_data)[0])
    print(struct.unpack("@e", current_data)[0])
    print(struct.unpack("h", current_data)[0])
    print(struct.unpack("H", current_data)[0])
    print(struct.unpack(">h", current_data)[0])
    print(struct.unpack("!h", current_data)[0])
    print(struct.unpack("@h", current_data)[0])