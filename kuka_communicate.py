# from utils import geometry_calculation as cal
import socket

class connect_kuka():
    def __init__(self):
        self.host = '172.31.1.147'
        self.port = 54604
        self.sock_connect()

    def sock_connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host, self.port))

    def data_to_KRL_bytes(self,send_data):
        send_string = ''
        for element in send_data:
            if type(element) == int:
                send_string = send_string + str(element) + ','
            elif type(element) == list:
                send_string = send_string + '{X %d,Y %d,Z %d,A %f,B %f,C %f}\r\n'% (element[0], element[1], element[2], element[3], element[4], element[5])
        if send_string.endswith(','):
            send_string = send_string[:-1] + '\r\n'
        return bytes(send_string, 'utf-8')

    def send_binary(self, send_data):
        byte_data = self.data_to_KRL_bytes(send_data)
        try:
            self.sock.sendall(byte_data)
        except Exception as ex:
            self.sock_connect()
            self.sock.sendall(byte_data)
        print('done', byte_data)

# import time
# kuka_connect = connect_kuka()

# kuka_connect.sock.recv(1024)
# count = 0
# while True:
#     try:
#         count += 1
#         s_time = time.time()
#         kuka_connect.send_binary([1])
#         kuka_connect.sock.recv(1024)
#         e_time = time.time()
#         print(count, 'elapse time =', e_time - s_time)
#         time.sleep(1)
#     except Exception as ex:
#         print(ex)
