# This program support the convertion of object location from camera by rotating it in 3D environment
# It needs the coorporation of robot hand so control robot function will be applied
# Robot position will be changed in config file

import kuka_communicate as robot
import configparser
from ast import literal_eval
import cicle_detection
import setup_camera as cam
import matplot_show as mat
import time
import fit_skspatial as fit
import geometry_calculation as cal
import numpy as np
import cam_rotation_eye_hand as rot

class eye_hand_cali():
    def __init__(self, robot_type, number_steps, angle_config):
        self.robot_type = robot_type
        self.num_step = number_steps
        self.angle_config = angle_config
        self.pos_list_x = []
        self.pos_list_y = []
        self.robo_pos_list = []
        self.camera_data = cam.setup_cam()
        self.conn = robot.connect_kuka()
        self.robot_origin_pos = self.conn.sock.recv(1024)
        self.robot_origin_pos = self.robot_origin_pos.replace(b'\r\n',b'')
        self.robot_origin_pos = [float(v) for v in self.robot_origin_pos.decode("utf-8").split(',')]
        self.get_config_data()
        # time.sleep(4)
        self.main_function()
    def get_config_data(self):
        self.config = configparser.ConfigParser()
        self.config.read("utils/configuration.cfg", encoding="utf-8")

    def set_config_data(self, section_name, key, value):
        if not self.config.has_section(section_name):
            self.config.add_section(section_name)
        self.config.set(section_name, key, value)
        with open("utils/configuration.cfg", 'w') as configfile:
            self.config.write(configfile)

    def axis_movement(self, axis):
        section_name = axis + '_axis_eye_' + self.robot_type
        start_pos = literal_eval(self.config[section_name]['start_pos'])
        end_pos = literal_eval(self.config[section_name]['end_pos'])
        
        if axis == 'x':
            print('xaxis_confirmation')
            step_range = (end_pos[1] - start_pos[1])/ self.num_step
            for i in range(self.num_step):
                new_pos = [start_pos[0],start_pos[1] + i*step_range,start_pos[2]]
                self.robo_pos_list.append(new_pos)
                self.conn.send_binary([[new_pos[0],new_pos[1] ,new_pos[2], self.angle_config[0], self.angle_config[1], self.angle_config[2]]])
                self.conn.sock.recv(1024)
                #get object position
                centroid_pos = cicle_detection.chessboard_detection(self.camera_data)
                if len(centroid_pos) > 0:
                    self.pos_list_x.append(centroid_pos)
        elif axis == 'y':
            print('yaxis_confirmation')
            step_range = (end_pos[0] - start_pos[0])/ self.num_step
            for i in range(self.num_step):
                new_pos = [start_pos[0] + i*step_range,start_pos[1],start_pos[2]]
                self.robo_pos_list.append(new_pos)
                self.conn.send_binary([[new_pos[0],new_pos[1],new_pos[2], self.angle_config[0], self.angle_config[1], self.angle_config[2]]])
                self.conn.sock.recv(1024)
                #get object position
                centroid_pos = cicle_detection.chessboard_detection(self.camera_data)
                if len(centroid_pos) > 0:
                    self.pos_list_y.append(centroid_pos)

    def eyes_to_tool(self, rx, ry, rz):
        self.conn.send_binary([[self.robot_origin_pos[0],
                                self.robot_origin_pos[1],
                                self.robot_origin_pos[2], 
                                180,0,180]])
        self.conn.sock.recv(1024)
        centroid_pos_rotation0 = cicle_detection.chessboard_detection(self.camera_data)
        centroid_pos_rotation0 = centroid_pos_rotation0*cal.Rx(np.radians(rx))*cal.Ry(np.radians(ry))*cal.Rz(np.radians(rz))
        self.conn.send_binary([[self.robot_origin_pos[0],
                                self.robot_origin_pos[1],
                                self.robot_origin_pos[2], 
                                0, 0, 180]])
        self.conn.sock.recv(1024)
        centroid_pos_rotation1 = cicle_detection.chessboard_detection(self.camera_data)
        centroid_pos_rotation1 = centroid_pos_rotation1*cal.Rx(np.radians(rx))*cal.Ry(np.radians(ry))*cal.Rz(np.radians(rz))
        eyes2tool = abs(-np.array(centroid_pos_rotation0) - np.array(centroid_pos_rotation1))/2
        print(eyes2tool)
    def main_function(self):
        # self.axis_movement('x')
        # self.axis_movement('y')
        # rz, rx, ry = rot.find_camera_rotation(self.pos_list_x, self.pos_list_y)
        rz, rx, ry = [-3.913, 0.474, -0.531]
        self.eyes_to_tool(rz, rx, ry)
        section_name = 'mapping_' + self.robot_type
        self.set_config_data(section_name, 'rz',rz)
        self.set_config_data(section_name, 'rx',rx)
        self.set_config_data(section_name, 'ry',ry)
        print(rz, rx, ry)


run = eye_hand_cali('kr60', 20, [180, 0, 180])
# run = eye_hand_cali('kr10', 20, [-134, -52, -93])
# robo_pos_list = []
# def get_config_data():
#     config = configparser.ConfigParser()
#     config.read("utils/configuration.cfg", encoding="utf-8")
#     return config

# def temp_func(axis, num_step):
#     config = get_config_data()
#     section_name = axis + '_axis_eye_kr10'
#     start_pos = literal_eval(config[section_name]['start_pos'])
#     end_pos = literal_eval(config[section_name]['end_pos'])
#     if axis == 'x':
#         print('xaxis_confirmation')
#         step_range = (end_pos[1] - start_pos[1])/ num_step
#         for i in range(num_step):
#             new_pos = [start_pos[0],start_pos[1] + i*step_range,start_pos[2]]
#             robo_pos_list.append(new_pos)

#     elif axis == 'y':
#         print('yaxis_confirmation')
#         step_range = (end_pos[0] - start_pos[0])/ num_step
#         for i in range(num_step):
#             self.conn.send_binary([[start_pos[0] + i*step_range,start_pos[1],start_pos[2], 179.84,-0.15,178.5]])#175,1,177 # 90 degree down
#             # self.conn.send_binary([[start_pos[0] + i*step_range,start_pos[1],start_pos[2], 170,7,175]]) #KR60
#             # new_pos = [start_pos[0] + i*step_range,start_pos[1],start_pos[2]]
#             robo_pos_list.append(new_pos)
#     print('done')
# temp_func('x', 40)
# temp_func('y', 40)