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
import cam_rotation_eye_base as rot

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
        self.conn.sock.recv(1024)
        self.get_config_data()
        time.sleep(6)
        self.main_function()
    def get_config_data(self):
        self.config = configparser.ConfigParser()
        self.config.read("utils/configuration.cfg", encoding="utf-8")

    def set_config_data(self, X, Y, Z):
        section_name = 'mapping_' + self.robot_type
        if not self.config.has_section(section_name):
            self.config.add_section(section_name)
        self.config.set(section_name, 'rx', X)
        self.config.set(section_name, 'ry', Y)
        self.config.set(section_name, 'rz', Z)
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
        centroid_pos = cicle_detection.chessboard_detection(self.camera_data)
        centroid_pos = centroid_pos*cal.Rx(np.radians(rx))*cal.Ry(np.radians(ry))*cal.Rz(np.radians(rz))
        self.conn.send_binary([[1200,0,1400, self.angle_config[0], self.angle_config[1], self.angle_config[2]]])

    def main_function(self):
        self.axis_movement('x')
        self.axis_movement('y')
        rz, rx, ry = rot.find_camera_rotation(self.pos_list_x, self.pos_list_y)
        self.set_config_data(rz, rx, ry)
        print(rz, rx, ry)

run = eye_hand_cali('kr10', 20, [-134, -52, -93])