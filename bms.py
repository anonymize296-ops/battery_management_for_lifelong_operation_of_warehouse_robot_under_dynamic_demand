from re import I, L
import matplotlib.pyplot as plt
import enum
import csv
import threading
import time
from data import *
from black_board import *
from rule_based_approach import *

sim_to_real = 1.0

class Charge_Station: 
    def __init__(self, g_id, m_id, global_blackboard, global_event_handler):
        #Global ID
        self.global_id = g_id
        self.module_id = m_id
        #Centralized Data Store
        self.black_board = global_blackboard
        self.node_id = self.black_board.register_node(self.global_key(), f"Node:Charge_Station {self.global_id}")

        self.event_handler = global_event_handler
        self.event_id = self.event_handler.register_node(f"Event:Charge_Station {self.global_id}")
        self.event_queue = None

        self.charge_id = m_id 
        self.mode = Charge_Station_Mode.Free 
        self.rid = -1
        

        self.setup()

    def global_key(self):
        return (self.global_id * 100) + self.module_id

    def setup(self):
        self.black_board.register(self.global_key(), 1, Data.GID)
        self.black_board.register(self.global_key(), 1, Data.Charge_Station_Mode)

        self.black_board.write((self.global_key(), Data.GID), self.global_key())
        self.black_board.write((self.global_key(), Data.Charge_Station_Mode), self.mode)
        self.black_board.write((self.global_key(), Data.Charge_Station_Robot), self.rid)


class Battery:
    def __init__(self, g_id, m_id, global_blackboard, global_event_handler):
        #Global ID
        self.global_id = g_id
        self.module_id = m_id
        #Centralized Data Store
        self.black_board = global_blackboard
        self.node_id = self.black_board.register_node(self.global_key(), f"Node:Battery {self.global_id}")

        self.event_handler = global_event_handler
        self.event_id = self.event_handler.register_node(f"Event:Battery {self.global_id}")
        self.event_queue = None

        self.rid = m_id
        self.charge_robot_info = [-1, -1, False]
        self.charge_time = 0
        self.critical = False
        self.battery_filepath = "./data/battery_data.csv" 
        open(self.battery_filepath, 'w').close()
        self.battery_file = open(self.battery_filepath, "a")
        self.battery_writer = csv.writer(self.battery_file)
        if self.module_id == 0: 
            self.battery_writer.writerow(['time', 'sim2real','rid', 'cid', 'gid', 'mode', 'battery', 'critical'])
        self.battery_percentage = None 
        self.mode = Mode_Of_Operation.Off_Mode 
        self.battery_rate = 0.
        self.start_thread = False
        self.thread = None
        self.dt = 0.1
        self.battery_threshold = 50
        self.battery_threshold_critical = 35
        self.setup()


    def global_key(self):
        return (self.global_id * 100) + self.module_id

    def setup(self):
        self.black_board.register(self.global_key(), 1, Data.GID)
        self.black_board.register(self.global_key(), 1, Data.Battery)
        self.black_board.register(self.global_key(), 1, Data.Battery_Mode)
        self.black_board.register(self.global_key(), 1, Data.Charge_Robot_Info)

        self.black_board.write((self.global_key(), Data.GID), self.global_key())
        self.black_board.write((self.global_key(), Data.Battery_Mode), self.mode)
        self.black_board.write((self.global_key(), Data.Charge_Time), 0)
        self.black_board.write((self.global_key(), Data.Charge_Robot_Info), self.charge_robot_info)

    def set_battery(self, battery):
        self.battery_percentage = battery

    def get_battery(self, battery):
        return self.battery_percentage

    def set_mode(self, mode):
        self.mode = mode

    def initiate_mode(self):
        self.battery_percentage = self.black_board.read((self.global_key(), Data.Battery))
        self.mode = self.black_board.read((self.global_key(), Data.Battery_Mode))

    def update_charge_time(self):
        self.charge_robot_info = self.black_board.read((self.global_key(), Data.Charge_Robot_Info))

        if self.charge_robot_info[0] != -1:

            if self.charge_robot_info[0] != 0:
                self.charge_time = self.black_board.read((self.global_key(), Data.Charge_Time))
                self.charge_time = max(self.charge_time - 1, 0)

                if (self.charge_time == 0) and (self.mode == Mode_Of_Operation.Charge_Mode):
                    self.mode = Mode_Of_Operation.Work_Mode
                    self.black_board.write((self.charge_robot_info[0], Data.Charge_Station_Mode), Charge_Station_Mode.Free)
                    self.charge_station_id = -1
                    self.black_board.write((self.global_key(), Data.Charge_Robot_Info), [-1, -1, False])
                elif (self.charge_time > 0):
                    self.mode = Mode_Of_Operation.Charge_Mode
            else:
                self.mode = Mode_Of_Operation.Queue_Mode
        else:
            #forcefully removed robot from charge station 
            if self.mode == Mode_Of_Operation.Charge_Mode:
                self.mode = Mode_Of_Operation.Work_Mode
        #else:
        #    print(f"{self.global_key()} Unknown State in Update charge time")

        if self.charge_robot_info[0] != -1:
            print(f"Robot {self.rid} Charge Station ID {self.charge_robot_info[0]} Queue ID: {self.charge_robot_info[1]}")
        self.black_board.write((self.global_key(), Data.Charge_Time), self.charge_time)
        self.black_board.write((self.global_key(), Data.Battery_Mode), self.mode)

    def set_rate(self):
        if self.mode == Mode_Of_Operation.Charge_Mode:
            #self.battery_rate = 6.
            self.battery_rate = +0.12
        elif self.mode == Mode_Of_Operation.Work_Mode:
            #self.battery_rate = -4.
            self.battery_rate = -0.00012
        elif self.mode == Mode_Of_Operation.Queue_Mode:
            self.battery_rate = -0.0022
        elif self.mode == Mode_Of_Operation.Off_Mode:
            self.battery_rate = 0.
        elif self.mode == Mode_Of_Operation.On_Mode:
            self.battery_rate = -0.00002

    def update_battery(self):
       # print(f"Battery Percentage {self.battery_percentage} ,Rate {self.battery_rate}")
       
        self.battery_percentage += self.battery_rate
        self.battery_percentage = min(self.battery_percentage, 100.)
        self.battery_percentage = max(self.battery_percentage, 0.)

        if self.battery_percentage < self.battery_threshold_critical:
            self.critical = True

        #if (self.critical) and (self.mode == Mode_Of_Operation.Work_Mode):
        #    self.critical = False

        #kself.black_board.write((self.global_key(), Data.Critical), self.critical)
        self.black_board.write((self.global_key(), Data.Battery), self.battery_percentage)

    def store_data(self):
        self.battery_file = open(self.battery_filepath, "a")
        self.battery_writer = csv.writer(self.battery_file) 
        self.battery_writer.writerow([time.time(), 1 ,self.rid, self.charge_robot_info[0], self.charge_robot_info[1],self.mode.value,self.battery_percentage,self.charge_robot_info[2]])
        self.battery_file.close()

    def behaviour(self):
        self.initiate_mode()
        if self.mode is not None:
            self.update_charge_time()
            self.set_rate()
            self.update_battery()
            self.store_data()

