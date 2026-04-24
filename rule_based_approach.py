from math import ceil, floor
from operator import itemgetter, mod
from data import *
from black_board import *


class Rule_Based_Approach:
    def __init__(self, g_id, m_id, global_blackboard, global_event_handler):
        # Global ID
        self.global_id = g_id
        self.module_id = m_id

        self.black_board = global_blackboard
        self.node_id = self.black_board.register_node(
            self.global_key(), f"Node:Rule_Based_Approach {self.global_id}"
        )

        self.event_handler = global_event_handler
        self.event_id = self.event_handler.register_node(
            f"Event:Rule_Based_Approach {self.global_id}"
        )
        self.event_queue = None

        self.Tmin = 3
        self.Tmax = 12
        self.T = -1.0
        self.Tcharge = -1.0
        self.tau_c = 0.03
        self.tau_d = 0.003
        self.Bmin = 30.0
        self.Bth = 50.0
        self.Nc = -1
        self.Nc_free = -1
        self.Nth = -1
        self.Nd = -1
        self.Nsd = -1
        self.Ns = -1
        self.TTW = -1
        self.alpha_d = 0.03
        self.alpha_u = 1.2
        self.battery_percentage = []
        self.assign_robot = []

    def global_key(self):
        return (self.global_id * 100) + self.module_id

    def setup(self):
        self.black_board.register(self.global_key(), 0, Data.GID)
        self.black_board.register(self.global_key(), 0, Data.RID)
        self.black_board.register(self.global_key(), 0, Data.Battery)
        self.black_board.register(self.global_key(), 1, Data.Charge_Time)

    def reset_parameter(self):
        self.alpha_d = 1
        self.alpha_u = 1

    def initialize_parameter(self):
        self.Nth = 0
        self.Nd = 0
        self.Ns = 0
        self.Nsd = 0
        self.Nc = 0
        self.free_charge_station = set()
        for mode in self.charge_station:
            self.Nc += 1

        for rid, battery in self.battery_percentage:
            if battery <= self.Bth:
                self.Nth += 1
            if battery <= self.Bmin:
                self.Nd += 1
        self.Ns = ceil(self.Nth / self.Nc)
        self.Nsd = ceil(self.Nd / self.Nc)

    def get_battery_percentage(self):
        self.battery_percentage = self.black_board.merge_all([Data.RID, Data.Battery])
        # print(f"Battery Percentage {self.battery_percentage}")

    def get_charge_station(self):
        self.charge_station = self.black_board.read_all((500, Data.Charge_Station_Mode))
        # print(f"Charge Station {self.charge_station}")

    def compute_ttw(self):
        total = 0.0
        for rid, battery in self.battery_percentage:
            total += battery
        self.TTW = ((total / self.Nth) - (self.alpha_d * self.Bmin)) / self.tau_d

    def algorithm(self):
        self.get_battery_percentage()
        self.get_charge_station()
        self.initialize_parameter()
        if self.Nth == 0:
            return
        if self.Nd == 0:
            self.compute_ttw()
            self.T = self.TTW / self.Ns

            if self.T > self.Tmax:
                self.Tcharge = self.Tmax
            elif self.Tmax > self.T > self.Tmin:
                self.Tcharge = self.T
            else:
                self.alpha_d = 0.8
                self.Tcharge = self.Tmin
        else:
            self.Tcharge = self.alpha_u * self.Tmax

    def solve(self):

        self.algorithm()
        for rid, battery in self.battery_percentage:
            self.black_board.write((300 + rid, Data.Charge_Time), self.Tcharge)
