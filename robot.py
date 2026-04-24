# from simulation import *
# import imp
from pstats import Stats
from re import A, I
from turtle import pos
import csv

# from pyrsistent import T
from black_board import *
from data import *
import multiprocessing
import concurrent.futures
import random
import networkx as nx
import time
import re
import logging


logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

formater = logging.Formatter("%(asctime)s:%(name)s:%(message)s")

file_handler = logging.FileHandler("robot.log")
file_handler.setLevel(logging.DEBUG)
file_handler.setFormatter(formater)

logger.addHandler(file_handler)

task_logger = logging.getLogger(__name__)
task_logger.setLevel(logging.DEBUG)


task_file_handler = logging.FileHandler("task.log")
task_file_handler.setLevel(logging.DEBUG)
task_file_handler.setFormatter(formater)

task_logger.addHandler(task_file_handler)


class Battery:
    def __init__(self, g_id, m_id, global_blackboard, global_event_handler):
        # Global ID
        self.global_id = g_id
        self.module_id = m_id
        # Centralized Data Store
        self.black_board = global_blackboard
        self.node_id = self.black_board.register_node(
            self.global_key(), f"Node:Battery {self.global_id}"
        )

        self.event_handler = global_event_handler
        self.event_id = self.event_handler.register_node(
            f"Event:Battery {self.global_id}"
        )
        self.event_queue = None

        self.rid = m_id
        self.charge_robot_info = [-1, -1, False]
        self.charge_time = 0
        self.critical = False
        self.battery_filepath = "./data/battery_data.csv"
        open(self.battery_filepath, "w").close()
        self.battery_file = open(self.battery_filepath, "a")
        self.battery_writer = csv.writer(self.battery_file)
        if self.module_id == 0:
            self.battery_writer.writerow(
                ["time", "sim2real", "rid", "cid", "gid", "mode", "battery", "critical"]
            )
        self.battery_percentage = None
        self.mode = Mode_Of_Operation.Off_Mode
        self.battery_rate = 0.0
        self.start_thread = False
        self.thread = None
        self.time = 0
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
        # self.black_board.register(self.global_key(), 1, Data.Load_Outbound)
        # self.black_board.register(self.global_key(), 1, Data.Drop)

        self.black_board.write((self.global_key(), Data.GID), self.global_key())
        self.black_board.write((self.global_key(), Data.Battery_Mode), self.mode)
        self.black_board.write((self.global_key(), Data.Charge_Time), 0)
        self.black_board.write(
            (self.global_key(), Data.Charge_Robot_Info), self.charge_robot_info
        )
        # self.black_board.write((self.global_key(), Data.Load_Outbound), [])
        # self.black_board.write((self.global_key(), Data.Drop), [])

    def set_battery(self, battery):
        self.battery_percentage = battery

    def get_battery(self, battery):
        return self.battery_percentage

    def set_mode(self, mode):
        self.mode = mode

    def initiate_mode(self):
        self.battery_percentage = self.black_board.read(
            (self.global_key(), Data.Battery)
        )
        self.mode = self.black_board.read((self.global_key(), Data.Battery_Mode))

    def update_charge_time(self):
        self.charge_robot_info = self.black_board.read(
            (self.global_key(), Data.Charge_Robot_Info)
        )

        if self.charge_robot_info[0] != -1:

            if self.charge_robot_info[0] != 0:
                self.charge_time = self.black_board.read(
                    (self.global_key(), Data.Charge_Time)
                )
                self.charge_time = max(self.charge_time - 1, 0)

                if (self.charge_time == 0) and (
                    self.mode == Mode_Of_Operation.Charge_Mode
                ):
                    self.mode = Mode_Of_Operation.Work_Mode
                    self.black_board.write(
                        (self.charge_robot_info[0], Data.Charge_Station_Mode),
                        Charge_Station_Mode.Free,
                    )
                    self.charge_station_id = -1
                    self.black_board.write(
                        (self.global_key(), Data.Charge_Robot_Info), [-1, -1, False]
                    )
                elif self.charge_time > 0:
                    self.mode = Mode_Of_Operation.Charge_Mode
            else:
                self.mode = Mode_Of_Operation.Queue_Mode
        else:
            # forcefully removed robot from charge station
            if self.mode == Mode_Of_Operation.Charge_Mode:
                self.mode = Mode_Of_Operation.Work_Mode
        # else:
        #    print(f"{self.global_key()} Unknown State in Update charge time")

        if self.charge_robot_info[0] != -1:
            print(
                f"Robot {self.rid} Charge Station ID {self.charge_robot_info[0]} Queue ID: {self.charge_robot_info[1]}"
            )
        self.black_board.write((self.global_key(), Data.Charge_Time), self.charge_time)
        # self.black_board.write((self.global_key(), Data.Battery_Mode), self.mode)

    def set_rate(self):
        self.mode = self.black_board.read((self.global_key(), Data.Battery_Mode))
        if self.mode == Mode_Of_Operation.Charge_Mode:
            self.battery_rate = (100.0 / (20.0 * 60.0)) * self.dt * 2
        elif self.mode == Mode_Of_Operation.Work_Mode:
            self.battery_rate = -(100.0 / (2 * 60.0 * 60.0)) * self.dt
        elif self.mode == Mode_Of_Operation.Queue_Mode:
            self.battery_rate = -(100.0 / (8 * 60.0 * 60.0)) * self.dt
        elif self.mode == Mode_Of_Operation.Off_Mode:
            self.battery_rate = 0.0
        elif self.mode == Mode_Of_Operation.On_Mode:
            self.battery_rate = -0.00002

    def update_battery(self):
        # print(f"Battery Percentage {self.battery_percentage} ,Rate {self.battery_rate}")

        self.battery_percentage += self.battery_rate
        self.battery_percentage = min(self.battery_percentage, 100.0)
        self.battery_percentage = max(self.battery_percentage, 0.0)

        if self.battery_percentage < self.battery_threshold_critical:
            self.critical = True

        # if (self.critical) and (self.mode == Mode_Of_Operation.Work_Mode):
        #    self.critical = False

        # kself.black_board.write((self.global_key(), Data.Critical), self.critical)
        self.black_board.write(
            (self.global_key(), Data.Battery), self.battery_percentage
        )

    def store_data(self):
        self.time += self.dt
        self.battery_file = open(self.battery_filepath, "a")
        self.battery_writer = csv.writer(self.battery_file)
        self.battery_writer.writerow(
            [
                self.time,
                1,
                self.rid,
                self.charge_robot_info[0],
                self.charge_robot_info[1],
                self.mode.value,
                self.battery_percentage,
                self.charge_robot_info[2],
            ]
        )
        self.battery_file.close()

    def behaviour(self):
        self.initiate_mode()
        if self.mode is not None:
            self.update_charge_time()
            self.set_rate()
            self.update_battery()
            self.store_data()


class Reservation(enum.Enum):
    Move = 0
    Wait = 1


class Robot:
    def __init__(self, g_id, r_id, global_black_board, global_event_handler, planner):
        # Global ID
        self.global_id = g_id
        self.module_id = r_id
        # centralized data store
        self.black_board = global_black_board
        self.node_id = self.black_board.register_node(
            self.global_key(), f"Node:Robot {self.global_id}"
        )

        self.event_handler = global_event_handler
        self.event_id = self.event_handler.register_node(
            f"Event:Robot {self.global_id}"
        )
        self.event_queue = None

        self.planner = planner
        # Robot State
        # 1.ID
        self.r_id = r_id

        # 2.Global Position and Velocity
        self.x_pos = None
        self.y_pos = None
        self.pos = Data.Pos
        self.battery = Battery(
            self.global_id, self.module_id, self.black_board, self.event_handler
        )
        self.initial_battery = None
        self.initialized = False

        self.theta = 0.0
        self.theta_bb = Data.Theta

        self.x_vel = 0.0
        self.y_vel = 0.0
        self.theta_vel = 0.0
        self.vel = Data.Vel

        # 3.Response rate
        self.dt = 0.04
        self.itr = 0

        # 4.Goal Pose
        self.last_goal = None
        self.goal = None
        self.final_goal = None

        # 5.Robot Routine state [on Goal, Move, Charge]
        self.current_task = Task.WaitForTask
        self.status = Status.InOutboundStation
        # self.current_task = Task.Inbound
        # self.status = Status.Unloaded

        self.set_goal = False

        self.got_new_plan = False

        self.task_id = -1
        self.inbound_task_list = []
        self.inbound_load_task = []
        self.inbound_task_start_time = []

        self.outbound_task_list = []
        self.outbound_load_task = []
        self.outbound_task_start_time = []

        self.outbound_load_task_list = []
        self.outbound_load_load_task = []
        self.outbound_load_task_start_time = []

        self.task_index_list = []
        self.all_task_path = []
        self.last_task = -1
        # Parameter for Controller and Loading and Unloading wait
        # Controller
        self.angular_speed = 0.8  # 0.8 rad/s
        self.linear_speed = 0.7  # 0.7 m/s
        # Loading and Unloading
        self.wait_count = 0
        self.inbound_load_count = 406
        self.outbound_load_count = 58
        self.load_count = 500
        self.unload_count = 500

        # safety related parameter
        self.safety_node = 1
        self.reservation = Reservation.Wait
        self.curr_index = None
        self.goal_index = None

        self.plan_index = None
        # planner paramert
        self.task_horizon = 1

        self.inbound_task_completed = False
        self.outbound_task_completed = False
        self.outbound_load_task_completed = False

        # Load in Robot
        self.number_of_inbound_task = 0
        self.number_of_outbound_task = 0
        self.current_outbound_count = 0

        self.on_rotation = False

        self.assign_inbound = False
        self.assign_outbound = False
        self.charging_station = []

        self.rotation = ["6,34", "8,35", "5,35", "2,18", "1,18", "3,18"]

        # robot inbound station
        self.robot_inbound_station = -1
        self.assign_task = False

        self.park_pos = None

        self.path_node = None
        self.path_node_reservation = None
        self.full_path = []

        self.last_current_task = None
        self.subgoal = []
        self.target_battery = 0
        self.setup()
        self.initiate()

    def global_key(self):
        return (self.global_id * 100) + self.module_id

    def setup(self):
        # Data
        self.black_board.register(self.global_key(), 1, Data.RID)
        self.black_board.register(self.global_key(), 1, Data.Pos)
        self.black_board.register(self.global_key(), 1, Data.Theta)
        self.black_board.register(self.global_key(), 1, Data.Vel)
        self.black_board.register(self.global_key(), 1, Data.Goal)
        self.black_board.register(self.global_key(), 0, Data.Initial_Pose)
        self.black_board.register(self.global_key(), 0, Data.Task)
        self.black_board.register(self.global_key(), 1, Data.Task_ID)
        self.black_board.register(self.global_key(), 1, Data.RobotStatus)
        self.black_board.register(self.global_key(), 1, Data.Assign_Inbound)
        self.black_board.register(self.global_key(), 1, Data.Assign_Outbound)
        self.black_board.register(self.global_key(), 1, Data.Number_of_Inbound_Task)
        self.black_board.register(self.global_key(), 1, Data.Number_of_Outbound_Task)
        self.black_board.register(self.global_key(), 1, Data.Robot_Inbound_Station)
        self.black_board.register(self.global_key(), 0, Data.Assign_Task)
        self.black_board.register(self.global_key(), 0, Data.Assign_Outbound)
        self.black_board.register(self.global_key(), 0, Data.Parking_Station)
        self.black_board.register(self.global_key(), 1, Data.Charging_station)
        self.black_board.register(self.global_key(), 1, Data.Battery_Mode)
        self.black_board.register(self.global_key(), 0, Data.Charge_Time)
        self.black_board.register(self.global_key(), 0, Data.CS_Battery_level)
        self.black_board.register(self.global_key(), 1, Data.Full_Path)
        self.black_board.register(self.global_key(), 1, Data.Load_Outbound)

        # Events
        # self.event_handler.register((self.global_id, self.module_id), self.event_id, 0, Event.Goal_Reached)
        # self.event_handler.register((self.global_id, self.module_id), self.event_id, 1, Event.Goal_Reached)
        # self.event_handler.register((self.global_id, self.module_id), self.event_id, 0, Event.Got_New_Goal)
        # self.event_handler.register((self.global_id, self.module_id), self.event_id, 1, Event.Got_New_Goal)
        # self.event_handler.register((self.global_id, self.module_id), self.event_id, 1, Event.Moved)
        # self.event_handler.register((self.global_id, self.module_id), self.event_id, 1, Event.Got_LocalPlan)
        # self.event_handler.register((self.global_id, self.module_id), self.event_id, 1, Event.Join_Robot)
        # self.event_handler.register((self.global_id, self.module_id), self.event_id, 1, Event.Leave_Robot)
        # self.event_handler.register((self.global_id, self.module_id), self.event_id, 1, Event.Initialize_Pose)

    def initiate(self):
        self.black_board.write((self.global_key(), Data.RID), self.r_id)
        # self.black_board.write((self.global_key(),Data.Pos), Point(self.x_pos, self.y_pos))
        # self.black_board.write((self.global_key(),Data.Theta), self.theta)
        self.black_board.write(
            (self.global_key(), Data.Vel), Point(self.x_vel, self.y_vel)
        )
        self.black_board.write((self.global_key(), Data.RobotStatus), self.status)
        self.black_board.write(
            (self.global_key(), Data.Assign_Inbound), self.assign_inbound
        )
        self.black_board.write(
            (self.global_key(), Data.Assign_Outbound), self.assign_outbound
        )
        self.black_board.write(
            (self.global_key(), Data.Number_of_Inbound_Task),
            self.number_of_inbound_task,
        )
        self.black_board.write(
            (self.global_key(), Data.Number_of_Outbound_Task),
            self.number_of_outbound_task,
        )
        self.black_board.write((self.global_key(), Data.TaskStatus), self.current_task)
        self.black_board.write((self.global_key(), Data.Assign_Task), self.assign_task)
        self.black_board.write(
            (self.global_key(), Data.Assigned_Task), self.current_task
        )
        self.black_board.write(
            (self.global_key(), Data.Robot_Inbound_Station), self.robot_inbound_station
        )
        self.black_board.write((self.global_key(), Data.RID), self.module_id)
        self.black_board.write((self.global_key(), Data.Battery), self.initial_battery)
        self.black_board.write(
            (self.global_key(), Data.Battery_Mode), Mode_Of_Operation.Off_Mode
        )
        self.black_board.write((self.global_key(), Data.CS_Battery_level), 50)
        # self.black_board.write((self.global_key(),Data.Goal), Point(self.goal.x, self.goal.y))

        # Events
        # self.event_handler.pub((self.global_id, Event.Join_Robot))
        # self.event_handler.pub((self.global_id, Event.Goal_Reached))
        # self.event_handler.pub((self.global_id, Event.Initialize_Pose))

    def set_position(self, x_pos, y_pos, theta):
        self.black_board.write((self.global_key(), Data.Pos), Point(x_pos, y_pos))

        if theta == 0:
            self.theta = 0
        elif theta == 1:
            self.theta = 1.57
        elif theta == 2:
            self.theta = 3.14
        elif theta == 3:
            self.theta = -1.57

        self.black_board.write((self.global_key(), Data.Theta), self.theta)

    def get_initialize(self):
        initial_pos = self.black_board.read((200, Data.Initial_Pose))
        parking_station = self.black_board.read((200, Data.Parking_Station))

        if initial_pos is not None:
            for poses in initial_pos:
                if poses[0] == self.module_id:
                    self.x_pos = poses[1].x
                    self.y_pos = poses[1].y
                    self.theta = poses[2]
                    self.initial_battery = poses[3]
                    self.park_pos = parking_station[self.module_id]
                    # print("parking points x and y",self.park_pos)
                    self.last_task = [-1, Point(self.x_pos, self.y_pos)]
                    self.black_board.write(
                        (self.global_key(), Data.Pos), Point(self.x_pos, self.y_pos)
                    )
                    self.black_board.write((self.global_key(), Data.Theta), self.theta)
                    self.black_board.write(
                        (self.global_key(), Data.Battery), self.initial_battery
                    )
                    self.black_board.write(
                        (self.global_key(), Data.Battery_Mode),
                        Mode_Of_Operation.On_Mode,
                    )
                    self.initialized = True

    def pos_to_node(self, x, y):
        node = f"{int(x)},{int(y)}"
        return str(node)

    def node_to_pos(self, node):
        data = re.split(r",", node)
        pos = (int(data[0]), int(data[1]))
        return pos

    """
    Given planned path in [(x1,y1),(x2,y2),...] to theta plan [(x1,y1,theta1),(x2,y2,theta2),...]
    """

    def theta_plan(self, planned_nodes, initial_theta):
        if len(planned_nodes) == 0:
            return []

        prev_angle = [initial_theta]

        planned_pos = []
        for node in planned_nodes:
            planned_pos.append(self.node_to_pos(node))
        new_path = [(planned_pos[0][0], planned_pos[0][1], initial_theta)]

        if len(planned_pos) > 1:
            diff_x = planned_pos[1][0] - planned_pos[0][0]
            diff_y = planned_pos[1][1] - planned_pos[0][1]
            if initial_theta == 0:
                if abs(diff_y) > 0:
                    new_path.append((planned_pos[0][0], planned_pos[0][1], 1))
                    prev_angle = [1]
            elif initial_theta == 1:
                if abs(diff_x) > 0:
                    new_path.append((planned_pos[0][0], planned_pos[0][1], 0))
                    prev_angle = [0]

        for i in range(1, len(planned_pos)):
            if (i + 1) < len(planned_pos):
                diff_x = planned_pos[i + 1][0] - planned_pos[i - 1][0]
                diff_y = planned_pos[i + 1][1] - planned_pos[i - 1][1]

                if abs(diff_x) > 0 and abs(diff_y) > 0:
                    diff_x = planned_pos[i + 1][0] - planned_pos[i][0]
                    diff_y = planned_pos[i + 1][1] - planned_pos[i][1]

                    if ((diff_x > 0) and (diff_y == 0)) or (
                        (diff_x < 0) and (diff_y == 0)
                    ):
                        prev_angle.append(0)
                    elif ((diff_x == 0) and (diff_y > 0)) or (
                        (diff_x == 0) and (diff_y < 0)
                    ):
                        prev_angle.append(1)

            for angle in prev_angle:
                new_path.append((planned_pos[i][0], planned_pos[i][1], angle))

            prev_angle = [prev_angle[-1]]

        return new_path

    def disc_theta(self, last_theta):
        if abs(last_theta - 3.14) < 0.1:
            last_theta = 0
        elif abs(last_theta - 1.57) < 0.1:
            last_theta = 1
        elif abs(last_theta + 1.57) < 0.1:
            last_theta = 1
        elif abs(last_theta) < 0.1:
            last_theta = 0
        elif abs(last_theta + 3.14) < 0.1:
            last_theta = 0

        return last_theta

    def get_full_path(self):
        # self.full_path = []
        new_full_path = []
        for i in range(len(self.full_path)):
            if (self.path_node is None) or ((i + 1) < len(self.path_node)):
                continue
            node = self.full_path[i]
            new_full_path.append(node)
            task_list = []
        task_load_list = []
        if self.current_task == Task.Outbound_Task:
            task_list = self.outbound_load_task_list
            task_load_list = self.outbound_load_load_task
        else:
            task_list = self.inbound_task_list
            task_load_list = self.inbound_load_task

        task_horizon = min(self.task_horizon, len(task_list))

        start = self.last_task[1]
        last_theta = self.theta

        last_theta = self.disc_theta(last_theta)

        for i in range(task_horizon):
            source_node = self.pos_to_node(start.x, start.y)
            goal = task_list[i][1]
            target_node = self.pos_to_node(goal.x, goal.y)
            if task_load_list[i]:
                # self.full_path += self.planner.plan(source_node,target_node)
                if len(new_full_path) == 0:
                    new_full_path += self.theta_plan(
                        self.planner.plan(source_node, target_node), last_theta
                    )
                    last_theta = new_full_path[-1][2]
                else:
                    new_full_path.pop(-1)
                    new_full_path += self.theta_plan(
                        self.planner.plan(source_node, target_node), last_theta
                    )
                    last_theta = new_full_path[-1][2]
                if self.current_task == Task.Outbound_Task:
                    self.outbound_load_load_task[i] = False
                else:
                    self.inbound_load_task[i] = False
            start = goal

        self.full_path = new_full_path

    def get_nearest_outbound(self):
        outbound_station = self.black_board.read((200, Data.Outbound))
        min_dist = 20000
        min_outbound_id = -1
        min_outbound_pos = None
        min_outbound_wait = -1
        start = self.last_task[1]
        start_node = self.planner.pos_to_node(start)
        for outbound_id, outbound_pos, outbound_wait in outbound_station:
            target = self.planner.pos_to_node(outbound_pos)
            dist = self.planner.plan_distance(start_node, target)
            if dist <= min_dist:
                min_dist = dist
                min_outbound_id = outbound_id
                min_outbound_pos = outbound_pos
                min_outbound_wait = outbound_wait

        return (min_outbound_id, min_outbound_pos, min_outbound_wait)

    def get_nearest_inbound(self):
        inbound_station = self.black_board.read((200, Data.Inbound))
        # pick_id = random.randint(0, len(pick_point)-1)
        min_dist = 10000
        min_inbound_id = -1
        min_inbound_pos = None
        min_inbound_wait = -1
        start = self.last_task[1]
        start_node = self.planner.pos_to_node(start)
        for inbound_id, inbound_pos, inbound_wait, inbound_load in inbound_station:
            target = self.planner.pos_to_node(inbound_pos)
            dist = self.planner.plan_distance(start_node, target)
            if dist <= min_dist:
                min_dist = dist
                min_inbound_id = inbound_id
                min_inbound_pos = inbound_pos
                min_inbound_wait = inbound_wait

        return (min_inbound_id, min_inbound_pos, min_inbound_wait)

    def get_inbound_state(self, inbound_id):
        print("inbount id", inbound_id)
        inbound_station = self.black_board.read((200, Data.Inbound))
        print("Inbound station", inbound_station)
        inbound_idx: int = -1
        for i in range(len(inbound_station)):
            if inbound_station[i][0] == inbound_id:
                inbound_idx = i
        return (
            inbound_id,
            inbound_station[inbound_idx][1],
            inbound_station[inbound_idx][2],
        )

    """
    All robot state change happen here
    """

    def robot_state_machine(self):
        self.target_battery = self.black_board.read(
            (self.global_key(), Data.CS_Battery_level)
        ) + 3 * self.black_board.read((self.global_key(), Data.Charge_Time))

        print(
            f"RID: {self.r_id} Set Goal {self.set_goal} Status {self.status} current Task {self.current_task}"
        )
        if self.current_task == Task.WaitForTask:
            self.assign_task = self.black_board.read(
                (self.global_key(), Data.Assign_Task)
            )
            # print(f"RID:{self.r_id}, AT:{self.assign_task}, Current_Task:{self.current_task}")
            if not self.assign_task:
                return
            self.assign_task = False
            if self.status == Status.Parked:
                self.current_task = self.black_board.read(
                    (self.global_key(), Data.Assigned_Task)
                )
            elif self.status == Status.InOutboundStation:
                self.current_task = self.black_board.read(
                    (self.global_key(), Data.Assigned_Task)
                )
            self.black_board.write(
                (self.global_key(), Data.Assign_Task), self.assign_task
            )

        elif self.set_goal == False and self.status == Status.ReachGoal:
            if self.current_task == Task.InboundStation:
                self.status = Status.Loading
            elif self.current_task == Task.Inbound:
                self.status = Status.Unloading
            elif self.current_task == Task.OutboundStation:
                self.status = Status.Unloading
            elif self.current_task == Task.Outbound:
                self.status = Status.Loading
            elif self.current_task == Task.Outbound_Task:
                self.status = Status.Loading
            elif self.current_task == Task.ParkStation:
                self.status = Status.Parked
            elif self.current_task == Task.ChargingStation:
                self.status = Status.Chargig
                self.black_board.write(
                    (self.global_key(), Data.Battery_Mode),
                    Mode_Of_Operation.Charge_Mode,
                )

        elif (
            self.set_goal == False
            and self.status == Status.Chargig
            and self.black_board.read((self.global_key(), Data.Battery))
            >= min(99, self.target_battery - 1)
        ):
            self.black_board.write((self.global_key(), Data.Assign_Task), False)
            Charge_Station_id = self.black_board.read(
                (self.global_key(), Data.Charge_Station_Robot)
            )
            self.black_board.write(
                (500 + Charge_Station_id, Data.Charge_Station_Mode),
                Charge_Station_Mode.Free,
            )
            self.current_task = Task.WaitForTask
            self.black_board.write(
                (self.global_key(), Data.Assigned_Task), self.current_task
            )
            self.status = Status.Parked
            self.last_task = self.inbound_task_list.pop(0)
            self.inbound_load_task.pop(0)
            last_task_start_time = self.inbound_task_start_time.pop(0)
            self.black_board.write((self.global_key(), Status), self.status)

        elif (
            self.set_goal == False
            and self.current_task == Task.ParkStation
            and (self.status == Status.InOutboundStation)
        ):
            self.inbound_task_list.append(
                [-5, Point(self.park_pos[1].x, self.park_pos[1].y)]
            )
            self.inbound_load_task.append(True)
            self.inbound_task_start_time.append(time.time())
            self.task_id = -5
            self.set_goal = True
            self.status = Status.Planning

        elif (
            self.set_goal == False
            and self.current_task == Task.ChargingStation
            and (not self.status == Status.ReachGoal)
            and (not self.status == Status.Chargig)
        ):
            self.charging_station = self.black_board.read_all(
                (200, Data.Charging_station)
            )
            battery = self.black_board.read((self.global_key(), Data.Battery))
            Charge_Station_id = self.black_board.read(
                (self.global_key(), Data.Charge_Station_Robot)
            )
            # print(Charge_Station_id)
            charging_station = self.black_board.read_all(
                (200, Data.Charge_Station_Mode)
            )  # idx = 0

            self.inbound_task_list.append(
                [-2, self.charging_station[0][Charge_Station_id - 1][1]]
            )
            self.inbound_load_task.append(True)
            # ? For Viualization purpose
            self.task_id = -2
            self.set_goal = True
            self.status = Status.Planning
            self.inbound_task_start_time.append(time.time())

        elif (
            self.set_goal == False
            and self.current_task == Task.InboundStation
            and (self.status in [Status.Parked, Status.InOutboundStation])
        ):
            inbound_id = self.black_board.read(
                (self.global_key(), Data.Robot_Inbound_Station)
            )
            if inbound_id == -1:
                print("No Task Assigned")
                return
            (min_inbound_id, min_inbound_pos, min_inbound_wait) = (
                self.get_inbound_state(inbound_id)
            )
            self.inbound_task_list.append([min_inbound_id, min_inbound_pos])
            self.inbound_load_task.append(True)
            self.inbound_task_start_time.append(time.time())

            self.robot_inbound_station = min_inbound_id
            self.task_id = min_inbound_id + 1
            self.set_goal = True
            self.inbound_load_count = min_inbound_wait
            self.status = Status.Planning

        elif self.set_goal == False and self.status == Status.Unloaded:
            print("Waiting for PickGoal")
            if self.current_task == Task.Inbound:
                self.last_goal = self.goal
                if len(self.inbound_task_list) > 0:
                    self.last_task = self.inbound_task_list.pop(0)
                    self.inbound_load_task.pop(0)
                    last_task_start_time = self.inbound_task_start_time.pop(0)
                    self.number_of_inbound_task -= 1
                    task_logger.info(
                        f",RID,{self.r_id},Inbound:{self.number_of_inbound_task} Outbound:{self.number_of_outbound_task} Current_Outbound:{self.current_outbound_count} InboundTask,{self.last_task[0]},Pos,{self.last_task[1]},Start_time,{last_task_start_time},End_time,{time.time()},"
                    )
                    self.inbound_task_completed = True
                else:
                    self.last_task = [-1, Point(self.x_pos, self.y_pos)]

                inbound_count1 = self.black_board.read(
                    (self.global_key(), Data.Number_of_Inbound_Task)
                )
                outbound_count1 = self.black_board.read(
                    (self.global_key(), Data.Number_of_Outbound_Task)
                )
                available_slot1 = min(1, 1 - (inbound_count1 + outbound_count1))

                # all task completed
                if len(self.inbound_task_list) == 0:
                    if available_slot1 <= 2:
                        (min_outbound_id, min_outbound_pos, min_outbound_wait) = (
                            self.get_nearest_outbound()
                        )
                        self.inbound_task_list.append(
                            [min_outbound_id, min_outbound_pos]
                        )
                        self.inbound_load_task.append(True)
                        self.inbound_task_start_time.append(time.time())
                        self.task_id = min_outbound_id
                        self.set_goal = True
                        self.current_task = Task.OutboundStation
                        self.outbound_load_count = min_outbound_wait
                        self.status = Status.Planning
                    else:
                        self.set_goal = False
                        self.status = Status.WaitForLoadOutbound

                else:
                    self.task_id = self.inbound_task_list[0][0]
                    self.set_goal = True
                    self.current_task = Task.Inbound
                    self.status = Status.Planning

            elif self.current_task == Task.OutboundStation:
                self.last_goal = self.goal
                if len(self.inbound_task_list) > 0:
                    self.last_task = self.inbound_task_list.pop(0)
                    self.inbound_load_task.pop(0)
                #! Check if there is a bug in the system due to this???
                elif len(self.outbound_load_task_list) > 0:
                    self.last_task = self.outbound_load_task_list.pop(0)
                    self.outbound_load_load_task.pop(0)

                # all task completed
                if (
                    len(self.inbound_task_list) == 0
                    and len(self.outbound_load_task_list) == 0
                ):
                    self.status = Status.InOutboundStation
                    self.current_task = Task.WaitForTask

        elif self.set_goal == False and self.status == Status.Loaded:
            print("Waiting for Inbound Task")
            if self.current_task == Task.InboundStation:
                # drop_point = self.black_board.read((200, Data.Drop))
                self.last_goal = self.goal
                self.last_task = self.inbound_task_list.pop(0)
                self.inbound_load_task.pop(0)
                self.inbound_task_start_time.pop(0)

                self.inbound_task_list = [self.last_task]
                self.inbound_load_task = [False]
                self.inbound_task_start_time = [time.time()]

                self.inbound_task = []
                self.status = Status.WaitForInboundTask

            elif self.current_task == Task.Outbound:
                # mark the outbound task
                if len(self.outbound_task_list) > 0:
                    self.last_task = self.outbound_task_list.pop(0)
                    self.outbound_load_task.pop(0)
                    last_task_start_time = self.outbound_task_start_time.pop(0)
                    self.current_outbound_count += 1
                    task_logger.info(
                        f",RID,{self.r_id},Inbound:{self.number_of_inbound_task} Outbound:{self.number_of_outbound_task} Current_Outbound:{self.current_outbound_count} OutBoundTask,{self.last_task[0]},Pos,{self.last_task[1]},Start_time,{last_task_start_time},End_time,{time.time()},"
                    )
                    self.outbound_task_completed = True
                    # all task completed
                if len(self.outbound_task_list) == 0:
                    self.task_id = self.inbound_task_list[0][0]
                    self.goal = self.inbound_task_list[0][1]
                    self.current_task = self.last_current_task
                    self.status = Status.MoveToGoal
                    self.set_goal = True
                else:
                    self.task_id = self.outbound_task_list[0][0]
                    self.goal = self.outbound_task_list[0][1]
                    self.set_goal = True
                    self.current_task = Task.Outbound
                    self.status = Status.MoveToGoal

        elif self.set_goal == False and self.current_task == Task.Outbound_Task:

            if len(self.outbound_load_task_list) > 0:
                self.last_task = self.outbound_load_task_list.pop(0)
                self.outbound_load_load_task.pop(0)
                last_task_start_time = self.outbound_load_task_start_time.pop(0)
                self.current_outbound_count += 1
                task_logger.info(
                    f",RID,{self.r_id},Inbound:{self.number_of_inbound_task} Outbound:{self.number_of_outbound_task} Current_Outbound:{self.current_outbound_count} OutBound_Task,{self.last_task[0]},Pos,{self.last_task[1]},Start_time,{last_task_start_time},End_time,{time.time()}"
                )
                self.outbound_load_task_completed = True
                # all task completed
            if len(self.outbound_load_task_list) == 0:
                ##* DOne
                (min_outbound_id, min_outbound_pos, min_outbound_wait) = (
                    self.get_nearest_outbound()
                )
                self.inbound_task_list.append([min_outbound_id, min_outbound_pos])
                self.inbound_load_task.append(True)
                self.inbound_task_start_time.append(time.time())

                self.task_id = min_outbound_id
                self.set_goal = True
                self.current_task = Task.OutboundStation
                self.outbound_load_count = min_outbound_wait
                self.status = Status.Planning

            else:
                self.task_id = self.outbound_load_task_list[0][0]
                self.set_goal = True
                self.current_task = Task.Outbound_Task
                self.status = Status.Planning

            # print(self.full_path)
        elif self.set_goal == False and self.status == Status.WaitForInboundTask:
            self.assign_inbound = self.black_board.read(
                (self.global_key(), Data.Assign_Inbound)
            )

            if not self.assign_inbound:
                return

            self.inbound_task = self.black_board.read((self.global_key(), Data.Pick))
            self.number_of_inbound_task = 0
            for task in self.inbound_task:
                self.inbound_task_list.append(task)
                self.inbound_load_task.append(True)
                self.inbound_task_start_time.append(time.time())
                self.number_of_inbound_task += 1

            self.last_task = self.inbound_task_list.pop(0)
            self.inbound_load_task.pop(0)
            self.inbound_task_start_time.pop(0)

            self.current_task = Task.Inbound
            self.set_goal = True
            self.status = Status.Planning
            self.task_id = self.inbound_task_list[0][0]
            self.assign_inbound = False
            self.black_board.write(
                (self.global_key(), Data.Assign_Inbound), self.assign_inbound
            )

        #!  Changes for the load balancing task
        elif self.set_goal == False and self.status == Status.WaitForLoadOutbound:

            self.load_task = self.black_board.read(
                (self.global_key(), Data.Load_Outbound)
            )
            print("Outbound_load_task", self.load_task)
            # self.number_of_outbound_task= 0
            if self.load_task is not None:
                if len(self.load_task):
                    for task in self.load_task:
                        self.outbound_load_task_list.append(task)
                        self.outbound_load_load_task.append(True)
                        self.outbound_load_task_start_time.append(time.time())
                        self.number_of_outbound_task += 1

                    if len(self.outbound_load_task_list) > 0:

                        self.current_task = Task.Outbound_Task
                        self.set_goal = True
                        self.status = Status.Planning
                        self.task_id = self.outbound_load_task_list[0][0]

            else:
                (min_outbound_id, min_outbound_pos, min_outbound_wait) = (
                    self.get_nearest_outbound()
                )
                self.inbound_task_list.append([min_outbound_id, min_outbound_pos])
                self.inbound_load_task.append(True)
                self.inbound_task_start_time.append(time.time())
                self.task_id = min_outbound_id
                self.set_goal = True
                self.current_task = Task.OutboundStation
                self.outbound_load_count = min_outbound_wait
                self.status = Status.Planning

        elif self.set_goal == True and self.status == Status.WaitForOutboundTask:
            print(f"RID: {self.r_id}, Status {self.status}, Task {self.current_task}")
            self.assign_outbound = self.black_board.read(
                (self.global_key(), Data.Assign_Outbound)
            )
            if not self.assign_outbound:
                return

            outbound_task = self.black_board.read((self.global_key(), Data.Pick))
            if outbound_task is None:
                return

            for task in outbound_task:
                self.outbound_task_list.append(task)
                self.outbound_load_task.append(True)
                self.outbound_task_start_time.append(time.time())
                self.number_of_outbound_task += 1

            print(f"Path:{self.path_node} outboundTask:{self.outbound_task_list}")

            if len(outbound_task) > 0:
                self.last_current_task = self.current_task
                self.current_task = Task.Outbound
                self.task_id = self.outbound_task_list[0][0]
                self.goal = self.outbound_task_list[0][1]

            self.status = Status.MoveToGoal
            self.assign_outbound = False

        elif self.set_goal == False and self.status == Status.Loading:
            # print(f"Waiting for {self.status} {self.wait_count}")
            if self.current_task == Task.Outbound:
                self.wait_count += 1
                if self.wait_count > self.load_count:
                    self.wait_count = 0
                    self.status = Status.Loaded
            elif self.current_task == Task.InboundStation:
                self.wait_count += 1
                if self.wait_count > self.inbound_load_count:
                    self.wait_count = 0
                    self.status = Status.Loaded

            elif self.current_task == Task.Outbound_Task:
                self.wait_count += 1
                if self.wait_count > self.load_count:
                    self.wait_count = 0
                    self.status = Status.Loaded

        elif self.set_goal == False and self.status == Status.Unloading:
            if self.current_task == Task.Inbound:
                # print(f"Waiting for {self.status} {self.wait_count}")
                self.wait_count += 1
                if self.wait_count > self.unload_count:
                    self.wait_count = 0
                    self.status = Status.Unloaded
            elif self.current_task == Task.OutboundStation:
                self.wait_count += 1
                if self.wait_count > (self.outbound_load_count):
                    self.wait_count = 0
                    self.status = Status.Unloaded
                    self.number_of_outbound_task = 0
                    self.current_outbound_count = 0
        elif self.set_goal == True and self.status == Status.Planning:
            start = self.last_task[-1]
            source_node = f"{int(start.x)},{int(start.y)}"

            if self.current_task == Task.Outbound_Task:
                self.goal = self.outbound_load_task_list[0][1]
                self.final_goal = self.outbound_load_task_list[0][1]
            else:
                self.goal = self.inbound_task_list[0][1]
                self.final_goal = self.inbound_task_list[0][1]
            target_node = f"{int(self.goal.x)},{int(self.goal.y)}"
            self.get_full_path()
            self.path_node = []
            target_node = self.node_to_pos(target_node)
            for i in range(len(self.full_path)):
                full_node = self.full_path[i]
                # full_path_node = self.node_to_pos(full_node)
                self.path_node.append(full_node)
                if full_node[0] == target_node[0] and full_node[1] == target_node[1]:
                    break

            # self.print_full_path()
            # self.print_path()
            self.curr_index = 0
            self.goal_index = 0
            # self.get_subgoal()
            if len(self.path_node) > 0:
                self.status = Status.MoveToGoal
                self.path_node_reservation = [True for i in range(len(self.path_node))]
                # if self.current_task == Task.Inbound:
                #     self.status = Status.WaitForOutboundTask
                # else:
                #     self.status = Status.MoveToGoal

            self.got_new_plan = True
            self.black_board.write((self.global_key(), Data.Path), self.path_node)

        elif self.set_goal == True and self.status == Status.MoveToGoal:
            if self.goal is None:
                return
            # check for goal reach
            dist_squared = (self.goal.x - self.x_pos) ** 2 + (
                self.goal.y - self.y_pos
            ) ** 2
            dist = math.sqrt(dist_squared)
            if dist < 0.15:
                print("Goal reached")
                if self.current_task == Task.OutboundStation:
                    self.load_task = []
                self.status = Status.ReachGoal
                self.set_goal = False
        elif self.set_goal == False and self.status == Status.Parked:
            self.last_goal = self.goal
            if len(self.inbound_task_list) > 0:
                self.last_task = self.inbound_task_list.pop(0)
                self.inbound_load_task.pop(0)
            # print("Parked waiting for task")
            self.current_task = Task.WaitForTask

        else:
            print("unknown state")

        # self.black_board.write((self.global_key(), Data.Pick), self.inbound_task)
        # self.black_board.write((self.global_key(), Data.Assign_Task), self.assign_task)
        # self.black_board.write((self.global_key(), Data.Robot_Inbound_Station), self.robot_inbound_station)
        self.black_board.write((self.global_key(), Data.Task_ID), self.task_id)
        self.black_board.write(
            (self.global_key(), Data.Assign_Outbound), self.assign_outbound
        )
        self.black_board.write(
            (self.global_key(), Data.Number_of_Inbound_Task),
            self.number_of_inbound_task,
        )
        self.black_board.write(
            (self.global_key(), Data.Number_of_Outbound_Task),
            self.number_of_outbound_task,
        )
        self.black_board.write((self.global_key(), Data.TaskStatus), self.current_task)

        self.black_board.write((self.global_key(), Data.RobotStatus), self.status)

    def print_task(self):
        logger.info("==========")
        for task_id, task_pos in self.inbound_task_list:
            logger.info(f"Task ID:{task_id} Pos:{task_pos}")
        logger.info("==========")

    def print_full_path(self):
        logger.info(f"Robot ID:{self.r_id} FullPath:{self.full_path}")

    def print_path(self):
        logger.info(f"Robot ID:{self.r_id} Path:{self.path_node}")

    """
    From full path get all node
    """

    def get_subgoal(self):
        if self.path_node is not None:
            if len(self.path_node) >= 1:
                self.subgoal = []
                last_theta = self.disc_theta(self.theta)
                curr_theta = self.disc_theta(self.theta)

                for i in range(self.curr_index, len(self.path_node) - 1):
                    curr_node = self.path_node[i]
                    if i == self.goal_index:
                        self.subgoal.append(curr_node)
                        break

                    next_node = self.path_node[i + 1]

                    if (
                        abs(self.goal.x - curr_node[0])
                        + abs(self.goal.y - curr_node[1])
                    ) == 0:
                        self.subgoal.append(curr_node)
                        break

                    if (i + 1 <= self.goal_index) and (curr_node[2] != next_node[2]):
                        self.subgoal.append(next_node)
                        break

                    if (i + 2 < len(self.path_node)) and (i + 1 <= self.goal_index):
                        next_next_node = self.path_node[i + 2]
                        if curr_node == next_next_node:
                            self.subgoal.append(next_node)
                            break

                if self.goal_index == len(self.path_node) - 1:
                    self.subgoal.append(self.path_node[-1])

            # logger.info(f"Robot {self.r_id} and subgoal {self.subgoal}  -> curr_idx {self.curr_index} , goal idx {self.goal_index}")

    """
    Update Robot position based on velocity
    """

    def update_position(self):
        if self.x_vel is not None:
            self.x_pos += self.x_vel * self.dt
            self.y_pos += self.y_vel * self.dt
            self.theta += self.theta_vel * self.dt

            self.black_board.write(
                (self.global_key(), Data.Pos), Point(self.x_pos, self.y_pos)
            )
            self.black_board.write((self.global_key(), Data.Theta), self.theta)

    def check_reservation(self):
        if (self.goal_index is None) or (self.curr_index is None):
            return

        safe_node = min(self.safety_node, (len(self.path_node) - self.curr_index - 1))

        if (self.goal_index - self.curr_index) >= safe_node:
            self.reservation = Reservation.Move
        else:
            self.reservation = Reservation.Wait

    def update_curr_idx(self):
        if self.curr_index is None:
            return
        if self.curr_index + 1 < len(self.path_node):
            next_node = self.path_node[self.curr_index + 1]
            dist = abs(self.x_pos - next_node[0]) + abs(self.y_pos - next_node[1])
            theta_dist2 = 10.0
            if next_node[2] == 0:
                theta_dist2 = min(abs(self.theta), abs(self.theta - 3.14))
                theta_dist2 = min(theta_dist2, abs(self.theta + 3.14))
            elif next_node[2] == 1:
                theta_dist2 = min(abs(self.theta - 1.57), abs(self.theta + 1.57))
            elif next_node[2] == 2:
                theta_dist2 = min(abs(self.theta - 3.14), abs(self.theta + 3.14))
            elif next_node[2] == 3:
                theta_dist2 = abs(self.theta + 1.57)

            if dist < 0.2 and theta_dist2 < 0.1:
                self.curr_index += 1

    def update_curr_index(self):
        if self.curr_index is None:
            return

        curr_node = self.path_node[self.curr_index]
        dist_squared = (self.x_pos - curr_node[0]) ** 2 + (
            self.y_pos - curr_node[1]
        ) ** 2
        dist = math.sqrt(dist_squared)

        theta_dist = 10.0

        if curr_node[2] == 0:
            theta_dist = abs(self.theta)
        elif curr_node[2] == 1:
            theta_dist = abs(self.theta - 1.57)
        elif curr_node[2] == 2:
            theta_dist = min(abs(self.theta - 3.14), abs(self.theta + 3.14))
        elif curr_node[2] == 3:
            theta_dist = abs(self.theta + 1.57)

        if dist >= (0.8) or (theta_dist > 0.78):
            if self.goal_index >= (self.curr_index + 1) and (
                len(self.path_node) >= (self.curr_index + 1)
            ):
                next_node = self.path_node[self.curr_index + 1]
                dist_squared = (self.x_pos - next_node[0]) ** 2 + (
                    self.y_pos - next_node[1]
                ) ** 2
                dist2 = math.sqrt(dist_squared)
                theta_dist2 = 10.0
                if next_node[2] == 0:
                    theta_dist2 = abs(self.theta)
                elif next_node[2] == 1:
                    theta_dist2 = abs(self.theta - 1.57)
                elif next_node[2] == 2:
                    theta_dist2 = min(abs(self.theta - 3.14), abs(self.theta + 3.14))
                elif next_node[2] == 3:
                    theta_dist2 = abs(self.theta + 1.57)
                if (dist2 <= dist) and (theta_dist2 <= 0.2):
                    self.curr_index += 1

    """
    Move the robot
    """

    def controller(self):
        if self.status == Status.MoveToGoal and self.reservation == Reservation.Move:
            #
            if len(self.subgoal) == 0:
                return
            # print(self.subgoal)
            current_goal_x = self.subgoal[0][0]
            current_goal_y = self.subgoal[0][1]
            current_theta = self.subgoal[0][2]

            if current_theta == 0:
                current_theta = 0.0
            elif current_theta == 1:
                current_theta = 1.57
            elif current_theta == 2:
                current_theta = 3.14
            elif current_theta == 3:
                current_theta = -1.57

            rotation = self.subgoal[0][1]

            dist_squared = (self.x_pos - current_goal_x) ** 2 + (
                self.y_pos - current_goal_y
            ) ** 2
            dist = math.sqrt(dist_squared)

            if dist > 0.1:
                self.theta_vel = 0.0

                diff_x = current_goal_x - self.x_pos
                diff_y = current_goal_y - self.y_pos

                direction_theta = math.atan2(diff_y, diff_x)

                self.x_vel = self.linear_speed * math.cos(direction_theta)
                self.y_vel = self.linear_speed * math.sin(direction_theta)

                self.on_rotation = False
                # print(f"VelX{self.x_vel}, VelY{self.y_vel}")
            elif min(
                abs(current_theta - self.theta), abs(current_theta + self.theta)
            ) > 0.01 and (rotation):

                self.x_vel = 0.0
                self.y_vel = 0.0

                if abs(current_theta - self.theta) > abs(current_theta + self.theta):
                    self.theta = -1.0 * self.theta

                if (current_theta - self.theta) > 0.0:
                    self.theta_vel = self.angular_speed
                else:
                    self.theta_vel = -self.angular_speed

                self.x_pos = round(self.x_pos)
                self.y_pos = round(self.y_pos)

                self.on_rotation = True
                # print(f"Desired theta{current_theta} and Current theta {self.theta} -> Theta_vel {self.theta_vel}")
            else:
                self.x_vel = 0.0
                self.y_vel = 0.0
                self.theta_vel = 0.0

                if rotation:
                    # self.theta = self.subgoal[0][0][2]
                    self.theta = current_theta

                self.on_rotation = False
                print(
                    f"Subgoal Reached xPos:{self.x_pos}, yPos:{self.y_pos}, theta:{self.theta}"
                )
                print(self.subgoal.pop(0))

            # update in blackboard
            # self.black_board.write((self.global_id,Data.Vel), Point(self.x_vel, self.y_vel))

        elif self.status == Status.ReachGoal:
            self.x_vel = 0.0
            self.y_vel = 0.0
            self.theta_vel = 0.0

        elif self.reservation == Reservation.Wait:
            self.x_vel = 0.0
            self.y_vel = 0.0
            self.theta_vel = 0.0

    def reset_task(self):
        self.inbound_task_completed = False
        self.outbound_task_completed = False
        self.outbound_load_task_completed = False

    def update_battery_state(self):
        if self.status == Status.Parked:
            self.battery.mode = Mode_Of_Operation.Queue_Mode
            self.black_board.write(
                (self.global_key(), Data.Battery_Mode), self.battery.mode
            )

        elif self.status == Status.Chargig:
            self.battery.mode = Mode_Of_Operation.Charge_Mode
            self.black_board.write(
                (self.global_key(), Data.Battery_Mode), self.battery.mode
            )

        else:
            self.battery.mode = Mode_Of_Operation.Work_Mode
            self.black_board.write(
                (self.global_key(), Data.Battery_Mode), self.battery.mode
            )

    """
    Behaviour of robot
    """

    def behaviour(self):
        if self.initialized == False:
            return
        self.reset_task()
        self.robot_state_machine()
        self.check_reservation()
        # self.update_curr_index()
        self.update_curr_idx()
        self.get_subgoal()
        self.controller()
        self.update_position()
        self.battery.behaviour()
        self.update_battery_state()

    """
    Event behaviour of robot
    """

    def event_behaviour(self):

        if self.initialized == False:
            self.get_initialize()
        else:
            pass
