from itertools import count
from logging import critical
from xml.dom.minidom import CharacterData
from operator import itemgetter
from matplotlib.style import available
from black_board import *
from data import *
import random
from task_planner import *
import xml.etree.ElementTree as ET
import re
from rule_based_approach import *
from bms import *
from geometry import *
from collections import *
import logging
import copy

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

formater = logging.Formatter("%(asctime)s:%(name)s:%(message)s")

file_handler = logging.FileHandler("logger/robot.log")
file_handler.setLevel(logging.DEBUG)
file_handler.setFormatter(formater)

logger.addHandler(file_handler)

task_logger = logging.getLogger(__name__)
task_logger.setLevel(logging.DEBUG)


task_file_handler = logging.FileHandler("logger/load_balancing.log")
task_file_handler.setLevel(logging.DEBUG)
task_file_handler.setFormatter(formater)

task_logger.addHandler(task_file_handler)

# task_logger.info(f",RID,{self.r_id}, ")


class EdgeData:
    def __init__(self, id, zone_id, capacity, start, end, task_id):
        self.id = id
        self.capacity = capacity
        self.start = start
        self.end = end
        self.task_id = task_id
        self.zone_id = zone_id
        self.available_task = []

    def getattr(self, attr):
        """
        Get the value of the attribute attr.
        """
        return self.__dict__[attr]

    def setattr(self, attr, value):
        """
        Set the value of the attribute attr to value.
        """
        self.__dict__[attr] = value

    def increment_capacity(self, value):
        self.capacity += value

    def print_(self):
        print("id of the edge", self.id)
        print("zone id of the edge", self.zone_id)
        print("capacity of the edge", self.capacity)
        print("start of the edge", self.start)
        print("end of the edge", self.end)
        print("task_id of the edge", self.task_id)
        print("available task of the edge", self.available_task)


class EdgeComputation:
    """
    @method: run() copmutes the edges for the layout
    @comment : Computes the edges in the layout and assign the zones to the edges.
    """

    def __init__(self):
        # self.black_board = black_board
        self.task_file = "./config/research_layout_2/research_layout.xml"
        self.task_visual_file = "./config/research_layout_2/task.xml"
        self.task_points = []
        self.edges = []
        self.zones = {}
        self.current_capacity = 0
        self.capacity = 0
        # self.aisle = self.edges
        self.run()

    def run(self):
        self.load_task_position()
        self.load_task_visual()
        self.compute_edges()
        self.edges_to_zone()
        self.print_zone_edges()
        for i in self.edges:
            i.print_()
        # self.print_edges()
        # self.update_edge_capacity()

    def pos_to_node(self, x, y):
        node = f"{int(x)},{int(y)}"
        return str(node)

    def node_to_pos(self, node):
        data = re.split(r",", node)
        pos = (int(data[0]), int(data[1]))
        return pos

    def load_task_visual(self):
        """
        To load the task from the task visual file
        """
        self.task_visual = []

        tree = ET.parse(self.task_visual_file)
        root = tree.getroot()

        task_point = root.find("Task_Station")

        for task in task_point:
            id_ = task.find("id")
            pt = task.find("pt")
            x = pt.find("x")
            y = pt.find("y")
            # wait = task.find('wait')
            self.task_visual.append([int(id_.text), float(x.text), float(y.text)])

        # self.black_board.write((self.global_key(), Data.Task), self.task_points)

    def load_task_position(self):
        """
        To load the task position from the task file
        """
        self.task_points = []

        tree = ET.parse(self.task_file)
        root = tree.getroot()

        task_point = root.find("task_station")

        for task in task_point:
            id_ = task.find("id")
            pt = task.find("pt")
            x = pt.find("x")
            y = pt.find("y")
            zone_id = task.find("zone_id")
            zone_value = 1
            if zone_id is not None:
                zone_value = int(zone_id.text)
            # wait = task.find('wait')
            self.task_points.append(
                [int(id_.text), float(x.text), float(y.text), zone_value]
            )

        # self.black_board.write((self.global_key(), Data.Task), self.task_points)

    def order_of_edges(self):
        """
        @comments: This will return the orientation of the aisles in the layout, so that we can compute the edges
        #!!!!!!!!!!This can also be done using the graph file, which makes it more robust.!!!!!!!!!!!!!!!!!!!!!!!!
        """

        order_edges = []
        for i in self.task_points:
            if self.task_points[0][1] == i[1] and self.task_points[0][2] == i[2]:
                order_edges.append(i)

        task_order = []
        # nmber_of_task = len(order_edges)
        for i in self.task_visual:
            for j in order_edges:
                if i[0] == j[0]:
                    task_order.append(i)
        count_x = 0
        count_y = 0
        for i in task_order:
            if task_order[0][1] == i[1]:
                count_x += 1
            elif task_order[0][2] == i[2]:
                count_y += 1
        if count_x > count_y:
            return False
        else:
            return True

    def compute_edges(self):
        """
        This function will compute all the edges in the layout, here edge means the aisle, where robot have to pick/place the totes.
        """

        self.edges = []
        order = self.order_of_edges()
        if order:

            self.task_points.sort(key=lambda x: x[1])
            edge = []
            x_factor = self.task_points[0][1]
            count = 0
            zone = self.task_points[0][3]
            for i in self.task_points:
                if i[1] == x_factor:
                    tasks = []
                    edge.append(i)
                    if i == self.task_points[-1]:
                        for j in edge:
                            tasks.append(j[0])
                        start_node = self.pos_to_node(edge[0][1], edge[0][2])
                        end_node = self.pos_to_node(edge[-1][1], edge[-1][2])
                        self.edges.append(
                            EdgeData(
                                id=count,
                                zone_id=zone,
                                capacity=0,
                                start=start_node,
                                end=end_node,
                                task_id=tasks,
                            )
                        )
                else:
                    tasks = []
                    for j in edge:
                        tasks.append(j[0])
                    start_node = self.pos_to_node(edge[0][1], edge[0][2])
                    end_node = self.pos_to_node(edge[-1][1], edge[-1][2])
                    self.edges.append(
                        EdgeData(
                            id=count,
                            zone_id=zone,
                            capacity=0,
                            start=start_node,
                            end=end_node,
                            task_id=tasks,
                        )
                    )
                    edge = []
                    edge.append(i)
                    x_factor = i[1]
                    count += 1
                    zone = i[3]

        else:
            self.task_points.sort(key=lambda x: x[2])
            edge = []
            y_factor = self.task_points[0][2]
            count = 0
            zone = self.task_points[0][3]
            for i in self.task_points:
                if i[2] == y_factor:
                    edge.append(i)

                    if i == self.task_points[-1]:
                        for j in edge:
                            tasks.append(j[0])
                        start_node = self.pos_to_node(edge[0][1], edge[0][2])
                        end_node = self.pos_to_node(edge[-1][1], edge[-1][2])
                        self.edges.append(
                            EdgeData(
                                id=count,
                                zone_id=zone,
                                capacity=0,
                                start=start_node,
                                end=end_node,
                                task_id=tasks,
                            )
                        )

                else:
                    tasks = []
                    for j in edge:
                        tasks.append(j[0])
                    start_node = self.pos_to_node(edge[0][1], edge[0][2])
                    end_node = self.pos_to_node(edge[-1][1], edge[-1][2])
                    self.edges.append(
                        EdgeData(
                            id=count,
                            zone_id=zone,
                            capacity=0,
                            start=start_node,
                            end=end_node,
                            task_id=tasks,
                        )
                    )
                    edge = []
                    edge.append(i)
                    x_factor = i[2]
                    count += 1
                    zone = i[3]

    def find_edge_id_for_task(self, task_id):
        """
        This will find the edge id for a given task id
        """
        for i in self.edges:
            if task_id in i.task_id:
                return i.id

        return None

    def update_edge_capacity(self, total_outbound_task):
        """
        For updating the edge capacity in real time as the task list is updated.
        """
        # self.print_zone_edges()
        self.available_outbound_task = []
        self.current_capacity = 0
        for i in self.edges:
            i.available_task = []
            i.capacity = 0

        for k in total_outbound_task.keys():
            for j in total_outbound_task[k]:
                if j[1] == False:
                    self.available_outbound_task.append(j)
                    self.current_capacity += 1
        self.capacity = self.current_capacity

        for l in self.available_outbound_task:
            edge_id = self.find_edge_id_for_task(l[0])
            if edge_id is None:
                continue

            self.edges[edge_id].increment_capacity(1)
            self.edges[edge_id].available_task.append(l)

        #!!!!!!!!!!!!! logger changes!!!!!!!!!!!!!!!!!
        # task_logger.info(f",Available Task,{self.available_outbound_task},  current capacity,{self.current_capacity}")
        # for i in self.edges:
        # task_logger.info(f"Edge {i.id}, {i.available_task}, {i.capacity}")

    def edges_to_zone(self):
        """
        This will get the zone id for each edge in the layout and return a dictionary containing the zone id as key and the edge id(s) as value.
        """
        edge = []
        zone_id = self.edges[0].zone_id
        for i in self.edges:
            if zone_id == i.zone_id:
                edge.append(i.id)
                if i == self.edges[-1]:
                    self.zones[zone_id] = edge

            else:
                self.zones[zone_id] = edge
                edge = []
                zone_id = i.zone_id
                edge.append(i.id)

    def print_zone_edges(self):
        """
        This will print the zone id and the edge id(s) for each zone in the layout.
        """
        for i in self.zones.keys():
            print(i, self.zones[i])


class Load_Balancing:
    """
    Containts all the parameters for the load balancing algorithm, and the functions for the algorithm.
    """

    def __init__(
        self,
        gid,
        mid,
        global_black_board,
        global_event_handler,
        edges,
        zones,
        wms,
        capacity,
    ):
        self.global_id = gid
        self.module_id = mid
        self.blackboard = global_black_board
        self.event_handler = global_event_handler
        self.load = None
        self.update_time = None
        self.number_of_robot = None
        self.edges = edges
        self.zones = zones
        self.threshold = 10
        self.wms = wms
        self.capacity = capacity
        # self.outbound = self.Outbound()

    def getattr(self, attr):
        """
        Get the value of the attribute attr.
        """
        return self.__dict__[attr]

    def setattr(self, attr, value):
        """
        Set the value of the attribute attr to value.
        """
        self.__dict__[attr] = value

    def update_load(self):
        """
        Update the task load
        """
        inbound_task = self.blackboard.read((400, Data.Inbound_Completed))
        # outbound_task = self.blackboard.read((400, Data.Outbound_Completed))
        outbound_task = self.capacity
        self.load = inbound_task - outbound_task
        return self.load

    def compute_threshold(self):
        """
        Compute the threshold for the load balancing algorithm.
        """
        # self.threshold = self.load * 0.5
        self.threshold = 10
        return self.threshold

    def get_load(self):
        """
        Get the latest load
        """
        if self.capacity > self.threshold:
            return True
        else:
            return False

    def No_of_robot(self, load=0, number_of_robots=0):
        """
        This will compute the max permissible robot that can be made available for the load balancing algorithm. One method can be the percentage of the total robot available.
        """
        self.number_of_robot = 1
        # return

    def find_zone_id_for_task(self, task_id):
        """
        This will find the zone id for a given task id
        """
        for i in self.edges:
            if task_id[0] in i.task_id:
                return i.zone_id

    def get_edges_task(self, task_id):
        """
        To get the edges lying in the same zone as the task.
        """
        # self.edges
        zone_id = self.find_zone_id_for_task(task_id)
        edges = []
        for i in self.zones[zone_id]:
            edges.append(self.edges[i])
        return edges

    def get_all_edges(self):
        """
        To get all the edges in the layout.
        """
        return self.edges

    def edges_in_order_of_capacity(self, edges):
        """
        To  get the edges in order of decending capacity
        """
        # for i in edges:
        #     self.edges[i]

        y = edges.sort(key=lambda x: x.cap, reverse=True)
        return y

    def assign_task(self, robot, available_slot, task_id=None):

        if task_id == None:
            edges = self.get_all_edges()

        else:
            edges = self.get_edges_task(task_id)

        task_logger.info(
            f",Task Id,{task_id},  edges ,{edges}, available slot ,{available_slot}"
        )
        task = self.assign_outbound(edges, available_slot)
        task_logger.info(f",Task Id,{task_id},  task ,{task} , robot ,{robot}")
        # print("Task that has to be assigned to the robot as per the load balancing",task)
        return task

    def assign_outbound(self, edges, available_slot):
        """
        It will assign the outbound task to the robot.
        """

        task = []

        for i in self.edges:
            while available_slot > 0:
                if i.capacity > 0:
                    # self.assign_task_from_edge
                    task.append(i.available_task[0])
                    i.available_task.pop(0)
                    i.increment_capacity(-1)
                    available_slot -= 1

                else:
                    break
        print("Task that are being assigned are", task)
        return task


class Inbound_State(enum.Enum):
    Assigned = 0
    Free = 1


class WMS:
    """
    Warehoouse Management System - simulate behaviour of Task Flow into the system
    """

    def __init__(self, gid, mid, global_black_board, global_event_handler):
        self.global_id = gid
        self.module_id = mid
        self.black_board = global_black_board
        self.event_handler = global_event_handler
        self.event_queue = None
        self.inbound_file = "./config/research_layout_2/rand_inbound_task.xml"
        self.outbound_file = "./config/research_layout_2/rand_outbound_task.xml"
        self.inbound_buffer = {}
        self.inbound_rand_task = {}
        self.rand_inbound_task = []
        self.rand_outbound_task = {}
        self.inbound_task_count = 0
        self.outbound_task_count = 0
        self.charging_station = []
        self.last_outbound_count = 0
        self.loop_count = 0
        self.setup()
        self.initiate()
        self.initialized = False

    def global_key(self):
        return (self.global_id * 100) + self.module_id

    def setup(self):
        self.black_board.register(self.global_key(), 0, Data.Robot_Member)
        self.black_board.register(self.global_key(), 0, Data.Task)
        self.black_board.register(self.global_key(), 0, Data.Inbound)
        # self.black_board.register(self.global_key(), 1, Data.)

    def initiate(self):
        self.inbound_station = self.black_board.read((200, Data.Inbound))
        # self.outbound_station = self.black_board.read((200, Data.Outbound))

        if self.inbound_station is None:
            return

        self.load_rand_inbound_task()
        self.load_charging()
        # self.load_rand_outbound_task()

        for idx, pos, wait, load in self.inbound_station:
            self.inbound_buffer[idx] = [0, 0, load, Inbound_State.Free, pos]

        self.initialized = True

        self.black_board.write(
            (self.global_key(), Data.Inbound_Station_State), self.inbound_buffer
        )

    """
    Prediocally Load Task for inbound buffer
    """

    def load_inbound(self):
        for idx in self.inbound_buffer:
            if (self.loop_count % self.inbound_buffer[idx][2]) == 0:
                self.inbound_buffer[idx][0] += 1

        self.black_board.write(
            (self.global_key(), Data.Inbound_Station_State), self.inbound_buffer
        )

    def load_rand_inbound_task(self):
        self.rand_inbound_task = []

        tree = ET.parse(self.inbound_file)
        root = tree.getroot()

        inbound_point = root.find("Random_Tasks")

        for task in inbound_point:
            id_ = task.find("id")
            pt = task.find("pt")
            x = pt.find("x")
            y = pt.find("y")
            self.rand_inbound_task.append(
                [int(id_.text), Point(float(x.text), float(y.text))]
            )

    def load_rand_outbound_task(self):
        self.rand_outbound_task = {}
        tree = ET.parse(self.outbound_file)
        root = tree.getroot()
        outbound_point = root.find("Random_Tasks")
        for task in outbound_point:
            id_ = task.find("id")
            pt = task.find("pt")
            x = pt.find("x")
            y = pt.find("y")
            # self.rand_outbound_task.append([int(id_.text),Point(float(x.text), float(y.text))])
            # self.rand_outbound_task[int(id_.text)] = [False, Point(float(x.text), float(y.text))]
            pos = (int(float(x.text)), int(float(y.text)))
            if pos in self.rand_outbound_task:
                self.rand_outbound_task[pos].append(
                    [int(id_.text), False, Point(float(x.text), float(y.text))]
                )
            else:
                self.rand_outbound_task[pos] = [
                    [int(id_.text), False, Point(float(x.text), float(y.text))]
                ]

    def load_outbound(self):
        tree = ET.parse(self.outbound_file)
        root = tree.getroot()
        outbound_point = root.find("Random_Tasks")
        count = -1
        for task in outbound_point:
            count += 1
            if not (self.last_outbound_count <= count < (self.last_outbound_count + 2)):
                continue

            id_ = task.find("id")
            pt = task.find("pt")
            x = pt.find("x")
            y = pt.find("y")
            # self.rand_outbound_task.append([int(id_.text),Point(float(x.text), float(y.text))])
            # self.rand_outbound_task[int(id_.text)] = [False, Point(float(x.text), float(y.text))]
            pos = (int(float(x.text)), int(float(y.text)))

            print(
                f"count : {count} Length : {len(self.rand_outbound_task)} and count {count} and last count {self.last_outbound_count}"
            )
            if pos in self.rand_outbound_task:
                self.rand_outbound_task[pos].append(
                    [int(id_.text), False, Point(float(x.text), float(y.text))]
                )
            else:
                self.rand_outbound_task[pos] = [
                    [int(id_.text), False, Point(float(x.text), float(y.text))]
                ]

    def load_charging(self):
        self.charging_station = self.black_board.read((200, Data.Charging_station))
        if self.charging_station is None:
            return

        for idx, pos in self.charging_station:
            Charge_Station(
                g_id=self.global_id,
                m_id=idx,
                global_blackboard=self.black_board,
                global_event_handler=self.event_handler,
            )

    def behaviour(self):
        if self.initialized == False:
            self.initiate()
            return
        self.load_inbound()
        if (self.loop_count % 1500) == 0:
            self.load_outbound()
            self.last_outbound_count += 2
        self.loop_count += 1


"""
Virtual TaskManagerAgent announce Task_Manager Global Event
Leader policy - which control the flow of all the member in it
"""


class Task_Manager:
    def __init__(self, gid, global_black_board, global_event_handler, planner):
        # self.layout = Config("map.xml")

        self.global_id = gid
        self.module_id = 0
        self.black_board = global_black_board
        self.event_handler = global_event_handler
        self.event_queue = None

        self.wms = WMS(
            self.global_id, self.module_id, self.black_board, self.event_handler
        )
        self.edges = EdgeComputation()
        self.load_balance = Load_Balancing(
            self.global_id,
            self.module_id,
            self.black_board,
            self.event_handler,
            self.edges.edges,
            self.edges.zones,
            wms=self.wms,
            capacity=self.edges.capacity,
        )
        self.planner = planner
        self.task_planner = Task_Planner(
            self.global_id, 1, self.black_board, self.event_handler, self.planner
        )
        self.robot_member = []
        self.inbound_station = []
        self.outbound_station = []
        self.tasks = []

        # For charging_station
        self.charging_station = []
        self.parking_station = []
        self.battery = []

        self.initialized = False
        self.battery_manager = Rule_Based_Approach(
            self.global_id, 0, self.black_board, self.event_handler
        )
        self.setup()
        # self.initiate()

    def global_key(self):
        return (self.global_id * 100) + self.module_id

    def setup(self):
        # self.black_board.register(self.global_id, self.node_id, 1, Data.Initial_Pose)
        self.black_board.register(self.global_key(), 0, Data.Robot_Member)
        self.black_board.register(self.global_key(), 0, Data.Task)
        self.black_board.register(self.global_key(), 1, Data.CS_Battery_level)
        self.black_board.register(self.global_key(), 1, Data.Load_Outbound)
        self.black_board.register(self.global_key(), 1, Data.Drop)
        self.black_board.write((self.global_key(), Data.CS_Battery_level), 50)

    def initiate(self):
        self.robot_member = self.black_board.read((400, Data.Robot_Member))

        self.inbound_station = self.black_board.read((200, Data.Inbound))
        self.outbound_station = self.black_board.read((200, Data.Outbound))
        self.tasks = self.black_board.read((200, Data.Task))

        self.charging_station = self.black_board.read((200, Data.Charging_station))
        self.parking_station = self.black_board.read((200, Data.Parking_Station))
        # self.battery = self.black_board.read((, Data.Battery))

        if len(self.robot_member) == 0:
            self.initialized = False
            return

        if len(self.inbound_station) == 0:
            self.initialized = False
            return

        if len(self.outbound_station) == 0:
            self.initialized = False
            return

        if len(self.tasks) == 0:
            self.initialized = False
            return

        if len(self.charging_station) == 0:
            self.initialized = False
            return

        self.initialized = True

    def pos_to_node(self, x, y):
        node = f"{int(x)},{int(y)}"
        return str(node)

    def node_to_pos(self, node):
        data = re.split(r",", node)
        pos = (int(data[0]), int(data[1]))
        return pos

    def assign_inbound_task(self, robot_id):
        assign_inbound = self.black_board.read((robot_id, Data.Assign_Inbound))
        inbound_station = self.black_board.read((robot_id, Data.Robot_Inbound_Station))
        # inbound_station -= 1
        if assign_inbound:
            return

        task_status = self.black_board.read((robot_id, Data.TaskStatus))

        if task_status == Task.ParkStation or task_status == Task.ChargingStation:
            self.black_board.write((robot_id, Data.Pick), [])
            self.black_board.write((robot_id, Data.Assign_Outbound), True)
            return

        # Free Inbound Station
        self.wms.inbound_buffer[inbound_station][3] = Inbound_State.Free
        task_list = []
        total_task = (
            self.wms.inbound_buffer[inbound_station][0]
            - self.wms.inbound_buffer[inbound_station][1]
        )
        # get new random 7 task
        available_task = min(1, total_task)
        for i in range(available_task):
            if len(self.wms.rand_inbound_task) < (self.wms.inbound_task_count + 1):
                continue
            idx = (
                len(self.wms.inbound_buffer)
                * self.wms.inbound_buffer[inbound_station][1]
            ) + inbound_station
            task_list.append(self.wms.rand_inbound_task[idx])
            # self.wms.inbound_task_count += 1
            self.wms.inbound_buffer[inbound_station][1] += 1
        # get optimal task order
        task_list = self.task_planner.optimal_task(task_list)

        self.black_board.write((robot_id, Data.Assign_Inbound), True)
        self.black_board.write((robot_id, Data.Pick), task_list)

    def assign_outbound_task(self, robot_id):
        task_status = self.black_board.read((robot_id, Data.TaskStatus))
        status = self.black_board.read((robot_id, Data.RobotStatus))

        if status == Status.InOutboundStation:
            return

        if task_status == Task.ParkStation or task_status == Task.ChargingStation:
            self.black_board.write((robot_id, Data.Pick), [])
            self.black_board.write((robot_id, Data.Assign_Outbound), True)
            return

        assign_outbound = self.black_board.read((robot_id, Data.Assign_Outbound))
        if assign_outbound:
            return
        inbound_count = self.black_board.read((robot_id, Data.Number_of_Inbound_Task))
        outbound_count = self.black_board.read((robot_id, Data.Number_of_Outbound_Task))

        available_slot = min(1, 1 - (inbound_count + outbound_count))

        path = self.black_board.read((robot_id, Data.Path))

        # choose
        task = []
        for node in path[1:-1]:
            # data = re.split(r',', node)
            curr_node = (int(node[0]), int(node[1]))
            if available_slot <= 0:
                break
            if curr_node in self.wms.rand_outbound_task:
                state = self.wms.rand_outbound_task[curr_node]

                for i in range(len(state)):
                    if state[i][1] == False:
                        task.append([state[i][0], state[i][2]])
                        self.wms.rand_outbound_task[curr_node][i][1] = True
                        available_slot -= 1
                        if available_slot <= 0:
                            break
        print(
            f"Possible avaible slot IC:{inbound_count} OC:{outbound_count} availablecount:{available_slot} TotalOutbound:{len(task)} outbound {task} Length {len(self.wms.rand_outbound_task)}"
        )
        print(f"Robot ID {robot_id} waiting for outbound task")
        self.black_board.write((robot_id, Data.Pick), task)
        self.black_board.write((robot_id, Data.Assign_Outbound), True)
        task_logger.info(
            f", Assign Outbound, not load outbound, Outbound Task assigned to the robot,{task}, robot,{robot_id}"
        )

    def assign_inbound_station(self, robot):
        min_dist = 10000
        min_inbound_id = -1
        start = self.black_board.read((robot, Data.Pos))
        start_node = self.planner.pos_to_node(start)
        for idx in self.wms.inbound_buffer:
            if True or self.wms.inbound_buffer[idx][3] == Inbound_State.Free:
                total_task = (
                    self.wms.inbound_buffer[idx][0] - self.wms.inbound_buffer[idx][1]
                )

                if total_task == 0:
                    continue
                inbound_pos = self.wms.inbound_buffer[idx][4]
                target = self.planner.pos_to_node(inbound_pos)
                dist = self.planner.plan_distance(start_node, target)
                if dist <= min_dist:
                    min_dist = dist
                    min_inbound_id = idx

        if min_inbound_id != -1:
            self.wms.inbound_buffer[min_inbound_id][3] = Inbound_State.Assigned
            self.black_board.write((robot, Data.Assign_Task), True)
            self.black_board.write((robot, Data.Assigned_Task), Task.InboundStation)
            self.black_board.write((robot, Data.Robot_Inbound_Station), min_inbound_id)

    def assign_task(self, robot):
        min_dist = 10000
        min_inbound_id = -1
        start = self.black_board.read((robot, Data.Pos))
        start_node = self.planner.pos_to_node(start)
        for idx in self.wms.inbound_buffer:
            if self.wms.inbound_buffer[idx][3] == Inbound_State.Free:
                total_task = (
                    self.wms.inbound_buffer[idx][0] - self.wms.inbound_buffer[idx][1]
                )
                if total_task == 0:
                    continue
                inbound_pos = self.wms.inbound_buffer[idx][4]
                target = self.planner.pos_to_node(inbound_pos)
                dist = self.planner.plan_distance(start_node, target)
                if dist <= min_dist:
                    min_dist = dist
                    min_inbound_id = idx

        if min_inbound_id != -1:
            self.wms.inbound_buffer[min_inbound_id][3] = Inbound_State.Assigned
            self.black_board.write((robot, Data.Assign_Task), True)
            self.black_board.write((robot, Data.Assigned_Task), Task.InboundStation)
            self.black_board.write((robot, Data.Robot_Inbound_Station), min_inbound_id)
        else:
            self.black_board.write((robot, Data.Assign_Task), True)
            self.black_board.write((robot, Data.Assigned_Task), Task.ParkStation)

    def assign_load_outbound_task(self, task_list1, robot_id):
        """
        To be done: Assign outbound task to the robot for the load.
        """
        task_list = task_list1
        m = []
        for i in task_list1:
            m.append((i[2].x, i[2].y))

        # task_list = task_list[:min(7, len(task_list))]
        task = []
        for i1 in task_list:
            task.append([i1[0], i1[2]])

        count = 0
        for k in m:
            for j in self.wms.rand_outbound_task[k]:
                if j[0] == task[count][0] and j[1] == False:
                    j[1] = True
                    break
            count += 1

        for i in task:
            if i == True:
                print("yesss")

        task_list = copy.deepcopy(self.task_planner.optimal_task(task))
        self.black_board.write((robot_id, Data.Assign_Load_Outbound), True)
        self.black_board.write((robot_id, Data.Load_Outbound), task_list)
        task_logger.info(
            f", in Assign Load oubtound function, Task assigned by Load Outbound,{task_list}, robot id,{robot_id}"
        )

    def assign_charging_station(self, robot_id):
        # charging_station = self.black_board.read_all(
        #     (self.global_key(), Data.Charge_Station_Mode)
        # )

        charging_station = {
            501: self.black_board.read((501, Data.Charge_Station_Mode)),
            502: self.black_board.read((502, Data.Charge_Station_Mode)),
            503: self.black_board.read((503, Data.Charge_Station_Mode)),
            504: self.black_board.read((504, Data.Charge_Station_Mode)),
            505: self.black_board.read((505, Data.Charge_Station_Mode)),
            506: self.black_board.read((506, Data.Charge_Station_Mode)),
        }
        charging_station_id = 1
        for key in charging_station.keys():
            if charging_station[key] == Charge_Station_Mode.Free:
                charging_station_id = key
                break

        if charging_station_id == 1:
            return

        self.black_board.write((robot_id, Data.Assign_Task), True)
        self.black_board.write((robot_id, Data.Assigned_Task), Task.ChargingStation)
        self.black_board.write(
            (robot_id, Data.Charge_Station_Robot), charging_station_id - 500
        )
        self.black_board.write(
            (charging_station_id, Data.Charge_Station_Mode),
            Charge_Station_Mode.Occupied,
        )
        self.black_board.write(
            (robot_id, Data.CS_Battery_level),
            self.black_board.read((robot_id, Data.Battery)),
        )

    def available_charging_station(self):
        charging_station = self.black_board.read_all(
            (self.global_key(), Data.Charge_Station_Mode)
        )
        for mode in charging_station:
            if mode == Charge_Station_Mode.Free:
                return True
        return False

    def behaviour(self):
        if not self.initialized:
            self.initiate()
            return

        self.wms.behaviour()
        self.battery_manager.solve()
        #! For updating the load for the load balancing algorithm
        self.load_balance.update_load()
        self.edges.update_edge_capacity(self.wms.rand_outbound_task)
        print("Total Outbound task till now", self.wms.rand_outbound_task)
        self.robot_battery = []
        self.robot_capacity = []

        for robot in self.robot_member:
            self.robot_battery.append(
                [robot, self.black_board.read((robot, Data.Battery))]
            )
            inbound_count1 = self.black_board.read((robot, Data.Number_of_Inbound_Task))
            outbound_count1 = self.black_board.read(
                (robot, Data.Number_of_Outbound_Task)
            )
            available_slot1 = min(1, 1 - (inbound_count1 + outbound_count1))
            self.robot_capacity.append([robot, available_slot1])

        self.robot_battery = sorted(self.robot_battery, key=itemgetter(1))
        self.robot_capacity = sorted(self.robot_capacity, key=itemgetter(1))

        for robot1 in self.robot_battery:
            robot = robot1[0]
            task_status = self.black_board.read((robot, Data.TaskStatus))
            status = self.black_board.read((robot, Data.RobotStatus))
            assign = self.black_board.read((robot, Data.Assign_Task))
            assign_inbound = self.black_board.read((robot, Data.Assign_Inbound))
            battery = self.black_board.read((robot, Data.Battery))
            inbound_count = self.black_board.read((robot, Data.Number_of_Inbound_Task))
            outbound_count = self.black_board.read(
                (robot, Data.Number_of_Outbound_Task)
            )
            available_slot = min(1, 1 - (inbound_count + outbound_count))

            if assign or assign_inbound:
                continue
            if 0 < inbound_count <= 2:
                if len(self.black_board.read((robot, Data.Pick))):
                    self.task_id = self.black_board.read((robot, Data.Pick))[-1]
                else:
                    self.task_id = None
                # print(self.task_id)

            if (
                status == Status.WaitForLoadOutbound
                and available_slot > 2
                and self.edges.current_capacity > 2
            ):
                # self.load_balance.assign_outbound_task()

                # * available_slot > 2, is a temporary solution, we can have a dynmaic solution for this later
                # * inbound_count < 2, is done as we are having a 2 task window, so if modify this later, it has to be done in a dynamic way
                # self.assign_outbound(robot)
                x = copy.deepcopy(
                    self.load_balance.assign_task(
                        robot=robot, available_slot=available_slot, task_id=self.task_id
                    )
                )
                print("Task for the load outbound", x)
                task_logger.info(
                    f" in task manager 1 line, Outbound task assigned to the bot,{x},  robot,{robot} , available slot,{available_slot}"
                )
                self.assign_load_outbound_task(x, robot)
                task_logger.info(
                    f",In task manager last line Outbound task assigned to the bot,{x},  robot,{robot} , available slot,{available_slot}, task in robobt {self.black_board.read((self.global_key(), Data.Load_Outbound))}"
                )

            elif status == Status.WaitForInboundTask:
                self.assign_inbound_task(robot)

            elif status == Status.WaitForOutboundTask:
                self.assign_outbound_task(robot)

            elif task_status == Task.WaitForTask and battery >= 50:
                if status == Status.Parked or status == Status.Charged:
                    self.assign_inbound_station(robot)
                elif status == Status.InOutboundStation:
                    if self.load_balance.get_load():
                        x = self.load_balance.assign_task(
                            robot=robot, available_slot=1, task_id=None
                        )
                        self.assign_load_outbound_task(x, robot)
                    else:
                        self.assign_task(robot)

            elif task_status == Task.WaitForTask and battery < 50:
                # print("state of charging station", self.available_charging_station())
                if self.available_charging_station():
                    self.assign_charging_station(robot)

                elif not self.available_charging_station() and battery < 35:
                    self.black_board.write((robot, Data.Assign_Task), True)
                    self.black_board.write(
                        (robot, Data.Assigned_Task), Task.ParkStation
                    )

                elif not self.available_charging_station() and battery >= 35:
                    if status == Status.Parked:
                        self.assign_inbound_station(robot)
                    elif status == Status.InOutboundStation:
                        if self.load_balance.get_load():
                            x = self.load_balance.assign_task(
                                robot=robot, available_slot=1, task_id=None
                            )
                            self.assign_load_outbound_task(x, robot)
                        else:
                            self.assign_task(robot)

    def event_behaviour(self):
        # check for new event
        if self.event_handler.is_there_event((self.global_id, self.module_id)):
            # Get all event
            event_queue = self.event_handler.get_event((self.global_id, self.module_id))
            print(f"{event_queue.qsize()} and size {event_queue.empty()}")
            # Response to all event
            while not event_queue.empty():
                event = event_queue.get()
                print(event)
                if event[1] == Event.Goal_Reached:
                    self.set_new_goal(event[0])
                elif event[1] == Event.Initialize_Pose:
                    self.initialize_robot_pos()
