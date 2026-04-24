from logging import root
import xml.etree.ElementTree as ET
import networkx as nx
from black_board import *
from data import *
import random


class Simulation_Model:
    def __init__(self, g_id, global_blackboard, global_event_handler):
        # Global ID of the node
        self.global_id = g_id
        self.module_id = 0
        # Centralized data access / store
        self.black_board = global_blackboard
        self.node_id = self.black_board.register_node(
            self.global_key(), f"Node:Simulation_Mode1 {self.global_id}"
        )

        self.event_handler = global_event_handler
        self.event_id = self.event_handler.register_node(
            f"Event:Simulation_Model {self.global_id}"
        )
        self.event_queue = None

        self.start = False
        self.graph_file = "./config/research_layout_2/layout_graph.gml"
        self.polygon_file = "./config/research_layout_2/sample.xml"
        self.task_file = "./config/research_layout_2/research_layout.xml"
        self.waitspot_file = "./config/research_layout_2/waitspot.xml"
        self.charging_station_file = "./config/research_layout_2/charging_station.xml"
        self.parking_station_file = "./config/research_layout_2/parking_station.xml"

        self.graph = None

        self.initial_pose = []
        self.polygons = []
        self.points = []
        self.lines = []
        self.inbound_points = []
        self.outbound_points = []
        self.task_points = []
        self.waitspot = []
        self.charging_station = []
        self.parking_station = []
        self.colors = []
        self.random_task = []
        self.task_generation_count = 2000
        self.setup()
        self.initiate()
        # self.load_polygons()
        # self.load_points()
        # self.load_lines()

    def global_key(self):
        return (self.global_id * 100) + self.module_id

    def setup(self):
        # Data
        self.black_board.register(self.global_key(), 0, Data.Start)
        self.black_board.register(self.global_key(), 1, Data.Polygon)
        self.black_board.register(self.global_key(), 1, Data.Node_Point)
        self.black_board.register(self.global_key(), 1, Data.Edge_Line)
        self.black_board.register(self.global_key(), 1, Data.Initial_Pose)
        self.black_board.register(self.global_key(), 1, Data.Inbound)
        self.black_board.register(self.global_key(), 1, Data.Outbound)
        self.black_board.register(self.global_key(), 1, Data.Task)
        self.black_board.register(self.global_key(), 1, Data.Color)
        self.black_board.register(self.global_key(), 1, Data.Parking_Station)
        self.black_board.register(self.global_key(), 1, Data.Charging_station)

        # Event
        self.event_handler.register(self.global_key(), ((1 * 100) + 1, Event.Start), 0)
        self.event_handler.register(
            self.global_key(), ((1 * 100) + 1, Event.Initialize_Layout), 1
        )

    def initiate(self):
        self.load_initial_pose()
        # self.load_polygons()
        self.load_lines()
        self.load_points()
        self.load_inbound_position()
        self.load_outbound_position()
        self.load_task_position()
        self.load_waitspot()
        self.load_charging_station()
        self.load_parking_station()

        self.create_random_inbound_task()
        self.create_random_outbound_task()
        self.event_handler.pub((self.global_key(), Event.Initialize_Layout))

    def load_graph(self):
        self.graph = nx.read_gml(self.graph_file)

    def generate_random_task(self, count):
        self.random_task = []
        for _ in range(count):
            task = random.randint(0, len(self.task_points) - 1)
            self.random_task.append(self.task_points[task])

    def create_random_inbound_task(self):
        self.generate_random_task(self.task_generation_count)

        tree = ET.ElementTree()
        root = ET.Element("root")
        task_station = ET.Element("Random_Tasks")
        root.append(task_station)

        for key, task_pos in self.random_task:
            task = ET.Element("task")
            task_station.append(task)
            id_ = ET.Element("id")
            id_.text = str(key)

            task.append(id_)

            point = ET.Element("pt")
            task.append(point)

            x = ET.Element("x")
            x.text = str(task_pos.x)
            y = ET.Element("y")
            y.text = str(task_pos.y)

            point.append(x)
            point.append(y)
        tree._setroot(root)
        tree.write("./config/research_layout_2/rand_inbound_task.xml", "")

    def create_random_outbound_task(self):
        self.generate_random_task(self.task_generation_count)

        tree = ET.ElementTree()
        root = ET.Element("root")
        task_station = ET.Element("Random_Tasks")
        root.append(task_station)

        for key, task_pos in self.random_task:
            task = ET.Element("task")
            task_station.append(task)
            id_ = ET.Element("id")
            id_.text = str(key)

            task.append(id_)

            point = ET.Element("pt")
            task.append(point)

            x = ET.Element("x")
            x.text = str(task_pos.x)
            y = ET.Element("y")
            y.text = str(task_pos.y)

            point.append(x)
            point.append(y)
        tree._setroot(root)
        tree.write("./config/research_layout_2/rand_outbound_task.xml", "")

    def load_initial_pose(self):
        self.initial_pose = []
        self.colors = []

        tree = ET.parse(self.polygon_file)
        root = tree.getroot()

        # load initial pose
        initial_pose = root.find("Initial_Pose")

        count = 0
        for pose in initial_pose:
            data = [count]
            pos = pose.find("pt")
            x = pos.find("x")
            y = pos.find("y")
            data.append(Point(float(x.text), float(y.text)))
            theta = pose.find("theta")
            data.append(float(theta.text))
            b_percentage = pose.find("battery_charge")
            data.append(float(b_percentage.text))
            color = pose.find("color")
            self.colors.append(color.text)
            self.initial_pose.append(data)
            count += 1

        self.black_board.write(
            (self.global_key(), Data.Initial_Pose), self.initial_pose
        )
        self.black_board.write((self.global_key(), Data.Color), self.colors)

    def load_polygons(self):
        self.polygons = []

        tree = ET.parse(self.polygon_file)
        root = tree.getroot()

        # load polygon
        polygon = root.find("polygon")

        for poly in polygon:
            points = []
            for pt in poly:
                x = pt.find("x")
                y = pt.find("y")
                # points.append(int(x.text),int(y.text))
                points.append(Point(int(x.text), int(y.text)))

            self.polygons.append(points)

        self.black_board.write((self.global_key(), Data.Polygon), self.polygons)

    def load_points(self):
        self.points = []

        tree = ET.parse(self.polygon_file)
        root = tree.getroot()

        point = root.find("point")
        for pos in point:
            x = pos.find("x")
            y = pos.find("y")
            # print(int(x.text),int(y.text))
            # self.points.append([int(x.text), int(y.text)])
            self.points.append(Point(int(x.text), int(y.text)))

        self.black_board.write((self.global_key(), Data.Node_Point), self.points)

    def load_lines(self):
        self.lines = []

        tree = ET.parse(self.polygon_file)
        root = tree.getroot()

        line = root.find("lines")
        for l in line:
            pt = []
            for pos in l:
                x = pos.find("x")
                y = pos.find("y")
                # pt.append([int(x.text), int(y.text)])
                pt.append(Point(int(x.text), int(y.text)))
            self.lines.append(pt)

        self.black_board.write((self.global_key(), Data.Edge_Line), self.lines)

    def load_inbound_position(self):
        self.inbound_points = []

        tree = ET.parse(self.task_file)
        root = tree.getroot()

        inbound_point = root.find("inbound_station")

        for inbound in inbound_point:
            id_ = inbound.find("id")
            pt = inbound.find("pt")
            wait = inbound.find("wait_time")
            load = inbound.find("load")
            x = pt.find("x")
            y = pt.find("y")
            self.inbound_points.append(
                [
                    int(id_.text),
                    Point(float(x.text), float(y.text)),
                    float(wait.text),
                    float(load.text),
                ]
            )

        self.black_board.write((self.global_key(), Data.Inbound), self.inbound_points)

    def load_outbound_position(self):
        self.outbound_points = []

        tree = ET.parse(self.task_file)
        root = tree.getroot()

        outbound_point = root.find("outbound_station")

        for outbound in outbound_point:
            id_ = outbound.find("id")
            pt = outbound.find("pt")
            wait = outbound.find("wait_time")
            x = pt.find("x")
            y = pt.find("y")
            self.outbound_points.append(
                [int(id_.text), Point(float(x.text), float(y.text)), float(wait.text)]
            )

        self.black_board.write((self.global_key(), Data.Outbound), self.outbound_points)

    def load_task_position(self):
        self.task_points = []

        tree = ET.parse(self.task_file)
        root = tree.getroot()

        task_point = root.find("task_station")

        for task in task_point:
            id_ = task.find("id")
            pt = task.find("pt")
            x = pt.find("x")
            y = pt.find("y")
            self.task_points.append(
                [int(id_.text), Point(float(x.text), float(y.text))]
            )

        self.black_board.write((self.global_key(), Data.Task), self.task_points)

    def load_waitspot(self):
        self.waitspot = []

        tree = ET.parse(self.waitspot_file)
        root = tree.getroot()

        waitspot = root.find("waitspot")

        for spot in waitspot:
            id_ = spot.find("id")
            pt = spot.find("pt")
            x = pt.find("x")
            y = pt.find("y")

            self.waitspot.append(Point(int(x.text), int(y.text)))

        self.black_board.write((self.global_key(), Data.Waitspot), self.waitspot)

    def load_charging_station(self):
        self.charging_station = []

        tree = ET.parse(self.charging_station_file)
        root = tree.getroot()
        charging_station = root.find("charging_station")

        for cstation in charging_station:
            id_ = cstation.find("id")
            pt = cstation.find("pt")
            x = pt.find("x")
            y = pt.find("y")

            self.charging_station.append(
                [int(id_.text), Point(float(x.text), float(y.text))]
            )

        self.black_board.write(
            (self.global_key(), Data.Charging_station), self.charging_station
        )

    def load_parking_station(self):
        self.parking_stationparking_station = []

        tree = ET.parse(self.parking_station_file)
        root = tree.getroot()

        parking_station = root.find("parking_station")

        for parking in parking_station:
            id_ = parking.find("id")
            pt = parking.find("pt")
            x = pt.find("x")
            y = pt.find("y")

            self.parking_station.append(
                [int(id_.text), Point(float(x.text), float(y.text))]
            )

        self.black_board.write(
            (self.global_key(), Data.Parking_Station), self.parking_station
        )
