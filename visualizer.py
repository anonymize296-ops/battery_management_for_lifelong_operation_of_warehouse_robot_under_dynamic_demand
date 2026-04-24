# import imp
from pydoc import text
from re import I
import tkinter as tk
from tkinter import ttk
from turtle import pos
from PIL import Image, ImageTk
from enum import Enum
import matplotlib
import matplotlib.pyplot as plt

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import xml.etree.ElementTree as ET
import networkx as nx
import re

from data import *
from black_board import *


class Mode(Enum):
    CreateMap = 0
    Simulation = 1


class Map_Controller:
    def __init__(self, root, g_id, global_blackboard, global_event_handler):

        # Global ID of the node
        self.global_id = g_id
        self.module_id = 2
        # Centralized data access / store
        self.black_board = global_blackboard
        self.node_id = self.black_board.register_node(
            self.global_key(), f"Node:Map_Controller {self.global_id}"
        )

        self.event_handler = global_event_handler
        self.event_id = self.event_handler.register_node(
            f"Event:Map_Controller {self.global_id}"
        )
        self.event_queue = None

        self.frame = tk.Frame(root)

        Map = []
        # root = tk.Tk()
        # create Mainframe
        self.command_frame = tk.Frame(self.frame)
        self.command_frame.pack(side=tk.TOP)
        self.main_frame = tk.Frame(self.frame)
        self.main_frame.pack(fill=tk.BOTH, expand=1)
        # create canvas
        self.my_canvas = tk.Canvas(self.main_frame)
        self.my_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=1)

        # Add a scrollbar to the canvas
        self.my_scrollbar = ttk.Scrollbar(
            self.main_frame, orient=tk.VERTICAL, command=self.my_canvas.yview
        )
        self.my_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # configure the canvas
        self.my_canvas.configure(yscrollcommand=self.my_scrollbar.set)
        self.my_canvas.bind(
            "<Configure>",
            lambda e: self.my_canvas.configure(scrollregion=self.my_canvas.bbox("all")),
        )

        # Create Another Frame inside the canvas
        self.second_frame = tk.Frame(self.my_canvas)
        # add the New frame to a window in the canvas
        self.my_canvas.create_window((0, 0), window=self.second_frame, anchor="nw")

        self.grid_frame = tk.Frame(self.second_frame)
        row = 10
        col = 10
        self.data = [[0.0 for i in range(col)] for j in range(row)]

        for i in range(row):
            for j in range(col):
                self.data[i][j] = tk.Entry(self.grid_frame, width=5)
                if i == 0:
                    self.data[i][j].insert(0, str(j))
                elif j == 0:
                    self.data[i][j].insert(0, str(i))
                else:
                    self.data[i][j].insert(0, "1")

                self.data[i][j].grid(row=i, column=j)

        self.grid_frame.grid(row=1, column=0)

        self.disp = tk.IntVar()
        self.disp.set(0)
        self.reset = tk.IntVar()
        self.reset.set(0)

        self.view = tk.Button(self.command_frame, text="View", command=self.visualize)
        self.reset = tk.Button(self.command_frame, text="Reset", command=self.reset)
        self.get_data = tk.Button(
            self.command_frame, text="Get Data", command=self.store_data
        )

        self.view.grid(row=0, column=0)
        self.reset.grid(row=0, column=1)
        self.get_data.grid(row=0, column=2)

        self.command = tk.Entry(self.command_frame, width=50)
        self.command.insert(0, "Enter Command")
        self.command.grid(row=0, column=3)
        self.command_button = tk.Button(
            self.command_frame,
            text="Exec Command",
            command=lambda: self.execute_command(self.command.get()),
        )
        self.command_button.grid(row=0, column=4)

    def global_key(self):
        return (self.global_id * 100) + self.module_id

    def reset(self):
        for i in range(1, len(self.data)):
            for j in range(1, len(self.data[i])):
                self.data[i][j].config({"background": "white"})
                self.data[i][j].delete(0, "end")
                self.data[i][j].insert(0, "1")

    def visualize(self):
        for i in range(len(self.data)):
            for j in range(len(self.data[i])):
                if int(self.data[i][j].get()) == 0:
                    self.data[i][j].config({"background": "black"})
                elif int(self.data[i][j].get()) == 1:
                    self.data[i][j].config({"background": "white"})
                elif int(self.data[i][j].get()) == 2:
                    self.data[i][j].config({"background": "red"})

    def store_data(self):
        self.Map = [
            [0.0 for i in range(len(self.data[j]))] for j in range(len(self.data))
        ]

        for i in range(len(self.data)):
            for j in range(len(self.data[i])):
                self.Map[i][j] = int(self.data[i][j].get())

    def store_graph(self):

        points = ET.Element("point")
        root.append(points)

        for node in A.nodes:
            # print(A.nodes[node]["pos"])
            pos = A.nodes[node]["pos"]
            point = ET.Element("pt")
            points.append(point)

            x = ET.Element("x")
            x.text = str(pos[0])
            y = ET.Element("y")
            y.text = str(pos[1])

            point.append(x)
            point.append(y)

        # root = ET.Element("root")
        lines = ET.Element("lines")
        root.append(lines)

        for edge in A.edges:
            # print(A.nodes[node]["pos"])
            pos = []
            pos.append(A.nodes[edge[0]]["pos"])
            pos.append(A.nodes[edge[1]]["pos"])
            # print(A.nodes[edge[1]]["pos"])
            line = ET.Element("line")
            lines.append(line)

            for i in range(2):
                point = ET.Element("pt")
                x = ET.Element("x")
                x.text = str(pos[i][0])
                y = ET.Element("y")
                y.text = str(pos[i][1])

                point.append(x)
                point.append(y)

                line.append(point)

        tree._setroot(root)
        tree.write("./config/research_layout_2/sample1.xml", "")

    def store_polygon(self):
        root = ET.Element("root")
        polygon = ET.Element("polygon")
        root.append(polygon)

        with open("./config/polygon.ply", newline="") as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                poly = ET.Element("poly")
                polygon.append(poly)
                for i in range(4):
                    point = ET.Element("pt")
                    poly.append(point)

                    x = ET.Element("x")
                    x.text = row[2 * i]
                    y = ET.Element("y")
                    y.text = row[2 * i + 1]
                    point.append(x)
                    point.append(y)
                print(row[2 * i], row[2 * i + 1])

    def add_row(self, value):
        for i in range(value):
            rows = [0.0 for j in range(len(self.data[-1]))]
            self.data.append(rows)
            for j in range(len(self.data[-1])):
                self.data[-1][j] = tk.Entry(self.grid_frame, width=5)
                if j == 0:
                    self.data[-1][j].insert(0, str(len(self.data) - 1))
                else:
                    self.data[-1][j].insert(0, "1")

                self.data[-1][j].grid(row=len(self.data) - 1, column=j)

    def add_col(self, value):
        for i in range(len(self.data)):
            extra = [0.0 for _ in range(value)]
            self.data[i] += extra
            for j in range(len(self.data[i]) - value, len(self.data[i])):
                self.data[i][j] = tk.Entry(self.grid_frame, width=5)
                if i == 0:
                    self.data[i][j].insert(0, str(j))
                else:
                    self.data[i][j].insert(0, "1")

                self.data[i][j].grid(row=i, column=j)

    def fill(self, r1, r2, c1, c2, value):
        for i in range(r1, r2):
            for j in range(c1, c2):
                self.data[i][j].delete(0, "end")
                self.data[i][j].insert(0, value)

    def load_data(self, file):
        f = open(file)
        pass

    def execute_command(self, command):
        part = command.split("/")

        if part[0] == "v":
            if part[1] == "row":
                row = int(part[2])
                print("getting row")

            if part[1] == "col":
                col = int(part[2])
                print("getting col")

        if part[0] == "o":
            if part[1] == "add":
                if part[2] == "row":
                    # row += int(part[3])
                    self.add_row(int(part[3]))
                    print(f"Increasing row {int(part[3])}")
                if part[2] == "col":
                    # col += int(part[3])
                    self.add_col(int(part[3]))

            if part[1] == "fill":
                self.fill(
                    int(part[2]), int(part[3]), int(part[4]), int(part[5]), int(part[6])
                )

            if part[1] == "load":
                self.load_data(part[2])


class Simulation_Controller:
    def __init__(self, root, g_id, global_blackboard, global_event_handler):
        # Global ID of the node
        self.global_id = g_id
        self.module_id = 1
        # Centralized data access / store
        self.black_board = global_blackboard
        self.node_id = self.black_board.register_node(
            self.global_key(), f"Node:Simulation_Controller {self.global_id}"
        )

        self.event_handler = global_event_handler
        self.event_id = self.event_handler.register_node(
            f"Event:Simulation_Controller {self.global_id}"
        )
        self.event_queue = None

        self.parent = root
        self.frame = tk.Frame(root)

        self.command_frame = tk.Frame(self.frame)
        self.command_frame.pack(side=tk.TOP)

        self.main_frame = tk.Frame(self.frame)
        self.main_frame.pack(fill=tk.BOTH, expand=1)
        # create canvas
        self.my_canvas = tk.Canvas(self.main_frame)
        # self.my_canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        self.my_canvas.grid(row=0, column=0, sticky=(tk.N, tk.S, tk.W, tk.E))
        # Add a scrollbar to the canvas
        self.my_scrollbar = ttk.Scrollbar(
            self.main_frame, orient=tk.VERTICAL, command=self.my_canvas.yview
        )
        # self.my_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.my_scrollbar.grid(row=0, column=100, sticky=(tk.N, tk.S))
        self.my_scrollbar_h = ttk.Scrollbar(
            self.main_frame, orient=tk.HORIZONTAL, command=self.my_canvas.xview
        )
        # self.my_scrollbar_h.pack(side=tk.BOTTOM, fill=tk.X)
        self.my_scrollbar_h.grid(row=100, column=0, sticky=(tk.W, tk.E))

        self.main_frame.rowconfigure(0, weight=1)
        self.main_frame.columnconfigure(0, weight=1)
        # configure the canvas
        self.my_canvas.configure(yscrollcommand=self.my_scrollbar.set)
        self.my_canvas.bind(
            "<Configure>",
            lambda e: self.my_canvas.configure(scrollregion=self.my_canvas.bbox("all")),
        )

        self.my_canvas.configure(xscrollcommand=self.my_scrollbar_h.set)
        self.my_canvas.bind(
            "<Configure>",
            lambda e: self.my_canvas.configure(scrollregion=self.my_canvas.bbox("all")),
        )

        # Create Another Frame inside the canvas
        self.second_frame = tk.Frame(self.my_canvas)
        # add the New frame to a window in the canvas
        self.my_canvas.create_window((0, 0), window=self.second_frame, anchor="nw")

        self.grid_frame = tk.Frame(self.second_frame)
        self.grid_frame.grid(row=1, column=0)

        self.visualizer = tk.Canvas(
            self.grid_frame, bg="white", height=1000, width=4000
        )
        self.visualizer.grid(row=0, column=0)
        self.resize = tk.Button(self.command_frame, text="resize", command=self.resize)
        self.command = tk.Entry(self.command_frame, width=50, borderwidth=5)
        self.command.insert(0, "Enter Command")
        self.command_button = tk.Button(
            self.command_frame,
            text="Exec Command",
            command=lambda: self.execute_command(self.command.get()),
        )

        self.inbound_img = Image.open("./images/inbound_station.png")
        self.inbound_image = ImageTk.PhotoImage(self.inbound_img.resize([50, 50]))

        self.outbound_img = Image.open("./images/outbound_station.png")
        self.outbound_image = ImageTk.PhotoImage(self.outbound_img.resize([50, 50]))

        self.charging_station_img = Image.open("./images/charging_station.jpg")
        self.charging_station_image = ImageTk.PhotoImage(
            self.charging_station_img.resize([50, 50])
        )

        self.park_img = Image.open("./images/parkings.png")
        self.park_image = ImageTk.PhotoImage(self.park_img.resize([45, 45]))

        self.display()

        self.robots = []
        self.polygons = []
        self.lines = []
        self.points = []
        self.inbound_station = []
        self.outbound_station = []
        self.charging_station = []
        self.parking_station = []
        self.task_station = []
        self.colors = []
        self.overlap_object = {}
        self.count = 0
        self.setup()

        self.object_wscale = 42.0
        self.object_hscale = 42.0

        self.x_off = 2.0
        self.y_off = 0.0

        self.reservation = []
        self.last_reserved = []
        self.unreserve = []
        self.high_priority_robot = []

        self.load_task_station()
        # self.initiate()
        # self.parent.bind('<Motion>', self.motion)

    def motion(self, event):
        self.x, self.y = event.x, event.y

        # print("X")
        # pass

        print("{}, {}".format(self.x, self.y))

    def global_key(self):
        return (self.global_id * 100) + self.module_id

    def load_task_station(self):
        self.task_station = []

        tree = ET.parse("./config/research_layout_2/task.xml")
        root = tree.getroot()

        task_point = root.find("Task_Station")

        for task in task_point:
            id_ = task.find("id")
            pt = task.find("pt")
            x = pt.find("x")
            y = pt.find("y")
            self.task_station.append(
                [int(id_.text), Point(float(x.text), float(y.text))]
            )

        # print(self.task_station)

    def setup(self):
        # Data
        self.black_board.register(self.global_key(), 1, Data.Start)
        self.black_board.register(self.global_key(), 0, Data.Polygon)
        self.black_board.register(self.global_key(), 0, Data.Node_Point)
        self.black_board.register(self.global_key(), 0, Data.Edge_Line)
        self.black_board.register(self.global_key(), 0, Data.Color)

        # Event [my_key, subscribe_event_key ]
        self.event_handler.register(self.global_key(), (2 * 100 + 0, Event.Start), 1)
        self.event_handler.register(
            self.global_key(), (2 * 100 + 0, Event.Initialize_Layout), 0
        )

    def initiate(self):
        self.black_board.write((self.global_key(), Data.Start), 0)

    def rotation_matrix(self, x, y, angle):
        x_rot = (math.cos(angle) * x) - (math.sin(angle) * y)
        y_rot = (math.sin(angle) * x) + (math.cos(angle) * y)
        return (x_rot, y_rot)

    def get_robot_polygon(self, x, y, theta):
        theta1 = theta + (math.pi * (1 / 4))
        theta2 = theta - (math.pi * (1 / 4))
        theta3 = theta - (math.pi * (3 / 4))
        theta4 = theta + (math.pi * (3 / 4))

        (b1x, b1y) = self.rotation_matrix(0.38, 0.38, theta)
        (b2x, b2y) = self.rotation_matrix(0.38, -0.38, theta)
        (b3x, b3y) = self.rotation_matrix(-0.38, -0.38, theta)
        (b4x, b4y) = self.rotation_matrix(-0.38, 0.38, theta)

        # print(b1x, b1y, b2x, b2y, b3x, b3y, b4x, b4y)
        polygon = [[x * self.object_wscale, y * self.object_hscale] for i in range(4)]
        polygon[0][0] += b1x * self.object_wscale
        polygon[0][1] += b1y * self.object_hscale
        polygon[1][0] += b2x * self.object_wscale
        polygon[1][1] += b2y * self.object_hscale
        polygon[2][0] += b3x * self.object_wscale
        polygon[2][1] += b3y * self.object_hscale
        polygon[3][0] += b4x * self.object_wscale
        polygon[3][1] += b4y * self.object_hscale

        poly = [round(polygon[i][j], 2) for i in range(4) for j in range(2)]
        return poly

    def resize(self):
        wscale = self.parent.winfo_width() / self.visualizer.winfo_width()
        hscale = self.parent.winfo_height() / self.visualizer.winfo_height()

        print(f"{self.parent.winfo_width()} {self.parent.winfo_height()}")

        wscale = wscale / 1.1
        hscale = hscale / 1.1

        new_width = self.visualizer.winfo_width() * wscale
        new_height = self.visualizer.winfo_height() * hscale
        self.visualizer.config(width=new_width, height=new_height)

    def xscale(self, wscale):
        new_width = self.visualizer.winfo_width() * wscale
        height = self.visualizer.winfo_height()
        self.visualizer.config(width=new_width, height=height)

    def yscale(self, hscale):
        width = self.visualizer.winfo_width()
        new_height = self.visualizer.winfo_height() * hscale
        self.visualizer.config(width=width, height=new_height)

    def object_scale(self, wscale, hscale):
        self.object_wscale = self.object_wscale * wscale
        self.object_hscale = self.object_hscale * hscale

        objects = self.visualizer.find_withtag("all")
        for obj in objects:
            coor = self.visualizer.coords(obj)
            new_coor = [coor[i] for i in range(len(coor))]
            for j in range(len(coor)):
                if (j % 2) == 0:
                    new_coor[j] = coor[j] * wscale
                elif (j % 2) == 1:
                    new_coor[j] = coor[j] * hscale

            self.visualizer.coords(obj, new_coor)

    def execute_command(self, command):
        part = command.split("/")

        if part[0] == "o":
            if part[1] == "scale":
                if part[2] == "x":
                    # row += int(part[3])
                    self.xscale(float(part[3]))
                    print(f"Increasing row {float(part[3])}")
                if part[2] == "y":
                    # col += int(part[3])
                    self.yscale(float(part[3]))
                if part[2] == "object":
                    self.object_scale(float(part[3]), float(part[4]))

            if part[1] == "fill":
                self.fill(
                    int(part[2]), int(part[3]), int(part[4]), int(part[5]), int(part[6])
                )

            if part[1] == "load":
                self.load_data(part[2])

    def display(self):
        self.resize.grid(row=0, column=0)
        self.command.grid(row=0, column=1)
        self.command_button.grid(row=0, column=2)

    """
    Read latest robot position and plot it
    """

    def update_robots_position(self):
        self.count += 1
        self.robots_state = self.black_board.merge_all(
            [
                Data.RID,
                Data.Pos,
                Data.Theta,
                Data.Task_ID,
                Data.TaskStatus,
                Data.Battery,
            ]
        )
        for RID, pose, theta, task_id, state, battery in self.robots_state:
            if pose is not None:
                # Most of the robot position in these theta, reducing floating error
                if abs(theta - 3.14) < 0.1:
                    theta = 3.14
                elif abs(theta - 1.57) < 0.1:
                    theta = 1.57
                elif abs(theta + 1.57) < 0.1:
                    theta = -1.57
                elif abs(theta) < 0.1:
                    theta = 0.0
                elif abs(theta + 3.14) < 0.1:
                    theta = 0.0

                poly = self.get_robot_polygon(
                    pose.x + self.x_off, pose.y + self.y_off, theta
                )

                ex1 = poly[0] + (poly[6] - poly[0]) * 0.1
                ey1 = poly[1] + (poly[7] - poly[1]) * 0.1

                ex2 = poly[2] + (poly[4] - poly[2]) * 0.1
                ey2 = poly[3] + (poly[5] - poly[3]) * 0.1

                ecx = poly[0] + (poly[2] - poly[0]) * 0.5
                ecy = poly[1] + (poly[3] - poly[1]) * 0.5

                edx = ex1 + (ex2 - ex1) * 0.5
                edy = ey1 + (ey2 - ey1) * 0.5

                hcx = poly[6] + (poly[4] - poly[6]) * 0.5
                hcy = poly[7] + (poly[5] - poly[7]) * 0.5

                tx = ecx + (hcx - ecx) * 0.2
                ty = ecy + (hcy - ecy) * 0.2

                ttx = ecx + (hcx - ecx) * 0.6
                tty = ecy + (hcy - ecy) * 0.6

                fx = poly[2] + (poly[4] - poly[2]) * 0.3
                fy = poly[3] + (poly[5] - poly[3]) * 0.3

                gx = poly[0] + (poly[6] - poly[0]) * 0.3
                gy = poly[1] + (poly[7] - poly[1]) * 0.3

                obj = self.visualizer.find_withtag(f"robot_{RID}")
                if obj:
                    self.visualizer.coords(obj, poly)
                    obj1 = self.visualizer.find_withtag(f"robot_name_{RID}")
                    # obj2 = self.visualizer.find_withtag(f"robot_eye1_{RID}")
                    # obj3 = self.visualizer.find_withtag(f"robot_eye2_{RID}")
                    obj4 = self.visualizer.find_withtag(f"robot_task_{RID}")
                    obj5 = self.visualizer.find_withtag(f"robot_taskid_{RID}")
                    # obj6 = self.visualizer.find_withtag(f"task_{task_id}")

                    # self.visualizer.itemconfigure(obj6, fil1=self.colors[RID])
                    self.visualizer.coords(obj1, tx, ty)
                    # print(self.colors[RID-1])
                    # self.visualizer.itemconfigure(obj, fill=self.colors[RID-1])
                    self.visualizer.itemconfigure(obj1, angle=math.degrees(-theta))
                    # self.visualizer.coords(
                    #     obj2,
                    #     poly[0],
                    #     poly[1],
                    #     round(ecx, 2),
                    #     round(ecy, 2),
                    #     round(edx, 2),
                    #     round(edy, 2),
                    #     round(ex1, 2),
                    #     round(ey1, 2),
                    # )
                    # self.visualizer.coords(
                    #     obj3,
                    #     poly[2],
                    #     poly[3],
                    #     round(ecx, 2),
                    #     round(ecy, 2),
                    #     round(edx, 2),
                    #     round(edy, 2),
                    #     round(ex2, 2),
                    #     round(ey2, 2),
                    # )
                    self.visualizer.coords(
                        obj4,
                        round(fx, 2),
                        round(fy, 2),
                        round(gx, 2),
                        round(gy, 2),
                        poly[6],
                        poly[7],
                        poly[4],
                        poly[5],
                    )
                    self.visualizer.coords(obj5, round(ttx, 2), round(tty, 2))
                    self.visualizer.itemconfigure(obj5, angle=math.degrees(-theta))
                    self.visualizer.itemconfigure(obj5, text=f"{int(battery)}")

                    if state == Task.Inbound:
                        self.visualizer.itemconfigure(obj5, fill="blue")
                    elif state == Task.Outbound:
                        self.visualizer.itemconfigure(obj5, fill="white")
                    elif state == Task.InboundStation:
                        self.visualizer.itemconfigure(obj5, fill="black")
                    elif state == Task.OutboundStation:
                        self.visualizer.itemconfigure(obj5, fill="black")
                    elif state == Task.Outbound_Task:
                        self.visualizer.itemconfigure(obj5, fill="red")
                else:
                    print(self.colors[RID])
                    obj1 = self.visualizer.create_polygon(
                        poly,
                        outline="black",
                        width=1,
                        smooth=True,
                        fill="white",
                        joinstyle=tk.MITER,
                        tags=[f"robot_{RID}", "robot"],
                    )
                    # obj1 = self.visualizer.create_polygon(poly, outline="black", width=2, fill=self.colors[RID], joinstyle=tk.ROUND, tags=[f"robot_{RID}", "robot"])
                    obj2 = self.visualizer.create_text(
                        tx,
                        ty,
                        text=str(RID),
                        font="Times 8 bold",
                        angle=0,
                        tags=f"robot_name_{RID}",
                    )
                    # self.visualizer.create_polygon(
                    #     poly[0] + self.x_off,
                    #     poly[1] + self.y_off,
                    #     round(ecx, 2) + self.x_off,
                    #     round(ecy, 2) + self.y_off,
                    #     round(edx, 2) + self.x_off,
                    #     round(edy, 2) + self.y_off,
                    #     round(ex1, 2) + self.x_off,
                    #     round(ey1, 2) + self.y_off,
                    #     fill="blue",
                    #     outline="black",
                    #     tags=f"robot_eye1_{RID}",
                    # )
                    # self.visualizer.create_polygon(
                    #     poly[2] + self.x_off,
                    #     poly[3] + self.y_off,
                    #     round(ecx, 2) + self.x_off,
                    #     round(ecy, 2) + self.y_off,
                    #     round(edx, 2) + self.x_off,
                    #     round(edy, 2) + self.y_off,
                    #     round(ex2, 2) + self.x_off,
                    #     round(ey2, 2) + self.y_off,
                    #     fill="blue",
                    #     outline="black",
                    #     tags=f"robot_eye2_{RID}",
                    # )
                    self.visualizer.create_polygon(
                        round(fx, 2) + self.x_off,
                        round(fy, 2) + self.y_off,
                        round(gx, 2) + self.x_off,
                        round(gy, 2) + self.y_off,
                        poly[6] + self.x_off,
                        poly[7] + self.y_off,
                        poly[4] + self.x_off,
                        poly[5] + self.y_off,
                        outline="black",
                        fill=self.colors[RID - 1],
                        tags=f"robot_task_{RID}",
                    )
                    print(battery)
                    self.visualizer.create_text(
                        ttx + self.x_off,
                        tty + self.y_off,
                        text=f"{int(battery)}",
                        font="Times 8 bold",
                        angle=0,
                        tags=f"robot_taskid_{RID}",
                        fill="blue",
                    )
        # self.visualizer.after(40, self.update_robots_position)

        for node_id in self.last_reserved:
            self.visualizer.itemconfigure(node_id, fill="blue")

        self.last_reserved = []
        self.reservation = self.black_board.read((400, Data.Reservation))
        # print(f"Reservation in visulaizer {self.reservation}")
        for RID, node in self.reservation:
            # print(f"Reservation {node} type:{type(node)}")
            # print(node)
            node_id = self.visualizer.find_withtag(
                f"node_{int(node[0])},{int(node[1])}"
            )

            self.last_reserved.append(node_id)
            # print(node_id)
            self.visualizer.itemconfigure(node_id, fill=self.colors[RID - 1])

    def statistics(self):
        self.total_inbound_completed = self.black_board.read(
            (400, Data.Inbound_Completed)
        )
        obj = self.visualizer.find_withtag(f"fms_inbound_task")

        if obj:
            self.visualizer.itemconfigure(
                obj, text=f"Inbound : {self.total_inbound_completed}"
            )
        else:
            self.visualizer.create_text(
                ((42.0 + self.x_off) * self.object_wscale),
                ((2.0 + self.y_off) * self.object_hscale),
                text=f"Inbound : {self.total_inbound_completed}",
                font="Times 18 bold",
                angle=0,
                tags=f"fms_inbound_task",
                fill="blue",
            )

        self.total_outbound_completed = self.black_board.read(
            (400, Data.Outbound_Completed)
        )

        obj = self.visualizer.find_withtag(f"fms_outbound_task")

        if obj:
            self.visualizer.itemconfigure(
                obj, text=f"Outbound : {self.total_outbound_completed}"
            )
        else:
            self.visualizer.create_text(
                ((42.0 + self.x_off) * self.object_wscale),
                ((4.0 + self.y_off) * self.object_hscale),
                text=f"Outbound : {self.total_outbound_completed}",
                font="Times 18 bold",
                angle=0,
                tags=f"fms_outbound_task",
                fill="blue",
            )

        self.total_time = self.black_board.read((400, Data.Total_Time))
        obj = self.visualizer.find_withtag(f"fms_time")

        if obj:
            self.visualizer.itemconfigure(
                obj, text=f"Time(s) : {round(self.total_time)}"
            )
        else:
            self.visualizer.create_text(
                ((42.0 + self.x_off) * self.object_wscale),
                ((6.0 + self.y_off) * self.object_hscale),
                text=f"Time(s) : {round(self.total_time)}",
                font="Times 18 bold",
                angle=0,
                tags=f"fms_time",
                fill="blue",
            )

        self.inbound_station_state = self.black_board.read(
            (500, Data.Inbound_Station_State)
        )

        self.inb_pos = {
            118: [52.0, 6.0],
            218: [52.0, 9.0],
            154: [52.0, 18.0],
            254: [52.0, 11.0],
            323: [52.0, 13.0],
            1018: [52.0, 15],
        }
        # self.hover = HoverInfo(self, 'while hovering press return \n for an exciting msg',0 , self.print_test )
        # self.command_frame
        for idx in self.inbound_station_state:
            obj = self.visualizer.find_withtag(f"Inbound_Station_State_{idx}")
            if obj:
                self.visualizer.itemconfigure(
                    obj,
                    text=f"Inbound Station : {idx} Buffer : {self.inbound_station_state[idx][0]} Assigned : {self.inbound_station_state[idx][1]}",
                )
            else:
                # pos = self.inb_pos[idx]
                print("Inbound station idx ======>", idx)
                print("Inbound station state ========>", self.inbound_station_state)
                pos = self.inb_pos[idx]
                self.visualizer.create_text(
                    ((pos[0] + self.x_off) * self.object_wscale),
                    ((pos[1] + self.y_off) * self.object_hscale),
                    text=f"Inbound Station : {idx} Buffer : {self.inbound_station_state[idx][0]} Completed : {self.inbound_station_state[idx][1]}",
                    font="Times 14 bold",
                    angle=0,
                    tags=f"Inbound_Station_State_{idx}",
                    fill="blue",
                )

    """
    Initialize Map Configuration
    """

    def initialize_layout(self):
        self.inbound_station = self.black_board.read((200, Data.Inbound))
        self.outbound_station = self.black_board.read((200, Data.Outbound))
        # self.task_station = self.black_board.read((200, Data.Task))
        self.polygons = self.black_board.read((200, Data.Polygon))
        self.lines = self.black_board.read((200, Data.Edge_Line))
        self.points = self.black_board.read((200, Data.Node_Point))
        self.colors = self.black_board.read((200, Data.Color))

        self.parking_station = self.black_board.read((200, Data.Parking_Station))
        self.charging_station = self.black_board.read((200, Data.Charging_station))

        for i in range(-1, 24):
            self.visualizer.create_line(
                (
                    0 + self.x_off,
                    (i - 0.5 + self.y_off) * self.object_hscale,
                    (140.0 + self.x_off) * self.object_wscale,
                    (i - 0.5 + self.y_off) * self.object_hscale,
                ),
                tags="lines",
            )
        for i in range(-1, 110):
            self.visualizer.create_line(
                (
                    ((i) - 0.5 + self.x_off) * self.object_wscale,
                    0 + self.y_off,
                    ((i) - 0.5 + self.x_off) * self.object_wscale,
                    (24.0 + self.y_off) * self.object_hscale,
                ),
                tags="lines",
            )

        for idx, pos in self.charging_station:
            if pos.x < 45:
                self.visualizer.create_text(
                    (pos.x + self.x_off) * self.object_wscale,
                    (pos.y + self.y_off) * self.object_hscale,
                    text=f"CS",
                    font="Times 16 bold",
                )
                self.visualizer.create_image(
                    (pos.x + self.x_off) * self.object_wscale,
                    (pos.y + 0.3 + self.y_off) * self.object_hscale,
                    image=self.charging_station_image,
                )
            else:
                self.visualizer.create_text(
                    (pos.x + 0.5 + self.x_off) * self.object_wscale,
                    (pos.y + self.y_off) * self.object_hscale,
                    text=f"{idx}",
                    font="Times 16 bold",
                )
                self.visualizer.create_image(
                    (pos.x + 1.5 + self.y_off) * self.object_wscale,
                    (pos.y + self.y_off) * self.object_hscale,
                    image=self.charging_station_image,
                )

        for idx, pos, wait, load in self.inbound_station:
            if pos.x < 15:
                self.visualizer.create_text(
                    (pos.x - 1.0 + self.x_off) * self.object_wscale,
                    (pos.y + self.y_off) * self.object_hscale,
                    text=f"IS",
                    font="Times 16 bold",
                )
                self.visualizer.create_image(
                    (pos.x - 2.0 + self.x_off) * self.object_wscale,
                    (pos.y + self.y_off) * self.object_hscale,
                    image=self.inbound_image,
                )
            else:
                self.visualizer.create_text(
                    (pos.x + 1.0 + self.x_off) * self.object_wscale,
                    (pos.y + self.y_off) * self.object_hscale,
                    text=f"IS",
                    font="Times 16 bold",
                )
                self.visualizer.create_image(
                    (pos.x + 2.0 + self.x_off) * self.object_wscale,
                    (pos.y + self.y_off) * self.object_hscale,
                    image=self.inbound_image,
                )

        for idx, pos, wait in self.outbound_station:
            if pos.x < 15:
                self.visualizer.create_text(
                    (pos.x - 1.0 + self.x_off) * self.object_wscale,
                    (pos.y + self.y_off) * self.object_hscale,
                    text=f"OS",
                    font="Times 16 bold",
                )
                self.visualizer.create_image(
                    (pos.x - 2.0 + self.x_off) * self.object_wscale,
                    (pos.y + self.y_off) * self.object_hscale,
                    image=self.outbound_image,
                )
            else:
                self.visualizer.create_text(
                    (pos.x + 1.0 + self.x_off) * self.object_wscale,
                    (pos.y + self.y_off) * self.object_hscale,
                    text=f"OS",
                    font="Times 16 bold",
                )
                self.visualizer.create_image(
                    (pos.x + 2.0 + self.x_off) * self.object_wscale,
                    (pos.y + self.y_off) * self.object_hscale,
                    image=self.outbound_image,
                )

        for key, pt in self.task_station:
            data = [
                (
                    (pt.x - 0.5 + self.x_off) * self.object_wscale,
                    (pt.y + 0.5 + self.y_off) * self.object_hscale,
                ),
                (
                    (pt.x - 0.5 + self.x_off) * self.object_wscale,
                    (pt.y - 0.5 + self.y_off) * self.object_hscale,
                ),
                (
                    (pt.x + 0.5 + self.x_off) * self.object_wscale,
                    (pt.y - 0.5 + self.y_off) * self.object_hscale,
                ),
                (
                    (pt.x + 0.5 + self.x_off) * self.object_wscale,
                    (pt.y + 0.5 + self.y_off) * self.object_hscale,
                ),
            ]
            obj = self.visualizer.create_polygon(
                data,
                outline="black",
                width=2,
                fill="pink",
                joinstyle=tk.ROUND,
                tags=f"task_{key}",
            )
            obj2 = self.visualizer.create_text(
                (pt.x + self.x_off) * self.object_wscale,
                (pt.y + self.y_off) * self.object_hscale,
                text=str(key),
                font="Times 8 bold",
                tags=f"task_{key}",
            )

        for pt in self.points:
            data = [
                (
                    (pt.x - 0.05 + self.x_off) * self.object_wscale,
                    (pt.y + 0.05 + self.y_off) * self.object_hscale,
                ),
                (
                    (pt.x - 0.05 + self.x_off) * self.object_wscale,
                    (pt.y - 0.05 + self.y_off) * self.object_hscale,
                ),
                (
                    (pt.x + 0.05 + self.x_off) * self.object_wscale,
                    (pt.y - 0.05 + self.y_off) * self.object_hscale,
                ),
                (
                    (pt.x + 0.05 + self.x_off) * self.object_wscale,
                    (pt.y + 0.05 + self.y_off) * self.object_hscale,
                ),
            ]
            obj = self.visualizer.create_polygon(
                data, tags=["node", f"node_{int(pt.x)},{int(pt.y)}"], fill="blue"
            )

        for idx, pos in self.parking_station:
            self.visualizer.create_text(
                (pos.x + self.x_off) * self.object_wscale,
                (pos.y + self.y_off) * self.object_hscale,
                text=f"P",
                font="Times 16 bold",
            )

    def event_behaviour(self):
        # check for new event
        if self.event_handler.is_there_event(self.global_key()):
            # Get all event
            event_queue = self.event_handler.get_event(self.global_key())
            print(f"{event_queue.qsize()} and size {event_queue.empty()}")
            # Response to all event
            while not event_queue.empty():
                event = event_queue.get()
                print(event)
                if event[1] == Event.Initialize_Layout:
                    self.initialize_layout()


class HoverInfo(tk.Menu):
    def __init__(self, parent, text, robotId, command=None):
        self.robotId = robotId
        self._com = command
        self.parent = parent
        tk.Menu.__init__(self, parent, tearoff=0)
        if not isinstance(text, str):
            raise TypeError(
                "Trying to initialise a Hover Menu with a non string type: "
                + text.__class__.__name__
            )
        toktext = re.split("\n", text)
        for t in toktext:
            self.add_command(label=t)
            self._displayed = False
            parent.frame.bind("<Enter>", self.Display)
            parent.frame.bind("<Leave>", self.Remove)

    def __del__(self):
        self.parent.frame.unbind("<Enter>")
        self.parent.frame.unbind("<Leave>")

    def Display(self, event):
        if not self._displayed:
            self._displayed = True
            self.post(event.x_root, event.y_root)
        if self._com != None:
            self.frame.unbind_all("<Return>")
            self.frame.bind_all("<Return>", self.Click)

    def Remove(self, event):
        if self._displayed:
            self._displayed = False
            self.unpost()
        if self._com != None:
            self.unbind_all("<Return>")

    def Click(self, event):
        self._com()


class Controller:
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

        self.root = tk.Tk()
        self.root.title("Robot")

        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=1)
        self.root.columnconfigure(2, weight=1)
        self.root.rowconfigure(1, weight=1)
        self.child = []
        self.mode = tk.IntVar()
        self.mode.set(1)

        for modex in Mode:
            rad1 = tk.Radiobutton(
                self.root,
                text=modex.name,
                variable=self.mode,
                value=modex.value,
                command=lambda: self.display(self.mode.get()),
            )
            rad1.grid(row=0, column=modex.value + 1)
            # rad1.pack(side=tk.TOP, expand=1)

        self.last_mode = -1

        self.child = [
            Map_Controller(
                self.root, self.global_id, self.black_board, self.event_handler
            ),
            Simulation_Controller(
                self.root, self.global_id, self.black_board, self.event_handler
            ),
        ]

    def global_key(self):
        return (self.global_id * 100) + self.module_id

    def display(self, value):
        # forget last mode
        if self.last_mode != -1:
            self.child[self.last_mode].frame.grid_forget()
            # self.child[self.last_mode].frame.pack_forget()
        # draw new mode
        self.child[value].frame.grid(row=1, column=0, sticky="NSEW", columnspan=3)
        # self.child[value].frame.pack(side=tk.BOTTOM, expand=1)
        # self.sim_frame.grid(row=1, column=0, columnspan=3)

        self.last_mode = value
