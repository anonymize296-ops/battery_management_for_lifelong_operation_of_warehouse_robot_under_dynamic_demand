from itertools import count
import operator
from os import path
import matplotlib.pyplot as plt
import math
import time
import random

from more_itertools import first
from black_board import *
from data import *
from robot import *
from task_manager import *
from fleet_core import *
from range_tree import *
import threading
import re
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

formater = logging.Formatter("%(asctime)s:%(name)s:%(message)s")

file_handler = logging.FileHandler("schedule.log")
file_handler.setLevel(logging.DEBUG)
file_handler.setFormatter(formater)

logger.addHandler(file_handler)

"""
Collision Avoidance through Centralized Fleet
Check collision segment and allow message between robot
"""


class FMS:
    def __init__(
        self, g_id, global_black_board, global_event_handler, planner, range_tree
    ):
        self.robot_list = {}
        self.coupling_graph = {}
        self.global_id = g_id
        self.module_id = 0
        self.id = 0
        self.black_board = global_black_board
        self.event_handler = global_event_handler
        self.manager_list = {}

        # self.planner = Task_Planner(self.global_id, 1,self.black_board, self.event_handler)
        self.planner = planner
        # self.fleet_core = Priority_Algorithm()
        self.range_tree = range_tree
        self.fleet_core = Global_Planner(self.range_tree, self.black_board)

        self.on_going_plan = {}
        self.reservation = []
        self.unreserve = []
        self.high_priority_robot = []
        self.nodes = []
        self.waitspot = []
        self.current_task = []
        self.buffer = 5
        self.initiate = False
        self.first_run = True
        self.robot_member = []

        self.total_inbound_task_completed = 0
        self.total_outbound_task_completed = 0
        self.total_time = 0.0
        self.initialized_range_tree = False
        self.setup()

    def global_key(self):
        return (self.global_id * 100) + self.module_id

    def setup(self):
        # Data
        # self.black_board.register(self.global_key(), 1, Data.Start)
        self.black_board.register(self.global_key(), 0, Data.Node_Point)
        self.black_board.register(self.global_key(), 1, Data.Robot_Member)
        self.black_board.register(self.global_key(), 1, Data.Inbound_Completed)
        self.black_board.register(self.global_key(), 1, Data.Outbound_Completed)
        self.black_board.register(self.global_key(), 1, Data.Total_Time)

        self.black_board.write(
            (self.global_key(), Data.Inbound_Completed),
            self.total_inbound_task_completed,
        )
        self.black_board.write(
            (self.global_key(), Data.Outbound_Completed),
            self.total_outbound_task_completed,
        )
        self.black_board.write((self.global_key(), Data.Total_Time), self.total_time)
        # Event [my_key, subscribe_event_key ]
        # self.event_handler.register(self.global_key(),(2 *100 +0, Event.Start), 1)
        self.event_handler.register(
            self.global_key(), (2 * 100 + 0, Event.Initialize_Layout), 0
        )

    def initialize_range_tree(self):
        self.nodes = self.black_board.read((200, Data.Node_Point))

        if self.nodes is None:
            self.initialized_range_tree = False
            return

        # Initialize range Tree
        r_point = []
        for point in self.nodes:
            r_point.append([int(point.x), int(point.y)])

        self.range_tree.set_points(r_point)
        self.range_tree.sort_x()
        self.range_tree.get_and_store_ysubtree()

        self.initialized_range_tree = True

    def initialize_fleet_core(self):
        self.nodes = self.black_board.read((200, Data.Node_Point))
        self.waitspot = self.black_board.read((200, Data.Waitspot))

        # initialize goal node
        for point in self.nodes:
            if point.y == 17:
                self.fleet_core.planner_data.goal_node[(int(point.x), int(point.y))] = 1
                # self.fleet_core.goal_node[f'{int(point.x)},{point.y}'] = 1
        # initialize waitspot
        for point in self.waitspot:
            self.fleet_core.planner_data.waitspot[(int(point.x), int(point.y))] = 1

        # Initialize pathnode
        for point in self.nodes:
            self.fleet_core.priority_algorithm.pathnode[
                (int(point.x), int(point.y))
            ] = BST()
            # self.fleet_core.pathnode[f'{int(point.x)},{int(point.y)}'] = BST()

        for point in self.nodes:
            for angle in [0, 1, 2, 3]:
                self.on_going_plan[(int(point.x), int(point.y), angle)] = []

        self.initiate = True

    def initialize_agent_path(self):
        logger.info("Start of initiate path")
        self.print_on_going_reservation()
        for robot_id in self.robot_list:
            curr_index = self.robot_list[robot_id].curr_index
            goal_index = self.robot_list[robot_id].goal_index
            logger.info(
                f"Robot {robot_id} logger info Curr Index : {curr_index} , Goal Index : {goal_index}"
            )
            if curr_index is None:
                continue
            plan_index = -1

            test_path = {}
            # for i in range(self.buffer + 1):
            for i in range(len(self.robot_list[robot_id].full_path)):
                if (curr_index + i) < len(self.robot_list[robot_id].full_path):
                    future_index = curr_index + i
                    plan_node = self.robot_list[robot_id].full_path[future_index]
                    plan_nodes = self.fleet_core.planner_data.get_all_collision(
                        plan_node[0], plan_node[1], plan_node[2]
                    )

                    if future_index < goal_index:
                        for plan_node in plan_nodes:
                            if plan_node in test_path:
                                test_path[plan_node] += 1
                            else:
                                test_path[plan_node] = 1
                    else:
                        for plan_node in plan_nodes:
                            if plan_node not in test_path:
                                test_path[plan_node] = 0

            # logger.info(f"Before Plan Path Index {plan_index}")
            # for node in test_path:
            #    logger.info(f"Node : {node} Count : {test_path[node]}")
            #    logger.info(f"Node : {node} OnGoingPlan : {self.on_going_plan[node]}")

            path_node = {}
            # for i in range(self.buffer + 1):
            for i in range(len(self.robot_list[robot_id].full_path)):
                if (curr_index + i) < len(self.robot_list[robot_id].full_path):
                    future_index = curr_index + i
                    plan_node = self.robot_list[robot_id].full_path[future_index]
                    plan_nodes = self.fleet_core.planner_data.get_all_collision(
                        plan_node[0], plan_node[1], plan_node[2]
                    )
                    if future_index <= goal_index:
                        plan_index = curr_index + i

                    if future_index < goal_index:
                        for plan_node in plan_nodes:
                            if plan_node in path_node:
                                path_node[plan_node] += 1
                            else:
                                path_node[plan_node] = 1
                    else:
                        for plan_node in plan_nodes:
                            if plan_node not in path_node:
                                path_node[plan_node] = 0
            # update OnGoing
            # logger.info(f"After Plan Path Index {plan_index}")
            for node in path_node:
                robot_count = 0
                for agent_id in self.on_going_plan[node]:
                    if robot_id == agent_id:
                        robot_count += 1

                remove = robot_count - path_node[node]

                self.on_going_plan[node].reverse()
                for i in range(remove):
                    self.on_going_plan[node].remove(robot_id)
                self.on_going_plan[node].reverse()
                # logger.info(f"Node : {node} OnGoindPlan Updated : {self.on_going_plan[node]}")

            if (curr_index == goal_index) and (curr_index == 0):
                for node in self.on_going_plan:
                    while robot_id in self.on_going_plan[node]:
                        self.on_going_plan[node].remove(robot_id)

            self.robot_list[robot_id].plan_index = plan_index
            robot_path = self.robot_list[robot_id].full_path[plan_index:]
            robot_path_corrected = []
            for node in robot_path:
                # data = re.split(r',',node)
                robot_path_corrected.append(node)
            self.fleet_core.planner_data.agents_path[robot_id] = robot_path_corrected
            logger.info(
                f"Robot {robot_id} full path {self.robot_list[robot_id].path_node} curr_index:{curr_index} goal_index:{goal_index} plan_index:{plan_index} Path:{robot_path_corrected}"
            )

        self.print_on_going_reservation()
        logger.info("End of initiate path")
        self.black_board.write((self.global_key(), Data.Unreserve), self.unreserve)

    """
    Add robot in fleet
    """

    def add_robot(self):
        self.robot_member = []
        self.robot_list[self.id] = Robot(
            3, self.id, self.black_board, self.event_handler, self.planner
        )
        self.id += 1

        for robot in self.robot_list:
            self.robot_member.append(self.robot_list[robot].global_key())

        self.black_board.write(
            (self.global_key(), Data.Robot_Member), self.robot_member
        )

    def print_on_going_reservation(self):
        self.reservation = []
        # logger.info("===============================================================>")
        for node in self.on_going_plan.keys():
            if len(self.on_going_plan[node]) > 0:
                self.reservation.append([self.on_going_plan[node][0], node])
                # logger.info(f"{node} {self.on_going_plan[node]}")
        # logger.info("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
        self.black_board.write((self.global_key(), Data.Reservation), self.reservation)

    def clear_on_going_reservation(self):
        for node in self.on_going_plan:
            if len(self.on_going_plan[node]) > 0:
                curr_robot = self.on_going_plan[node][0]

                found = False
                for i in range(
                    self.robot_list[curr_robot].curr_index,
                    self.robot_list[curr_robot].curr_index + self.buffer + 1,
                ):
                    if i < len(self.robot_list[curr_robot].path_node):
                        path_node = self.robot_list[curr_robot].path_node[i]
                        if node[0] == path_node[0] and node[1] == path_node[1]:
                            found = True

                # delete reservation
                if not found:
                    self.on_going_plan[node].pop(0)
                    logger.info(
                        f"DELETED RESERVATION Robot:{curr_robot} Node:{node}********************"
                    )

    def print_time_schedule(self):
        # print("===============================================================>")
        logger.info("**********************************************************")
        for node in self.fleet_core.planner_data.time_schedule.keys():
            if len(self.fleet_core.planner_data.time_schedule[node]) > 0:
                # print(f"Node:{node} Robot:{self.fleet_core.planner_data.time_schedule[node]}")
                logger.info(
                    f"Node:{node} Robot:{self.fleet_core.planner_data.time_schedule[node]}"
                )
        # print("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX")
        logger.info("**********************************************************")

    def update_schedule(self):
        self.high_priority_robot = []
        for agent in self.robot_list:
            curr_index = self.robot_list[agent].curr_index
            if curr_index is None:
                continue

            curr_node = None
            try:
                curr_node = self.robot_list[agent].path_node[curr_index]
            except IndexError:
                pass

            if (curr_index + 1) < len(self.robot_list[agent].path_node):
                next_index = curr_index + 1
                next_node = self.robot_list[agent].path_node[next_index]
                path_node = {}
                # Update OnGoing Reservation from time schedule
                for i in range(self.buffer):
                    if (curr_index + i) < len(self.robot_list[agent].path_node):
                        future_index = curr_index + i
                        future_path_node = self.robot_list[agent].path_node[
                            future_index
                        ]

                        future_nodes = self.fleet_core.planner_data.get_all_collision(
                            future_path_node[0],
                            future_path_node[1],
                            future_path_node[2],
                        )
                        ######FOR ALL POINTS
                        for future_node in future_nodes:
                            if future_node in path_node:
                                path_node[future_node] += 1
                            else:
                                path_node[future_node] = 1
                        # update OnGoing
                        for future_node in future_nodes:
                            found = False
                            count = 0
                            planned_robot = self.on_going_plan[future_node]
                            for planned_rid in planned_robot:
                                if planned_rid == agent:
                                    count += 1

                            if count >= path_node[future_node]:
                                found = True

                            if not found:
                                first = -1
                                for idx in range(
                                    len(
                                        self.fleet_core.planner_data.time_schedule[
                                            future_node
                                        ]
                                    )
                                ):
                                    if (
                                        self.fleet_core.planner_data.time_schedule[
                                            future_node
                                        ][idx]
                                        == agent
                                    ):
                                        first = idx
                                        break

                                if first != -1:
                                    for idx in range(first + 1):
                                        r_id = (
                                            self.fleet_core.planner_data.time_schedule[
                                                future_node
                                            ].pop(0)
                                        )
                                        self.on_going_plan[future_node].append(r_id)
                            else:
                                continue

                path_node = {}

                for i in range(self.buffer + 1):
                    next_index = curr_index + i

                    if (next_index + 1) > len(self.robot_list[agent].path_node):
                        break

                    if not self.robot_list[agent].path_node_reservation[next_index]:
                        continue
                    next_path_node = self.robot_list[agent].path_node[next_index]
                    next_nodes = self.fleet_core.planner_data.get_all_collision(
                        next_path_node[0], next_path_node[1], next_path_node[2]
                    )
                    for next_node in next_nodes:
                        if next_node in path_node:
                            path_node[next_node] += 1
                        else:
                            path_node[next_node] = 1
                    # Reservation
                    reserved = True
                    if (len(self.on_going_plan[next_path_node])) >= path_node[
                        next_path_node
                    ]:
                        count = 0
                        for r_id in self.on_going_plan[next_path_node]:
                            if r_id == agent:
                                count += 1
                            else:
                                break

                        if count < path_node[next_path_node]:
                            reserved = False
                            break
                    else:
                        reserved = False
                        break

                    if reserved:
                        self.robot_list[agent].goal_index = next_index
                    else:
                        break

        self.black_board.write(
            (self.global_key(), Data.High_Priority_robot), self.high_priority_robot
        )

    def unreservation(self):
        self.unreserve = []
        for agent in self.robot_list.keys():
            if self.robot_list[agent].curr_index is None:
                continue
            curr_index = self.robot_list[agent].curr_index
            for i in range(1, 4):
                prev_index = curr_index - i
                if prev_index < 0:
                    continue

                if self.robot_list[agent].path_node_reservation[prev_index]:
                    prev_node = self.robot_list[agent].path_node[prev_index]
                    reserved = True
                    prev_nodes = [prev_node]
                    for prev_node in prev_nodes:
                        if len(self.on_going_plan[prev_node]) == 0:
                            reserved = False
                            break
                        if self.on_going_plan[prev_node][0] != agent:
                            reserved = False
                            break

                    prev_nodes = self.fleet_core.planner_data.get_all_collision(
                        prev_node[0], prev_node[1], prev_node[2]
                    )
                    if reserved:
                        for prev_node in prev_nodes:
                            value = self.on_going_plan[prev_node].remove(agent)
                            self.unreserve.append(
                                (int(prev_node[0]), int(prev_node[1]))
                            )
                        self.robot_list[agent].path_node_reservation[prev_index] = False

        self.black_board.write((self.global_key(), Data.Unreserve), self.unreserve)

    """
    Behaviour of fleet interface
    """

    def behaviour(self):
        self.total_time += 0.04

        if not self.initialized_range_tree:
            self.initialize_range_tree()

        # Run controller for period (0.3)
        for robot_id in self.robot_list:
            self.robot_list[robot_id].behaviour()
            self.robot_list[robot_id].event_behaviour()

            if self.robot_list[robot_id].inbound_task_completed:
                self.total_inbound_task_completed += 1
            elif self.robot_list[robot_id].outbound_task_completed:
                self.total_outbound_task_completed += 1
            elif self.robot_list[robot_id].outbound_load_task_completed:
                self.total_outbound_task_completed += 1

            self.black_board.write(
                (self.global_key(), Data.Inbound_Completed),
                self.total_inbound_task_completed,
            )
            self.black_board.write(
                (self.global_key(), Data.Outbound_Completed),
                self.total_outbound_task_completed,
            )

        replan = False
        for robot_id in self.robot_list:
            if self.robot_list[robot_id].got_new_plan:
                print(f"{robot_id} got new plan")
                replan = True
                self.robot_list[robot_id].got_new_plan = False

        self.unreservation()

        if replan:
            if not self.initiate:
                self.initialize_fleet_core()
            print("Replaningg")
            start_time = time.time()
            # self.clear_on_going_reservation()
            self.initialize_agent_path()

            for node in self.on_going_plan:
                if len(self.on_going_plan[node]) > 0:
                    logger.info(
                        f"Node:{node} -> OnGoingPlan:{self.on_going_plan[node]}"
                    )

            self.fleet_core.solve()

            end_time = time.time()
            print(f"Time taken to solve priority algorithm {end_time - start_time}")
            for node in self.fleet_core.planner_data.time_schedule:
                if len(self.fleet_core.planner_data.time_schedule[node]) > 1:
                    print(
                        f"{node}: {self.fleet_core.planner_data.time_schedule[node]}",
                        end=",,",
                    )

            # update path
            for robot_id in self.robot_list:
                curr_index = self.robot_list[robot_id].curr_index

                if curr_index is None:
                    continue
                new_path = []
                new_reservation = []
                for i in range(0, self.robot_list[robot_id].goal_index):
                    new_path.append(self.robot_list[robot_id].path_node[i])

                for i in range(self.robot_list[robot_id].goal_index):
                    status = self.robot_list[robot_id].path_node_reservation[i]
                    new_reservation.append(status)

                reach = False
                target_node = (
                    int(self.robot_list[robot_id].final_goal.x),
                    int(self.robot_list[robot_id].final_goal.y),
                )

                for i in range(len(self.fleet_core.planner_data.agents_path[robot_id])):
                    node = self.fleet_core.planner_data.agents_path[robot_id][i]
                    # pos_node = self.fleet_core.planner.pos_to_node(node)
                    new_path.append(node)
                    if not reach:
                        new_reservation.append(True)

                    if node[0] == target_node[0] and node[1] == target_node[1]:
                        reach = True

                print(f"Robot:{robot_id} old path")
                for node in self.robot_list[robot_id].path_node:
                    print(node, end=",")
                print()
                self.robot_list[robot_id].full_path = new_path

                self.robot_list[robot_id].path_node_reservation = new_reservation

                target_node = (
                    int(self.robot_list[robot_id].final_goal.x),
                    int(self.robot_list[robot_id].final_goal.y),
                )
                self.robot_list[robot_id].path_node = []
                for i in range(len(self.robot_list[robot_id].full_path)):
                    full_node = self.robot_list[robot_id].full_path[i]
                    self.robot_list[robot_id].path_node.append(full_node)
                    if (
                        full_node[0] == target_node[0]
                        and full_node[1] == target_node[1]
                    ):
                        break

                print(f"Robot:{robot_id} new path")
                for node in self.robot_list[robot_id].path_node:
                    print(node, end=",")
                print()

                logger.info(f"Robot:{robot_id} Path Update ****************")
                logger.info(
                    f"Planned Path: {self.fleet_core.planner_data.agents_path[robot_id]}"
                )
                logger.info(
                    f"Robot target node: {target_node} and Goal index: {self.robot_list[robot_id].goal_index}"
                )
                logger.info(f"Robot full Path: {self.robot_list[robot_id].full_path}")
                logger.info(f"Robot path node: {self.robot_list[robot_id].path_node}")
                logger.info(
                    f"RObot path reservation {self.robot_list[robot_id].path_node_reservation}"
                )
                logger.info(f"Robot:{robot_id} *****************************")
                # self.robot_list[robot_id].path_node_reservation = new_reservation
                # self.robot_list[robot_id].get_subgoal()

            self.first_run = True

        self.update_schedule()

        self.print_on_going_reservation()

        # self.clear_on_going_reservation()

        self.black_board.write((self.global_key(), Data.Total_Time), self.total_time)

    def event_behaviour(self):
        # check for new event
        if self.event_handler.is_there_event(self.global_key()):
            # Get all event
            event_queue = self.event_handler.get_event(self.global_key())
            print(f"Insdie fleet {event_queue.qsize()} and size {event_queue.empty()}")
            # Response to all event
            while not event_queue.empty():
                event = event_queue.get()
                print(f"Inside fleet {event}")
                if event[1] == Event.Initialize_Layout:
                    self.initialize_fleet_core()
