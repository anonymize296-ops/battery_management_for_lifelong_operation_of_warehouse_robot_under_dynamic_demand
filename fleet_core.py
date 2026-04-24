from cmath import log
from email import iterators
from enum import Enum

# import imp
from importlib.resources import path
from logging.handlers import WatchedFileHandler
import time
import networkx as nx
from datastructure import *
from operator import itemgetter
from itertools import permutations
from geometry import *
import re
import logging
import copy
from black_board import *
from operator import itemgetter

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

formater = logging.Formatter("%(asctime)s:%(name)s:%(message)s")

file_handler = logging.FileHandler("fleet_core.log")
file_handler.setLevel(logging.DEBUG)
file_handler.setFormatter(formater)

logger.addHandler(file_handler)


class Direction(Enum):
    Left = 0
    Right = 1
    Up = 2
    Down = 3


class Planner_Data:
    def __init__(self, range_tree):
        # input data necessary for all algorithm
        self.agents_path = {}
        self.width = 0.45
        self.length = 0.45
        self.angle_to_footprint = {
            0: [self.length, self.width],
            1: [self.width, self.length],
            2: [self.length, self.width],
            3: [self.width, self.length],
        }
        self.outer_box = 3.0  # 3m width and length
        self.range_tree = range_tree
        # 1.Priority Algorithm input
        self.goal_node = {}
        self.end_node = {}
        self.deadlock_nodes = set()

        # 2.LazyWait input
        self.waitspot = {}
        self.assigned_waitspot = {}

        # output data of all algorithm
        # 1.Priority Algorithm output
        self.time_schedule = {}
        self.deadlock = {}
        self.deadlock_cycle = nx.DiGraph()
        self.robot_start_idx = {}
        self.deadlock_nodes = set()
        # 2.Lazy Wait
        self.robot_waitspot_path = {}

    def print_path(self):
        for robot_id in self.agents_path:
            logger.info(f"Robot:{robot_id}:Path:{self.agents_path[robot_id]}")

    def print_time_schedule(self):
        logger.info("Start of schedule")
        for node in self.time_schedule:
            if len(self.time_schedule[node]) > 0:
                logger.info(f"Node:{node}:{self.time_schedule[node]},")
        logger.info("End of schedule")

    def to_gnode(self, node):
        theta = -1
        if node[2] == 0 or node[2] == 2:
            theta = 0
        elif node[2] == 1 or node[2] == 3:
            theta = 1
        else:
            theta = -1
            logger.info(f"Error in theta conversion")

        return (node[0], node[1], theta)

    """
    Given node(x,y) position and discretize angle [0,1,2,3] [0,90,180,-90], return all the node angle pair footprint collision
    """

    def get_all_collision(self, x, y, rangle):
        # Get all the point inside big box
        dx = self.outer_box
        dy = self.outer_box
        self.range_tree.box_range(x - dx, x + dx, y - dy, y + dy)

        [r_dx, r_dy] = self.angle_to_footprint[rangle]
        ego_rect = Rectangle(x - r_dx, y - r_dy, x + r_dx, y + r_dy)

        overlap_pos = []
        for points in self.range_tree.inside_points:
            for obs_angle in [0, 1, 2, 3]:
                [obs_dx, obs_dy] = self.angle_to_footprint[obs_angle]
                obs_rect = Rectangle(
                    points[0] - obs_dx,
                    points[1] - obs_dy,
                    points[0] + obs_dx,
                    points[1] + obs_dy,
                )

                if ego_rect.is_rectangle_overlap(obs_rect):
                    overlap_pos.append((points[0], points[1], obs_angle))

        return overlap_pos


class MultiMachine_Scheduling:
    def __init__(self, planner_data):
        self.planner_data = planner_data
        self.agents = []
        self.time_schedule = {}

    def initiate(self, agents):
        self.agents = agents

    """
    Smaller path length gets high priority
    """

    def earlier_finish_first(self):
        least_priority = []

        for agent in self.agents:
            least_priority.append([agent, len(self.planner_data.agents_path[agent])])

        modified_least_priority = sorted(least_priority, key=itemgetter(1))
        modified_least_priority.reverse()

        robot_order = []
        logger.info(f"Robot and Path {modified_least_priority}")

        for agent, path in modified_least_priority:
            robot_order.append(agent)

        logger.info(f"Earlier Finish First Robot order {robot_order}")

        return robot_order

    def longer_path_first(self):
        least_priority = []

        for agent in self.agents:
            least_priority.append([agent, len(self.planner_data.agents_path[agent])])

        modified_least_priority = sorted(least_priority, key=itemgetter(1))

        robot_order = []
        logger.info(f"Robot and Path {modified_least_priority}")

        for agent, path in modified_least_priority:
            robot_order.append(agent)

        logger.info(f"Longer Path First Robot order {robot_order}")

        return robot_order

    """
    Measure quality of solution
    """

    def set_schedule(self, robot_order):
        self.time_schedule = {}
        # update schedule
        for i in range(len(robot_order) - 1, -1, -1):
            agent = robot_order[i]
            for j in range(
                self.planner_data.robot_start_idx[agent],
                len(self.planner_data.agents_path[agent]),
            ):
                node = self.planner_data.agents_path[agent][j]
                if node in self.time_schedule:
                    self.time_schedule[node].append(agent)
                else:
                    self.time_schedule[node] = [agent]

    def find_overall_finish_time(self, robot_order, verbose):
        traversal_node = {}
        for agent in robot_order:
            for node in self.planner_data.agents_path[agent][
                self.planner_data.robot_start_idx[agent] :
            ]:
                traversal_node[node] = 0

        agents_idx = {}
        agents_path_time = {}
        agents_finish_time = {}
        agents_done = {}
        agents_count = {}

        count = 8
        for agent in robot_order:
            agents_idx[agent] = self.planner_data.robot_start_idx[agent]
            agents_path_time[agent] = len(self.planner_data.agents_path[agent]) - 1
            agents_finish_time[agent] = 0
            agents_done[agent] = False
            agents_count[agent] = 0

        iteration = 0
        for agent in robot_order:
            iteration += agents_path_time[agent] + max(agents_path_time[agent], 10)
        # Check Initial schedule validity
        for agent in robot_order:
            start_node = self.planner_data.agents_path[agent][
                self.planner_data.robot_start_idx[agent]
            ]
            if agent != self.time_schedule[start_node][0]:
                return 100 * iteration

        for time in range(iteration):
            finish = True
            for agent in robot_order:
                if not agents_done[agent]:
                    finish = False
                    break
            if finish:
                if verbose:
                    logger.info(f"Agent Finish Time {agents_finish_time}")
                    logger.info(f"Agent Path Time {agents_path_time}")
                overall_wait_time = 0
                for agent in robot_order:
                    overall_wait_time += (
                        agents_finish_time[agent] - agents_path_time[agent]
                    )

                if verbose:
                    logger.info(f"Overall Solution Quality {overall_wait_time}")

                return overall_wait_time

            for agent in robot_order:
                if agents_done[agent]:
                    continue

                if (agents_idx[agent] + 1) >= len(self.planner_data.agents_path[agent]):
                    agents_count[agent] += 1

                    if agents_count[agent] >= count:
                        agents_done[agent] = True
                        agents_finish_time[agent] = (
                            time + self.planner_data.robot_start_idx[agent]
                        )
                        curr_node = self.planner_data.agents_path[agent][
                            agents_idx[agent]
                        ]
                        traversal_node[curr_node] += 1
                    else:
                        continue

                if agents_done[agent]:
                    continue

                next_node = self.planner_data.agents_path[agent][agents_idx[agent] + 1]
                robot_id = self.time_schedule[next_node][traversal_node[next_node]]

                if robot_id == agent:
                    curr_node = self.planner_data.agents_path[agent][agents_idx[agent]]
                    traversal_node[curr_node] += 1
                    agents_idx[agent] += 1

    def dfs(self, robot_order):
        variable = robot_order
        value = robot_order
        node_list = []
        for i in range(len(value) - 1, -1, -1):
            node_list.append([value[i]])

        while len(node_list) > 0:
            curr_node = node_list.pop()

            # fill partial order
            if len(curr_node) != len(variable):
                for i in range(len(value) - 1, -1, -1):
                    if value[i] in curr_node:
                        continue
                    new_node = []
                    for val in curr_node:
                        new_node.append(val)
                    new_node.append(value[i])
                    node_list.append(new_node)
            else:
                print(curr_node)

    def solve(self):
        best_soln_quality = 100000
        best_robot_order = []

        # Strategy I:
        robot_order = self.earlier_finish_first()

        self.set_schedule(robot_order)
        soln_quality = self.find_overall_finish_time(robot_order, False)

        logger.info(
            f"Earlier Finish Time Robot order:{robot_order} Soln Quality:{soln_quality}"
        )

        if soln_quality <= best_soln_quality:
            best_robot_order = robot_order
            best_soln_quality = soln_quality

        # Strategy II:
        # robot_order = self.longer_path_first()

        # self.set_schedule(robot_order)
        # soln_quality = self.find_overall_finish_time(robot_order)

        # logger.info(f"Longer Path First Robot Order:{robot_order} Soln Quality:{soln_quality}")

        # if soln_quality <= best_soln_quality:
        #    best_robot_order = robot_order
        #    best_soln_quality = soln_quality

        # Strategy III:
        # return self.agents
        # start_time = time.time()
        # best_robot_order = self.brute_force(self.agents)
        # end_time = time.time()
        logger.info(
            f"Robot Idx:{self.planner_data.robot_start_idx} Best Robot Order:{best_robot_order}"
        )
        # logger.info(f"Overall time takent by BruteForce {end_time - start_time}")
        return best_robot_order

    def brute_force(self, robot_order):
        perm = permutations(robot_order)

        best_sol = 10000
        best_robot_order = None

        for robots in perm:
            self.set_schedule(robots)
            sol = self.find_overall_finish_time(robots, False)
            if sol <= best_sol:
                best_sol = sol
                best_robot_order = robots

        self.set_schedule(best_robot_order)
        self.find_overall_finish_time(best_robot_order, True)
        return best_robot_order


class Path_Planner:
    def __init__(self):
        self.graph = nx.read_gml("./config/research_layout_2/layout_d.gml")
        self.Grid = []
        self.row = -1
        self.col = -1
        self.res = -1
        self.pos2node = {}
        self.store_node_pos()

        self.set_weight()

    def store_node_pos(self):
        for node in self.graph.nodes:
            pos = self.graph.nodes[node]["pos"]
            self.pos2node[(int(pos[0]), int(pos[1]))] = node

    def set_weight(self):
        for edge in self.graph.edges:
            self.graph[edge[0]][edge[1]]["weight"] = 1.0

    def node_to_pos(self, node):
        return (self.graph.nodes[node]["pos"][0], self.graph.nodes[node]["pos"][1])

    def pos_to_node(self, pos):
        return self.pos2node[(int(pos[0]), int(pos[1]))]

    def plan(self, source, target):
        source = self.pos_to_node(source)
        target = self.pos_to_node(target)
        path = nx.dijkstra_path(self.graph, source, target)
        pos_path = []
        for node in path:
            pos_path.append(self.node_to_pos(node))
        return pos_path

    """
    Return total cost of the given path
    """

    def cost(self, path):
        cost = 0.0
        prev_node = self.pos_to_node(path[0])
        for i in range(1, len(path)):
            curr_node = self.pos_to_node(path[i])
            cost += self.graph[prev_node][curr_node]["weight"]
            prev_node = curr_node

        return cost

    def block_path(self, path):
        if len(path) == 0:
            return
        prev_node = self.pos_to_node(path[0])
        for i in range(1, len(path)):
            curr_node = self.pos_to_node(path[i])
            if prev_node != curr_node:
                if curr_node in self.graph[prev_node]:
                    self.graph[prev_node][curr_node]["weight"] = 2000.0
            prev_node = curr_node

    def reset_path(self, path):
        if len(path) == 0:
            return
        prev_node = self.pos_to_node(path[0])
        for i in range(1, len(path)):
            curr_node = self.pos_to_node(path[i])
            if prev_node != curr_node:
                self.graph[prev_node][curr_node]["weight"] = 1.0
            prev_node = curr_node


class Priority_Algorithm:
    def __init__(self, planner_data, range_tree):
        self.planner_data = planner_data
        self.range_tree = range_tree
        self.pathnode = {}
        self.high_priority_agent = []
        self.start_node = {}
        self.max_iteration = 5000

        self.schedular = MultiMachine_Scheduling(self.planner_data)
        # self.goal_node = {}

    def set_start_node(self):
        self.start_node = {}
        for agent in self.high_priority_agent:
            node = self.planner_data.agents_path[agent][
                self.planner_data.robot_start_idx[agent]
            ]
            self.start_node[node] = agent

    def balanced_iterator(self, n):
        iterator = []
        count = 0
        while count < n:
            left = int(n / 2) - count
            right = int(n / 2) + count
            if left < 0 and right > n:
                break
            else:
                if (count > 0) and (left >= 0):
                    iterator.append(left)
                if right < n:
                    iterator.append(right)
                count += 1
        return iterator

    def get_direction(self, prev_node, curr_node):
        diff_x = curr_node[0] - prev_node[0]
        diff_y = curr_node[1] - prev_node[1]
        direction = None
        if diff_x > 0 and diff_y == 0:
            # if curr_node[2] == 0:
            direction = Direction.Right
        elif diff_x < 0 and diff_y == 0:
            # elif curr_node[2] == 2:
            direction = Direction.Left
        elif diff_y > 0 and diff_x == 0:
            # elif curr_node[2] == 1:
            direction = Direction.Down
        elif diff_y < 0 and diff_x == 0:
            # elif curr_node[2] == 3:
            direction = Direction.Up
        else:
            if curr_node[2] == 0:
                direction = Direction.Right
            elif curr_node[2] == 2:
                direction = Direction.Left
            elif curr_node[2] == 1:
                direction = Direction.Down
            elif curr_node[2] == 3:
                direction = Direction.Up

            # logger.error(f"Wrong direction Current_Node:{curr_node} Previous_Node:{prev_node}")

        return direction

    def is_block_direction(self, r_dir, ego_dir):
        if r_dir == Direction.Up and ego_dir == Direction.Down:
            return True
        elif r_dir == Direction.Down and ego_dir == Direction.Up:
            return True
        elif r_dir == Direction.Left and ego_dir == Direction.Right:
            return True
        elif r_dir == Direction.Right and ego_dir == Direction.Left:
            return True
        else:
            return False

    def is_footprint_collision(self, node, r_id, block_node):
        is_collision = False
        self.pos = self.planner_data.get_all_collision(node[0], node[1], node[2])
        for x, y, angle in self.pos:
            if (
                (x == block_node[0])
                and (y == block_node[1])
                and (angle == block_node[2])
            ):
                is_collision = True
                return is_collision
            gnode = self.planner_data.to_gnode((x, y, angle))
            if gnode in self.robot_path_node:
                self.robot_path_node[gnode].add(r_id)
            else:
                self.robot_path_node[gnode] = set({r_id})

        return is_collision

    def is_footprint_overlap(self, node, block_node):
        is_collision = False
        self.pos = self.planner_data.get_all_collision(node[0], node[1], node[2])
        for x, y, angle in self.pos:
            if (
                (x == block_node[0])
                and (y == block_node[1])
                and (angle == block_node[2])
            ):
                is_collision = True
                return is_collision

        return is_collision

    def deadlock_node(self, agent, agent_idx, block_robot, block_idx):
        self.robot_path_node = {}
        agent_last_idx = -1
        block_last_idx = -1
        agent_node = self.planner_data.agents_path[agent][agent_idx]
        block_node = self.planner_data.agents_path[block_robot][block_idx]
        # Move agent Until Collision with block robot or End
        agent_collision = False
        for j in range(agent_idx, len(self.planner_data.agents_path[agent])):
            node = self.planner_data.agents_path[agent][j]
            if self.is_footprint_collision(node, agent, block_node):
                agent_collision = True
                agent_last_idx = j
                break

        # End No Deadlock
        if not agent_collision:
            return (None, None, True)
        # Collision, Move block robot untill collision with agent or End
        block_collision = False
        for j in range(block_idx, len(self.planner_data.agents_path[block_robot])):
            node = self.planner_data.agents_path[block_robot][j]
            if self.is_footprint_collision(node, block_robot, agent_node):
                block_collision = True
                block_last_idx = j
                break
        # END from Last Deadlock node
        if not block_collision:
            for j in range(agent_last_idx, agent_idx - 1, -1):
                node = self.planner_data.agents_path[agent][j]
                gnode = self.planner_data.to_gnode(node)
                self.planner_data.deadlock_nodes.add(gnode)
                found = True
                if len(self.robot_path_node[gnode]) != 1:
                    found = False
                if found:
                    return (node, j, True)
            node = self.planner_data.agents_path[agent][agent_idx]
            return (node, agent_idx, True)

        # Collision, possible deadlock (Four Cases)
        agent_free_node = False
        agent_free_idx = -1
        block_free_node = False
        block_free_idx = -1

        for j in range(agent_idx + 1, agent_last_idx):
            node = self.planner_data.agents_path[agent][j]
            gnode = self.planner_data.to_gnode(node)
            found = True
            if len(self.robot_path_node[gnode]) != 1:
                found = False
            if found:
                agent_free_node = True
                agent_free_idx = j
                break

        for j in range(block_idx + 1, block_last_idx):
            node = self.planner_data.agents_path[block_robot][j]
            gnode = self.planner_data.to_gnode(node)
            found = True
            if len(self.robot_path_node[gnode]) != 1:
                found = False
            if found:
                block_free_node = True
                block_free_idx = j
                break

        if (not agent_free_node) and (not block_free_node):
            # all nodes until other robot position
            first_hit = False
            for j in range(agent_idx, len(self.planner_data.agents_path[agent])):
                node = self.planner_data.agents_path[agent][j]
                gnode = self.planner_data.to_gnode(node)
                self.planner_data.deadlock_nodes.add(gnode)
                if self.is_footprint_overlap(node, block_node):
                    if not first_hit:
                        first_hit = True
                else:
                    if first_hit:
                        break
            return (self.planner_data.agents_path[agent][agent_idx], agent_idx, False)
        elif (not agent_free_node) and block_free_node:
            (coll_node, coll_idx, enter) = self.deadlock_node(
                agent, agent_idx, block_robot, block_free_idx
            )
            first_hit = False
            for j in range(agent_idx, len(self.planner_data.agents_path[agent])):
                node = self.planner_data.agents_path[agent][j]
                gnode = self.planner_data.to_gnode(node)
                self.planner_data.deadlock_nodes.add(gnode)
                if self.is_footprint_overlap(node, block_node):
                    if not first_hit:
                        first_hit = True
                else:
                    if first_hit:
                        break
            if (coll_idx is not None) and agent_idx == coll_idx and enter == False:
                node = self.planner_data.agents_path[agent][agent_idx]
                # all nodes until block free idx
                return (node, agent_idx, enter)
            else:
                # all nodes until block free idx
                return (coll_node, coll_idx, enter)
        elif (agent_free_node) and (not block_free_node):
            (node, j, enter) = self.deadlock_node(
                agent, agent_free_idx, block_robot, block_idx
            )
            if (j is not None) and agent_free_idx == j and enter == False:
                node = self.planner_data.agents_path[agent][agent_idx]
                return (node, agent_idx, enter)
            else:
                return (node, j, enter)
        elif (agent_free_node) and (block_free_node):
            (node, j, enter) = self.deadlock_node(
                agent, agent_free_idx, block_robot, block_free_idx
            )
            if (j is not None) and agent_free_idx == j:
                node = self.planner_data.agents_path[agent][agent_idx]
                return (node, agent_idx, enter)
            else:
                return (node, j, enter)

    def find_deadlock_node(self, agent, start_node, block_robot):
        # check agent in start of block_robot
        block_start_node = self.planner_data.agents_path[block_robot][
            self.planner_data.robot_start_idx[block_robot]
        ]

        agent_idx = self.planner_data.robot_start_idx[agent]
        block_idx = self.planner_data.robot_start_idx[block_robot]
        (node, j, enter) = self.deadlock_node(agent, agent_idx, block_robot, block_idx)

        logger.info(f"Node:{node} Index:{j} Enter:{enter}")
        return (node, j)

    """
    Return List of other robot blocked by this robot 
    """

    def get_blocked_robot(self, robot_id, start_node):

        block_set = []
        non_block_set = []

        current_node = self.pathnode[start_node].find(robot_id)

        for j in range(len(current_node.data.index)):
            next_node = None
            if j + 1 < len(self.planner_data.agents_path[robot_id]):
                next_node = self.planner_data.agents_path[robot_id][j + 1]
            # check for already in final goal
            if next_node is None:
                return (block_set, block_set)

            robot_direction = self.get_direction(start_node, next_node)

            super_next_node = {}
            self.pos = self.planner_data.get_all_collision(
                next_node[0], next_node[1], next_node[2]
            )
            for x, y, angle in self.pos:
                super_next_node[(x, y, angle)] = 1

            next_next_node = None
            if j + 2 < len(self.planner_data.agents_path[robot_id]):
                next_next_node = self.planner_data.agents_path[robot_id][j + 2]

            super_next_next_node = {}
            if next_next_node is not None:
                self.pos = self.planner_data.get_all_collision(
                    next_next_node[0], next_next_node[1], next_next_node[2]
                )
                for x, y, angle in self.pos:
                    super_next_next_node[(x, y, angle)] = 1

            # get all other robot which pass through same node with different direction
            iterator = {}
            self.pos = self.planner_data.get_all_collision(
                start_node[0], start_node[1], start_node[2]
            )
            super_curr_node = {}
            for x, y, angle in self.pos:
                super_curr_node[(x, y, angle)] = 1
                if (x, y, angle) not in self.pathnode:
                    continue
                iters = self.pathnode[(x, y, angle)].get_iterator()
                for ego_robot in iters:
                    r_id = ego_robot.data.r_id
                    if r_id == robot_id:
                        continue
                    if r_id in iterator:
                        for idx in ego_robot.data.index:
                            iterator[r_id].append(idx)
                    else:
                        iterator[r_id] = copy.deepcopy(ego_robot.data.index)

            for ego_robot in iterator:
                robot_found = False
                # move untill outside box
                for ego_idx in iterator[ego_robot]:
                    if robot_found:
                        break
                    curr_idx = ego_idx
                    prev_next_found = False
                    next_found = False
                    for index in range(
                        ego_idx, self.planner_data.robot_start_idx[r_id] - 1, -1
                    ):
                        node = self.planner_data.agents_path[ego_robot][index]
                        if node not in super_curr_node:
                            curr_idx = index
                            break

                    if curr_idx == 0:
                        if ego_robot not in block_set:
                            block_set.append(ego_robot)
                    else:
                        curr_node = self.planner_data.agents_path[ego_robot][curr_idx]
                        if (curr_node in super_next_node) or (
                            curr_node in super_next_next_node
                        ):
                            if ego_robot not in block_set:
                                block_set.append(ego_robot)
                            robot_found = True
                        else:
                            if ego_robot not in non_block_set:
                                non_block_set.append(ego_robot)

        logger.info(f"BS:{block_set} NBS:{non_block_set}")
        return (block_set, non_block_set)

    """
    Initiate pathnode 
    """

    def initiate(self):
        self.planner_data.end_node = {}
        self.planner_data.time_schedule = {}
        self.planner_data.deadlock = {}
        self.high_priority_agent = []
        self.planner_data.robot_start_idx = {}
        self.planner_data.deadlock_cycle = nx.DiGraph()

        for agent in self.planner_data.agents_path.keys():
            end_node = self.planner_data.agents_path[agent][-1]
            self.planner_data.end_node[end_node] = 1

        # Reseting pathnode
        for node in self.pathnode.keys():
            self.pathnode[node] = BST()

        # for node in self.pathnode.keys():
        for node in self.planner_data.time_schedule:
            self.planner_data.time_schedule[node] = []

        # set high_priority_agent
        for agent in self.planner_data.agents_path.keys():
            self.high_priority_agent.append(agent)

        # set robot start idx
        for agent in self.high_priority_agent:
            self.planner_data.robot_start_idx[agent] = 0

        # set start node
        self.set_start_node()

        n = len(self.high_priority_agent)

        balance_iter = self.balanced_iterator(n)

        self.initiate_pathnode(balance_iter)

    """
    Traverse agent path and store in BST
    """

    def initiate_pathnode(self, balance_iter):
        for agent in balance_iter:
            prev_node = None
            for j in range(len(self.planner_data.agents_path[agent])):
                node = self.planner_data.agents_path[agent][j]
                next_node = None
                if (j + 1) < len(self.planner_data.agents_path[agent]):
                    next_node = self.planner_data.agents_path[agent][j + 1]

                if node not in self.pathnode:
                    self.pathnode[node] = BST()

                node_bst = self.pathnode[node].insert(agent)
                node_bst.data.index.append(j)

                node_bst.data.prev_node.append(prev_node)
                node_bst.data.next_node.append(next_node)

                prev_node = node

    def initiate_pathnode_footprint(self):
        pass

    """
    Remove agent
    """

    def remove_agent(self, agent):
        # remove robot from high priority
        self.high_priority_agent.remove(agent)
        # remove path of node
        for j in range(
            self.planner_data.robot_start_idx[agent],
            len(self.planner_data.agents_path[agent]),
        ):
            node = self.planner_data.agents_path[agent][j]
            self.pathnode[node].remove(agent)

    def is_path_overlap(self, node, r_id):
        is_overlap = False
        self.pos = self.planner_data.get_all_collision(node[0], node[1], node[2])
        for x, y, angle in self.pos:
            if (x, y, angle) not in self.pathnode:
                continue
            if len(self.pathnode[(x, y, angle)]) > 1:
                is_overlap = True
                return is_overlap

            if (len(self.pathnode[(x, y, angle)]) == 1) and (
                self.pathnode[(x, y, angle)].root.key != r_id
            ):
                is_overlap = True
                return is_overlap

        return is_overlap

    """
    Robot Wait in its start node to avoid collision then enter 
    """

    def wait_enter_heuristic(self):
        self.set_start_node()
        safe_start_node = []
        least_priority_agent = []
        # for each agent,their start node is safe, then make least priority(wait there)
        for start_node in self.start_node:
            is_overlap = self.is_path_overlap(start_node, self.start_node[start_node])
            if not is_overlap:
                safe_start_node.append(start_node)

        # remove least priority agent and start node
        for safe_start in safe_start_node:
            agent = self.start_node[safe_start]
            least_priority_agent.append(agent)
            logger.info(f"Wait Enter Heuristic {agent}")

        # remove least priority agent
        for agent in least_priority_agent:
            self.remove_agent(agent)

        return least_priority_agent

    def is_start_collision(self, node, r_id):
        is_collision = False
        self.pos = self.planner_data.get_all_collision(node[0], node[1], node[2])
        for x, y, angle in self.pos:
            if ((x, y, angle) in self.start_node) and self.start_node[
                (x, y, angle)
            ] != r_id:
                is_collision = True
                logger.info(
                    f"Agent:{r_id}, Node:{node}, In Collision with point:{(x,y,angle)}"
                )
                return is_collision

        return is_collision

    """
    Robot can reach either Goal node or safespot(which is ) and wait there
    """

    def reach_safespot_wait(self):
        self.set_start_node()
        removable_robot = []
        new_robot_start = {}
        for agent in self.high_priority_agent:
            # move untill reach goal or Non-Blocking Spot
            for j in range(
                self.planner_data.robot_start_idx[agent],
                len(self.planner_data.agents_path[agent]),
            ):
                node = self.planner_data.agents_path[agent][j]
                # check collision with other robots
                is_collision = self.is_start_collision(node, agent)
                if is_collision:
                    new_robot_start[agent] = [None, False]
                    break

                # move untill reach goal
                if (node in self.planner_data.end_node) and (
                    j == (len(self.planner_data.agents_path[agent]) - 1)
                ):
                    logger.info(f"Robot {agent}, Reach Goal {node}")
                    new_robot_start[agent] = [j, True]
                    removable_robot.append(agent)
                    break

                # move untill Safespot
                is_overlap = self.is_path_overlap(node, agent)
                if not is_overlap:
                    logger.info(f"Robot {agent}, Reach safespot {node}")
                    new_robot_start[agent] = [j, False]
                    removable_robot.append(agent)
                    break

        goal_robot = []
        for agent in removable_robot:
            idx, goal = new_robot_start[agent]
            if goal:
                goal_robot.append(agent)

        optimal_robot_order = []
        if len(goal_robot) > 0:
            logger.info(f"Goal robot {goal_robot}")
            self.schedular.initiate(goal_robot)
            optimal_robot_order = self.schedular.solve()

        new_removable_robot = []
        for i in range(len(optimal_robot_order) - 1, -1, -1):
            new_removable_robot.append(optimal_robot_order[i])

        for agent in removable_robot:
            idx, goal = new_robot_start[agent]
            if not goal:
                new_removable_robot.append(agent)

        # remove least priority agent
        for robot in new_removable_robot:
            logger.info(f"Reach SafeSpot or Goal {robot}")
            # remove safe robot from high priority
            self.high_priority_agent.remove(robot)

        # update pathnode and schedule robot
        for agent in new_removable_robot:
            # remove from pathnode
            for j in range(
                self.planner_data.robot_start_idx[agent],
                len(self.planner_data.agents_path[agent]),
            ):
                node = self.planner_data.agents_path[agent][j]
                self.pathnode[node].remove(agent)

        least_priority_robot = []
        for agent in new_removable_robot:
            idx, goal = new_robot_start[agent]

            if goal:
                for j in range(
                    self.planner_data.robot_start_idx[agent],
                    len(self.planner_data.agents_path[agent]),
                ):
                    node = self.planner_data.agents_path[agent][j]
                    self.pos = self.planner_data.get_all_collision(
                        node[0], node[1], node[2]
                    )
                    for x, y, angle in self.pos:
                        if (x, y, angle) in self.planner_data.time_schedule:
                            self.planner_data.time_schedule[(x, y, angle)].append(agent)
                        else:
                            self.planner_data.time_schedule[(x, y, angle)] = [agent]
            else:
                least_priority_robot.append(agent)
                # update time schedule
                for j in range(
                    self.planner_data.robot_start_idx[agent], idx + 1
                ):  # Modified for footprint
                    # for j in range(self.planner_data.robot_start_idx[agent], idx): #Modified for footprint
                    node = self.planner_data.agents_path[agent][j]
                    self.pos = self.planner_data.get_all_collision(
                        node[0], node[1], node[2]
                    )
                    for x, y, angle in self.pos:
                        if (x, y, angle) in self.planner_data.time_schedule:
                            self.planner_data.time_schedule[(x, y, angle)].append(agent)
                        else:
                            self.planner_data.time_schedule[(x, y, angle)] = [agent]

                self.planner_data.robot_start_idx[agent] = idx + 1

        return (new_removable_robot, least_priority_robot)

    """
    Reduce Blocking, Robot move along path aim to reduce blocking other robot
    """

    def reduce_blocked_robot(self):
        self.set_start_node()
        new_robot_start = {}
        for agent in self.high_priority_agent:
            new_robot_start[agent] = None
            logger.info(f"Robot id {agent}")
            start_node = self.planner_data.agents_path[agent][
                self.planner_data.robot_start_idx[agent]
            ]
            logger.info(start_node)
            if (
                self.planner_data.robot_start_idx[agent]
                == len(self.planner_data.agents_path[agent]) - 1
            ):
                logger.info(f"Reached goal")
                new_robot_start[agent] = self.planner_data.robot_start_idx[agent]
                continue

            (initial_block_set, initial_nonblock_set) = self.get_blocked_robot(
                agent, start_node
            )
            logger.info(f"IBS:{initial_block_set} INBS:{initial_nonblock_set}")

            untill_node = set()
            for block_robot in self.high_priority_agent:
                if block_robot == agent:
                    continue
                (output, idx) = self.find_deadlock_node(agent, start_node, block_robot)
                logger.info(f"Blocked Node {output} {idx}")
                if output is not None:
                    untill_node.add((output, idx))

            found = False
            for node, idx in untill_node:
                if (
                    node[0] == start_node[0]
                    and node[1] == start_node[1]
                    and node[2] == start_node[2]
                ) and (idx == self.planner_data.robot_start_idx[agent]):
                    found = True

            if found:
                logger.info("In Deadlock")
                continue

            # move untill reach goal or Non-Blocking Spot
            for j in range(
                self.planner_data.robot_start_idx[agent] + 1,
                len(self.planner_data.agents_path[agent]),
            ):
                robot_node = self.planner_data.agents_path[agent][j]
                logger.info(robot_node)
                is_collision = self.is_start_collision(robot_node, agent)
                if is_collision:
                    logger.info("In Collision")
                    break
                found = False
                for node, idx in untill_node:
                    if (
                        node[0] == robot_node[0]
                        and node[1] == robot_node[1]
                        and node[2] == robot_node[2]
                    ) and (idx == j):
                        found = True
                        break

                if found:
                    logger.info("Enter into Deadlock")
                    break

                new_robot_start[agent] = j
                logger.info(f"New Index ==> {new_robot_start[agent]}")
                break

        update = 0
        # to make sure two robot wont overlap their position
        updated_start_node = []
        removable_agent = []

        moved_robot = []
        # update robot start and remove robot path from pathnode
        for agent in self.high_priority_agent:
            if new_robot_start[agent] is not None:
                found = False
                new_start_idx = new_robot_start[agent]
                node = self.planner_data.agents_path[agent][new_start_idx]
                if new_start_idx + 1 < len(self.planner_data.agents_path[agent]):
                    # Check new start create deadlock
                    untill_node = set()
                    for block_robot in moved_robot:
                        if block_robot == agent:
                            continue
                        block_idx = min(
                            new_robot_start[block_robot] + 1,
                            len(self.planner_data.agents_path[block_robot]) - 1,
                        )
                        (output, idx, enter) = self.deadlock_node(
                            agent, new_robot_start[agent] + 1, block_robot, block_idx
                        )
                        logger.info(
                            f"Recheck Blocked Node {agent} with {block_robot} {output} {idx}"
                        )
                        if output is not None:
                            untill_node.add((output, idx, enter))

                    # found = False
                    start_node = self.planner_data.agents_path[agent][
                        new_robot_start[agent] + 1
                    ]
                    for node, idx, enter in untill_node:
                        if enter:
                            continue
                        if (node[0] == start_node[0] and node[1] == start_node[1]) and (
                            idx == new_robot_start[agent] + 1
                        ):
                            found = True

                if found:
                    logger.info("In Deadlock")
                    continue

                moved_robot.append(agent)

                if node not in updated_start_node:
                    updated_start_node.append(node)
                else:
                    continue

                reach_goal = False
                for j in range(
                    self.planner_data.robot_start_idx[agent], new_robot_start[agent] + 1
                ):  # Modified for footprint
                    node = self.planner_data.agents_path[agent][j]

                    if j < new_robot_start[agent]:
                        self.pathnode[node].remove(agent)

                    self.pos = self.planner_data.get_all_collision(
                        node[0], node[1], node[2]
                    )
                    for x, y, angle in self.pos:
                        if (x, y, angle) in self.planner_data.time_schedule:
                            self.planner_data.time_schedule[(x, y, angle)].append(agent)
                        else:
                            self.planner_data.time_schedule[(x, y, angle)] = [agent]

                if new_robot_start[agent] == (
                    len(self.planner_data.agents_path[agent]) - 1
                ):
                    reach_goal = True
                    self.planner_data.robot_start_idx[agent] = new_robot_start[agent]
                else:
                    # update robot start index
                    self.planner_data.robot_start_idx[agent] = (
                        new_robot_start[agent] + 1
                    )

                if reach_goal:
                    removable_agent.append(agent)
                    logger.info(
                        f"Removing robot {agent} -> {len(self.planner_data.agents_path[agent])}"
                    )
                update += 1

        for agent in removable_agent:
            self.remove_agent(agent)
        return update

    """
    Find which robot in deadlock, which help in resolving deadlock
    """

    def detect_deadlock(self):
        node_deadlock = {}
        for agent in self.high_priority_agent:
            r_idx = self.planner_data.robot_start_idx[agent]
            for block_robot in self.high_priority_agent:
                if agent == block_robot:
                    continue
                block_idx = self.planner_data.robot_start_idx[block_robot]
                self.planner_data.deadlock_nodes = set()
                (deadlock_node, idx, enter) = self.deadlock_node(
                    agent, r_idx, block_robot, block_idx
                )

                for node in self.planner_data.deadlock_nodes:
                    gnode = self.planner_data.to_gnode(node)
                    if gnode in node_deadlock:
                        node_graph = node_deadlock[gnode]
                        node_graph.add_node(agent)
                        node_graph.add_node(block_robot)
                        node_graph.add_edge(agent, block_robot)
                    else:
                        node_deadlock[gnode] = nx.DiGraph()
                        node_graph = node_deadlock[gnode]
                        node_graph.add_node(agent)
                        node_graph.add_node(block_robot)
                        node_graph.add_edge(agent, block_robot)

        logger.info("Direct Deadlock")

        logger.info("Node Deadlock")
        for node in node_deadlock.keys():
            ng = node_deadlock[node]
            logger.info(f"N:{node}, E:{ng.edges}")

            iterator = nx.algorithms.cycles.simple_cycles(ng)

            for cycle in iterator:
                logger.info(cycle)
                all_pair = []
                # if len(cycle)  <= 2:
                #    continue

                for i in range(len(cycle)):
                    all_pair.append((cycle[i - 1], cycle[i]))

                print(f"ALLLLL")
                print(all_pair)
                for i in range(len(all_pair)):
                    r1 = all_pair[i][0]
                    r2 = all_pair[i][1]
                    r1_startnode = self.planner_data.agents_path[r1][
                        self.planner_data.robot_start_idx[r1]
                    ]
                    r2_startnode = self.planner_data.agents_path[r2][
                        self.planner_data.robot_start_idx[r2]
                    ]

                    path1 = []
                    for k in range(
                        self.planner_data.robot_start_idx[r1],
                        len(self.planner_data.agents_path[r1]),
                    ):
                        node = self.planner_data.agents_path[r1][k]
                        path1.append(node)
                        if node == r2_startnode:
                            break
                    new_pair = []
                    for j in range(len(all_pair)):
                        if i != j:
                            new_pair.append(all_pair[j])

                    if r1 not in self.planner_data.deadlock_cycle:
                        self.planner_data.deadlock_cycle.add_node(r1)

                    if r2 not in self.planner_data.deadlock_cycle:
                        self.planner_data.deadlock_cycle.add_node(r2)

                    self.planner_data.deadlock_cycle.add_edge(r1, r2)
                    # self.planner_data.deadlock_cycle.add_edge(r2,r1)

                    if (r1, r2) in self.planner_data.deadlock:
                        continue
                        self.planner_data.deadlock[(r1, r2)].append(
                            [r1_startnode, new_pair, path1]
                        )
                    else:
                        self.planner_data.deadlock[(r1, r2)] = [
                            [r1_startnode, new_pair, path1]
                        ]

    def priority(self):
        self.initiate()
        self.static_priority = []
        transformation = True
        iteration = 0
        while (transformation == True) and (iteration < self.max_iteration):
            iteration += 1
            transformation = False
            # Heuristic I:
            least_priority_robot = self.wait_enter_heuristic()

            if len(least_priority_robot) != 0:
                transformation = True

            if len(least_priority_robot) > 0:
                logger.info(f"Least Priority {least_priority_robot}")
                self.schedular.initiate(least_priority_robot)
                optimal_robot_order = self.schedular.solve()
                for robot in optimal_robot_order:
                    self.static_priority.append(robot)
            if transformation:
                continue

            # Heuristic II:
            (high_priority_robot, least_priority_robot) = self.reach_safespot_wait()
            logger.info(f"High priority {self.static_priority}")
            if (len(high_priority_robot) + len(least_priority_robot)) != 0:
                transformation = True
            if len(least_priority_robot) > 0:
                self.schedular.initiate(least_priority_robot)
                optimal_robot_order = self.schedular.solve()
                for robot in optimal_robot_order:
                    self.static_priority.append(robot)
            if transformation:
                continue
            # Heuristic III:
            update = self.reduce_blocked_robot()
            if update > 0:
                transformation = True

        if len(self.high_priority_agent) > 0:
            logger.info(f"Decting Deadlockkkkkkkkkk.....")
            self.detect_deadlock()
            return False

        # update schedule
        for i in range(len(self.static_priority) - 1, -1, -1):
            agent = self.static_priority[i]
            for j in range(
                self.planner_data.robot_start_idx[agent],
                len(self.planner_data.agents_path[agent]),
            ):
                node = self.planner_data.agents_path[agent][j]
                self.pos = self.planner_data.get_all_collision(
                    node[0], node[1], node[2]
                )
                for x, y, angle in self.pos:
                    if (x, y, angle) in self.planner_data.time_schedule:
                        self.planner_data.time_schedule[(x, y, angle)].append(agent)
                    else:
                        self.planner_data.time_schedule[(x, y, angle)] = [agent]

        return True


class Deadlock_Data:
    def __init__(self, r_id, start_node):
        self.r_id = r_id
        self.start = start_node
        self.occurance = 1
        self.resolved = False
        self.affected_pair = {}
        self.blocked_path = {}

    def __str__(self):
        return f"RID:{self.r_id} Occurance:{self.occurance} and Resolve:{self.resolved} Affected Pair:{self.affected_pair} Blocked Path:{self.blocked_path}"

    def get_deadlock_pair(self):
        return self.affected_pair.keys()


class Lazy_Wait:
    def __init__(self, planner_data, path_planner, blackboard):
        self.planner_data = planner_data
        self.path_planner = path_planner
        self.deadlock_robot = {}
        self.available_waitspot = {}
        self.robot_path = {}
        self.graph_file = "./config/research_layout_2/layout_d.gml"
        self.graph = None
        self.load_graph()
        self.black_board = blackboard

    def load_graph(self):
        self.graph = nx.DiGraph()
        self.graph = nx.read_gml(self.graph_file)

    def pos_to_node(self, x, y):
        node = f"{int(x)},{int(y)}"
        return str(node)

    def node_to_pos(self, node):
        data = re.split(r",", node)
        pos = (int(data[0]), int(data[1]))
        return pos

    def deadlock_pair_selection(self):
        self.free_waitspot()
        candidate_robot = []
        for agent in self.deadlock_robot:
            if self.deadlock_robot[agent].resolved:
                continue
            else:
                candidate = self.NoRobotBlockingJunction(
                    agent, self.find_junctions(agent)
                )
                if len(candidate):
                    candidate_robot.append(agent)
        if len(candidate_robot):
            robot_id = candidate_robot[0]
            number_robot = self.numberOfRobotInDeadlock(robot_id)

            for j in candidate_robot:
                if self.numberOfRobotInDeadlock(j) < number_robot:
                    robot_id = j
            return robot_id
        else:
            return None

    def NumberOfRobotBlocking(self, robot_id):
        """
        Number of robots which are blocking other robots which are making deadlock pair
        """
        junction = self.find_junctions(robot_id)
        robot_nodeid = self.find_nearest_nodeid(robot_id)
        robot_location = []
        for r_id in self.planner_data.agents_path.keys():
            r_location = self.find_nearest_nodeid(r_id)
            if robot_id != r_id:
                robot_location.append([r_id, r_location])
        candidate_junction = []
        for i in junction:
            path = self.path_planner.plan(
                self.node_to_pos(robot_nodeid), self.node_to_pos(i)
            )
            # print("Path of the robot and junction",path)
            number_of_blocking_robot = 0
            robot_blocking = []
            for agent_location in robot_location:
                if self.node_to_pos(agent_location[1]) in path:
                    number_of_blocking_robot += 1
                    robot_blocking.append(agent_location[0])
            candidate_junction.append(
                [robot_id, i, number_of_blocking_robot, robot_blocking]
            )
        candidate_junction.sort(key=itemgetter(2))
        return candidate_junction
        #! Changes are required here for the sorting and add the robot id for the deadlock

    def deadlock_robot_selection(self, robot_id):
        # print("vajfdbvds")
        candidate_robot = []
        master_robot = robot_id
        if self.deadlock_robot[robot_id].resolved:
            return None
        if robot_id is None:
            return
        robot_pair = []
        robot_pair.append(robot_id)
        # if robot_id is not None:
        for agent in self.deadlock_robot[
            robot_id
        ].affected_pair.keys():  #!!!!!!! Might have a issue at later stage.
            robot_pair.append(agent)
        for i in robot_pair:
            candidate = self.NoRobotBlockingJunction(i, self.find_junctions(i))
            if len(candidate):
                candidate_robot.append(i)
        print("candidate robobt", candidate_robot)
        candidate_for_resolution = []
        if len(candidate_robot):
            robot_id = candidate_robot[0]
            # number_robot = self.numberOfRobotInDeadlock(robot_id)
            for j in candidate_robot:
                print("agent in error", j)
                print("Nearest node id in error", self.find_nearest_nodeid(j))
                print("waitspot in error", self.waitspot_for_robot(j))
                # if self.waitspot_for_robot(j) is None:
                #     waitspot = self.assign_static_waitspot(j)
                # else:
                waitspot = self.waitspot_for_robot(j)
                dist = self.path_planner.cost(
                    self.path_planner.plan(
                        self.node_to_pos(self.find_nearest_nodeid(j)),
                        self.node_to_pos(waitspot),
                    )
                )
                candidate_for_resolution.append(
                    [j, self.numberOfRobotInDeadlock(j), dist]
                )
                # if self.numberOfRobotInDeadlock(j) < number_robot:
                #     robot_id = j
            candidate_for_resolution.sort(key=itemgetter(1), reverse=True)
            robot_id_resolution = []
            number_deadlock = candidate_for_resolution[0][1]
            for agent, number, dist in candidate_for_resolution:
                if number_deadlock == number:
                    robot_id_resolution.append([agent, dist])
                else:
                    continue
            robot_id_resolution.sort(key=itemgetter(1))
            return robot_id_resolution[0][0]
        else:
            return master_robot
        # else:
        #     junction_traffic = []
        #     for agent in self.deadlock_robot:
        #         if self.deadlock_robot[agent].resolved:
        #             continue
        #         else:
        #             junction_traffic.append(self.NumberOfRobotBlocking(agent))
        #     print("Junction Traffic",junction_traffic)
        #     junction_load = []
        #     for i in junction_traffic:
        #         for j in i:
        #             junction_load.append(j)
        #     print("Junction load", junction_load)
        #     junction_load.sort(key=itemgetter(2))
        #     if len(junction_load):
        #         # robot_id = self.Nearest_to_junction(junction_load[0][0] , junction_load[0][3] , junction_load[0][1])
        #         print("Robots for deadlock resolution",junction_load[0][0] )
        #         return junction_load[0][0]

    def numberOfRobotInDeadlock(self, robot_id):
        """
        Return number of deadlock a robot is with another robots
        """
        return self.deadlock_robot[robot_id].occurance

    def assign_static_waitspot(self, robot_id):
        """ "
        @comment: Considers only the robots in deadlock for finding the waitspot
        """
        all_path = (
            self.planner_data.agents_path
        )  #!!! change the robot list to those which are in deadlock.
        position = self.pos_to_node(
            self.planner_data.agents_path[robot_id][0][0],
            self.planner_data.agents_path[robot_id][0][1],
        )
        waitspot = self.planner_data.waitspot
        path = []
        for agent in all_path.keys():
            for node_id in all_path[agent]:
                node = self.pos_to_node(node_id[0], node_id[1])
                path.append(node)
        distance = 0
        node = None
        count = 0
        array_1 = []
        for i in waitspot:
            count += 1
            i = self.pos_to_node(i[0], i[1])
            x, y = self.node_to_pos(i)
            try:
                dist = self.path_planner.cost(
                    self.path_planner.plan(self.node_to_pos(position), (x, y))
                )
                if count == 1:
                    distance = dist
                    node = i
                if dist < distance:
                    distance = dist
                    node = i
                array_1.append([dist, i])
            except:
                continue

        array_1.sort(key=itemgetter(0))

        for i, j in array_1:
            if j in path:
                continue
            else:
                return j

    # def perform_check(self, list1, list2 , count):

    def check_path(self, robot_id, waitspot_path):
        print("Waitspot Path", waitspot_path)
        robot_path = []
        wait_path = []
        for pos in self.planner_data.agents_path[robot_id]:
            node = self.pos_to_node(pos[0], pos[1])
            robot_path.append(node)
        for pos in waitspot_path:
            node = self.pos_to_node(pos[0], pos[1])
            wait_path.append(node)
        for count1 in range(len(robot_path)):
            count = count1
            check = 0
            for wait_node in wait_path:
                if count == len(robot_path):
                    return False
                if wait_node == robot_path[count]:
                    check += 1
                count += 1
            if check == len(wait_path):
                print("YEssss")
                return True
            else:
                print("No the path doesn't matcehs")
                return False

    def waitspot_for_robot(self, robot_id):
        # if len(self.deadlock_robot[robot_id].affected_pair)==0:
        #     return None
        print("Robot id in waitspot is", robot_id)
        all_path = (
            self.planner_data.agents_path
        )  #! Check the format of data and then how we will utilise the
        position = self.find_nearest_nodeid(
            robot_id
        )  #! blackboard has to be integrated.
        nodes_all = self.graph.nodes
        path = []
        final_list = []
        for agent in all_path.keys():
            if agent != robot_id:
                for node_id in all_path[agent]:
                    node = self.pos_to_node(node_id[0], node_id[1])
                    path.append(node)
            # else:
            # continue
        for pos in self.planner_data.waitspot:
            # print("Waitspot data is", node)
            node = self.pos_to_node(pos[0], pos[1])
            if node not in path:
                final_list.append(node)
            else:
                continue
        # for node in nodes_all:
        #     if node not in path:
        #         final_list.append(node)
        #     else:
        #         continue
        # path_free_nodes = set(nodes_all) - set(path)
        # final_list = list(path_free_nodes)
        if len(final_list) == 0:
            # return self.assign_static_waitspot(robot_id)
            return self.dynamic_waitspot_for_robot(robot_id)
        else:
            distance = 0
            node = None
            count = 0
            for i in final_list:
                count += 1
                x, y = self.node_to_pos(i)
                if self.planner_data.assigned_waitspot[i] == True:
                    continue

                else:
                    try:
                        dist = self.path_planner.cost(
                            self.path_planner.plan(self.node_to_pos(position), (x, y))
                        )
                        if count == 1:
                            distance = dist
                            node = i
                        if dist < distance:
                            distance = dist
                            node = i
                    except:
                        continue
                    # dist = ((position[0] - x)**2 + (position[1] - y)**2)**0.5
            print("Node id for deadlock resolution is", node)
            print("Robot which has to be resolved first is: ", robot_id)
            if node is None:
                node = self.dynamic_waitspot_for_robot(robot_id)

            return node

    def dynamic_waitspot_for_robot(self, robot_id):
        # if len(self.deadlock_robot[robot_id].affected_pair)==0:
        #     return None
        print("Robot id in waitspot is", robot_id)
        all_path = (
            self.planner_data.agents_path
        )  #! Check the format of data and then how we will utilise the
        position = self.find_nearest_nodeid(
            robot_id
        )  #! blackboard has to be integrated.
        nodes_all = self.graph.nodes
        path = []
        final_list = []
        for agent in all_path.keys():
            if agent != robot_id:
                for node_id in all_path[agent]:
                    node = self.pos_to_node(node_id[0], node_id[1])
                    path.append(node)
            # else:
            # continue
        for node in nodes_all:
            # print("Waitspot data is", node)
            # node = self.pos_to_node(pos[0], pos[1])
            if node not in path:
                final_list.append(node)
            else:
                continue
        # for node in nodes_all:
        #     if node not in path:
        #         final_list.append(node)
        #     else:
        #         continue
        # path_free_nodes = set(nodes_all) - set(path)
        # final_list = list(path_free_nodes)
        if len(final_list) == 0:
            return self.assign_static_waitspot(robot_id)
        else:
            distance = 0
            node = None
            count = 0
            for i in final_list:
                count += 1
                x, y = self.node_to_pos(i)
                if self.planner_data.assigned_waitspot[i] == True:
                    continue

                else:
                    try:
                        dist = self.path_planner.cost(
                            self.path_planner.plan(self.node_to_pos(position), (x, y))
                        )
                        if count == 1:
                            distance = dist
                            node = i
                        if dist < distance:
                            distance = dist
                            node = i
                    except:
                        continue
                    # dist = ((position[0] - x)**2 + (position[1] - y)**2)**0.5
            print("Node id for deadlock resolution is", node)
            print("Robot which has to be resolved first is: ", robot_id)
            if node is None:
                node = self.assign_static_waitspot(robot_id)

            return node

    def find_nearest_nodeid(self, robot_id):
        """
        @comment: Return the nearest nodeid for the robot based on the current position of the robot using robot path, if path is not a correct, then
        search space can be extended to the entire grid space.
        """
        #! Check the depenedency on blackboard and the function call is apt or not?
        path = []

        for i in self.planner_data.agents_path[robot_id]:
            # print("Node of path", i)
            path.append(self.pos_to_node(i[0], i[1]))
        position = self.black_board.read((300 + robot_id, Data.Pos))
        distance = 0
        node = None
        count = 0
        for i in path:
            count += 1
            x, y = self.node_to_pos(i)
            dist = ((position.x - x) ** 2 + (position.y - y) ** 2) ** 0.5
            if count == 1:
                distance = dist
                node = i
            if dist < distance:
                distance = dist
                node = i
        # node = self.pos_to_node(self.planner_data.agents_path[robot_id][0][0] , self.planner_data.agents_path[robot_id][0][1])
        return node

    def neighbours(self, node):
        all_neighbours = []
        for l in nx.all_neighbors(self.graph, node):
            if l not in all_neighbours:
                all_neighbours.append(l)
        total_degree = len(all_neighbours)
        return all_neighbours

    def find_junctions(self, robot_id):
        node_id = self.find_nearest_nodeid(robot_id)
        all_neighbours = []
        for l in nx.all_neighbors(self.graph, node_id):
            if l not in all_neighbours:
                all_neighbours.append(l)
        total_degree = len(all_neighbours)
        junction = []
        for i in all_neighbours:
            j = i
            last_node = node_id
            while len(self.neighbours(j)) <= 2:
                neighbour = self.neighbours(j)
                neighbour.remove(last_node)
                if len(neighbour) == 0:
                    break
                next_node = neighbour[0]
                last_node = j
                j = next_node
            junction.append(j)
        if node_id in junction:
            junction.remove(node_id)
        return junction

    def NoRobotBlockingJunction(self, robot_id, junction):
        robot_nodeid = self.find_nearest_nodeid(robot_id)
        #!! Add blackboard data access system, here i need to only read the data in the blackboard.
        robot_location = []
        for r_id in self.planner_data.agents_path.keys():
            r_location = self.find_nearest_nodeid(r_id)
            if robot_id != r_id:
                robot_location.append(r_location)
        candidate_junction = []
        for i in junction:
            path = self.path_planner.plan(
                self.node_to_pos(robot_nodeid), self.node_to_pos(i)
            )
            # print("Path of the robot and junction",path)
            count = 0
            for agent_location in robot_location:
                if self.node_to_pos(agent_location) in path:
                    break
                count += 1
                if count == len(robot_location):
                    candidate_junction.append(i)
        return candidate_junction

    def Nearest_to_junction(self, deadlock_robot, robot_list, junction):
        """
        Find the robot which is nearest to the junction, so that we can move it.
        """
        count = 0
        agent_id = -1
        agent_list = []
        print("Robot List", robot_list)
        for agent in robot_list:
            print("Agent", agent)
            node_id = self.find_nearest_nodeid(agent)
            x, y = self.node_to_pos(node_id)
            count += 1
            try:
                dist = self.path_planner.cost(
                    self.path_planner.plan(self.node_to_pos(junction), (x, y))
                )
                agent_list.append([agent, dist])
                if count == 1:
                    distance = dist
                    agent_id = agent
                if dist < distance:
                    distance = dist
                    agent_id = agent
            except:
                continue
        agent_list.sort(key=itemgetter(1))
        final_robot_list = []
        for agent in agent_list:
            final_robot_list.append(agent[0])
        final_robot_list.append(deadlock_robot)
        print(
            "Robot list for the deadlock resolution in Nearest to Junction Function",
            final_robot_list,
        )
        return final_robot_list

    def DRS(self):
        robot_id = self.deadlock_pair_selection()
        print("Robot id in WMS deadock pair", robot_id)
        if robot_id is None:
            return (None, None)
        else:
            agent_id = self.deadlock_robot_selection(robot_id)
            print("Robot id in WMS deadlock robot", agent_id)
            waitspot = self.waitspot_for_robot(agent_id)
            print("Waitspot for robot", waitspot)
            # for i in range(10):
            #     for agent in self.deadlock_robot:
            #         if self.deadlock_robot[agent].resolved:
            #             continue
            #         print("Deadlock Pairs are", agent)
            #         print("Agent id", agent_id)
            return (agent_id, waitspot)

    def get_min_deadlock(self):
        """
        Return robot id which create maximum deadlock with another robots
        """
        min_occurance = self.deadlock_robot[0].occurance
        min_r_id = -1
        for robot in self.deadlock_robot:
            if self.deadlock_robot[robot].resolved:
                continue
            if min_occurance > self.deadlock_robot[robot].occurance:
                min_occurance = self.deadlock_robot[robot].occurance
                min_r_id = robot
        return min_r_id

    def free_waitspot(self):
        path = []
        for agent in self.planner_data.agents_path.keys():
            for pos in self.planner_data.agents_path[agent]:
                path.append(self.pos_to_node(pos[0], pos[1]))
        nodes_all = self.graph.nodes
        for node in nodes_all:
            if node in path:
                self.planner_data.assigned_waitspot[node] = True
            else:
                self.planner_data.assigned_waitspot[node] = False

    def initiate(self):
        self.available_waitspot = {}
        self.planner_data.robot_waitspot_path = {}
        self.deadlock_robot = {}

        for r1, r2 in self.planner_data.deadlock:
            for r1_start_node, deadlock_pair, path in self.planner_data.deadlock[
                (r1, r2)
            ]:
                logger.info(f"{r1} and {r2} are in deadlock {path}")
                self.insert(r1, r2, deadlock_pair, r1_start_node, path)

    def insert(self, r_id, r2, deadlock_pair, start_node, blockedpath):
        if r_id in self.deadlock_robot:
            self.deadlock_robot[r_id].occurance += 1
            self.deadlock_robot[r_id].affected_pair[r2] = deadlock_pair
            self.deadlock_robot[r_id].blocked_path[r2] = blockedpath
        else:
            self.deadlock_robot[r_id] = Deadlock_Data(r_id, start_node)
            self.deadlock_robot[r_id].affected_pair[r2] = deadlock_pair
            self.deadlock_robot[r_id].blocked_path[r2] = blockedpath

    def print_deadlock(self):
        for robot in self.deadlock_robot:
            logger.info(self.deadlock_robot[robot])

    """
    Return robot id which create maximum dealock with another robots
    """

    def get_max_deadlock(self):
        max_occurance = 0
        max_r_id = -1
        for robot in self.deadlock_robot:
            if self.deadlock_robot[robot].resolved:
                continue
            if max_occurance < self.deadlock_robot[robot].occurance:
                max_occurance = self.deadlock_robot[robot].occurance
                max_r_id = robot
        return max_r_id

    def near_waitspot_dist(self, robot):
        # assign nearest waitspot
        min_dist = 2000.0
        min_waitspot = -1
        for waitspot in self.planner_data.waitspot:
            if self.planner_data.waitspot[waitspot] == 0:
                continue
            # path = self.path_planner.plan(self.deadlock_robot[robot].start, waitspot)
            # dist_original = len(path)
            dist = abs(self.deadlock_robot[robot].start[0] - waitspot[0])

            if dist == 0:
                dist = abs(self.deadlock_robot[robot].start[0] - waitspot[0]) + abs(
                    self.deadlock_robot[robot].start[1] - waitspot[1]
                )
            else:
                aisle_top = [self.deadlock_robot[robot].start[0], 1]
                aisle_bottom = [self.deadlock_robot[robot].start[0], 16]

                dist1 = abs(self.deadlock_robot[robot].start[0] - aisle_top[0]) + abs(
                    self.deadlock_robot[robot].start[1] - aisle_top[1]
                )
                dist1 += abs(aisle_top[0] - waitspot[0]) + abs(
                    aisle_top[1] - waitspot[1]
                )

                dist2 = abs(
                    self.deadlock_robot[robot].start[0] - aisle_bottom[0]
                ) + abs(self.deadlock_robot[robot].start[1] - aisle_bottom[1])
                dist2 += abs(aisle_bottom[0] - waitspot[0]) + abs(
                    aisle_bottom[1] - waitspot[1]
                )

                dist = min(dist1, dist2)
            logger.info(
                f"Robot:{robot} Start:{self.deadlock_robot[robot].start} Waitspot:{waitspot} Dist:{dist}"
            )
            if dist < min_dist:
                min_dist = dist
                min_waitspot = waitspot

        return min_dist

    def near_waitspot(self):
        # choose robot which create maximum deadlock
        max_deadlock = self.get_max_deadlock()
        print("Max deadlock in deadlock pair", max_deadlock)
        for i in self.deadlock_robot.keys():
            print("robot in deadlock", self.deadlock_robot[i])
        print("number of robot in deadlock", len(self.deadlock_robot.keys()))

        if max_deadlock == -1:
            return -1

        deadlock_pair = self.deadlock_robot[max_deadlock].get_deadlock_pair()

        all_robot = [[max_deadlock, self.near_waitspot_dist(max_deadlock)]]

        for pair in deadlock_pair:
            all_robot.append([pair, self.near_waitspot_dist(pair)])

        logger.info(deadlock_pair)

        modified_priority = sorted(all_robot, key=itemgetter(1))

        logger.info(modified_priority)

        return modified_priority[0][0]

    def lazy_wait(self):
        self.initiate()
        self.print_deadlock()
        for _ in range(1):
            self.free_waitspot()
            # for deadlock_robot in self.WMS():
            deadlock = self.DRS()
            if deadlock[0] is not None:

                deadlock_robot = deadlock[0]
                waitspot = deadlock[1]
                print("Deaclock robot is", deadlock_robot)
                print("Waitspot for robot", waitspot)
                if waitspot is None:
                    return
                # self.deadlock_robot[deadlock_robot].get_deadlock_pair()
                # waitspot = self.waitspot_for_robot(deadlock_robot)
                if deadlock_robot == -1:
                    logger.info("All Deadlock Got New Path")
                    return
                if self.check_path(
                    deadlock_robot,
                    self.path_planner.plan(
                        self.deadlock_robot[deadlock_robot].start,
                        self.node_to_pos(waitspot),
                    ),
                ):
                    return
                logger.info(f"Deadlock:{deadlock_robot}")
                # update path
                for key in self.deadlock_robot[deadlock_robot].blocked_path:
                    blocked_path = self.deadlock_robot[deadlock_robot].blocked_path[key]
                    self.path_planner.block_path(blocked_path)

                self.planner_data.assigned_waitspot[waitspot] = True

                path = self.path_planner.plan(
                    self.deadlock_robot[deadlock_robot].start,
                    self.node_to_pos(waitspot),
                )

                logger.info(f"Path :{path}")
                cost = self.path_planner.cost(path)

                logger.info(f"Path cost :{cost}")

                self.planner_data.robot_waitspot_path[deadlock_robot] = path

                self.deadlock_robot[deadlock_robot].occurance = 0
                self.deadlock_robot[deadlock_robot].resolved = True

                iterator = nx.all_neighbors(
                    self.planner_data.deadlock_cycle, deadlock_robot
                )
                seen_edge = {}
                for iter in iterator:
                    if self.planner_data.deadlock_cycle.has_edge(deadlock_robot, iter):
                        if (deadlock_robot, iter) not in seen_edge:
                            del self.deadlock_robot[deadlock_robot].affected_pair[iter]
                            del self.deadlock_robot[deadlock_robot].blocked_path[iter]

                            seen_edge[(deadlock_robot, iter)] = 1

                    if self.planner_data.deadlock_cycle.has_edge(iter, deadlock_robot):
                        if (iter, deadlock_robot) not in seen_edge:
                            del self.deadlock_robot[iter].affected_pair[deadlock_robot]
                            del self.deadlock_robot[iter].blocked_path[deadlock_robot]

                            seen_edge[(iter, deadlock_robot)] = 1

                self.planner_data.deadlock_cycle.remove_node(deadlock_robot)
                iterator = nx.algorithms.cycles.simple_cycles(
                    self.planner_data.deadlock_cycle
                )

                count = 0
                has_cycle = False
                for iter in iterator:
                    if len(iter) > 0:
                        has_cycle = True

                if not has_cycle:
                    logger.info("All Deadlock Got New Path")
                    return

                for robot in self.deadlock_robot:
                    logger.info(self.deadlock_robot[robot])

                self.path_planner.set_weight()
                del self.deadlock_robot[deadlock_robot]

            else:
                deadlock_robot = self.near_waitspot()
                if deadlock_robot in self.deadlock_robot.keys():
                    if self.deadlock_robot[deadlock_robot].resolved:
                        print("Robot Deadlock has been resolved already")
                        return
                # if self.waitspot_for_robot(deadlock_robot) is None:
                #     return

                if deadlock_robot == -1:
                    logger.info("All Deadlock Got New Path")
                    return
                min_waitspot = self.waitspot_for_robot(deadlock_robot)
                print("Deaclock robot is", deadlock_robot)
                print("Waitspot for robot", min_waitspot)
                if self.check_path(
                    deadlock_robot,
                    self.path_planner.plan(
                        self.deadlock_robot[deadlock_robot].start,
                        self.node_to_pos(min_waitspot),
                    ),
                ):
                    return
                logger.info(f"Deadlock:{deadlock_robot}")
                # update path
                for key in self.deadlock_robot[deadlock_robot].blocked_path:
                    blocked_path = self.deadlock_robot[deadlock_robot].blocked_path[key]
                    self.path_planner.block_path(blocked_path)
                self.planner_data.assigned_waitspot[min_waitspot] = True

                logger.info(min_waitspot)

                # self.planner_data.waitspot[min_waitspot] = 0

                self.planner_data.robot_waitspot_path[deadlock_robot] = (
                    self.path_planner.plan(
                        self.deadlock_robot[deadlock_robot].start,
                        self.node_to_pos(min_waitspot),
                    )
                )

                self.deadlock_robot[deadlock_robot].occurance = 0
                self.deadlock_robot[deadlock_robot].resolved = True

                iterator = nx.all_neighbors(
                    self.planner_data.deadlock_cycle, deadlock_robot
                )
                seen_edge = {}
                for iter in iterator:
                    if self.planner_data.deadlock_cycle.has_edge(deadlock_robot, iter):
                        if (deadlock_robot, iter) not in seen_edge:
                            del self.deadlock_robot[deadlock_robot].affected_pair[iter]
                            del self.deadlock_robot[deadlock_robot].blocked_path[iter]

                            seen_edge[(deadlock_robot, iter)] = 1

                    if self.planner_data.deadlock_cycle.has_edge(iter, deadlock_robot):
                        if (iter, deadlock_robot) not in seen_edge:
                            del self.deadlock_robot[iter].affected_pair[deadlock_robot]
                            del self.deadlock_robot[iter].blocked_path[deadlock_robot]

                            seen_edge[(iter, deadlock_robot)] = 1

                self.planner_data.deadlock_cycle.remove_node(deadlock_robot)
                iterator = nx.algorithms.cycles.simple_cycles(
                    self.planner_data.deadlock_cycle
                )

                count = 0
                has_cycle = False
                for iter in iterator:
                    if len(iter) > 0:
                        has_cycle = True

                if not has_cycle:
                    logger.info("All Deadlock Got New Path")
                    return

                for robot in self.deadlock_robot:
                    logger.info(self.deadlock_robot[robot])

                self.path_planner.set_weight()
                del self.deadlock_robot[deadlock_robot]


class Global_Planner:
    def __init__(self, range_tree, blackboard):
        self.planner_data = Planner_Data(range_tree)
        self.blackboard = blackboard
        self.planner = Path_Planner()
        self.range_tree = range_tree
        self.priority_algorithm = Priority_Algorithm(self.planner_data, range_tree)
        self.lazy_wait = Lazy_Wait(self.planner_data, self.planner, self.blackboard)
        self.max_iter = 20

    def update_path(self):
        for robot in self.planner_data.robot_waitspot_path:
            logger.info(self.planner_data.robot_start_idx[robot])
            new_path = []
            start_index = self.planner_data.robot_start_idx[robot]

            for i in range(start_index + 1):
                new_path.append(self.planner_data.agents_path[robot][i])

            wait_path = self.planner_data.robot_waitspot_path[robot]

            if len(wait_path) <= 1:
                continue

            prev_angle = [new_path[-1][2]]

            if new_path[-1][0] == wait_path[1][0]:
                if prev_angle[0] not in [1, 3]:
                    new_path.append((new_path[-1][0], new_path[-1][1], 1))
            elif new_path[-1][1] == wait_path[1][1]:
                if prev_angle[0] not in [0, 2]:
                    new_path.append((new_path[-1][0], new_path[-1][1], 0))

            prev_angle = [new_path[-1][2]]
            for i in range(1, len(wait_path)):
                if (i + 1) < len(wait_path):
                    diff_x = wait_path[i + 1][0] - wait_path[i - 1][0]
                    diff_y = wait_path[i + 1][1] - wait_path[i - 1][1]

                    if abs(diff_x) > 0 and abs(diff_y) > 0:
                        diff_x = wait_path[i + 1][0] - wait_path[i][0]
                        diff_y = wait_path[i + 1][1] - wait_path[i][1]

                        if ((diff_x > 0) and (diff_y == 0)) or (
                            (diff_x < 0) and (diff_y == 0)
                        ):
                            prev_angle.append(0)
                        elif ((diff_x == 0) and (diff_y > 0)) or (
                            (diff_x == 0) and (diff_y < 0)
                        ):
                            prev_angle.append(1)

                for angle in prev_angle:
                    new_path.append((wait_path[i][0], wait_path[i][1], angle))

                prev_angle = [prev_angle[-1]]

            wait_path.reverse()

            for i in range(1, len(wait_path)):
                if (i + 1) < len(wait_path):
                    diff_x = wait_path[i + 1][0] - wait_path[i - 1][0]
                    diff_y = wait_path[i + 1][1] - wait_path[i - 1][1]

                    if abs(diff_x) > 0 and abs(diff_y) > 0:
                        diff_x = wait_path[i + 1][0] - wait_path[i][0]
                        diff_y = wait_path[i + 1][1] - wait_path[i][1]

                        if ((diff_x > 0) and (diff_y == 0)) or (
                            (diff_x < 0) and (diff_y == 0)
                        ):
                            prev_angle.append(0)
                        elif ((diff_x == 0) and (diff_y > 0)) or (
                            (diff_x == 0) and (diff_y < 0)
                        ):
                            prev_angle.append(1)

                for angle in prev_angle:
                    new_path.append((wait_path[i][0], wait_path[i][1], angle))

                prev_angle = [prev_angle[-1]]

            prev_angle = [new_path[-1][2]]

            if start_index + 2 < len(self.planner_data.agents_path[robot]):
                old_path = self.planner_data.agents_path[robot][start_index + 1]
                if new_path[-1][0] == old_path[0]:
                    if prev_angle[0] not in [1, 3]:
                        new_path.append((new_path[-1][0], new_path[-1][1], 1))
                elif new_path[-1][1] == old_path[1]:
                    if prev_angle[0] not in [0, 2]:
                        new_path.append((new_path[-1][0], new_path[-1][1], 0))

            for i in range(start_index + 1, len(self.planner_data.agents_path[robot])):
                new_path.append(self.planner_data.agents_path[robot][i])

            logger.info(self.planner_data.agents_path[robot])
            logger.info(new_path)
            self.planner_data.agents_path[robot] = new_path
            logger.info(self.planner_data.agents_path[robot])

    def time_to_finish_schedule(self):
        traversal_node = {}
        for agent in self.planner_data.agents_path:
            for x, y, angle in self.planner_data.agents_path[agent]:
                self.pos = self.planner_data.get_all_collision(x, y, angle)
                for x, y, angle in self.pos:
                    traversal_node[(x, y, angle)] = 0

        agents_idx = {}
        agents_path_time = {}
        agents_finish_time = {}
        agents_done = {}
        agents_count = {}

        count = 8
        for agent in self.planner_data.agents_path:
            agents_idx[agent] = 0
            agents_path_time[agent] = len(self.planner_data.agents_path[agent]) - 1
            agents_finish_time[agent] = 0
            agents_done[agent] = False
            agents_count[agent] = 0

        for time in range(10000):
            finish = True
            for agent in self.planner_data.agents_path:
                if not agents_done[agent]:
                    finish = False
                    break
            if finish:
                logger.info(f"Agent Finish Time {agents_finish_time}")
                logger.info(f"Agent Path Time {agents_path_time}")
                overall_waittime = 0

                for agent in self.planner_data.agents_path:
                    overall_waittime += (
                        agents_finish_time[agent] - agents_path_time[agent]
                    )

                logger.info(f"Overall Solution Quality {overall_waittime}")
                return

            for agent in self.planner_data.agents_path:
                if agents_done[agent]:
                    continue

                if (agents_idx[agent] + 1) >= len(self.planner_data.agents_path[agent]):
                    agents_count[agent] += 1

                    if agents_count[agent] >= count:
                        agents_done[agent] = True
                        agents_finish_time[agent] = time
                        curr_node = self.planner_data.agents_path[agent][
                            agents_idx[agent]
                        ]
                        self.pos = self.planner_data.get_all_collision(
                            curr_node[0], curr_node[1], curr_node[2]
                        )
                        for x, y, angle in self.pos:
                            traversal_node[(x, y, angle)] += 1
                    else:
                        continue

                if agents_done[agent]:
                    continue

                path_index = {}
                curr_node = self.planner_data.agents_path[agent][agents_idx[agent]]

                self.pos = self.planner_data.get_all_collision(
                    curr_node[0], curr_node[1], curr_node[2]
                )
                for x, y, angle in self.pos:
                    if (x, y, angle) in path_index:
                        path_index[(x, y, angle)] += 1
                    else:
                        path_index[(x, y, angle)] = 0

                all_schedule = True

                for x, y, angle in self.pos:
                    robot_id = self.planner_data.time_schedule[(x, y, angle)][
                        traversal_node[(x, y, angle)] + path_index[(x, y, angle)]
                    ]
                    if robot_id != agent:
                        all_schedule = False
                        break

                if all_schedule:
                    curr_node = self.planner_data.agents_path[agent][agents_idx[agent]]
                    self.pos = self.planner_data.get_all_collision(
                        curr_node[0], curr_node[1], curr_node[2]
                    )
                    for x, y, angle in self.pos:
                        traversal_node[(x, y, angle)] += 1
                    agents_idx[agent] += 1
        logger.info(f"INFEASIBLE SOLUTION CHOICE ")
        logger.info(
            f"Agent PathTime:{agents_path_time},Agent idx:{agents_idx} Agent finishTime:{agents_finish_time}"
        )

    def solve(self):
        start_time = time.time()
        logger.info(
            "*************************************************************************************"
        )
        logger.info("Start of the fleet core")

        # Initialize Waitspot
        for waitspot in self.planner_data.waitspot:
            self.planner_data.waitspot[waitspot] = 1
            logger.info(waitspot)

        for _ in range(self.max_iter):
            self.planner_data.print_path()

            self.planner.set_weight()
            if self.priority_algorithm.priority():
                self.planner_data.print_path()
                self.planner_data.print_time_schedule()
                # Check Feasibilty of solution and Calculate Overall wait time
                # robot_time = self.time_to_finish_schedule()
                logger.info("End of fleet core")
                end_time = time.time()
                logger.info(
                    f"Overall time taken by fleet algorithm {end_time - start_time}"
                )
                logger.info(
                    "**************************************************************************************"
                )
                return True
            else:
                self.lazy_wait.lazy_wait()
                self.update_path()
