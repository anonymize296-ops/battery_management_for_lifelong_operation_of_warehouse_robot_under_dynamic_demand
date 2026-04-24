from calendar import c
from re import I
import networkx as nx
from itertools import permutations
import random as rand
import time
# from numpy import source
from planner import *
from black_board import *
from data import *


class Task_Planner:
    def __init__(self, g_id, m_id, global_black_board, global_event_handler, planner):
         #Global ID
        self.global_id = g_id
        self.module_id = m_id
        #centralized data store
        self.black_board = global_black_board
        self.node_id = self.black_board.register_node(self.global_key(), f"Node:Robot {self.global_id}")

        self.event_handler = global_event_handler 
        self.event_id = self.event_handler.register_node(f"Event:Robot {self.global_id}")
        self.event_queue = None
        #self.start_time = time.time()

        self.planner = planner 

        self.tasks = None
        self.task_to_node = {}
        #self.set_weight()

    def global_key(self):
        return (self.global_id * 100) + self.module_id

    def setup(self):
        #Data
        self.black_board.register(self.global_key(), 0, Data.Task)

    def initiate(self):
        if self.tasks is None or (len(self.tasks) == 0):
            self.tasks = self.black_board.read((200, Data.Task))
            for task_id, pos in self.tasks:
                #self.task_to_node[task_id] = f"{int(pos.x)},{int(pos.y)}" 
                self.task_to_node[task_id] = self.planner.pos_to_node(pos)
                #print(self.graph.nodes[self.task_to_node[task_id]])

    def task_to_path(self, tasks):
        path = [tasks[0]]
        last_task = tasks[0]
        for task in tasks[1:]:
            path += self.planner.johnson_path[last_task][task][1:]
            last_task = task

        return path
        #print(f"Path length {len(path)} and path is {path}")
  
    def optimal_task(self, tasks):
        #self.initiate()
        tasks_id = []
        for task_id, pos in tasks:
            self.task_to_node[task_id] = pos
            tasks_id.append(task_id)
        
        optimal_tasks_id = self.brute_force_faster(tasks_id)
        return  [[task_id, self.task_to_node[task_id]] for task_id in optimal_tasks_id] 
        
    """
    Greedy Strategy I:Select Minimum distance to goal first
    """
    def greedy_next_min_goal(self, tasks):     
        #start_time = time.time()
        M_new = [tasks[i] for i in range(1,len(tasks))]
        last_task = tasks[0]
        best_task = [tasks[0]]
        for i in range(len(tasks)-1):
            #Greedy Strategy I: visit minimum distance to goal first
            min_length = 1000
            min_path = []
            min_task = -1
            for i in range(len(M_new)):
                s_pos = self.task_to_node[last_task]
                #source = f"{int(s_pos.x)},{int(s_pos.y)}"
                source = self.planner.pos_to_node(s_pos)
                t_pos = self.task_to_node[M_new[i]] 
                #target = f"{int(t_pos.x)},{int(t_pos.y)}" 
                target = self.planner.pos_to_node(t_pos)

                path_length = len(self.planner.johnson_path[source][target])
                if path_length < min_length:
                    min_length = path_length
                    min_task = i
            best_task.append(M_new[min_task]) 
            #update the last task
            last_task = M_new[min_task]
            #print(last_task)
            #update the task
            M_new.remove(M_new[min_task])   

        return best_task
        #end_time = time.time()
        #print(f"Greedy Best task {best_task}")
        #self.task_to_path(best_task)
        #print(f"Total time taken {end_time - start_time}")

    """
    Greedy Strategy II: visit path contains maximum set of goals and minimum distance as well
    """
    def greedy_maximum_task(self, tasks):
        #start_time = time.time()
        P = [tasks[0]]
        M_new = [tasks[i] for i in range(1,len(tasks))]
        last_task = tasks[0]
        while len(M_new) > 0:
            #Greedy Strategy II: visit path contains maximum set of goals and minimum distance as well 
            min_length = 1000
            max_task_visited = 0
            max_task_list = []
            min_path = []

            for i in range(len(M_new)):
                s_pos = self.task_to_node[last_task]
                #source = f"{int(s_pos.x)},{int(s_pos.y)}"
                source = self.planner.pos_to_node(s_pos)
                t_pos = self.task_to_node[M_new[i]] 
                #target = f"{int(t_pos.x)},{int(t_pos.y)}" 
                target = self.planner.pos_to_node(t_pos)
                path = self.planner.johnson_path[source][target]
                #print(path)
                task_cover = 0
                task_list = []
                for node in path:
                    if node in M_new:
                        task_cover += 1
                        task_list.append(node)
                
                    if task_cover > max_task_visited:
                        min_path = path
                        max_task_visited = task_cover
                        min_length = len(path)
                        max_task_list = task_list
                    elif task_cover == max_task_visited and len(path) < min_length:
                        min_path = path
                        min_length = len(path)
                        max_task_list = task_list
        
            #update the path
            P += min_path[1:]
            #update the last task
            last_task = max_task_list[-1]
            #update the task
            for task in max_task_list:
                M_new.remove(task)
        #end_time = time.time()
        #print(f"Overall length {len(P)} Path{P} ") 
        #print(f"Total time taken {end_time - start_time}")
    
    def brute_force_faster(self, task):
        min_path_length = 1000
        #start_time = time.time()
        seq = permutations(task[1:])
        P = None
        for sequence in seq:
            last_node = task[0] 
            new_path = [task[0]]
            path_length = 1
            for node in sequence:
                s_pos = self.task_to_node[last_node]
                #source = f"{int(s_pos.x)},{int(s_pos.y)}"
                source = self.planner.pos_to_node(s_pos)
                t_pos = self.task_to_node[node] 
                #target = f"{int(t_pos.x)},{int(t_pos.y)}" 
                target = self.planner.pos_to_node(t_pos)
                path_length += len(self.planner.johnson_path[source][target]) - 1
                last_node = node

            if path_length < min_path_length:
                min_path_length = path_length
                #print(sequence)
                P = sequence
        #end_time = time.time()
        optimal_task = [task[0]]
        #print(P)
        #update optimal task
        for task_id in P:
            optimal_task.append(task_id)
        #self.task_to_path(optimal_task)
        #print(f"Total time taken {end_time - start_time}")
        return optimal_task