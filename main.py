# import imp
from itertools import count
import operator
from os import path
import matplotlib.pyplot as plt
import math
import time
import random
import signal
import multiprocessing
import concurrent.futures
from black_board import *
from data import *
from task_manager import *
from simulation_model import Simulation_Model
from visualizer import *
from fleet_system import *
from planner import *
import threading
import re
import cProfile
import pstats
import logging


# simulation time - 0.04, rotation speed 0.8 rad/s -> 2s for 90degree, Unloading time 7s , Loading time 16s
def handler(signum, frame):
    # res = input("Ctrl-c was pressed. Do you really want to exit? y/n ")
    # if res == 'y':
    # exit(1)
    exit(1)


def simulation_loop(sim):
    # sim.root.update()
    # gb.plot_graph()
    while True:
        sim.child[1].event_behaviour()
        sim.child[1].update_robots_position()
        sim.child[1].statistics()
        # sim.root.update()
        time.sleep(0.02)
        # count  += 1

    sim.root.quit()


def visualizer(fleet, task_manager):
    while True:
        task_manager.behaviour()
        fleet.event_behaviour()
        fleet.behaviour()
        time.sleep(0.05)
        # count += 1


def main():
    global_blackboard = BlackBoard()
    global_event_handler = Event_Handler()
    planner = Planner()
    range_tree = Range_Tree()
    fleet = FMS(4, global_blackboard, global_event_handler, planner, range_tree)
    simulation = Controller(1, global_blackboard, global_event_handler)
    simulation_model = Simulation_Model(2, global_blackboard, global_event_handler)
    task_manager = Task_Manager(5, global_blackboard, global_event_handler, planner)
    for i in range(40):
        fleet.add_robot()
    # simulation.root.update()
    # simulation.child[1].visualizer.after(10, simulation.child[1].update_robots_position())
    t = threading.Thread(target=visualizer, args=[fleet, task_manager])
    t1 = threading.Thread(target=simulation_loop, args=[simulation])
    t.start()
    t1.start()

    # p1 = multiprocessing.Process(target=visualizer, args=[fleet])
    # p2 = multiprocessing.Process(target=simulation_loop, args=[simulation])
    # p1.start()
    # p2.start()
    # simulation.child[1].event_behaviour()
    # simulation.child[1].update_robots_position()
    simulation.root.mainloop()
    t.join()
    t1.join()
    # p1.join()
    # p2.join()
    # simulation.root.mainloop()
    # fleet.add_robot()
    # fleet.add_robot()
    # fleet.global_black_board.plot_graph()
    # fleet.add_robot()
    # fleet.global_black_board.plot_graph()
    # fleet.global_event_handler.plot_graph()

    # for i in range(1000):
    # if i == 1:
    # print("Robot Member ", end = " ")
    # print(fleet.global_black_board.read((0, Data.Robot_Member)))
    # fleet.behaviour()
    # time.sleep(0.05)
    # time.sleep(10)


if __name__ == "__main__":
    # profile = cProfile.Profile()
    # profile.runcall(main)
    # ps = pstats.Stats(profile)
    # ps.print_stats()
    main()
