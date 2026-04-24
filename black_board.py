import enum
from queue import Queue
import threading

import networkx as nx
from data import *
import matplotlib.pyplot as plt


"""
BlackBoard - Centralized way of accessing data
Key format is (Global_id, Key_ID)
"""
class BlackBoard:
    def __init__(self):
        #Related to data
        self.data = {}
        self.topic_list = {}
        self.data_graph = nx.Graph(n = 0, directed = True)
        self.data_lock = threading.Lock()
    """
    Register node
    """
    def register_node(self, global_id, name):
        #register node in graph
        self.data_graph.add_node(global_id, name=name)

    """
    Create Data repository [0 - read, 1 - write]
    """
    def register(self, global_id, access, data_type):
        if access == 0:
            pass
            #topic_node = self.topic_list[(global_id, data_type)]
            #self.data_graph.add_edges([(topic_node.index, node_id)])
            #self.data[(global_id, data_type)] = None
        elif access == 1:
            node_name = f"{data_type.name}"
            topic_node = self.data_graph.add_node(data_type.value*1000,name=node_name)
            self.topic_list[data_type.value * 1000] = topic_node
            self.data_graph.add_edge(global_id, data_type.value*1000)
            self.data[(global_id, data_type)] = None
    """
    Read Data Value
    """
    def read(self, key):
        return self.data[key]

    """
    Read all the Data with ending datatype
    """
    def read_all(self, data_type):
        datas = []
        for key in self.data.keys():
            #key match with any data_type
            if key[1] in data_type:
                datas.append(self.read(key))
        return datas

    """
    Read combinely all the Data with list of matched datatype
    """
    def merge_all(self, data_type_list):
        self.data_lock.acquire()
        datas = []
        for key in self.data.keys():
            #key match with any data_type
            if key[1] == data_type_list[0]:
                data = []
                for data_type in data_type_list:
                    data.append(self.read((key[0],data_type)))
                datas.append(data)
        self.data_lock.release()
        return datas
    """
    Write Data into BlackBoard
    """
    def write(self, key, data):
        self.data_lock.acquire()
        self.data[key] = data
        self.data_lock.release()

    def plot_graph(self):
        #self.data_graph.vs["label"] = self.data_graph.vs["name"]
        nx.draw(self.data_graph, with_labels=True, node_size=10)
        plt.show()

"""
Event Handler - Centralized way of handling event
Key format is (Global_id, Event_ID)
"""
class Event_Handler:
    def __init__(self):
        #Events subscriber
        self.events_subscriber = {}
        self.event_list = {}
        self.event_neighbor = {}
        #Events Publisher
        self.events_queue = Queue(maxsize = 0)
        self.update_new_event_lock = threading.Lock()
        self.event_graph = nx.Graph(n = 0, directed = True)


    def register_node(self, name):
        return self.event_graph.add_node(1,name=name)

    """
    Create Event repository access[0 - Subscribe, 1 - Publish]
        my_key - global_id
        subscribe_event_key - (global_id , Event_id)

    """
    def register(self, my_key, subscribe_event_key, access):        
        event_key = self.event_list.keys()
        event_node = None
            #print(event_key)
        if subscribe_event_key in event_key:
            event_node = self.event_list[subscribe_event_key]
        else:
            #node_name = f"{event_type.name}_{subscribe_event_key[0]}"
            #event_node = self.event_graph.add_node(4,name=node_name)
            #self.event_list[(key[0], event_type)] = event_node
            self.event_neighbor[subscribe_event_key] = []

        #print(f"key{key} event_node{event_node.index} event_type {event_type}")
        if access == 0:
            self.event_neighbor[subscribe_event_key].append(my_key)
            self.events_subscriber[my_key] = Queue(maxsize = 0)
            #self.event_graph.add_edge([(event_node.index, node_id.index)])
            #print(f"Register node {node_id}")
            #print(f"Connection between Event {event_type.value} and Node: {node_id.index}")
        elif access == 1:
            pass
            #self.event_graph.add_edge([(node_id.index, event_node.index)])
            #print(f"Connection between Node:{node_id.index} and Event:{event_type.value}")

    """
    Publish Event
    """
    def pub(self, key):
        self.update_new_event_lock.acquire()    
        #self.events_queue.put(event_type)
        #get all subscriber
        neighbors = self.event_neighbor[key]
        for neigh in neighbors:
            self.events_subscriber[neigh].put(key)

        #print(f"{self.events_subscriber[neigh].qsize()}")
        self.update_new_event_lock.release()
    
    """
    Check individual event queue, to find any new event
    """
    def is_there_event(self, key):
        return not self.events_subscriber[key].empty()

    """
    Update event queue
    """
    def get_event(self, key):
        self.update_new_event_lock.acquire()
        event_queue = Queue()
        #Take all event 
        while not self.events_subscriber[key].empty():
            event_queue.put(self.events_subscriber[key].get())   
        self.update_new_event_lock.release()
        return event_queue

    def plot_graph(self):
        #self.event_graph.vs["label"] = self.event_graph.vs["name"]
        #nx.plot(self.event_graph)
        nx.draw(self.event_graph, with_labels=True, node_size=10)