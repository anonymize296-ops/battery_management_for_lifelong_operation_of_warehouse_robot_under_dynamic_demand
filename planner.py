import networkx as nx
import time


class Planner:
    def __init__(self):
        self.graph = nx.read_gml("./config/research_layout_2/layout_d.gml")
        self.set_weight()
        # all pair shortes path algorithm
        self.floyd_dist = None
        self.johnson_path = None
        self.johnson()

    def set_weight(self):
        for edge in self.graph.edges:
            self.graph[edge[0]][edge[1]]["weight"] = 1.0

    def pos_to_node(self, pos):
        return f"{int(pos.x)},{int(pos.y)}"

    """
    All Pair Shortest Path algorithm, Floyd Warshall
    """

    def floyd_warshall_graph(self):
        start_time = time.time()
        self.floyd_dist = nx.floyd_warshall(self.graph)
        end_time = time.time()
        # print(f"Time taken to complete Floyed Warshall {end_time - start_time}")

    def johnson(self):
        # start_time = time.time()
        self.johnson_path = nx.johnson(self.graph)
        # end_time = time.time()
        # print(f"Time taken to complete Johnson algorithm {end_time - start_time}")

    def plan(self, source, target):
        return self.johnson_path[source][target]

    # def plan
    def plan_distance(self, source, target):
        return len(self.johnson_path[source][target])
