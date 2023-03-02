import matplotlib.pyplot as plt

from Road import *
from scipy.spatial import distance
from Intersections import *
import networkx as nx

class TrafficSystemParameterised:
    """
    Parameterised Traffic Simulator that simulates traffic with variable input rate.
    The three methods are

    build_network() -- builds the network given the edge and node list, as well as experimental parameters
    update() -- The master update method that updates every object in the time step
    draw() -- Draws the simulation configuration at a time step when called

    """
    def __init__(self):
        self.traffic_grid = None  # a networkx object?
        self.roads = []
        self.inflows = []
        self.outflows = []
        self.intersections = []
        self.time = 0
        self.system_size = 0 # in term of cells
        self.net_flow_rate = 0.8
        self.cars_in_system = 0
        self.density = []

    def build_network(self, input_dataframe, nodes, node_config, netflow_rate = 0.8, dpi = 300):
        self.net_flow_rate = netflow_rate
        self.original_data = input_dataframe
        # builds networkx directed graph from dataframe and node types
        G = nx.from_pandas_edgelist(input_dataframe, source="In", target="Out",
                                    edge_attr=True, create_using=nx.DiGraph)
        # Build Roads Based on Data Frame
        for u, v, a in G.edges(data=True):
            # build new road
            new_road = Road(ID=a["Edge_ID"], start=[a["start_x"], a["start_y"]],
                            end=[a["end_x"], a["end_y"]], speed_limit=a["speedLimit"] * kmh_to_ms_conversion,
                            start_node= u, end_node= v, master_system = self)

            self.system_size += new_road.length

            G.edges[u, v]["sim_distance"] = distance.euclidean([a["start_x"], a["start_y"]], [a["end_x"], a["end_y"]])
            G.edges[u, v]["road"] = new_road

            self.roads.append(new_road)
        # Compute the shortest paths used to generate graph objects
        self.paths = nx.shortest_path(G, weight="sim_distance")
        # Build Nodes Based on Node Type
        for i in sorted(G.nodes):
            G.nodes[i]["Node_ID"] = nodes['Node_ID'][i]
            G.nodes[i]["type"] = nodes["type"][i]
            G.nodes[i]["x_pos"] = nodes["x"][i]
            G.nodes[i]["y_pos"] = nodes["y"][i]

            if nodes["type"][i] == "inflow":
                possible_paths = [self.paths[i][v] for v in sorted(nodes[nodes.type == "outflow"].Node_ID)]
                input_rate = node_config.loc[i]["relative_probability"]
                path_probabilities = node_config.loc[i]["path_distribution"]
                for u, v, data in G.out_edges(i, data=True):
                    road = data['road']

                G.nodes[i]["node"] = inflow(cycle=[36, 54], input_rate= input_rate*self.net_flow_rate,
                                            paths=possible_paths,
                                            # assume a uniform distribution for now
                                            probs=path_probabilities,
                                            arrival_model=sts.poisson, attached_road=road, master_system = self)

                self.inflows.append(G.nodes[i]["node"])

            if nodes["type"][i] == "outflow":
                for u, v, data in G.in_edges(i, data=True):
                    road = data['road']
                G.nodes[i]["node"] = outflow(cycle=[24, 36], attached_road=road, master_system = self)

                self.outflows.append(G.nodes[i]["node"])

            if nodes["type"][i] == "intersection":
                road_ins = []
                road_outs = []
                for u, v, data in G.in_edges(i, data=True):
                    road_ins.append(data['road'])
                for u, v, data in G.out_edges(i, data=True):
                    road_outs.append(data['road'])

                cycle_offset = 0

                G.nodes[i]["node"] = traffic_intersection(cycle=[36, 54], in_roads=road_ins, cycle_offset= cycle_offset,
                                                          out_roads=road_outs, master_system = self)

                self.intersections.append(G.nodes[i]["node"])

        self.sub_system = [self.inflows,self.roads,self.intersections,self.outflows]

        self.figure, self.axes = plt.subplots(dpi=dpi)  # Create a new figure for use in the draw() method

        self.traffic_grid = G

    def update(self):
        for sub_system in self.sub_system:
            for traffic_object in sub_system:
                traffic_object.update()
        if self.time >= data_collection_time:
            self.density.append(self.cars_in_system / self.system_size)
        self.time += 1

    def draw(self, title):
        self.axes.cla()
        for i, row in enumerate(self.original_data.loc[:, ["start_x", "start_y", "end_x", "end_y"]].iterrows()):
            row = row[1]
            self.axes.plot([row[0], row[2]], [row[1], row[3]], color="black", zorder = 1)
        car_x = []
        car_y = []
        car_v = []
        traffic_light_x = []
        traffic_light_y = []
        traffic_light_green = []
        for road in self.roads:
            for vehicle in road.vehicles:
                car_x.append(vehicle.x)
                car_y.append(vehicle.y)
                car_v.append(vehicle.v)
            traffic_light_x.append(road.end[0] - 10 * road.cos)
            traffic_light_y.append(road.end[1] - 10 * road.sin)
            traffic_light_green.append(road.green_light)
        self.axes.scatter(car_x, car_y, c = car_v, cmap="RdYlGn", s = 5 ,
                          marker = "s",
                          linewidths = 0.25,  edgecolors = "black", zorder = 2)
        self.axes.set_xlim(0,650)
        self.axes.set_ylim(0, 400)
        self.axes.tick_params(axis='both', which='both', reset = True,  bottom='off', top='off', labelbottom='off', right='off',
                              left='off',
                              labelleft='off')
        self.axes.xaxis.set_visible(False)
        self.axes.yaxis.set_visible(False)
        self.axes.set_title(title)
        plot = self.axes.scatter(traffic_light_x, traffic_light_y, c=traffic_light_green, cmap="RdYlGn", zorder = 3)
        return plot
