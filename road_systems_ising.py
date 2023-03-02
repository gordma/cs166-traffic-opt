import matplotlib.pyplot as plt
import numpy as np
import Intersections_ising
from Road import *
from scipy.spatial import distance
from Intersections_ising import *
import networkx as nx

class TrafficManager:
    """
    Traffic Manager object that computes the energy values for the Ising-inspired intersections.
    Also manages the update process and flipping of intersections.

    Takes in the Ising parameters [T, mu, J, K] to control the behavior of the update process.
    """
    def __init__(self, master_system, nodes, cycle, T, mu,J,K):
        self.master_system = master_system
        self.nodes = nodes
        self.master_cycle = cycle
        self.cycle_count = 0
        self.current_field = 1
        self.update_frequency = 18
        self.magnetic_moment = mu
        self.coupling_strength = J
        self.gradient_field = K
        self.T = T
        self.signs_flipped = 0

    def update(self):
        #Main update loop for Manager Object

        #Flip Global Field with the pre-defined 36-54 90 second intervals
        if self.cycle_count > self.master_cycle[int((self.current_field +1 )/2)]:
            #Flip the field
            self.current_field *= -1
            self.cycle_count = 0

        #Update each node given its energy configuration
        for node in (self.master_system.inflows + self.master_system.outflows + self.master_system.intersections):
            E = self.compute_hamiltonian(node)
            node.energy = E
            if (self.cycle_count % self.update_frequency) == 1:
                log_p = (-2*E) / self.T
                if np.log(np.random.uniform(0,1)) < log_p:
                    node.green_light = (node.green_light + 1 ) % 2
                    self.signs_flipped += 1

        self.cycle_count += 1


    def compute_hamiltonian(self,node):
        #Computes the hamiltonian for each node, the energy given the three rules.

        # mu if aligned, -mu if not aligned
        global_field = self.magnetic_moment * (2* (node.green_light - 0.5)) * self.current_field

        #Compute Energy from Coupling with Neighbourhoods
        coupling = 0
        for neighbors in node.successors + node.predecessors:
            coupling += (2* (neighbors.green_light - 0.5))
        coupling *=  (self.coupling_strength * (2* (node.green_light - 0.5)))

        #Computes the Potential Difference Gradient
        if type(node) == Intersections_ising.traffic_intersection:
            if len(node.in_road)  == 2:
                vertical_in, horizontal_in = node.in_road
                vertical_out, horizontal_out = node.out_road
                horizontal_gradient = horizontal_in.current_density() - horizontal_out.current_density()
                vertical_gradient = vertical_in.current_density() - vertical_out.current_density()
            else:
                #This is simplified, since we know both intersection objects are horizontal only
                horizontal_in = node.in_road[0]
                horizontal_out = node.out_road[0]
                horizontal_gradient = horizontal_in.current_density() - horizontal_out.current_density()
                vertical_gradient = 0


        #Compute gradient caused by differences in density of neighbouring road
        else:
            if type(node) == Intersections_ising.outflow:
                #Density of the road we are flowing into
                in_density = node.attached_road.current_density()
                #We are assuming that the other roads are identical to the road in this system, hence the "out" roads
                #Are the mean density of our system
                out_density = (self.master_system.cars_in_system / self.master_system.system_size) /2

            elif type(node) == Intersections_ising.inflow:
                #We make a rough estimate for the in_flow density, since there are no roads physically present.
                in_density = len(node.pool) / 50
                out_density = node.attached_road.current_density()

            else:
                raise ValueError(f'Node type Error {type(node)}, Unrecognized')

            #Deal with the cases of the configurations of inflow and outflow nodes
            #If the attached road is horizontal
            if abs(node.attached_road.cos) > abs(node.attached_road.sin):
                #Horizontal Outflow Gradient
                horizontal_gradient = (in_density - out_density)

                #We assume the hidden vertical road has a gradient that follows a gaussian distribution
                #The gradient differential should be the opposite, since traffic flows alternatively.
                vertical_gradient = np.clip(
                                            np.random.normal( loc = -(in_density - out_density),
                                                              scale =  abs(in_density - out_density) / 5  ),
                                            a_min = -1,
                                            a_max = 1
                                            )
            else:
                #Vice versa for a vertical outflow/inflow
                horizontal_gradient = np.clip(
                                                np.random.normal( loc = -(in_density - out_density),
                                                                  scale = abs(in_density - out_density) / 5  ),
                                                a_min = -1,
                                                a_max = 1
                                            )
                vertical_gradient = (in_density - out_density)
        #Compute the gradient differential energy
        gradient_differential = self.gradient_field * (2* (node.green_light - 0.5)) * (1*horizontal_gradient + (-1)*vertical_gradient)
        #Compute final energy to be returned
        energy = (global_field + coupling + gradient_differential)
        return(energy)





class TrafficSystemIsing:
    """
    Traffic System accounting for the Ising Model's Traffic Manager object.
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

    def build_network(self, input_dataframe, nodes, node_config, netflow_rate = 0.8, dpi = 300, traffic_cycle = [36,54],
                      ising_params = [2,1,1.2,0.8]):
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
            #Compute the distance in simulation given the longitude and lattitude conversions
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

            try:
                successors = [G.nodes[j]["node"] for j in G.successors(i)]
            except Exception:
                successors = []
            try:
                predecessors = [G.nodes[j]["node"] for j in G.predecessors(i)]
            except Exception:
                predecessors = []

            #Create one of the three types of node given the node type specified in the node list.
            if nodes["type"][i] == "inflow":

                possible_paths = [self.paths[i][v] for v in sorted(nodes[nodes.type == "outflow"].Node_ID)]
                input_rate = node_config.loc[i]["relative_probability"]
                path_probabilities = node_config.loc[i]["path_distribution"]

                for u, v, data in G.out_edges(i, data=True):
                    road = data['road']



                G.nodes[i]["node"] = inflow(ID = i,
                                            cycle=[36, 54], input_rate= input_rate*self.net_flow_rate,
                                            paths=possible_paths,
                                            probs=path_probabilities,
                                            arrival_model=sts.poisson, attached_road=road, master_system = self,
                                            successors = successors, predecessors = predecessors)

                self.inflows.append(G.nodes[i]["node"])

            if nodes["type"][i] == "outflow":
                for u, v, data in G.in_edges(i, data=True):
                    road = data['road']


                G.nodes[i]["node"] = outflow(ID = i, cycle=[36, 54], attached_road=road, master_system = self,
                                             successors = successors, predecessors = predecessors)

                self.outflows.append(G.nodes[i]["node"])

            if nodes["type"][i] == "intersection":
                road_ins = []
                road_outs = []
                for u, v, data in G.in_edges(i, data=True):
                    road_ins.append(data['road'])
                for u, v, data in G.out_edges(i, data=True):
                    road_outs.append(data['road'])

                cycle_offset = 0

                G.nodes[i]["node"] = traffic_intersection(ID = i ,
                                                          cycle=[36, 54], in_roads=road_ins, cycle_offset= cycle_offset,
                                                          out_roads=road_outs, master_system = self,
                                                          successors = successors, predecessors = predecessors)

                self.intersections.append(G.nodes[i]["node"])

        self.sub_system = [self.inflows,self.roads,self.intersections,self.outflows]

        self.figure, self.axes = plt.subplots(dpi=dpi)  # Create a new figure for use in the draw() method

        self.traffic_grid = G
        #Compute the Ising Parameters based on the input list and create the traffic manager object.
        T = ising_params[0]
        mu = ising_params[1]
        J = ising_params[2] * ising_params[1]
        K = ising_params[3] * ising_params[1]
        self.traffic_manager = TrafficManager(self,nodes = self.inflows+self.outflows+self.intersections,
                                              cycle = traffic_cycle, T = T , mu =mu,
                                              J = J , K = K)
    #Main Update Loop
    def update(self):
        #Updates each of the objects in the system
        for sub_system in self.sub_system:
            for traffic_object in sub_system:
                traffic_object.update()
        self.traffic_manager.update()

        #Record Density Data for Analysis
        if self.time >= data_collection_time:
            self.density.append(self.cars_in_system / self.system_size)
        self.time += 1

    def draw(self, title):
        #Plotter
        self.axes.cla()
        for node in self.traffic_grid.nodes(data = True):
            nd = node[1]
            self.axes.text(nd['x_pos'] + 5, nd['y_pos'] - 18, s=-np.round(nd['node'].energy,3))

        self.axes.text(0.95, 1.1, f'Global Mag {self.traffic_manager.current_field}',
                       horizontalalignment='center', verticalalignment='center', transform=self.axes.transAxes)
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
