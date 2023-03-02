from abc import ABC, abstractmethod
from collections import deque
from config import *
from car import *
from Road import *
import scipy.stats as sts
class node(ABC):
    """
    An abstract class for the nodes we will be using: intersection, inflow nodes outflow nodes
    """
    def __init__(self, ID, cycle,master_system = None, initial_counter = 0, successors = [], predecessors = []):
        self.ID = ID
        self.master_system = master_system
        self.pool = deque()
        self.cycle = cycle
        self.green_light = 1
        self.counter = initial_counter
        self.successors = successors
        self.predecessors = predecessors
        self.next_light = 1
        self.energy = 0
    @abstractmethod
    def update(self):
        """
        Update based on the node's update rules
        """
        pass

class inflow(node):
    """
    Generator for creating vehicle entrance

    :param cycle: lst
    Cycle is a length 2 list, containing the seconds of red light, versus seconds of green light
    :param arrival_rate : float
    The mean arrival rate for vehicles, so how many vehicles per second
    """
    def __init__(self, ID, cycle, input_rate, paths, probs, arrival_model, attached_road : Road, successors, predecessors,
                 master_system = None):
        super().__init__(ID, cycle, master_system, successors, predecessors)
        self.arrival_rate = input_rate
        self.arrival_model = arrival_model
        self.paths = paths
        self.path_probs = probs
        self.attached_road = attached_road

    def update(self):
        #How many number of new cars to generate
        new_cars = self.arrival_model.rvs(mu = self.arrival_rate)
        #Generate Cars
        for __ in range(new_cars):
            self.pool.append(self.generate_car())

        #Retrofitted logic to accomodate green light rules in the Ising Model
        actual_green = False
        #In a horizontal road, green light means green light
        if abs(self.attached_road.cos) > abs(self.attached_road.sin) and self.green_light:
            actual_green = True
        #In a vertical road, a horizontal red light means vertical green light
        elif abs(self.attached_road.cos) <  abs(self.attached_road.sin) and not self.green_light:
            actual_green = True

        #Whether the light is actually green
        if actual_green:
            if self.pool:
                new_car = self.pool.popleft()
                #Check if there is a car at the start of the road
                if self.attached_road.vehicles:
                    #If there isn't we can add
                    if self.attached_road.vehicles[-1].rel_pos != 0:
                        self.attached_road.add_car(new_car)
                        self.master_system.cars_in_system += 1
                    #Otherwise we place the car back in the pool
                    else:
                        self.pool.appendleft(new_car)
                        pass
                else: #We add as normal
                    self.attached_road.add_car(new_car)
                    self.master_system.cars_in_system += 1

    def generate_car(self):
        #Car generator, we pick a path given that the car enters from this road.
        path = np.random.choice(len(self.paths), p = self.path_probs)

        return(Car(rel_pos= 2 + self.attached_road.length ,path = self.paths[path],
                   initial_v = 1, current_road=self.attached_road,
                   start_x = self.attached_road.start[0], start_y= self.attached_road.start[1], entranceTime= self.master_system.time))

class outflow(node):
    """
    Car exits the system from these points
    Assume that a traffic light governs each exit, so cars can only exit at green light
    """
    def __init__(self, ID, cycle , attached_road : Road , successors, predecessors,
                 master_system = None):
        super().__init__(ID, cycle, master_system, successors = successors, predecessors = predecessors)
        self.attached_road = attached_road
    def update(self):
        open_road = self.attached_road
        #If horizontal Road
        if abs(open_road.cos) > abs(open_road.sin):
            open_road.green_light = self.green_light
        #The road is vertical, and the road's light is opposite of the node's light status
        else:
            open_road.green_light = (self.green_light + 1) % 2
        if self.attached_road.vehicles:
            # For all the vehicles that have completed the journey of the road
            while self.attached_road.vehicles and self.attached_road.vehicles[0].rel_pos >= self.attached_road.length:
                #We take them out and record statistics of the vehicle.
                vehicles_out = self.attached_road.vehicles.popleft()
                vehicles_out.exitTime = self.master_system.time
                vehicles_out.current_road.travel_times.append(
                    self.master_system.time - vehicles_out.segment_entrance_time)
                self.pool.append(vehicles_out)
                self.master_system.cars_in_system -= 1




class traffic_intersection(node):
    """
    traffic_intersection objects contains roads of in-traffic and out-traffic and controls traffic light of in-traffics
    Handles the transition of the vehicle from one road to another road in its path.
    """
    def __init__(self, ID, cycle ,in_roads, out_roads, successors, predecessors,
                 master_system = None, cycle_offset = 0, initial_close = 0 ):
        super().__init__(ID, cycle,master_system, cycle_offset, successors = successors, predecessors = predecessors)
        #The roads that goes INTO the intersection
        self.in_road = in_roads
        self.green_light = (self.green_light + initial_close) % 2

        #The roads that goes OUT of the intersection
        self.out_road = out_roads

        # Ordering roads, vertical is in position 0, horizontal is in position 1
        self.in_road.sort(key=lambda x: abs(x.cos))
        self.out_road.sort(key=lambda x: abs(x.cos))

        #Close the vertical lane initially.
        if len(in_roads) > 1:
            self.in_road[ self.green_light - 1 ].green_light = 0

        for road in self.in_road:
            road.inroad_intersection= self
        for road in self.out_road:
            road.outroad_intersection= self

    def update(self):
        #Updates which light is green.
        if len(self.in_road) == 2:
            open_road = self.in_road[self.green_light]
            open_road.green_light = 1
            closed_road = self.in_road[(self.green_light + 1) %2]
            closed_road.green_light = 0
        if len(self.in_road) == 1:
            open_road = self.in_road[0]
            if abs(open_road.cos )> abs(open_road.sin):
                open_road.green_light = self.green_light
            else:
                open_road.green_light = (self.green_light + 1 ) %2

        if open_road.vehicles:
            # For all the vehicles that have completed the journey of the road
            while open_road.vehicles and open_road.vehicles[0].rel_pos >= open_road.length :
                transfer_success = False
                #Which road to add the vehicle to?
                for next_road in self.out_road:
                    if next_road.end_node in open_road.vehicles[0].path:
                        vehicle_out = open_road.vehicles.popleft()
                        vehicle_out.current_road.travel_times.append(self.master_system.time - vehicle_out.segment_entrance_time)
                        next_road.add_car(vehicle_out)
                        transfer_success = True
                        if not open_road.vehicles:
                            break
                #Just incase the path was poorly-defined
                if not transfer_success:
                    raise Exception(f"Car Don't Know Where to Go. Path {open_road.vehicles[0].path}, "
                                    f"Available Roads {[outroad.ID for outroad in self.out_road]}")


