from abc import ABC, abstractmethod
from collections import deque
from config import *
from car import *
from Road import *
import scipy.stats as sts
class node(ABC):
    """
    An abstract class for the nodes we will be using: intersection, inflow nodes outflow nodes

    :param cycle: lst



    """
    def __init__(self, cycle,master_system = None, initial_counter = 0):
        self.master_system = master_system
        self.pool = deque()
        self.cycle = cycle
        self.green_light = 1
        self.counter = initial_counter
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
    """
    def __init__(self, cycle, input_rate, paths, probs, arrival_model, attached_road : Road, master_system = None):
        super().__init__(cycle, master_system)
        self.arrival_rate = input_rate
        self.arrival_model = arrival_model
        self.paths = paths
        self.path_probs = probs
        self.attached_road = attached_road

    def update(self):
        new_cars = self.arrival_model.rvs(mu = self.arrival_rate)
        #Generate Cars
        for __ in range(new_cars):
            self.pool.append(self.generate_car())
        #Greenlight -- Add one car in Pool to flow freely into the system
        if self.green_light:
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


        if self.counter < self.cycle[self.green_light]:
            self.counter += 1
        #Change light, reset counter
        else:
            self.green_light = (self.green_light + 1) % 2
            self.counter = 0
    def generate_car(self):
        path = np.random.choice(len(self.paths), p = self.path_probs)

        return(Car(rel_pos= 2 + self.attached_road.length ,path = self.paths[path],
                   initial_v = 1, current_road=self.attached_road,
                   start_x = self.attached_road.start[0], start_y= self.attached_road.start[1], entranceTime= self.master_system.time))

class outflow(node):
    """
    Car exits the system from these points
    Assume that a traffic light governs each exit, so cars can only exit at green light
    """
    def __init__(self, cycle , attached_road : Road , master_system = None):
        super().__init__(cycle, master_system)
        self.attached_road = attached_road
    def update(self):
        if self.attached_road.vehicles:
            # For all the vehicles that have completed the journey of the road
            while self.attached_road.vehicles and self.attached_road.vehicles[0].rel_pos >= self.attached_road.length:
                vehicles_out = self.attached_road.vehicles.popleft()
                vehicles_out.exitTime = self.master_system.time
                vehicles_out.current_road.travel_times.append(
                    self.master_system.time - vehicles_out.segment_entrance_time)
                self.pool.append(vehicles_out)
                self.master_system.cars_in_system -= 1


        if self.counter < self.cycle[self.green_light]:
            self.counter += 1
        #Change light, reset counter
        else:
            self.green_light = (self.green_light + 1) % 2
            self.attached_road.green_light = self.green_light
            self.counter = 0

class traffic_intersection(node):
    """
    traffic_intersection objects
    contains roads of in-traffic and out-traffic and controls traffic light of in-traffics
    """
    def __init__(self, cycle ,in_roads, out_roads, master_system = None, cycle_offset = 0, initial_close = 0 ):
        super().__init__(cycle,master_system, cycle_offset)

        self.in_road = in_roads
        self.green_light = (self.green_light + initial_close) % 2
        #Ordering roads, vertical is in position 0, horizontal is in position 1
        self.in_road.sort(key=lambda x: abs(x.cos))
        #Close the vertical lane
        if len(in_roads) > 1:
            self.in_road[ self.green_light - 1 ].green_light = 0

        self.out_road = out_roads

        for road in self.in_road:
            road.inroad_intersection= self
        for road in self.out_road:
            road.outroad_intersection= self

    def update(self):
        if len(self.in_road) == 2:
            open_road = self.in_road[self.green_light]
        if len(self.in_road) == 1:
            open_road = self.in_road[0]
        if open_road.vehicles:
            # For all the vehicles that have completed the journey of the road
            while open_road.vehicles and open_road.vehicles[0].rel_pos >= open_road.length :
                transfer_success = False
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


        if self.counter < self.cycle[self.green_light]:
            self.counter += 1
        #Change light, reset counter
        else:
            if len(self.in_road) == 2:
                self.in_road[self.green_light].green_light = (self.in_road[self.green_light].green_light + 1 ) % 2
                self.green_light = (self.green_light + 1) % 2
                self.in_road[self.green_light].green_light = (self.in_road[self.green_light].green_light + 1) % 2
                self.counter = 0
            elif len(self.in_road) == 1:
                self.green_light = (self.green_light + 1) % 2
                self.in_road[0].green_light = (self.in_road[0].green_light + 1) % 2
                self.counter = 0

