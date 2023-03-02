from scipy.spatial import distance
from config import *
from collections import deque
import numpy as np
from car import *
from copy import deepcopy
import statistics

class Road:
    """
    The road class. Houses vehicle objects on a road segment, and handles the addition of a new vehicle to the road
    """
    def __init__(self,ID, start, end, speed_limit, master_system = None,  start_node = None, end_node = None):
        self.ID = ID
        self.vehicles = deque()
        self.master_system = master_system
        self.inroad_intersection = None
        self.outroad_intersection = None
        self.data = None
        self.start = start
        self.end = end
        self.green_light = 1
        self.length = int(distance.euclidean(start,end) / cell_size)
        self.speed_limit = int(speed_limit / cell_size)

        self.cos = (end[0] - start[0]) / distance.euclidean(start,end)
        self.sin = (end[1] - start[1]) / distance.euclidean(start,end)

        self.start_node = start_node
        self.end_node = end_node

        #Statistics for analysis
        self.total_car_passed  = 0
        self.data = []
        self.travel_times = []

    def update(self):
        '''
        Updates the position of each car in the road.

        Outputs the cars that have driven to the end of the road
        '''

        n = len(self.vehicles)
        #if there are cars in the road
        if n > 0:
            #we update from the last car to simulate updating in parallel
            for i in range(n-1,0,-1):
                #who is the car infront
                lead = self.vehicles[i-1]
                #update car i's position and velocity based on car i-1
                self.vehicles[i].update(lead)
            #Lead car does not have a car infront
            self.vehicles[0].update(None)

        if self.master_system.time > data_collection_time:
            self.record_data()

    def record_data(self):
        #only record if it is a non-empty lane

        car_speeds = np.array([cars.v for cars in self.vehicles])
        if len(car_speeds) >= 1:
            speed_harmonic_average = statistics.harmonic_mean((car_speeds+10e-12)*cell_size / kmh_to_ms_conversion)
            speed_mean = statistics.mean(car_speeds*cell_size / kmh_to_ms_conversion)
            try:
                speed_sd = statistics.stdev(car_speeds*cell_size / kmh_to_ms_conversion)
            except Exception:
                speed_sd = 0
            density = self.current_density()
            self.data.append([speed_harmonic_average,speed_mean,speed_sd,density])

    def current_density(self):
        return(len(self.vehicles) / self.length)

    def add_car(self, incoming_vehicle):
        """
        Add a car that have entered this road, adjusts its velocity if it cannot travel at its original velocity
        """
        new_car = incoming_vehicle
        #How much the car can travel if there is nothing in front
        distance_gain = new_car.rel_pos - new_car.current_road.length
        #Now we compute on the new road
        new_car.current_road = self
        #New lead car position
        if self.vehicles:
            last_car_pos = self.vehicles[-1].rel_pos

            #If our car cannot travel as far as it planned
            if last_car_pos <= (distance_gain - 1) :
                car_input_position = last_car_pos
                v_delta = abs(distance_gain - last_car_pos)
                #adjust amount travelled by car
                new_car.distanceTravelled -= v_delta
                # velocity is non-negative
                new_car.v = max(
                    # our car has to travel at a slower velocity not comptued by the previous update
                    min(new_car.v, new_car.v - v_delta)
                    , 0)
            else:
                car_input_position = distance_gain

            new_car.rel_pos = max(car_input_position - 1 ,0)
        else:
            new_car.rel_pos = max(distance_gain - 1,0)

        new_car.x = ( new_car.rel_pos * self.cos * cell_size ) + self.start[0]
        new_car.y = (new_car.rel_pos * self.sin * cell_size )  + self.start[1]
        new_car.segment_entrance_time = self.master_system.time
        self.vehicles.append(new_car)
        self.total_car_passed += 1

    def print_road_status(self):
        state = np.full(self.length, -1, dtype=int)
        for vehicle in self.vehicles:
            state[vehicle.rel_pos] = vehicle.v
        print(''.join('.' if x == -1 else str(x) for x in state), ('G' if self.green_light else 'R'))

