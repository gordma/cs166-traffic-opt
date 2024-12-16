from config import *
import numpy as np

class Car:
    """
    Base class of a car. Updates based on the NS model and adapted to cross intersections and stop at red lights
    The main function is the update function. When the car waits too long to enter another road segment,
    it calls the change_path() function that computes the shortest alternate path out of the system.
    """
    def __init__(self, rel_pos, current_road,initial_v ,path = None , start_x = 0, start_y = 0 , entranceTime = 0):

        self.path = path
        self.rel_pos = rel_pos #relative position on the road
        self.v = initial_v #initial velocity
        self.current_road = current_road #current road car is on
        self.traffic_jam_counter = 0
        #Recording Stats
        self.segment_entrance_time = 0
        self.entranceTime = entranceTime
        self.exitTime = None
        self.distanceTravelled = 0
        #for visualization purposes
        self.x = start_x
        self.y = start_y



    def update(self, lead):
        #Updates velocity based on the front car's position and velocity using the NS model

        #The lead car has "infinite" distance infront of them until the intersection
        if lead:
            self.distance_to_car = (lead.rel_pos - self.rel_pos )
        else:
            self.distance_to_car = np.inf
        #print(distance_to_car)

        #Under the green light we follow the NS model, where the car first accelerates
        if self.v < self.current_road.speed_limit:
            self.v += 1

        #If light isn't green, e.g. red, car slow down as it approaches the stop
        if not self.current_road.green_light:
            distance_to_red_light =  self.current_road.length - self.rel_pos
            if distance_to_red_light < slow_down_cells:
                if self.v > 1:
                    self.v -= 1
                if self.v + self.rel_pos >= self.current_road.length:
                    self.v = distance_to_red_light - 1
        #Decelerate if the car is too close to the next car

        self.v = max(min(self.v, self.distance_to_car - 1),0)
        #Randomize slowing
        if (
                (np.random.random() < prob_slow) and
                (self.v > 0) ) :
            self.v -= 1
        # Check ahead to the next road if there is space to drive
        if (self.rel_pos + self.v) >= self.current_road.length:
            #If the car is at an intersection
            if self.current_road.inroad_intersection:
                #Which road is the car going onto next?
                for next_road in self.current_road.inroad_intersection.out_road:
                    if next_road.end_node in self.path:
                        #If there is a car right infront of us in the next road, don't drive across
                        if next_road.vehicles:
                            if next_road.vehicles[-1].rel_pos == 0:
                                self.v = max(self.current_road.length - self.rel_pos - 1,0)
                                #We will only wait in traffic for a certain amount of time, drivers are impatient
                                if self.v == 0:
                                    self.traffic_jam_counter += 1
                                if self.traffic_jam_counter >= maximum_wait_time:
                                    self.change_path()
                                    self.traffic_jam_counter = 0

        # Advance 1 unit
        self.rel_pos += self.v
        self.distanceTravelled += self.v

        #Record Change in 2-D Basis
        self.x += self.v * self.current_road.cos * cell_size
        self.y += self.v * self.current_road.sin * cell_size


    def change_path(self):
        #Change path if we are waiting too long in our journey to the next road
        possible_paths = self.current_road.master_system.paths
        current_node = self.current_road.end_node
        possible_reroutes = []
        for outflow in self.current_road.master_system.outflows:
            v = outflow.attached_road.end_node
            if v is not self.path[-1]:
                try:
                    possible_reroutes.append(possible_paths[current_node][v])
                except:
                    pass
        if possible_reroutes:
            shortest_alternate_path = min(possible_reroutes, key = lambda i: len(i))
            new_path = self.path[:self.path.index(current_node)] + shortest_alternate_path
            self.path = new_path

    def time_taken(self):
        return(self.exitTime - self.entranceTime)



