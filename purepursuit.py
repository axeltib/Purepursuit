import json
import matplotlib
from matplotlib import pyplot as plt
import numpy as np
import time

class Bicycle_model:
    def __init__(self, look_ahead, speed):
        self.wheelbase = 1
        self.velocity = speed
        self.state = np.array([0, 0, np.pi/2])  # (x, y, yaw)
        self.look_ahead = look_ahead     #How long
        self.path_taken = ([], [])

    def update_state(self, steering_point, dt):
        """Updating the state of the bicycle"""

        steering_angle = self.get_steering_angle(steering_point)

        #Updates the state of the model
        u = np.array(
        [self.velocity * dt * np.cos(self.state[2]),
        self.velocity * dt * np.sin(self.state[2]),
        self.velocity * dt * np.tan(steering_angle) / self.wheelbase])

        self.state = np.add(self.state, u, casting = 'unsafe')
        #Updates path taken
        self.path_taken[0].append(self.state[0])
        self.path_taken[1].append(self.state[1])

    def get_steering_angle(self, steering_point):
        """Calculates the steering angle to a given point"""
        #Calculates relative vector from bicycle to look ahead point
        delta = [steering_point[0] - self.state[0],
                 steering_point[1] - self.state[1]]
        #Calculates the angle (argument) of the relative vector
        delta_len = np.sqrt(delta[0]**2 + delta[1]**2)
        abs_angle = np.arctan(delta[1]/delta[0])
        #Returns diff between said agnle and current yaw, results in st. angle
        return abs_angle - self.state[2]

class PurePursuit:
    """
    Notes:
    choose a starting index, then increase until the distance from the bicycle
    to the point of the path is the look ahead. Svae the last index
    """
    def __init__(self, look_ahead, dt = 0.1, speed = 10):
        plt.ion()   #Magic method for everythong to work...
        #Time between cycles
        self.dt = dt
        #Parameters used for plotting
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], lw=2)
        plt.title('Pure Pursuit')

        with open('path.json', 'r') as f:
            self.path = json.load(f)        #Saving path points
        f.close()

        #Coordinates for plotting the map
        self.graph_path = ([], [])
        for coordinate in self.path:
            self.graph_path[0].append(coordinate[0])
            self.graph_path[1].append(coordinate[1])

        #Used for checking if new index is out of bounds
        self.max_index = len(self.path) - 1

        self.bicycle = Bicycle_model(look_ahead, speed)  #starting bicycle instance


    def run_sim(self):
        """Main method, runs the simulation of pure pursuit"""
        #Gets point index near bicycle at start, la=look ahead
        la_point_index = self.get_start_index()
        while True: #The main loop
            #print('Next step: \n')
            #start = time.time()    #Used for performance check
            #Gets index for the look ahead point
            la_point_index = self.get_lookahead_point_index(la_point_index)
            #print(time.time() - start)
            self.bicycle.update_state(self.path[la_point_index], self.dt)
            self.update_plot(la_point_index)
            time.sleep(self.dt)

    def update_plot(self, la_point_index):
        """Updates the plot based on the current state and the path taken"""
        #Clear plot
        plt.clf()
        #plot path
        plt.plot(self.graph_path[0], self.graph_path[1], 'c', linewidth = 3)
        #Plot look ahead point
        plt.plot(self.path[la_point_index][0], self.path[la_point_index][1], 'kx')
        #Plot path taken
        plt.plot(self.bicycle.path_taken[0], self.bicycle.path_taken[1], 'm')
        #Plot current position
        plt.plot(self.bicycle.state[0], self.bicycle.state[1], 'bo')

        self.fig.canvas.flush_events()

    def get_lookahead_point_index(self, last_index):
        """Returns the point at which the steering angle point towards"""
        index = last_index
        #Looking for thecorresponding index of the point on the path that is at
        #the look ahead distance from the bicycle
        #Increases the index until the delta vector is the look ahead distance
        while np.linalg.norm(self._diff_list(self.path[index], [self.bicycle.state[0],
                              self.bicycle.state[1]])) < self.bicycle.look_ahead:
            index = self._increase_index(index)
            if index == last_index: #Walked around the path one time
                return last_index #Want to steer towards the path
        return index


    def get_start_index(self, margin = 0.1):
        """Returns the corresponding index of the starting point (within
        a certain margin). Assumes that the bicycle starts approximately on
        the path. """
        start_point = (self.bicycle.state[0], self.bicycle.state[1])
        for i in range(len(self.path)):
            if abs(self.path[i][0] - start_point[0]) < margin:
                if abs(self.path[i][1] - start_point[1]) < margin:
                    return i    #Returns corresponding index

    def _increase_index(self, index):
        """Increases path index with one and checks if out of bounds. Loops
        back otherwise"""
        return (index + 1) % self.max_index

    def _diff_list(self, list1, list2):
        """Returns the vector difference of two dim 2 lists"""
        return [list1[0] - list2[0], list1[1] - list2[1]]

def main():
    pp = PurePursuit(5, 0.05, 20)
    pp.run_sim()

if __name__ == '__main__':
    main()
