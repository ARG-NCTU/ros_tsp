import rospy
from std_msgs.msg import Int16MultiArray
from random import uniform
from dynamic_programming import solve_tsp_held_karp
from greedy_algorithm import solve_tsp_nearest_neighbor
from python_tsp.heuristics import solve_tsp_simulated_annealing
import numpy as np
import math

class Tsp():
    def __init__(self):
        self.pub_point = rospy.Publisher("/tsp_goalpoint", Int16MultiArray, queue_size=1)
        self.offset_x = 0
        self.offset_y = 0
        self.area_length_x = 200
        self.area_length_y = 50
        self.point_num = 100
        self.sub_area_num = 20
        self.point_set = []
        self.permutation = []
        self.distance = 0
    def dist(p1, p2):
        return math.sqrt(((p1-p2)**2).sum())

    def randomGenerate(self, mode):
        rand_point_x = []
        rand_point_y = []
        
        if mode == 'raa':
            rand_point_x = [(uniform(-area_length_x/2, area_length_x/2) + offset_x) for _ in range(point_num)]
            rand_point_y = [(uniform(-area_length_y/2, area_length_y/2) + offset_y) for _ in range(point_num)]
        elif mode == 'rsa':
            for _ in range(sub_area_num):
                generated_x = [(uniform(-area_length_x/2 + i*(area_length_x/sub_area_num), -area_length_x/2 + (i+1)*(area_length_x/sub_area_num))
                    + offset_x) for _ in range(int(point_num/sub_area_num))]
                generated_y = [(uniform(-area_length_y/2, area_length_y/2) + offset_y) for _ in range(int(point_num/sub_area_num))]
                rand_point_x.extend(generated_x)
                rand_point_y.extend(generated_y)
        else:
            print("Wrong input")
            return
        for i in range(point_num):
            self.point_set.append([rand_point_x[i], rand_point_y[i]])
        return

    def distanceGenerate(self, point_set):
        return np.asarray([[dist(np.array(p1), np.array(p2)) for p2 in point_set] for p1 in point_set])
    
    def run(self, mode):
        distance_matrix = distanceGenerate(self.point_set)
        if mode == 'heuristics':
            self.permutation, self.distance = solve_tsp_simulated_annealing(distance_matrix)
        elif mode == 'greedy':
            self.permutation, self.distance = solve_tsp_nearest_neighbor(distance_matrix)
        elif mode == 'dynamic':
            self.permutation, self.distance = solve_tsp_held_karp(distance_matrix)
        else:
            print('Wrong input')
        return
    def publish(self):
        tsp_result = Int16MultiArray()
        tsp_result.data = 0
        self.pub_point.publish(tsp_result)
        return

### assemble point set
for i in range(point_num):
    point_set.append([rand_point_x[i], rand_point_y[i]])
    

if __name__ == '__main__':
    rospy.init_node('tsp_generator', anonymous=False)
    multi_goalpoint = Tsp()
    # input: random in all area = raa, random in every sub area = rsa
    multi_goalpoint.randomGenerate('rsa')
    # input: heuristics, greedy, dynamic
    multi_goalpoint.run("heuristics")
    multi_goalpoint.publish()