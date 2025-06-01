#!/usr/bin/env python3
import time
import station_navigator as sn
import numpy as np
import rospy

def measure_path_times(stations):
    num_stations = len(stations)
    time_matrix = [[0] * num_stations for _ in range(num_stations)]

    for i in range(num_stations):
        for j in range(i + 1, num_stations):
            sn.move_loc(*stations[i])
            start_time = time.time()

            sn.move_loc(*stations[j])
            end_time = time.time()

            time_matrix[i][j] = end_time - start_time

    return time_matrix

def main():
    rospy.init_node('station_estim_client')
    repeat = 10

    for i in range(repeat):
        if i == 0:
            result_matrix = np.array(measure_path_times(sn.station_locs))

        else:
            result_matrix += np.array(measure_path_times(sn.station_locs))

    result_matrix /= repeat
    print("Average time matrix:")
    print(np.array(result_matrix))
    

if __name__ == '__main__':
    main()    
