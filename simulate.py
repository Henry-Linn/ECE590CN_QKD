import numpy as np
import math
from matplotlib import pyplot as plt

R_earth = 6371 # earth radius, km
leo_altitude = 780 # leo from earth, km
geo_altitude = 35786 # geo from earth, km
leo_period = 100 # LEO period, minutes
geo_period = 1440 # GEO period, minutes
max_range = 2000 # km, before a GS can't connect

# initialize positions
ground_station_1 = {'x': R_earth, 'y': 0, 'z': 0} # red point
ground_station_2 = {'x': R_earth * np.cos(math.radians(75)), 'y': 0, 'z': R_earth * np.cos(math.radians(15))} # green point
leo_sat_1 = {'x': R_earth + leo_altitude, 'y': 0, 'z': 0}
leo_sat_2 = {'x': 0, 'y': R_earth + leo_altitude, 'z': 0}
geo_sat_1 = {'x': R_earth + geo_altitude, 'y': 0, 'z': 0}



def durationVisibility(ground_station_number, satellite_number):
    time = 0; iteration = 0; duration = 0

    if satellite_number == 1 or satellite_number == 2:
        period = leo_period
    else:
        period = geo_period
    
    data = np.zeros(shape = (period, 3))
    while time < period:
        degrees = (360 / period) * time # 3.6 degrees per minute
        theta = math.radians(degrees)
        if satellite_number == 1: # no y direction
            x = (R_earth + leo_altitude) * np.cos(theta)
            y = 0
            z = (R_earth + leo_altitude) * np.sin(theta)
        elif satellite_number == 2: # no x direction
            x = (R_earth + leo_altitude) * np.cos(theta+ math.radians(45))
            y = 0
            z = (R_earth + leo_altitude) * np.sin(theta + math.radians(45))
        elif satellite_number == 3: # GEO 1
            x = (R_earth + geo_altitude) * np.cos(theta)
            y = (R_earth + geo_altitude) * np.sin(theta)
            z = 0
        data[iteration] = [x, y, z]

        if isVisible(data[iteration], ground_station_number):
            duration += 1

        time += 1; iteration += 1

    return data, duration

def isVisible(satellite, ground_station_number):
    if ground_station_number == 1:
        station = {'x': R_earth, 'y': 0, 'z': 0} # red point
    elif ground_station_number == 2:
        station = {'x': R_earth * np.cos(math.radians(75)), 'y': 0, 'z': R_earth * np.cos(math.radians(15))} # red/orange point

    distance = math.sqrt(math.pow(satellite[0] - station["x"], 2) + math.pow(satellite[1] - station["y"], 2) + math.pow(satellite[2] - station["z"], 2))
    
    if distance < max_range:
        return True
    else:
        return False

def main():
    # data _ (station number)(leo/geo)(number), duration _ (station)
    data_gs1l1, duration_gs1l1 = durationVisibility(1, 1)
    data_gs1l2, duration_gs1l2 = durationVisibility(1, 2)

    data_gs2l1, duration_gs2l1 = durationVisibility(2, 1)
    data_gs2l2, duration_gs2l2 = durationVisibility(2, 2)

    data_gs1g1, duration_gs1g1 = durationVisibility(1, 3)
    
    print("Duration of connectivity of GS1-LEO1:", duration_gs1l1)
    print("Duration of connectivity of GS1-LEO2:", duration_gs1l2)
    print("Duration of connectivity of GS2-LEO1:", duration_gs2l1)
    print("Duration of connectivity of GS2-LEO2:", duration_gs2l2)
    print("Duration of connectivity of GS1-GEO1:", duration_gs1g1)

    # transpose all the data
    x1, y1, z1 = data_gs1l1.T
    x2, y2, z2 = data_gs1l2.T
    x3, y3, z3 = data_gs1g1.T

    # plotting the data
    fig = plt.figure()
    ax = plt.axes(projection = "3d")
    ax.set_aspect("auto")
    ax.scatter3D(x1, y1, z1, c = "blue") # LEO sat 1
    ax.scatter3D(x2, y2, z2, c = "red")  # LEO sat 2
    # ax.scatter3D(x3, y3, z3, c = "pink")  # GEO sat 1


    ax.scatter3D(ground_station_1["x"], ground_station_1["y"], ground_station_1["z"], c = "green")
    ax.scatter3D(ground_station_2["x"], ground_station_2["y"], ground_station_2["z"], c = "black")
    # ax.scatter3D(geo_sat_1["x"], geo_sat_1["y"], geo_sat_1["z"], c = "black")
    plt.show()
        

if __name__ == "__main__":
    main()