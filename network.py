import math
import numpy as np
import random
from qiskit import QuantumCircuit, ClassicalRegister, QuantumRegister, execute, BasicAer
from matplotlib import pyplot as plt

# iridium constellation
# 6 orbits of 11 satellites each (66 satellites total)
# 
# 780 km above earth
# orbital period is 100.5 minutes
# number/location of nodes -> nodes = satellites
# can decide number/location of GS

"""
Q3: Design and simulate the topology -- nodes, node placement, mobility, connections

Analyze the duration of the connectivity availability between the GS and the satellites in your design
"""

# constants
leo_altitude = 780 # leo from earth, km
geo_altitude = 35786 # geo from earth, km
R_earth = 6378.1 # in km, or 3958.8 mi
leo_period = 100 # in minutes
geo_period = 1440 # in minutes
max_range = 2000 # km for GS-LEO
geo_leo_range = 90000 # km, for GEO-LEO

# initialize position for GS and satellites
ground_station_1 = {'x': R_earth, 'y': 0, 'z': 0}
ground_station_2 = {'x': R_earth * np.cos(math.radians(45)), 'y': 0, 'z': R_earth * np.sin(math.radians(45))}
leo_sat_1 = {'x': R_earth + leo_altitude, 'y': 0, 'z': 0}
leo_sat_2 = {'x': (R_earth + leo_altitude) * np.cos(math.radians(45)), 'y': 0, 'z':  (R_earth + leo_altitude) * np.sin(math.radians(45))}
geo_sat_1 = {'x': R_earth + geo_altitude, 'y': 0, 'z': 0}

# for the QKD
n = 1
qr = QuantumRegister(n, name = "qr")
cr = ClassicalRegister(n, name = "cr")

class Link:
    def __init__(self, node1, node2, t_start, t_end, distance, rate, keys):
        self.node1 = node1
        self.node2 = node2
        self.time_start = t_start
        self.time_end = t_end
        self.link_distance = distance
        self.secret_key_rate = rate
        self.available_keys = keys

    def __str__(self):
        return f"{self.node1}, {self.node2}"

class Node:
    def __init__(self, name, x, y, z):
        self.name = name
        self.x = x
        self.y = y
        self.z = z
        self.connections = []
        self.links = []

    def connect(self, node2, t_start, t_end, distance, rate, keys):
        link = Link(self, node2, t_start, t_end, distance, rate, keys)
        self.links.append(link)
        node2.links.append(self)
        self.connections.append(node2)
        node2.connections.append(self)

    def __str__(self):
        return f"{self.name}"

def distance(x1, x2, y1, y2, z1, z2):
    return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2) + math.pow(z2 - z1, 2))

def do_bb84():
    # initialize
    alice =  QuantumCircuit(qr, cr)
    bob = QuantumCircuit(qr, cr)

    # alice generating
    alice_basis = random.choices(["rectilinear", "diagonal"], k = 1)[0]
    alice_key = random.choices([0, 1], k = 1)[0]
    if alice_basis == "rectilinear":
        if alice_key == "1":
            alice.h(qr[0])
    else:
        if alice_key == "0":
            alice.h(qr[0])
            alice.s(qr[0])
    
    # bob randomly measuring qubits
    bob_basis = random.choices(["rectilinear", "diagonal"], k = 1)[0]
    if bob_basis == alice_basis:
        bob.h(qr[0])
    else:
        bob.h(qr[0])
        bob.s(qr[0])

    # measuring & simulate
    alice.measure(qr[0], cr[0])
    bob.measure(qr[0], cr[0])

    backend = BasicAer.get_backend("qasm_simulator")
    job = execute(alice, backend, shots = 1) # only want 1
    alice_result = job.result().get_counts()

    job = execute(bob, backend, shots = 1)
    bob_result = job.result().get_counts()

    if alice_result == bob_result:
        return 1
    else:
        return 0

def durationVisibility(ground_station_number, satellite_number):
    time = 0; iteration = 0; duration = 0; when = []

    if satellite_number == 1 or satellite_number == 2:
        period = leo_period
    else:
        period = geo_period
    
    data = np.zeros(shape = (period, 3))
    while time < period:
        degrees = (360 / period) * time
        theta = math.radians(degrees)
        if satellite_number == 1: # LEO 1
            x = (R_earth + leo_altitude) * np.cos(theta)
            y = 0
            z = (R_earth + leo_altitude) * np.sin(theta)
        elif satellite_number == 2: # LEO 2
            x = (R_earth + leo_altitude) * np.cos(theta + math.radians(45))
            y = 0
            z = (R_earth + leo_altitude) * np.sin(theta + math.radians(45))
        elif satellite_number == 3: # GEO 1
            x = (R_earth + geo_altitude) * np.cos(theta)
            y = (R_earth + geo_altitude) * np.sin(theta)
            z = 0
        data[iteration] = [x, y, z]

        if isVisible(data[iteration], ground_station_number):
            duration += 1
            when.append(time)

        time += 1; iteration += 1

    return data, duration, when

def isVisible(satellite, ground_station_number):
    if ground_station_number == 1:
        station = ground_station_1
    elif ground_station_number == 2:
        station = ground_station_2

    distance = math.sqrt(math.pow(satellite[0] - station["x"], 2) + math.pow(satellite[1] - station["y"], 2) + math.pow(satellite[2] - station["z"], 2))
    
    if distance < max_range:
        return True
    else:
        return False

def graphical():
    # data _ (station number)(leo/geo)(number), duration _ (station)
    data_gs1l1, duration_gs1l1, when_gs1l1 = durationVisibility(1, 1)
    data_gs1l2, duration_gs1l2, when_gs1l2 = durationVisibility(1, 2)

    data_gs2l1, duration_gs2l1, when_gs2l1 = durationVisibility(2, 1)
    data_gs2l2, duration_gs2l2, when_gs2l2 = durationVisibility(2, 2)

    data_gs1g1, duration_gs1g1, when_gs1g1 = durationVisibility(1, 3)
    
    print(f"Duration of connectivity of GS1-LEO1: {duration_gs1l1} minutes at time {when_gs1l1}")
    print(f"Duration of connectivity of GS1-LEO2: {duration_gs1l2} minutes at time {when_gs1l2}")
    print(f"Duration of connectivity of GS2-LEO1: {duration_gs2l1} minutes at time {when_gs2l1}")
    print(f"Duration of connectivity of GS2-LEO2: {duration_gs2l2} minutes at time {when_gs2l2}")
    print(f"Duration of connectivity of GS1-GEO1: {duration_gs1g1} minutes at time {when_gs1g1}")

    # plotting earth radius
    earth = np.zeros(shape = (180, 3))
    for i in range(180):
        degrees = (360 / 15) * i
        angle = math.radians(degrees)
        x = R_earth * np.cos(angle)
        y = 0
        z = R_earth * np.sin(angle)
        earth[i] = [x, y, z]

    earthx, earthy, earthz = earth.T   

    # plotting the data
    fig = plt.figure()
    ax = plt.axes(projection = "3d")
    ax.set_aspect('auto')
    # ax.scatter3D(x2, y2, z2, c = "red") # orbital path of LEO in same plane as GS
    ax.scatter3D(earthx, earthy, earthz, c = "blue") # earth

    ax.scatter3D(ground_station_1["x"], ground_station_1["y"], ground_station_1["z"], c = "green")
    ax.scatter3D(ground_station_2["x"], ground_station_2["y"], ground_station_2["z"], c = "black")
    ax.scatter3D(leo_sat_1["x"], leo_sat_1["y"], leo_sat_1["z"], c = "red")
    ax.scatter3D(leo_sat_2["x"], leo_sat_2["y"], leo_sat_2["z"], c = "red")
    # ax.scatter3D(geo_sat_1["x"], geo_sat_1["y"], geo_sat_1["z"], c = "black")
    plt.show()

def main():
    user = input("Enter g for graphical or s for simulation: ")
    if user == "g":
        graphical()
        return

    LEO1 = Node("LEO1", leo_sat_1["x"], leo_sat_1["y"], leo_sat_1["z"])
    LEO2 = Node("LEO2", leo_sat_2["x"], leo_sat_2["y"], leo_sat_2["z"])
    GEO1 = Node("GEO1", geo_sat_1["x"], geo_sat_1["y"], geo_sat_1["z"])
    GS1 = Node("GS1", ground_station_1["x"], ground_station_1["y"], ground_station_1["z"])
    GS2 = Node("GS2", ground_station_2["x"], ground_station_2["y"], ground_station_2["z"])

    simulation_time = 36
    total_attempts_GEO = 40
    total_attempts_LEO = 400
    total_attempts_GL = 100
    total_inter = 1000
    processing_delay = 20 # in ms
    # rate_LEO = total_attempts_LEO * (current_dis - max_range) / (0 - max_range)

    # setting arbitrary values - start time, end time, distance, rate, available keys
    geo_connection = (0, simulation_time, geo_altitude, total_attempts_GEO, 0)
    geo_leo1_connection1 = {0, 16, 35006, 100, 0}
    geo_leo1_conneciton2 = {80, simulation_time, 90000, 100, 0}
    geo_leo2_connection1 = {0, 4, 35006, 100, 0}
    geo_leo2_conneciton2 = {simulation_time-28, simulation_time, 90000, 100, 0}
    leo1_leo2_connection = (0, simulation_time, 4026, total_inter, 1000)

    # GS to LEO can be up ~485 to ~2000km
    gs1_leo1_connection1 = (0, 5, 485, 0, 0)
    gs1_leo1_connection2 = (simulation_time - 5, simulation_time, 2000, 0, 0)
    gs1_leo2_connection = ()

    gs2_leo2_connection1 = (0, 5, 485, 0, 0)
    gs2_leo2_connection2 = (simulation_time - 5, simulation_time, 2000, 0, 0)
    gs2_leo2_connection = ()

    # establish connection between GS and satellites
    GS1.connect(GEO1, *geo_connection); GS1.connect(LEO1, *gs1_leo1_connection1); GS1.connect(LEO1, *gs1_leo1_connection2); GS1.connect(LEO2, *gs1_leo2_connection)

    GS2.connect(GEO1, *geo_connection)
    GS2.connect(LEO2, *gs2_leo2_connection)

    LEO1.connect(LEO2, *leo1_leo2_connection)

    LEO_Prob = []
    GEO_Prob = []

    for t in range(20, simulation_time):
        # update positions of LEO at current timestep
        deg = (360 / 100) * t
        theta = math.radians(deg); offset = math.radians(45)
        x = (R_earth + leo_altitude) * np.cos(theta)
        z = (R_earth + leo_altitude) * np.sin(theta)
        x2 = (R_earth + leo_altitude) * np.cos(theta + offset)
        z2 = (R_earth + leo_altitude) * np.sin(theta + offset)

        LEO1.x = x; LEO1.z = z
        LEO2.x = x2; LEO2.z = z2

        # update for the time step
        for i in [GS1.links[0], GS2.links[0], GS1.links[1], GS2.links[1]]:
            # 0 is GS-GEO, 1 is GS-LEO if there is
            if i.node2 == GEO1:
                i.available_keys += i.secret_key_rate
                print(f"Time {t}: {i} {i.available_keys}")
            elif i.node2 == LEO1 or i.node2 == LEO2:
                if i.time_start <= t <= i.time_end:
                    time = abs(((i.time_end + i.time_start) / 2) - t)
                    i.link_distance = 485 + 300*time

                    rate = total_attempts_LEO * (i.link_distance - max_range) // (485 - max_range)
                    print(f"Time {t}: {i} {i.available_keys} + {rate}")
                    i.available_keys += rate

        both_leo_in_range = False
        one_leo_in_range = False
        attempts = total_attempts_GEO
        for link1 in GS1.links:
            for link2 in GS2.links:
                print(f"{link1} --> {link2}")
                if link1.node2 == LEO1 and link2.node2 == LEO2: # check if both LEOs have a connection to ground station
                    if link1.time_start <= t <= link1.time_end and link2.time_start <= t <= link2.time_end:
                        both_leo_in_range = True
                
                if (link1.node2 == GEO1 and link2.node2 == LEO1) or (link1.node2 == LEO1):
                    pass
                # # if GS1 to LEO1 and GS2 to LEO2 are within range
                # if link1.node2 == LEO1 and link1.link_distance < max_range and link2.node2 == LEO2 and link2.link_distance < max_range:
                #     if link1.time_start <= t <= link1.time_end and link2.time_start <= t <= link2.time_end:
                #         leo_leo_in_range = True
                #         link1.available_keys += link1.secret_key_rate - 1
                #         attempts = int(link1.secret_key_rate) + int(link2.secret_key_rate)
        
        # success = sum(do_bb84() for _ in range(attempts))
        # success_prob = success / attempts * 100
        if both_leo_in_range: # can use LEO-LEO
            # LEO_Prob.append(success_prob)
            print(f"Time {t}, using LEO-LEO satellites \n")
        elif one_leo_in_range: # can use GEO-LEO
            # GEO_Prob.append(success_prob)
            print(f"Time {t}, using GEO-LEO satellite \n")
        else: # need to use GEO-GEO
            print(f"Time {t}, using GEO satellite \n")

    # print(f"For LEO connection: {sum(LEO_Prob) / len(LEO_Prob)} where {len(LEO_Prob)} * 400 keys were attempted")
    # print(f"For GEO connection: {sum(GEO_Prob) / len(GEO_Prob)} where {len(GEO_Prob)} * 40 keys were attempted")

    #     if leo_in_range: # can use LEO-LEO
    #         print(f"At time {t}, GS1 and GS2 can connect via LEO Satellite.")
    #         # success = sum(do_bb84() for _ in range(total_attempts_LEO))
    #         # success_prob = success / total_attempts_LEO
    #         # print(success_prob)
    #     elif leo_geo_in_range: # need to use GEO-LEO
    #         print(f"At time {t}, GS1 and GS2 can connect via GEO-LEO Satellite.")            
    #     else:
    #         print(f"At time {t}, GS1 and GS2 must use GEO Satellite.")

if __name__ == "__main__":
    main()


# using distance to get the rate, because of the LEO location
# if i.node2 == LEO1:
#     update_distance = distance(LEO1.x, i.node1.x, LEO1.y, i.node1.y, LEO2.z, i.node1.z)
# elif i.node2 == LEO2:
#     update_distance = distance(LEO2.x, i.node1.x, LEO2.y, i.node1.y, LEO2.z, i.node1.z)

# i.link_distance = update_distance
