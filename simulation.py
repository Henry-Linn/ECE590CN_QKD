from network_component import Link
from network_component import Node
import numpy as np
import math
from matplotlib import pyplot as plt
import datetime
import random
from qiskit import QuantumCircuit, ClassicalRegister, QuantumRegister, execute, BasicAer

# Parameters
iridiumFromEarth = 780 # in km, or 485 mi
radiusEarth = 6378.1 # in km, or 3958.8 mi
orbital = 100 # in minutes

R_earth = 6371 # earth radius, km
leo_altitude = 780 # leo from earth, km
geo_altitude = 35786 # geo from earth, km
leo_period = 100 # LEO period, minutes
geo_period = 1440 # GEO period, minutes
max_range = 2000 # km, before a GS can't connect

connectivity_threshold = 2000
GEO_LEO_connectivity_threshold = 38000
R_LEO_max = 400
R_GEO_max = 40
R_interSate = 1000
R_GEO_LEO_max = 100

GSs = ['GS1', 'GS2']
LEOs = ['LEO1', 'LEO2']
GEOs = ['GEO1']

num_LEOs = len(LEOs)
num_GEOs = len(GEOs)
num_GSs = len(GSs)

GSs_indices = [0,1]
LEOs_indices = [2,3]
GEOs_indices = [4]

node_2_index = dict(zip(GSs+LEOs+GEOs, np.arange(num_LEOs+num_GEOs+num_GSs)))
index_2_node = dict(zip(np.arange(num_LEOs+num_GEOs+num_GSs), GSs+LEOs+GEOs))

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

def cal_distance(node1, node2, t):
  x1,y1,z1 = node1.dynamics[t]
  x2,y2,z2 = node2.dynamics[t]
  return np.sqrt((x1-x2)**2+(y1-y2)**2+(z1-z2)**2)

def visual(satellite, gs):
    x,y,z = satellite.T
    xg,yg,zg = gs.T
   # plotting the data
    fig = plt.figure()
    ax = plt.axes(projection = "3d")
    ax.set_aspect("auto")
    ax.scatter3D(x, y, z, c = "blue") # satellite
    ax.scatter3D(xg, yg, zg, c = "green")

def do_bb84(n):
  qr = QuantumRegister(n, name = "qr")
  cr = ClassicalRegister(n, name = "cr")
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

def construct_connectivities_matrix(links):
  connectivities_matrix = np.zeros((len(node_2_index), len(node_2_index))).astype(int)
  for link in links:
    if link.check_connectivity():
      row_idx = node_2_index[link.get_sourceName()]
      col_idx = node_2_index[link.get_destName()]
      connectivities_matrix[row_idx][col_idx] = 1
      connectivities_matrix[col_idx][row_idx] = 1
  return connectivities_matrix

def construct_partition_connectivity(set_indcies):
  GSs = []
  LEOs = []
  GEOs = []
  for index in set_indcies:
    if index in GSs_indices:
      GSs.append(index)
    elif index in LEOs_indices:
      LEOs.append(index)
    elif index in GEOs_indices:
      GEOs.append(index)
  return GSs, LEOs, GEOs

def routing_str(connectivities_matrix, curr_idx, strs):
  if curr_idx == 1:
    print(strs)
    return
  new_index = np.where(connectivities_matrix[curr_idx] == 1)[0]
  if 1 in new_index and [2,3] not in new_index:
     routing_str(connectivities_matrix, 1, strs+"GS2")
  else:
    for index in new_index:
      if index in LEOs_indices and index_2_node[index] not in strs:
        # LEO routing
        routing_str(connectivities_matrix, index, strs+f"{index_2_node[index]}->")
      elif index in GEOs_indices:
        # GEO routing
        routing_str(connectivities_matrix, index, strs+f"{index_2_node[index]}->")


def routing(connectivities_matrix, curr_idx, strs):
  if curr_idx == 1:
    return strs
  new_index = np.where(connectivities_matrix[curr_idx] == 1)[0]
  if 1 in new_index and [2,3] not in new_index:
     strs.append(index_2_node[1])
     return routing(connectivities_matrix, 1, strs)
  else:
    for index in new_index:
      if index in LEOs_indices and index_2_node[index] not in strs:
        # LEO routing
        temp_index = np.where(connectivities_matrix[index] == 1)[0]
        if (len(temp_index) == 1 and index_2_node[temp_index[0]] in strs):
          strs.append(index_2_node[4])
          return routing(connectivities_matrix, 4, strs)
        else:
          strs.append(index_2_node[index])
          return routing(connectivities_matrix, index, strs)
      elif index in GEOs_indices:
        # GEO routing
        strs.append(index_2_node[index])
        return routing(connectivities_matrix, index, strs)


def main():
    # Calculate dynamics
    dyn_LEO1, dur1 = durationVisibility(1, 1)
    dyn_LEO2, dur2= durationVisibility(1, 2)
    dyn_GEO1, du3 = durationVisibility(1, 3)

    dyn_GS1 = np.zeros((100,3))
    dyn_GS1[:, 0] = np.array([R_earth])

    dyn_GS2 = np.zeros((100,3))
    dyn_GS2[:, 0] = np.array([R_earth * np.cos(math.radians(45))])
    dyn_GS2[:, 2] = np.array([R_earth * np.sin(math.radians(45))])

    # Creating nodes and connection
    # ground stations
    GS1 = Node("GS1", dyn_GS1, 'GS')
    GS2 = Node("GS2", dyn_GS2, 'GS')

    # LEOs
    LEO1 = Node("LEO1", dyn_LEO1, 'LEO')
    LEO2 = Node("LEO2", dyn_LEO2, 'LEO')

    # GEOs
    GEO1 = Node("GEO1", dyn_GEO1, 'GEO')

    possible_links = []
    # Building the edges
    possible_links.append(GS1.connect(LEO1))
    possible_links.append(GS1.connect(LEO2))
    possible_links.append(GS1.connect(GEO1))

    possible_links.append(GS2.connect(LEO1))
    possible_links.append(GS2.connect(LEO2))
    possible_links.append(GS2.connect(GEO1))


    possible_links.append(LEO1.connect(LEO2))
    possible_links.append(LEO1.connect(GEO1))

    possible_links.append(LEO2.connect(GEO1))


    # Simulation! 100 minutes
    # key requested per minute
    n = 1
    for t in range(1, 100):
        print("Time = ", t)
        GS1.update(t)
        GS2.update(t)
        LEO1.update(t)
        LEO2.update(t)

        connectivities_matrix = construct_connectivities_matrix(possible_links)
        # routing_str(connectivities_matrix, 0, "GS1->")
        best_path = routing(connectivities_matrix, 0, ['GS1'])
        path="Selected Routing: "

        for i, elem in enumerate(best_path):
            if elem == 'GS1':
                link = GS1.query_rightLink(best_path[i+1])
                print(link)
                link.consume_key(n)
            elif elem == 'LEO1':
                link = LEO1.query_rightLink(best_path[i+1])
                print(link)
                link.consume_key(n)
            elif elem == "LEO2":
                link = LEO2.query_rightLink(best_path[i+1])
                print(link)
                link.consume_key(n)
            elif elem == 'GEO1':
                link = GEO1.query_rightLink(best_path[i+1])
                print(link)
                link.consume_key(n)

            if elem != 'GS2':
                path+=elem+" -> "
            else:
                path+=elem
            print(" ")

        if do_bb84(n):
            print("Encryption and Decryption Successed!")
        else:
            print("Encryption and Decryption Failed!")

        print(path)
        print("--------------------------")

if __name__ == "__main__":
    main()

