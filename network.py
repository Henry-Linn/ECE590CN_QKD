import datetime
import numpy as np
import random
from qiskit import QuantumCircuit, ClassicalRegister, QuantumRegister, execute, BasicAer

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

iridiumFromEarth = 780 # in km, or 485 mi
radiusEarth = 6378.1 # in km, or 3958.8 mi
orbital = 100 # in minutes

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

class Node:
    def __init__(self, name):
        self.name = name
        self.connections = []
        self.links = []

    def connect(self, node2, t_start, t_end, distance, rate, keys):
        link = Link(self, node2, t_start, t_end, distance, rate, keys)
        self.links.append(link)
        node2.links.append(self)
        self.connections.append(node2)
        node2.connections.append(self)


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

def main():
    LEO1 = Node("LEO1")
    # LEO2 = Node("LEO2")
    GEO1 = Node("GEO1")
    GS1 = Node("GS1")
    GS2 = Node("GS2")

    simulation_time = 100
    # setting arbitrary values - start time, end time, distance, rate, key
    geo_connection = (0, simulation_time, 100, 120, 100)
    gs1_leo1_connection = (20, 30, 80, 120, 100)
    gs2_leo1_connection = (25, 35, 80, 120, 100)

    # establish connection between ground stations and LEO1 satellite
    GS1.connect(GEO1, *geo_connection)
    GS2.connect(GEO1, *geo_connection)
    GS1.connect(LEO1, *gs1_leo1_connection)
    GS2.connect(LEO1, *gs2_leo1_connection)

    # share_key = do_bb84(GS1, GS2)
    total_attempts_GEO = 40
    total_attempts_LEO = 400
    processing_delay = 20 # in ms

    # success = sum(do_bb84() for _ in range(total_attempts_LEO))
    # success_prob = success / total_attempts_LEO
    # print(success_prob)

    for t in range(simulation_time):
        # check if LEO satellite is within range of both ground stations
        leo_in_range = False
        for link_1 in GS1.links: # gs1
            for link_2 in GS2.links: # gs2
                if link_1.node2 == LEO1 and link_2.node2 == LEO1:
                    if link_1.time_start <= t <= link_1.time_end and link_2.time_start <= t <= link_2.time_end:
                        leo_in_range = True

        if leo_in_range:
            print(f"At time {t}, GS1 and GS2 can connect via LEO Satellite.")
        else:
            print(f"At time {t}, GS1 and GS2 must use GEO Satellite.")

if __name__ == "__main__":
    main()