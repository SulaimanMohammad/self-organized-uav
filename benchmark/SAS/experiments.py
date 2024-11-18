"""
Experiments with ExSpanBal

"""
# ****************************************************************************
#
#       Copyright (C) 2019      David Coudert <david.coudert@inria.fr>
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 2 of the License, or
#  (at your option) any later version.
#                  https://www.gnu.org/licenses/
# ****************************************************************************
import sage
import re

from sage.graphs.graph import Graph
from sage.functions.other import sqrt
from sage.misc.prandom import uniform, choice, random

from statistics import mean, stdev, variance

#
# ## This ensures that the next import requests will be in the correct directory
# ## i.e., we get the path to this file and we move to it
import inspect
import os
import importlib.util
import argparse
import sys
working_directory = os.getcwd()
source_directory = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
os.chdir(source_directory)
import argparse
import sys
#
#
from drone import DroneState, Drone
from landscape import LandScape
from exspanbal import *
import math
import time


# Get the working directory and source directory paths



# # Change the current working directory to the source directory
# os.chdir(source_directory)
#
# # Define a helper function to load a module from a file dynamically
# def load_module_from_file(module_name, file_path):
#     spec = importlib.util.spec_from_file_location(module_name, file_path)
#     module = importlib.util.module_from_spec(spec)
#     spec.loader.exec_module(module)
#     return module
#
# # Load drone.py and landscape.py dynamically
# drone_module = load_module_from_file('drone', source_directory + '/drone.py')
# landscape_module = load_module_from_file('landscape', source_directory + '/landscape.py')
#
# # Now you can use DroneState, Drone, and LandScape from these modules
# DroneState = drone_module.DroneState
# Drone = drone_module.Drone
# LandScape = landscape_module.LandScape




def exp1(poi, nmax=200, runs=100, filename=None):
    nmin=5
    step=5
    # poi = [(4, 0), (2, 4), (-2, 4), (-6, 4), (-4, 0), (-2, -4), (2, -4), (6, -4)]
    #poi = [(5, 0), (2, 5), (-2, 5), (-7, 5), (-5, 0), (-2, -5), (2, -5), (7, -5)]

    def values(n, s):
        return n, min(s), float(mean(s)), float(stdev(s)), float(variance(s)), max(s)
    f = open("benchmark_predefined_targets_SAS.csv", 'w')
    f.write("n\tSAS-min\tSAS-mean\tSAS-std\tSAS-var\tSAS-max\tCDY-min\tCDY-mean\tCDY-std\tCDY-var\tCDY-max\n")
    random.seed(time.time())
    SAS = []
    CDY = []
    errors = 0
    for n in range(nmin, nmax , step):
        s1 = []
        s2 = []
        print("\n")
        i = 0
        while i < runs:
            # print("n = {}\t{} / {}".format(n, i+1, runs))

            E = ExSpanBal(n, C=100, epsilon=20, radius=20, points_of_interest=poi, verbose=False)
            E.expansion()
            s1.append(len(E.found_points_of_interest))
            E.spanning(first=True)
            while any(di.is_free() for di in E.drones) and len(E.found_points_of_interest) < len(E.points_of_interest):
                E.identify_new_border()
                E.balancing()
                E.expansion_from_border()
                E.spanning(first=False)
                E.identify_new_border(check_false_positive=True)
                E.spanning(extend_to_border_only=True)
            s2.append(len(E.found_points_of_interest))
            i += 1

        SAS.append(values(n, s1))
        CDY.append(values(n, s2))
        print(SAS[-1], CDY[-1])
        s = str(n)
        for A in [SAS[-1][1:], CDY[-1][1:]]:
            for x in A:
                if isinstance(x, int):
                    s += "\t" + str(x)
                else:
                    s += "\t" + str(round(x, 4))
        f.write(s+"\n")

    f.close()

    #print("{} errors, so {}".format(errors, float(errors/(nmax-nmin+1))))
    return SAS, CDY
    
    
def read_exp1(filename):
    """
    """
    SAS = []
    CDY = []
    with open(filename, 'r') as f:
        for i, line in enumerate(f):
            if not i:
                continue
            V = line.split()
            W = [int(v) if j in [0, 1, 5, 6, 10] else float(v) for j,v in enumerate(V)]
            SAS.append(W[:6])
            CDY.append(W[0]+W[6:])
    return SAS, CDY


import matplotlib.mlab as mlab
import matplotlib.pyplot as plt

def myplot1(SAS, CDY, filename="/Users/dcoudert/Recherche/Drones/Plots/avg-poi.png"):
    """
    """
    # Clear the figure
    plt.clf()
    plt.xlabel("Number of drones", fontsize=12)
    plt.ylabel("Average number of discovered points of interest", fontsize=12)
    X = [x[0] for x in SAS]
    S = [x[2] for x in SAS]
    C = [x[2] for x in CDY]
    plt.plot(X, S, 'b', label="SaS", linewidth=1)
    plt.plot(X, C, 'r', label="VESPA", linewidth=1)
    plt.xlim(xmin=min(X), xmax=max(X))
    plt.ylim(ymin=0)
    plt.grid(True, which='both', alpha=0.4)
    # plt.grid(True, which='major', alpha=0.8)
    plt.legend(loc=4, fontsize=12, frameon=True)
    plt.subplots_adjust(left=0.07, right=0.98, bottom=0.1, top=0.98)
    plt.savefig(filename)




def exp2(nmin=200, nmax=200, step=1, runs=1, filename=None):
    """
    Count # rounds and # expansions phases
    exp2(nmin=20, nmax=200, step=1, runs=200)
    """
    # poi = [(4, 0), (2, 4), (-2, 4), (-6, 4), (-4, 0), (-2, -4), (2, -4), (6, -4)]
    poi = [(5, 0), (2, 5), (-2, 5), (-7, 5), (-5, 0), (-2, -5), (2, -5), (7, -5)]

    def values(n, s):
        return n, min(s), float(mean(s)), float(stdev(s)), float(variance(s)), max(s)

    if filename is not None:
        f = open(filename + "-moves.txt", 'w')
        f.write("n\tSAS-min\tSAS-mean\tSAS-std\tSAS-var\tSAS-max\tCDY-min\tCDY-mean\tCDY-std\tCDY-var\tCDY-max\n")
        f2 = open(filename + "-expansions.txt", 'w')
        f2.write("n\tCDY-min\tCDY-mean\tCDY-std\tCDY-var\tCDY-max\n")

    SAS = []
    CDY = []
    CDY2 = []
    errors = 0
    for n in range(nmin, nmax + 1, step):
        s1 = []
        s2 = []
        s3 = []
        print("\n")
        i = 0
        while i < runs:
            print("n = {}\t{} / {}".format(n, i+1, runs))
            try:
                E = ExSpanBal(n, C=100, epsilon=20, radius=20, points_of_interest=poi, verbose=False)
                E.expansion()
                first_expansions = E.counter_expansion
                expansions = E.counter_expansion
                E.spanning(first=True)
                phases = 1
                while any(di.is_free() for di in E.drones) and len(E.found_points_of_interest) < len(E.points_of_interest):
                    E.identify_new_border()
                    E.balancing()
                    expansions += E.counter_balancing_1 + E.counter_balancing_2
                    E.expansion_from_border()
                    expansions += E.counter_expansion
                    E.spanning(first=False)
                    E.identify_new_border(check_false_positive=True)
                    E.spanning(extend_to_border_only=True)
                    phases += 1

                s1.append(first_expansions)
                s2.append(expansions)
                s3.append(phases)
                i += 1
            except:
                errors += 1

        SAS.append(values(n, s1))
        CDY.append(values(n, s2))
        CDY2.append(values(n, s3))
        print(SAS[-1], CDY[-1], CDY2[-1])
        if filename is not None:
            s = str(n)
            for A in [SAS[-1][1:], CDY[-1][1:]]:
                for x in A:
                    if isinstance(x, int):
                        s += "\t" + str(x)
                    else:
                        s += "\t" + str(round(x, 4))
            f.write(s+"\n")
            s = str(n)
            for x in CDY2[-1][1:]:
                if isinstance(x, int):
                    s += "\t" + str(x)
                else:
                    s += "\t" + str(round(x, 4))
            f2.write(s+"\n")
            

    if filename is not None:
        f.close()
        f2.close()

    #print("{} errors, so {}".format(errors, float(errors/(nmax-nmin+1))))
    return SAS, CDY, CDY2

        

# =================================================================================
# =================================================================================
# =================================================================================

def get_min_SAS(poi, nmin=1, nmax=1000, I=None):
    """
    """
    if abs(nmin - nmax) < 4:
        return nmax, I
    n = (nmax + nmin) // 2
    #print(nmin, n, nmax)
    E = ExSpanBal(n, C=100, epsilon=20, radius=20, points_of_interest=poi, verbose=False)
    E.expansion()
    E.spanning(first=True)
    E.final_state()
    Ip = sum(1 for di in E.drones if di.is_irremovable())

    if len(E.found_points_of_interest) < len(poi):
        return get_min_SAS(poi, nmin=n, nmax=nmax, I=I)
    else: # bigger or smaller
        return get_min_SAS(poi, nmin=nmin, nmax=n, I=Ip)

def get_min_CDY(poi, nmin=1, nmax=1000, I=None):
    """
    """
    if abs(nmin - nmax) < 4:
        return nmax, I
    n = (nmax + nmin) // 2
    try:
        E = ExSpanBal(n, C=100, epsilon=20, radius=20, points_of_interest=poi, verbose=False)
        E.expansion()
        E.spanning(first=True)
        while any(di.is_free() for di in E.drones) and len(E.found_points_of_interest) < len(poi):
            E.identify_new_border()
            E.balancing()
            E.expansion_from_border()
            E.spanning(first=False)
            E.identify_new_border(check_false_positive=True)
            E.spanning(extend_to_border_only=True)
    except ValueError as msg:
        # print("n = {}\tPOI = {}".format(n,poi))
        # E.show()
        # raise ValueError("STOP -- {}".format(msg))
        pass

    E.final_state()
    Ip = sum(1 for di in E.drones if di.is_irremovable())

    if len(E.found_points_of_interest) < len(poi):
        return get_min_CDY(poi, nmin=n, nmax=nmax, I=I)
    else:
        return get_min_CDY(poi, nmin=nmin, nmax=n, I=Ip)



def exp3(nmax=30, runs=200, filename=None, steiner=False):
    """
    """
    nmin=1
    def values(n, s):
        if len(s) == 0:
            # Handle case where `s` is empty
            return n, None, None, None, None, None
        elif len(s) == 1:
            # Handle case where `s` contains only one element
            return n, min(s), float(s[0]), None, None, max(s)
        else:
            # Normal case where `s` has more than one element
            return n, min(s), float(mean(s)), float(stdev(s)), float(variance(s)), max(s)

    f = open("num_drones_targets.csv", 'w')
    fI = open("num_irr_drones_targets.csv", 'w')
    s = "n\tSAS-min\tSAS-mean\tSAS-std\tSAS-var\tSAS-max\tCDY-min\tCDY-mean\tCDY-std\tCDY-var\tCDY-max"
    f.write(s + "\n")
    if steiner:
        fI.write(s + "\tST-min\tST-mean\tST-std\tST-var\tST-max\n")
    else:
        fI.write(s + "\n")

    random_targets_file = open("random_targets.txt", 'w')

    SAS, CDY = [], []
    SASI, CDYI, STE = [], [], []
    V = list(LandScape(layers=7).graph)
    V.remove((0, 0))
    L = LandScape()
    random.seed(time.time())
    for num_poi in range(nmin, nmax + 1):
        sas, cdy = [], []
        sasI, cdyI, ste = [], [], []
        random_targets_file.write("num_poi: "+ f"{num_poi} \n")
        for i in range(runs):
            random.shuffle(V)
            poi = V[:num_poi]
            poi_coord= coordinates_poi(poi)
            poi_string = "(" + ", ".join(f"{{{x[0]}, {x[1]}}}" for x in poi_coord) + "),\n"
            random_targets_file.write(poi_string)
            print("pio",poi)
            n, I = get_min_SAS(poi)
            sas.append(n)
            sasI.append(I)
            n, I = get_min_CDY(poi)
            cdy.append(n)
            cdyI.append(I)
            if steiner:
                while any(s not in L.graph for s in poi):
                    L.add_layer()
                ste.append(L.steiner_tree(poi).order())
                print(num_poi, sas[-1],cdy[-1], sasI[-1],cdyI[-1],ste[-1])
                #print("num_poi, sas[-1],cdy[-1], sasI[-1],cdyI[-1],ste[-1]", num_poi, sas[-1],cdy[-1], sasI[-1],cdyI[-1],ste[-1])

            else:
                pass
                #print("else num_poi, i, sas[-1],cdy[-1], sasI[-1],cdyI[-1]", num_poi, i, sas[-1],cdy[-1], sasI[-1],cdyI[-1])
                #print("num_poi, sas[-1],cdy[-1], sasI[-1],cdyI[-1],ste[-1] else", num_poi, i, SAS.append(values(num_poi, sas)), sas[-1],cdy[-1], sasI[-1],cdyI[-1])

        SAS.append(values(num_poi, sas))
        CDY.append(values(num_poi, cdy))
        SASI.append(values(num_poi, sasI))
        CDYI.append(values(num_poi, cdyI))
        if steiner:
            STE.append(values(num_poi, ste))

        s = str(num_poi)
        # print(" SAS is ",  SAS)
        # print( "[SAS[-1][1:], CDY[-1][1:]]", [SAS[-1][1:], CDY[-1][1:]])
        for A in [SAS[-1][1:], CDY[-1][1:]]:
            for x in A:
                if isinstance(x, int):
                    s += "\t" + str(x)
                elif x is not None:
                    s += "\t" + str(round(x, 4))
        # print("S after isinstance",s)
        f.write(s+"\n")
        s = str(num_poi)
        if steiner:
            LLL = [SASI[-1][1:], CDYI[-1][1:], STE[-1][1:]]
        else:
            LLL = [SASI[-1][1:], CDYI[-1][1:]]
        for A in LLL:
            for x in A:
                if isinstance(x, int):
                    s += "\t" + str(x)
                elif x is not None:
                    s += "\t" + str(round(x, 4))
        fI.write(s+"\n")

    random_targets_file.close()
    f.close()
    fI.close()
    return SAS, CDY, SASI, CDYI, STE

def coordinates_poi(pio, radius=20):
    coords=[]
    for x in pio:
        coordinates_x = round((radius *  math.cos(math.pi/6) / 2)*(x[0] * 2 + x[1]))
        coordinates_y = round(-(radius * math.sin(math.pi/6) / 2) * x[1] * 3)
        coords.append( (coordinates_x, coordinates_y) )
    return coords




def reverse_coordinates(coords, radius=20):
    sage_pio = []
    for coord in coords:
        coord_x, coord_y = coord
        # Reverse transformation for y-coordinate
        y = round( (2.0/3.0) *(-coord_y / (radius * math.sin(math.pi / 6.0))))
        # Reverse transformation for x-coordinate
        x = round( ((2*coord_x)/(radius * math.cos(math.pi/ 6.0)) - y) / 2.0)
        sage_pio.append((x, y))
    return sage_pio



with open('../parameters.txt', 'r') as file:
    for line in file:
        if 'predfined_targets=' in line:
            # Extract coordinates from the line
            targets = re.findall(r'\{([-]?\d+),\s*([-]?\d+)\}', line)
            poi_initia = [(int(x), int(y)) for x, y in targets]
            poi= reverse_coordinates(poi_initia, radius=20)

        elif 'number_runs=' in line:
            # Extract number of runs
            runs = int(re.search(r'number_runs=(\d+)', line).group(1))

        elif 'max_num_drones_to_test=' in line:
            # Extract max number of drones
            nmax = int(re.search(r'max_num_drones_to_test=(\d+)', line).group(1))

        elif 'max_number_targets=' in line:
            # Extract max number of targets
            max_number_targets = int(re.search(r'max_number_targets=(\d+)', line).group(1))

        elif 'number_configurations_per_targets=' in line:
            # Extract number of configurations per target
            number_configurations_per_targets = int(re.search(r'number_configurations_per_targets=(\d+)', line).group(1))

        elif 'max_drones_to_check=' in line:
            # Extract max drones to check
            max_drones_to_check = int(re.search(r'max_drones_to_check=(\d+)', line).group(1))


print("Run for defined Targets\n\n")
exp1(poi, nmax,runs)
print("Run for "+ f"{max_number_targets}" + " random Targets\n\n")
exp3(max_number_targets, number_configurations_per_targets)
