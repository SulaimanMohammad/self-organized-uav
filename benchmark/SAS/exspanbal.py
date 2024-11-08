"""
Class ExSpanBal

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
# from sage.graphs.graph import Graph
# from sage.functions.other import sqrt
# from sage.misc.prandom import uniform, choice, random


from statistics import mean

## This ensures that the next import requests will be in the correct directory
## i.e., we get the path to this file and we move to it
import inspect,os
working_directory = os.getcwd()
source_directory = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
os.chdir(source_directory)
import math
#import argparse
#import sys

import random
from drone import DroneState, Drone
from landscape import LandScape
# load(source_directory + '/drone.py')
# load(source_directory + '/landscape.py')

## Now that the code is loaded, we go back to the working directory
os.chdir(working_directory)




class ExSpanBal:
    """
    """

    def __init__(self, num_drones, num_poi=None, points_of_interest=None,
                     communication_radius=30, C=1, epsilon=1E-9,
                     radius=10, scale=.5,
                     visual=False, verbose=None):
        """
        INPUT:

        - ``num_drones`` -- number of drones

        - ``num_poi`` -- number (default: ``None``); when specified, points of
          interest are selected randomly

        - ``points_of_interest`` -- list (default: ``None``); list of points of
          interests expressed as spots

        - ``communication_radius`` -- int (default: 30); radius of the ball in
          which the drone is able to communicate

        - ``C`` -- a constant (default: 1);

        - ``epsilon`` -- a constant (default: 1E-9); a small value preventing
          overlaps in random choices

        - ``radius`` -- number (default: 10); radius of a spot

        - ``scale`` -- number in [0..1] (default: 0.5); scale for tikz output

        """
        self.landscape = LandScape(layers=math.ceil(math.sqrt(num_drones)/2), radius=radius)
        if points_of_interest is None:
            if num_poi is None:
                raise ValueError("num_poi and points_of_interest cannot be None at the same time")
            self.points_of_interest = set()
            while len(self.points_of_interest) < num_poi:
                spot = self.landscape.random_spot()
                if spot != self.landscape.sink:
                    self.points_of_interest.add(spot)
        else:
            self.points_of_interest = points_of_interest
        self.found_points_of_interest = dict()
        self.phase = 0
        self.action = 0

        self.drones = [Drone(i, spot=self.landscape.sink, communication_radius=communication_radius,
                                 C=C, epsilon=epsilon, landscape=self.landscape)
                           for i in range(num_drones)]
        self._current_border = set([self.landscape.sink])

        self.C = C
        self.epsilon = epsilon
        self.radius = radius
        self.scale = scale
        self.visual = visual
        self.verbose = verbose is not False

    def reset(self):
        """
        Reset the position and state of all drones
        """
        sink = self.landscape.sink
        for di in self.drones:
            di.reset(sink)
        self._current_border = set([self.landscape.sink])
        self.phase = 0
        self.action = 0
        self.found_points_of_interest = dict()

    def new_action(self, action=None):
        """
        """
        if action is None or action == 'expansion':
            self.phase += 1
            self.action = 0
        else:
            self.action += 1

    def get_action_stamp(self):
        """
        """
        return (self.phase, self.action)

    def get_drones_per_spot(self, drones=None):
        """
        Return a dictionary keyed by spots with all drones on that spot
        """
        drones = self.drones if drones is None else drones
        D = {}
        for di in drones:
            spot = di.get_spot()
            if spot in D:
                D[spot].append(di)
            else:
                D[spot] = [di]
        return D

    def move_all_free_drones_to_sink(self):
        """
        All free drones move back to the sink
        """
        for di in self.drones:
            if di.is_free():
                di.set_target_spot(sink)
                di.move_to_target()


    def _identify_drones_at_the_border(self, drones_per_spot):
        """
        Identify drones at the border

        INPUT:

        - ``drones_per_spot`` -- dict keyed by spot recording the current list
          of drones in each spot
        """
        # In theory, we should follow a path of irremovable drones from the
        # sink, possibly followed by free/border drones, until the end of
        # the path

        # In practice, we find the drone at maximal distance from the sink
        # in the x direction and bet it is on the external boundary...
        def blop(spot):
            if spot[0] <= 0 or spot[1] != 0:
                return 0
            return self.landscape.euclidian_distance(spot)

        di = max(self.drones, key=lambda x: blop(x.get_spot()))
        spot = di.get_spot()

        di.set_state(DroneState.BORDER)
        border = [(spot, di)]

        # We search for the next occupied spot
        spot_neighbors = self.landscape.get_spot_neighbors(spot)
        # We know that spot_neighbors[0] is empty (next spot on x axis)
        for i in range(1, 6):
            s = spot_neighbors[i]
            if s in drones_per_spot and drones_per_spot[s]:
                dj = drones_per_spot[s][0]
                dj.set_state(DroneState.BORDER)
                border.append((s, dj))
                break

        # We move along the external border until we come back to the first
        # spot
        while True:
            current_spot, current_drone = border[-1]
            previous_spot, _ = border[-2]
            next_spot = current_drone.next_occupied_right(previous_spot, drones_per_spot)

            if next_spot == border[0][0]:
                # We are done
                break

            di = drones_per_spot[next_spot][0]
            di.set_state(DroneState.BORDER)
            border.append((next_spot, di))

        if self.verbose:
            print("Identify drones at the border in {} steps".format(len(border)))
        self._current_border = border


    def identify_new_border(self, check_false_positive=False):
        """
        """
        for di in self.drones:
            di.set_border(False)

        # We identify drones at the border
        self._identify_drones_at_the_border(self.get_drones_per_spot())


    def _move_one_round_away(self, drones_per_spot, points_of_interest, C, epsilon):

        """
        All free drones move one spot away from the sink.

        INPUT:

        - ``drones_per_spot`` -- dict keyed by spot recording the current list
          of drones in each spot

        - ``points_of_interest`` -- set of points_of_interest that have not yet
          been found

        - ``C`` -- a constant

        - ``epsilon`` -- a small constant
        """
        sink = self.landscape.sink
        drones_to_move = []
        current_border = {s: i for i,(s,_) in enumerate(self._current_border)}

        # On each spot, all drones but one move away from the sink.  Each drone
        # thus decides its target spot independently.  We treat all drones on
        # the same spot at once for efficiency
        for spot, drones in drones_per_spot.items():
            if len(drones) == 1:
                continue

            # identify free drones on that spot
            free_drones = [di for di in drones if di.is_free()]

            # at least one drone must stay on that spot
            if len(free_drones) == len(drones):
                di = min(free_drones, key=lambda x: x.id)
                free_drones.remove(di)

            # All other drones will move
            if free_drones and spot == sink:
                # Special case for the sink
                neighbor_spots = self.landscape.get_spot_neighbors(spot)
                for i, di in enumerate(free_drones):
                    di.set_target_spot(neighbor_spots[i % 6])
                drones_to_move = free_drones

            elif free_drones:
                # We get neighbor spots and number of drones in each of them
                if spot in current_border:
                    # Free drones can only move outside the border
                    # We get previous / next spots on the border
                    if current_border[spot] > 0:
                        previous_spot = self._current_border[current_border[spot] - 1][0]
                    else:
                        previous_spot = self._current_border[-1][0]
                    if current_border[spot] < len(current_border) - 1:
                        next_spot = self._current_border[current_border[spot] + 1][0]
                    else:
                        next_spot = self._current_border[0][0]

                    neighbor_spots = self.landscape.get_spots_outside_border(previous_spot, spot, next_spot, strict=False)
                else:
                    neighbor_spots = self.landscape.get_spot_neighbors(spot)

                wi = {s: len(drones_per_spot[s]) if s in drones_per_spot else 0 for s in neighbor_spots}
                vi = dict()

                # Each unoccupied spot receives a small value, except at sink
                for s in neighbor_spots:
                    if wi[s]:
                        continue
                    vi[s] = (self.landscape.euclidian_distance(s) * C) / (4 * self.landscape.euclidian_distance(spot))

                # Each occupied spot that goes away from the sink receives a
                # random value in [wi.C + epsilon, (wi+1).C[
                spots_away_to_fill = [s for s in self.landscape.next_spots_away(spot, strict=True)
                                          if s in neighbor_spots and s not in vi]
                for di in free_drones:
                    for s in spots_away_to_fill:
                        vi[s] = random.uniform(wi[s] * C + epsilon, (wi[s] + 1) * C)

                    # The drone can now choose its new spot, if any available
                    di.choose_target_spot(vi)
                    drones_to_move.append(di)

        # We now move the drones to target spots
        for di in drones_to_move:
            current_spot = di.get_spot()
            target_spot = di.get_target_spot()
            if current_spot == target_spot:
                continue

            # Move to target and update list of drones in each spot
            di.move_to_target()
            drones_per_spot[current_spot].remove(di)
            if target_spot in drones_per_spot:
                drones_per_spot[target_spot].append(di)
            else:
                drones_per_spot[target_spot] = [di]

            # We check if a target has been found
            if target_spot in points_of_interest:

                # we found a new target !!!
                points_of_interest.discard(target_spot)
                self.found_points_of_interest[target_spot] = self.get_action_stamp()
                di.set_state(DroneState.IRREMOVABLE, when=self.get_action_stamp())

        return bool(drones_to_move)


    def expansion(self, strict=True, verbose=False, C=None, epsilon=None):
        """
        Expansion phase.

        The drones move away from the sink to detect the points_of_interest.

        """
        self.new_action('expansion')

        C = self.C if C is None else C
        epsilon = self.epsilon if epsilon is None else epsilon

        sink = self.landscape.sink
        points_of_interest = {s for s in self.points_of_interest if s not in self.found_points_of_interest}
        # We force a drone to stay at the sink
        self.drones[0].set_state(DroneState.IRREMOVABLE, when=self.get_action_stamp())

        # We identify all drones located at the same spot
        D = self.get_drones_per_spot()

        self.counter_expansion = 0
        while True:
            self.counter_expansion += 1

            has_moved = self._move_one_round_away(D, points_of_interest, C, epsilon)

            # Check for termination the bad way. The good way is to identify
            # drones at the border, circulate a message, etc.)
            if not has_moved:
                break

        if self.verbose:
            print("Expansion in {} rounds".format(self.counter_expansion))

        # We identify drones at the border
        self._identify_drones_at_the_border(D)




    def spanning(self, verbose=False, first=True, extend_to_border_only=False):
        """
        Spanning phase

        We search for a path from each found target to the sink and a path from
        each found target to the border
        """
        self.new_action('spanning')

        sink = self.landscape.sink

        # mapping from spots to drones. We have at most 1 drone per spot...
        spot_to_drones = self.get_drones_per_spot()
        queue_to_sink = []
        queue_to_border = []
        if extend_to_border_only:
            # We only want to extend paths of irremovable drones to the border
            irr_spot = {di.get_spot(): di for di in self.drones if di.is_irremovable() and not di.get_spot() == sink}
            queue_to_border = [di for spot, di in irr_spot.items()
                                if not di.is_border() and not any(s in irr_spot for s in self.landscape.next_spots_away(spot))]

            # We propagate a message from the sink along irremovable drones to
            # identify "leaves". Then each leaves will grow a path to border



        elif self.found_points_of_interest:
            # We want to find paths from each newly discovered point of interest to
            # the sink and to the border
            # Get the drones located on points_of_interest (are already irremovable)

            drones_on_poi = [spot_to_drones[spot][0] for spot,action_stamp in self.found_points_of_interest.items()
                                 if spot in spot_to_drones and action_stamp[0] == self.get_action_stamp()[0]]
            # for spot_irr in drones_on_poi:
            #     #print(zone.spot)
            #     print( self.landscape.spot_to_coordinate(spot_irr.spot) )
            queue_to_sink = list(drones_on_poi)
            queue_to_border = list(drones_on_poi)

        else:
            # No POI has been found yet.
            # We create a path from the sink to the border
            if len(self.drones) > 7 and (1, 0) in spot_to_drones:
                di = spot_to_drones[(1, 0)][0]
                queue_to_border = [di]
                di.set_state(DroneState.IRREMOVABLE, when=self.get_action_stamp())

        this_round = set()
        while queue_to_sink or queue_to_border:

            # One move to the sink
            if queue_to_sink:
                di = queue_to_sink.pop(0)  # ok, we could do better to manipulate a queue...
                spot = di.get_spot()
                this_round.add(spot)
                spots_to_sink = [s for s in self.landscape.next_spots_to(spot, sink)
                                    if s in spot_to_drones]
                drones_to_sink = [spot_to_drones[spot][0] for spot in spots_to_sink]
                if not spots_to_sink:
                    if first:
                        raise ValueError("somethink goes wrong, unable to reach the sink")

                    # We must follow the internal border
                    # We act as if we reached that internal border for the first time

                    # We first find the first occupied spot in right hand side
                    empty_spot = [s for s in self.landscape.next_spots_to(spot, sink)
                                    if s not in spot_to_drones][0]
                    next_spot = di.next_occupied_right(empty_spot, spot_to_drones)

                    path = [spot, next_spot]
                    if verbose:
                        print(path)
                    while True:
                        current_spot = path[-1]
                        current_drone = spot_to_drones[current_spot][0]
                        previous_spot = path[-2]
                        if verbose:
                            print(current_drone)
                        current_drone.set_state(DroneState.IRREMOVABLE, when=self.get_action_stamp())
                        this_round.add(current_spot)

                        X = [spot_to_drones[s][0] for s in self.landscape.get_spot_neighbors(current_spot)
                                              if s in spot_to_drones and s not in this_round]
                        if any(dj.is_irremovable() and dj.get_when_irremovable() < self.get_action_stamp() for dj in X):
                            # We reach a branch to the sink
                            break

                        next_spot = current_drone.next_occupied_right(previous_spot, spot_to_drones)
                        if next_spot in path:
                            self.show()
                            print("with {} drones".format(len(self.drones)))
                            raise ValueError("ARRRGGGGGHHHHHHLLLLLLLLLLLLLLL")
                        path.append(next_spot)

                elif any(dj.is_irremovable() and dj.get_when_irremovable() <= self.get_action_stamp()
                             for dj in drones_to_sink):
                    # We reached another path to the sink (or the sink itself)
                    pass
                else:
                    # otherwise, we select a drone and set it to irremovable
                    dj = min(drones_to_sink,
                                 key=lambda x: (self.landscape.euclidian_distance(x.get_spot()), x.get_spot()))
                    dj.set_state(DroneState.IRREMOVABLE, when=self.get_action_stamp())
                    queue_to_sink.append(dj)

            # One move to the border
            if queue_to_border:
                di = queue_to_border.pop(0)

                if di.is_border():
                    continue
                this_round.add(di.get_spot())
                spots_to_border = [spot for spot in self.landscape.next_spots_away(di.get_spot(), sink)
                                    if spot in spot_to_drones]
                if not spots_to_border:
                    # We have several cases:
                    # 1) di is on the border. We check if there is at least one drone
                    #    in its neighborhood that is a border or irremovable
                    # 2) di is not on the border but we cannot go further. This is
                    #    due to bad luck or bad choice of parameters. We do nothing
                    #
                    # print(di.id)
                    # print({spot: spot in spot_to_drone for spot in self.landscape.next_spots_away(di.get_spot(), sink)})
                    #
                    if any(spot_to_drones[spot][0].get_state() is not DroneState.FREE
                               for spot in di.get_spot_neighbors()
                               if spot in spot_to_drones):
                        continue
                    if self.visual:
                        self.show()
                    raise ValueError("somethink goes wrong, unable to reach the border from {}".format(di.get_spot()))

                drones_to_border = [spot_to_drones[spot][0] for spot in spots_to_border]
                if any(dj.get_state() is not DroneState.FREE for dj in drones_to_border):
                    # We reached another path to the border or the border itself
                    continue
                # otherwise, we select a drone and set it to irremovable
                dj = max(drones_to_border, key=lambda x: (self.landscape.euclidian_distance(x.get_spot()), x.get_spot()))
                dj.set_state(DroneState.IRREMOVABLE, when=self.get_action_stamp())
                queue_to_border.append(dj)

            if verbose:
                self.show()



    def balancing(self, C=None, epsilon=None):
        """
        Free drones move to positions at the border, i.e., in BORDER state

        WARNING: We assume that we have at most 1 drone per spot
        """
        self.new_action('balancing')

        C = self.C if C is None else C
        epsilon = self.epsilon if epsilon is None else epsilon
        sink = self.landscape.sink

        #
        # We build useful data structures
        #
        drones_per_spot = self.get_drones_per_spot()
        border_spots = set(s for s, _ in self._current_border)
        irr_spots = [di.get_spot() for di in self.drones if di.is_irremovable()]

        #
        # We move all free drones to the border
        #

        self.counter_balancing_1 = 0
        while True:
            self.counter_balancing_1 += 1
            if self.counter_balancing_1 > len(self.drones):
                if self.visual:
                    self.show()
                raise ValueError("we are lost")

            drones_to_move = []
            for spot, drones in drones_per_spot.items():
                if spot in border_spots:
                    continue
                free_drones = [di for di in drones if di.is_free()]
                if not free_drones:
                    continue

                spots_away = [s for s in self.landscape.next_spots_away(spot, strict=True)]

                if spot == sink:
                    for i, di in enumerate(free_drones):
                        di.set_target_spot(spots_away[i % len(spots_away)])

                else:
                    for di in free_drones:
                        di.set_target_spot(random.choice(spots_away))
                        drones_to_move.append(di)

            # Check for termination the bad way. The good way is to identify
            # drones at the border, circulate a message, etc.)
            if not drones_to_move:
                break

            # We now move the drones to target spots and update list of drones
            # in each spot
            for di in drones_to_move:
                current_spot = di.get_spot()
                target_spot = di.get_target_spot()
                if current_spot == target_spot:
                    continue
                di.move_to_target()
                drones_per_spot[current_spot].remove(di)
                if target_spot in drones_per_spot:
                    drones_per_spot[target_spot].append(di)
                else:
                    drones_per_spot[target_spot] = [di]
                if target_spot in self.points_of_interest and target_spot not in self.found_points_of_interest:
                    print("WARNING: we found a new POI !!!\t{}".format(target_spot))
                    # We check if a neighbor spot is irremovable
                    if any(s in irr_spots for s in di.get_spot_neighbors()):
                        di.set_state(DroneState.IRREMOVABLE, when=self.get_action_stamp())
                        self.found_points_of_interest[target_spot] = self.get_action_stamp()
                        print("\twe are lucky, we can monitor it")
                    else:
                        print("\tunfortunately, we skip it :((")

        if self.verbose:
            print("Balancing/1 in {} rounds".format(self.counter_balancing_1))

        #
        # We now balance drones at border
        #

        self.counter_balancing_2 = 0
        while True:
            self.counter_balancing_2 += 1
            drones_to_move = []
            # Each drone check the number of drones in neighbor spots
            for spot, drones in drones_per_spot.items():
                if spot not in border_spots:
                    continue
                spot_occupancy = len(drones)
                if spot_occupancy < 2:
                    continue
                # if this spot has more drones than some neighbor spots, some
                # drones move to neighbors to balance the number of drones per
                # spots
                neighbor_spots = [s for s in self.landscape.get_spot_neighbors(spot) if s in border_spots]
                wi = {s: len(drones_per_spot[s]) if (s in drones_per_spot and drones_per_spot[s]) else 0
                          for s in neighbor_spots}
                good_spots = [s for s in neighbor_spots if wi[s] and wi[s] < spot_occupancy - 1]
                if good_spots:
                    mean_occupancy = float(mean([wi[s] for s in good_spots] + [spot_occupancy]))
                    # A drone moves with a probability proportional to the
                    # number of drones needed on spot s to have overall mean
                    # drones
                    sum_proba = [(spot, mean_occupancy)]
                    for s in good_spots:
                        v = sum_proba[-1][1] + mean_occupancy - wi[s]
                        sum_proba.append((s, v))

                    for di in drones:
                        if not di.is_free():
                            continue
                        p = random.uniform(0, sum_proba[-1][1])
                        for s, v in sum_proba:
                            if p < v:
                                di.set_target_spot(s)
                                drones_to_move.append(di)
                                break

            if not drones_to_move:
                break

            # We now move the drones to target spots and update list of drones
            # in each spot
            for di in drones_to_move:
                current_spot = di.get_spot()
                target_spot = di.get_target_spot()
                if current_spot == target_spot:
                    continue
                di.move_to_target()
                drones_per_spot[current_spot].remove(di)
                drones_per_spot[target_spot].append(di)

        if self.verbose:
            print("Balancing/2 in {} rounds".format(self.counter_balancing_2))



    def expansion_from_border(self, C=None, epsilon=None):
        """
        """
        self.new_action('expansion')

        C = self.C if C is None else C
        epsilon = self.epsilon if epsilon is None else epsilon

        sink = self.landscape.sink
        points_of_interest = {s for s in self.points_of_interest if s not in self.found_points_of_interest}

        # We identify all drones located at the same spot
        D = self.get_drones_per_spot()

        self.counter_expansion = 0
        while True:
            has_moved = self._move_one_round_away(D, points_of_interest, C, epsilon)

            self.counter_expansion += 1
            if self.counter_expansion >= 2 * len(self.drones):
                print("Warning in expansion_from_border: {} rounds ??".format(self.counter_expansion))
            if not has_moved:
                break

        # We identify drones at the border
        self._identify_drones_at_the_border(D)

        if self.verbose:
            print("Expansion from border in {} rounds".format(self.counter_expansion))


    def final_state(self):
        """
        Release useless irremovable drones.
        Move free and border drones to the sink.
        """
        self.new_action('final_state')

        I = dict()
        for di in self.drones:
            if di.is_irremovable():
                I[di.get_spot()] = di
            else:
                di.set_target_spot(self.landscape.sink)
                di.move_to_target()

        sink = self.landscape.sink
        POI = set(self.points_of_interest)
        POI.add(sink)
        # Irremovable drones ending a path to border (so not on a POI) can be freed
        J = [di for si, di in I.items() if (si not in POI and
                                len([s for s in di.get_spot_neighbors() if s in I]) == 1)]
        while J:
            di = J.pop()
            spot = di.get_spot()
            di.set_target_spot(sink)
            di.move_to_target()
            del I[spot]
            for s in self.landscape.get_spot_neighbors(spot):
                if (s in I and s not in POI
                    and len([ss for ss in self.landscape.get_spot_neighbors(s) if ss in I]) == 1):
                    J.append(I[s])
                    break


    def show(self, box=False, talk=True, **args):
        """
        """
        G = Graph()
        state_to_color = {DroneState.FREE: 'blue',
                          DroneState.BORDER: 'green',
                          DroneState.IRREMOVABLE: 'red',
                          DroneState.BORDER_AND_IRREMOVABLE: 'brown',
                          '.': 'white'}
        colors = {col: [] for col in state_to_color.values()}
        colors['purple'] = []
        pos = {}
        for di in self.drones:
            G.add_vertex(di.get_id())
            pos[di.get_id()] = self.landscape.spot_to_coordinate(di.get_spot())
            if di.get_spot() in self.points_of_interest:
                colors['purple'].append(di.get_id())
            else:
                colors[state_to_color[di.get_state()]].append(di.get_id())
        if box:
            # Add spots in the last layer
            for spot in self.landscape.current_layer:
                u = G.add_vertex()
                pos[u] = self.landscape.spot_to_coordinate(spot)
                colors['white'].append(u)
        G.set_pos(pos)
        G.show(vertex_colors=colors, vertex_shape="h", **args)


    def tikz(self, free=False, scale=None, values=None, fast=False, box_ready=True):
        """
        Return the current situation as tikz string.
        """
        color_sink        = ("violet", "violet!40")
        color_poi         = ("black", "black!20")
        color_irremovable = ("red", "red!60")
        color_border      = ("green", "green!20")
        color_free        = ("cyan", "cyan!10")

        scale = self.scale if scale is None else scale
        RL = "\n" if True else ""
        sink = self.landscape.sink
        radius = self.landscape.radius

        if fast:
            hexagone = "circle, minimum size=8mm, inner sep=0pt, outer sep=0pt, draw"
        else:
            hexagone  = "regular polygon, regular polygon sides=6, shape border rotate=30,"
            hexagone += " shape aspect=0.5, inner sep=0pt,outer sep=0pt,"
            hexagone += " minimum width=1cm, minimum height=1cm, draw"

        # Macros
        s = "\\begin{tikzpicture}[transform canvas={scale=" + "{}".format(scale) + "}]"
        s += RL + "\\tikzstyle{hexagone}=[" + hexagone + "];"
        s += RL + "\\tikzstyle{point}=[circle, color=black, minimum size=2pt, inner sep=0pt, outer sep=0pt, draw];"
        s += RL + "\\tikzstyle{value}=[color=black];"
        s += RL + "\\tikzstyle{mystar}=[star,star points=7,star point ratio=.7, color=black, fill=black!80, minimum size=2pt, inner sep=2pt, outer sep=0pt, draw];"

        def hexagone_string(col, id, spot, name='', circle=True, value=None):
            """
            \node[hexagone, color=cyan, anchor=east] (h3) at (h2.corner 4) {};
            \node[ball, label=90:$s_{3}$,scale=.5] (s3) at (h3) {};
            """
            x, y, z = self.landscape.spot_to_coordinate(spot)
            ss = RL + "\\node[hexagone, color={}, fill={}] ({}) at ({}, {})".format(
                    col[0], col[1], id, x/radius, y/radius)
            ss += " {" + "{}".format(name) + "};"
            if circle:
                ss += RL + "\\node[point] (b{}) at ({})".format(id, id) + " {};"
            if value is not None:
                ss += RL + "\\node[value] (v{}) at ({})".format(id, id) + " {" + str(value) + "};"
            return ss

        def target_string(col, id, spot, name):
            """
            """
            x, y, z = self.landscape.spot_to_coordinate(spot)
            ss = RL + "\\node[mystar] ({}) at ({}, {})".format(id, x/radius, y/radius)
            ss += " {" + "{}".format(name) + "};"
            ss += RL + "\\node[value] (t{}) at ({})".format(id, id)
            ss += " {" + "{}".format(name) + "};"
            return ss

        # free, irremovable, border
        for di in self.drones:
            spot = di.get_spot()
            if spot == sink:
                continue

            if di.is_irremovable():
                s += hexagone_string(color_irremovable, "d{}".format(di.id),
                                         spot, circle=spot not in self.points_of_interest)
            elif di.is_border():
                if values:
                    s += hexagone_string(color_border, "d{}".format(di.id), spot, values[spot])
                else:
                    s += hexagone_string(color_border, "d{}".format(di.id), spot)
            elif free and di.is_free():
                s += hexagone_string(color_free, "d{}".format(di.id), spot)


        # Sink
        s += hexagone_string(color_sink, "S", sink, "{\\Large ${\\mathbf{\\mathcal{S}}}$}", circle=False)

        # Points of interest
        for t, spot in enumerate(self.points_of_interest):
            s += target_string(color_poi, "t{}".format(t), spot, "{\\Large $\\mathbf{"+"\\tau_{}".format(t)+"}$}")

        # Add string for bounding box
        if box_ready:
            s += RL + "_BOUNDING_BOX_"

        s += RL + "\\end{tikzpicture}"
        return s

    def tikz_bounding_box_string(self):
        """
        Return the string with the bounding box of current state
        """
        xmin,ymin,xmax,ymax = 0,0,0,0

        def my_update(spot, xmin, ymin, xmax, ymax):
            x, y, z =  self.landscape.spot_to_coordinate(spot)
            xmin = min(x, xmin)
            ymin = min(y, ymin)
            xmax = max(x, xmax)
            ymax = max(y, ymax)
            return xmin, ymin, xmax, ymax

        for di in self.drones:
            xmin, ymin, xmax, ymax = my_update(di.get_spot(), xmin, ymin, xmax, ymax)
        for s in self.points_of_interest:
            xmin, ymin, xmax, ymax = my_update(s, xmin, ymin, xmax, ymax)


        radius = self.landscape.radius
        x_offset = self.landscape.x_offset
        y_offset = self.landscape.y_offset
        s  = "\\draw ({}, {})".format(float((xmin - x_offset) / radius), float((ymin - y_offset) / radius))
        s += " node[anchor=south, color=black!1] {.};"
        s += "\\draw ({}, {})".format(float((xmax + x_offset) / radius), float((ymax + y_offset) / radius))
        s += " node[anchor=south, color=black!1] {.};"

        self._bounding_box = [float((xmin - x_offset) / radius), float((ymin - y_offset) / radius),
                              float((xmax + x_offset) / radius), float((ymax + y_offset) / radius)]

        return s


    def play(self, filename=None, scale=None, columns=3, fast=False, box_ready=True):
        """
        # drones = 1, 7, 19, 37, 61, 91, 127, 169, 217, 271, 331, 397, 469, 547,
        #          631, 71, 817, 919, 1027, 1141, 1261,..

        poi = {(-9, 0), (-9, 2), (-8, 10), (-4, -4), (1, 8), (3, -7), (3, 4), (8, 2)}
        E = ExSpanBal(469, 8, C=10, epsilon=.2, radius=30, points_of_interest=poi)
        %time E.play("/Users/dcoudert/Recherche/Drones/Code/current.tex")

        poi = set([(-3, 6), (-4, -3), (-14, 4), (6, 0), (-9, 9), (4, -6), (8, -4), (1, -3), (-6, 6), (13, 0)])
        E = ExSpanBal(217, C=100, epsilon=2, radius=30, points_of_interest=poi)
        %time E.play("/Users/dcoudert/Recherche/Drones/Code/current.tex", fast=False)

        sage: poi = [(4, 0), (2, 4), (-2, 4), (-6, 4), (-4, 0), (-2, -4), (2, -4), (6, -4)]
        sage: E = ExSpanBal(60, C=100, epsilon=2, radius=30, points_of_interest=poi)
        sage: %time E.play("/Users/dcoudert/Recherche/Drones/Code/current.tex", fast=False)
        """
        T = [self.tikz(free=True, scale=123456789, fast=fast, box_ready=box_ready)]
        self.expansion()
        T.append(self.tikz(free=True, scale=123456789, fast=fast, box_ready=box_ready))
        self.spanning(first=True)
        T.append(self.tikz(free=True, scale=123456789, fast=fast, box_ready=box_ready))
        self.balancing()
        D = self.get_drones_per_spot()
        T.append(self.tikz(free=False, scale=123456789, fast=fast, box_ready=box_ready,
                       values={s: len(l) if any(di.is_border() for di in l) else None
                                       for s,l in D.items()}))
        self.expansion_from_border()
        T.append(self.tikz(free=True, scale=123456789, fast=fast, box_ready=box_ready))
        self.spanning(first=False)
        T.append(self.tikz(free=True, scale=123456789, fast=fast, box_ready=box_ready))
        self.identify_new_border(check_false_positive=True)
        T.append(self.tikz(free=True, scale=123456789, fast=fast, box_ready=box_ready))
        # extend irremovable to new border
        self.spanning(extend_to_border_only=True)
        T.append(self.tikz(free=True, scale=123456789, fast=fast, box_ready=box_ready))
        # Restart from balancing
        self.balancing()
        T.append(self.tikz(free=False, scale=123456789, fast=fast, box_ready=box_ready))
        self.expansion_from_border()
        self.spanning(first=False)
        self.identify_new_border(check_false_positive=True)
        self.spanning(extend_to_border_only=True)
        T.append(self.tikz(free=True, scale=123456789, fast=fast, box_ready=box_ready))
        i = 0
        while any(di.is_free() for di in self.drones):
            i += 1
            if self.verbose:
                print("another round.. {}".format(i))
            self.balancing()
            self.expansion_from_border()
            self.spanning(first=False)
            self.identify_new_border(check_false_positive=True)
            self.spanning(extend_to_border_only=True)
        if self.verbose:
            print("So {} more rounds".format(i))
        T.append(self.tikz(free=True, scale=123456789, fast=fast, box_ready=box_ready))

        if box_ready:
            bounding_box = self.tikz_bounding_box_string()

        # To show final state
        self.final_state()
        T.append(self.tikz(free=False, scale=123456789, fast=fast, box_ready=box_ready))

        if box_ready:
            for i, t in enumerate(T):
                T[i] = t.replace("_BOUNDING_BOX_", bounding_box)


        if scale is None:
            xmin,ymin,xmax,ymax = self._bounding_box
            width = xmax - xmin
            true_width = float((21 - (columns - 1) - 4) / columns)
            scale = float( true_width / width)
        else:
            true_width = 7

        my_scale = "scale={}".format(.25 if scale is None else scale)
        #for i, t in enumerate(T):
        #    T[i] = t.replace("scale=123456789",  my_scale)

        if filename is None:
            for t in T:
                print(t)
        else:
            s = "\\begin{tikzpicture}\n\\node(FIG0) {\n" + T[0].replace("scale=123456789",  my_scale) + "\n};\n"
            for i, t in enumerate(T):
                if i == 0:
                    continue
                if i % 3 == 0:
                    s += "\\node[below = {}cm of FIG{}] (FIG{})".format(true_width + 1, i-3, i)
                else:
                    s += "\\node[right = {}cm of FIG{}] (FIG{})".format(true_width + 1, i-1, i)
                s += " {\n" + t.replace("scale=123456789", my_scale) + "\n};\n"
            s += "\\end{tikzpicture}\n"
            with open(filename, 'w') as f:
                f.write(s)

            for i,t in enumerate(T):
                with open(filename.replace(".tex", "-{}.tikz".format(i)), 'w') as f:
                    s = t.replace("scale=123456789", "scale=1")
                    s = s.replace("[transform canvas={scale=1}]", "")
                    f.write(s)


