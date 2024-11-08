"""
Class LandScape
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
from sage.graphs.graph import Graph
from sage.functions.other import sqrt
import math
class LandScape:
    """
    Class to interact with the landscape.

    The initial spot (0, 0) is at position (0, 0, 1).
    Then new spot are added in layers such that spot (x, y) is surrounded by
    spots (x + 1, y), (x, y + 1), (x - 1, y + 1), (x - 1, y), (x, y - 1),
    (x + 1, y - 1).

    New layers are added on demand.

    Given a spot (x, y), we can get its coordinates in the 3D space.
    """

    def __init__(self, radius=10, layers=1):
        """
        Initialize the landscape with a central spot and ``layers`` layers of
        spots around the central spot.
        """
        self.graph = Graph()
        self.sink = (0, 0)
        self.graph.add_vertex(self.sink)
        self.x_sink = 0
        self.y_sink = 0
        self.z_sink = 1
        self.x_offset = radius *  math.cos(math.pi/6) / 2
        self.y_offset = radius * math.sin(math.pi/6) / 2
        self.pos3d = {self.sink: (0, 0, 1)}
        self.graph.set_pos(self.pos3d)
        self.current_layer = set([self.sink])
        self.radius = radius
        for _ in range(layers):
            self.add_layer()
        self.distance_to_sink = {}  # Cache distances to sink

    def get_spot_neighbors(self, spot):
        """
        Return the 6 neighbors in the hexagonal grid of spot u.

               (x, y - 1)    (x + 1, y - 1)
        (x - 1, y)      (x, y)      (x + 1, y)
            (x - 1, y + 1)   (x, y + 1)
        """
        idx, idy = spot
        return [(idx + 1, idy), (idx, idy + 1), (idx - 1, idy + 1),
                 (idx - 1, idy), (idx, idy - 1), (idx + 1, idy - 1)]

    def get_spots_outside_border(self, previous_spot, spot, next_spot, strict=True):
        """
        Return the list of neighbor spots of spot between previous_spot and
        next_spot in clockwise order.
        """
        spot_neighbors = self.get_spot_neighbors(spot)
        L = [] if strict else [previous_spot]
        i = spot_neighbors.index(previous_spot) + 1
        while i < 6 and spot_neighbors[i] != next_spot:
            L.append(spot_neighbors[i])
            i += 1
        if i == 6:
            i = 0
            while spot_neighbors[i] != next_spot:
                L.append(spot_neighbors[i])
                i += 1
        if not strict:
            L.append(next_spot)
        return L

    
    def spot_to_coordinate(self, spot):
        """
        Return the 3D coordinates of the spot.

        See https://stackoverflow.com/questions/20734438/algorithm-to-generate-a-hexagonal-grid-with-coordinate-system
        """
        if spot in self.pos3d:
            return self.pos3d[spot]
        x, y = spot
        return (float(self.x_sink + self.x_offset * (x * 2 + y)),
                float(self.y_sink - self.y_offset * y * 3),
                1)

    def euclidian_distance(self, s, t=None):
        """
        Return the euclidian distance between spots ``s`` and ``t``.
        """
        if t is None or t == self.sink:
            if s not in self.distance_to_sink:
                cs = self.spot_to_coordinate(s)
                self.distance_to_sink[s] = sqrt(cs[0]**2 + cs[1]**2)
            return self.distance_to_sink[s]
        while s not in self.graph:
            self.add_layer()
        while t not in self.graph:
            self.add_layer()
        cs = self.spot_to_coordinate(s)
        ct = self.spot_to_coordinate(t)
        return sqrt((cs[0] - ct[0])**2 + (cs[1] - ct[1])**2)

    def graph_distance(self, s, t):
        """
        Return the shortest path distance in the graph from s to t
        """
        if s not in self.graph:
            self.add_layer()
        if t not in self.graph:
            self.add_layer()
        return self.graph.distance(s, t)

    def add_layer(self):
        """
        Add a layer of spots.
        """
        pos3d = self.graph.get_pos(dim=3)
        new_layer = set()
        for spot in self.current_layer:
            S = self.get_spot_neighbors(spot)
            Snew = [v for v in S if not self.graph.has_vertex(v)]
            new_layer.update(Snew)
            self.graph.add_vertices(Snew)
            self.graph.add_cycle(S)
            for v in Snew:
                self.graph.add_edge(spot, v)
            self.pos3d.update({v: self.spot_to_coordinate(v) for v in Snew})
        self.current_layer = new_layer

    def show(self, **args):
        self.graph.show(args)
        

    def random_spot(self):
        """
        Return a random spot in the existing spots
        """
        return self.graph.random_vertex()


    def next_spots_away(self, s, t=None, strict=True):
        """
        Return the neighboring spots of s that are farther of t than s.

        When strict is True, we return spots that are stricly farther that
        s. Otherwise, we return spots at distance at least the distance of s
        from t.

        if t in None, we use the sink.
        """
        if t is None:
            t = self.sink
        dist_st = self.euclidian_distance(s, t)
        if strict:
            return [ss for ss in self.get_spot_neighbors(s) if self.euclidian_distance(ss, t) > dist_st]
        else:
            return [ss for ss in self.get_spot_neighbors(s) if self.euclidian_distance(ss, t) >= dist_st]

    def next_spots_to(self, s, t, strict=True):
        """
        Return the neighboring spots of s that are on a shortest path to t.
        """
        dist_st = self.euclidian_distance(s, t)
        if strict:
            return [ss for ss in self.get_spot_neighbors(s) if self.euclidian_distance(ss, t) < dist_st]
        else:
            return [ss for ss in self.get_spot_neighbors(s) if self.euclidian_distance(ss, t) <= dist_st]


    def steiner_tree(self, poi, verbose=False):
        """
        Return a Steiner tree with targets and sink
        """
        while any(s not in self.graph for s in poi):
            self.add_layer()
        g = self.graph
        num_poi = len(poi)
            
        from sage.numerical.mip import MixedIntegerLinearProgram
        p = MixedIntegerLinearProgram(maximization=False)

        edges = p.new_variable(binary=True)

        p.set_objective(p.sum(edges[frozenset(e)] for e in g.edge_iterator(labels=False)))
        
        # Flow from each poi to the sink
        flow = p.new_variable(nonnegative=True)
        for s in poi:
            for u in g:
                if u == s:
                    val = -1
                elif u == (0, 0):
                    val = 1
                else:
                    val = 0
                p.add_constraint(p.sum(flow[s,v,u] for v in g.neighbor_iterator(u))
                                     - p.sum(flow[s,u,v] for v in g.neighbor_iterator(u)) == val)

        for u,v in g.edge_iterator(labels=False):
            p.add_constraint(p.sum(flow[s,u,v] + flow[s,v,u] for s in poi) <= num_poi * edges[frozenset([u, v])])

        p.solve(log=verbose)

        edges = p.get_values(edges)

        st =  g.subgraph(edges=[e for e in g.edge_iterator(labels=False) if edges[frozenset(e)] == 1],
                         immutable=False)
        st.delete_vertices(v for v in g if not st.degree(v))

        return st
