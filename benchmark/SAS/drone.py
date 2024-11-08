"""
Class Drone

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
# from sage.misc.prandom import choice, uniform
from operator import itemgetter
import random
from enum import Enum
class DroneState(Enum):
    """
    Enum class defining the possible states of a drone.

    Possible states are:
    - ``IRREMOVABLE`` -- the drone is part of the Steiner tree connecting
      discovered targets to the sink.

    - ``BORDER`` -- drones in the border are the ones most outside the covered
      area, representing the edge of the expansion zone, according to the sink

    - ``FREE`` -- the drone can move freely. It is neither in the border nor
      irremovable.

    See: https://docs.python.org/3/library/enum.html
    """
    FREE = 0
    BORDER = 1
    IRREMOVABLE = 2
    BORDER_AND_IRREMOVABLE = 3


class Drone:
    """
    Class defining a drone.

    TODO:
    - interact with landscape
    - record previous / next on border to ease second expansion
    - expansion 1 & 2
    - balancing
    - spanning
    """
    def __init__(self, id, spot=None, state=None, communication_radius=20, C=100, epsilon=2, landscape=None):
        """
        Initialization of the drone.

        INPUT:

        - ``id`` -- identifier of this drone

        - ``spot`` -- tuple (default: ``None``); current spot of the drone. By
          default (``None``), the drone is placed at spot (0, 0).

        - ``state`` -- DroneState (default: ``None``); state of the drone,
          ``DroneState.FREE`` by default

        - ``communication_radius`` -- int (default: 20); radius of the ball in
          which the drone is able to communicate

        - ``C`` -- a constant (default: 100);

        - ``epsilon`` -- a constant (default: 2); a small value preventing
          overlaps in random choices
        """
        self.id = id
        self.path = []  # to record all spots of the drone
        self.set_spot(spot)
        self.set_target_spot(spot)
        self.set_state(state)
        self.communication_radius = communication_radius
        self.landscape = landscape


    def __str__(self):
        return "Drone {} at spot {} with state {}".format(self.id, self.spot, self.get_state())

    def __repr__(self):
        return str(self)

    def reset(self):
        """
        Reset the spot and state of the drone
        """
        self.set_target((0, 0))
        self.move_to_target()
        self.set_state(DroneState.FREE)
        self.path = []
    
    def get_id(self):
        return self.id

    def set_spot(self, spot=None):
        """
        Set the spot of the drone.

        INPUT:

        - ``spot`` -- tuple (default: ``None``); current spot of the drone. By
          default (``None``), the drone is placed at spot (0, 0).
        """
        self.spot = (0, 0) if spot is None else spot
        self.path.append(self.spot)

    def get_spot(self):
        """
        Return the current spot of the drone.
        """
        return self.spot

    def get_spot_neighbors(self):
        """
        Return the 6 neighbors in the hexagonal grid of this spot.

               (x, y - 1)    (x + 1, y - 1)
        (x - 1, y)      (x, y)      (x + 1, y)
            (x - 1, y + 1)   (x, y + 1)
        """
        idx, idy = self.spot
        return [(idx + 1, idy), (idx, idy + 1), (idx - 1, idy + 1),
                 (idx - 1, idy), (idx, idy - 1), (idx + 1, idy - 1)]


    def next_occupied_right(self, previous_spot, drones_per_spot):
        """
        Return the first occupied spot after previous_spot in right hand side.

        INPUT::

        - ``previous_spot`` -- previous spot

        - ``drones_per_spot`` -- dictionary keyed by spots with list of drones
          on that spot
        """
        current_spot = self.spot
        spot_neighbors = self.get_spot_neighbors()
        occupied = [s in drones_per_spot and drones_per_spot[s] for s in spot_neighbors]
        # We get the relative position of the previous spot wrt current spot
        i = spot_neighbors.index(previous_spot)
        # We search for the next occupied spot
        j = i + 1
        while j < 6 and not occupied[j]:
            j += 1
        if j == 6:
            j = 0
            while not occupied[j]:
                j += 1
        return spot_neighbors[j]


    def choose_target_spot(self, vi, by_min=True):
        """
        Choose a new spot with smallest value and record it as target.

        INPUT:

        - ``vi`` -- dictionary keyed by spots associating each spot a value. If
          the dict is empty or ``None``, do nothing.

        - ``by_min`` -- boolean (default: ``True``); whether to choose the spot
          with minimum value or to chose a spot with a probability proportional
          to its value
        """
        if vi:
            if by_min:
                # self.target_spot = min(vi.items(), key=itemgetter(1))[0]
                min_val = min(vi.values())
                candidates = [s for s, v in vi.items() if v == min_val]
                self.target_spot = random.choice(candidates)

            else:
                tot = sum(vi.values())
                p = random.uniform(0, tot)
                s = 0
                for spot, val in vi.items():
                    if s + val >= p:
                        self.set_target_spot(spot)
                        break
                    else:
                        s += val

    def set_target_spot(self, spot):
        self.target_spot = spot

    def get_target_spot(self):
        return self.target_spot

    def move_to_target(self):
        """
        Move the drone to its target
        """
        if self.landscape is not None:
            # inform the landscape that this drone has moved
            pass
        self.set_spot(self.target_spot)


    def set_state(self, state=None, when=None):
        """
        Set the state of this drone

        INPUT:

        - ``state`` -- DroneState (default: ``None``); state of the drone,
          ``DroneState.FREE`` by default

        - ``when`` -- integer (default: ``None``); time stamp at which the
          current status has been set. This is especially important for
          irremovable state as this value is used in spanning phase.

        Possible states are: FREE, BORDER, IRREMOVABLE, and BORDER & IRREMOVABLE
        """
        if state is None or state is DroneState.FREE:
            self._border = False
            self._irremovable = False
        elif state is DroneState.BORDER:
            self.set_border(on=True, when=when)
        elif state is DroneState.IRREMOVABLE:
            self.set_irremovable(when=when)
        elif state is DroneState.BORDER_AND_IRREMOVABLE:
            self.set_border(on=True, when=when)
            self.set_irremovable(when=when)
        else:
            raise ValueError("unknown drone state")

    def get_state(self):
        """
        Return current state of this drone.
        """
        if self.is_irremovable():
            if self.is_border():
                return DroneState.BORDER_AND_IRREMOVABLE
            return DroneState.IRREMOVABLE
        if self.is_border():
            return DroneState.BORDER
        return DroneState.FREE

    def set_border(self, on=True, when=None):
        """
        Set/unset the drone as BORDER
        """
        self._border = on
        self._when_border = when

    def set_irremovable(self, when):
        """
        """
        if when is None:
            raise ValueError("a time stamp is needed when setting to irremovable")
        self._irremovable = True
        self._when_irremovable = when

    def set_free(self):
        self._irremovable = False
        self._border = False

    def is_free(self):
        return not (self._border or self._irremovable)
    
    def is_border(self):
        return self._border

    def get_when_border(self):
        if self.is_border():
            return self._when_border
        return None

    def is_irremovable(self):
        return self._irremovable

    def get_when_irremovable(self):
        if self.is_irremovable():
            return self._when_irremovable
        return None

