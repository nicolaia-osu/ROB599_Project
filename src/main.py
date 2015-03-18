"""
Description:
    Driver for our SDM adaptive view planning project.

Author: Kory Kraft
Date: 3-12-15
"""

import os
import os.path
import time
import sys
import pickle
import random
import math

import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import numpy as np

"""
"view_state_definitions.csv":

View State 1, X Degrees, Y Degrees
View State 2, X Degrees, Y Degrees
...

"region_mapping.csv":

True Knife Region 1, View State 1 Image Region, ... View State 9 Image Region
True Knife Region 2, View State 1 Image Region, ... View State 9 Image Region
...

"view_#_results.csv":

Image Region 1, Defect [y/n] (0/1)
Image Region 2, Defect [y/n] (0/1)

"view_#_confidence.csv":

Image Region 1, Confidence Level [0 - 1]
Image Region 2, Confidence Level [0 - 1]
"""

def load_view_state_definitions(fname):
    """ Returns dictionary of tuples.

            File in format:
                View State 1, X Degrees, Y Degrees
                View State 2, X Degrees, Y Degrees

            List of tuples in form:
                [1:(X degrees, y degrees),...2:(xdegrees, ydegrees)]

                To get tuple for state 1, you would use key 1.
    """
    #print "Data Source: ", fname
    #print "Loading view state definitions..."

    view_state_defs = {}

    with open(fname, "r") as f:
        for line in f:
            if line.startswith("#"):
                continue
            else:
                split_line = line.split(",")

                state = int(split_line[0])
                xDegrees = float(split_line[1])
                yDegrees = float(split_line[2])

                view_state_defs[state] = (xDegrees, yDegrees)

    return view_state_defs

def load_view_state_region_mappings(fname):
    """ Returns dictionary of "region_tuples".

        Key in region_mappings list correspond to view angle/state.

        Index into region_tuple correspond to:
            0 = True region
            1 = mapping at view from 1
            2 = mapping at view from 2
            3 = mapping at view from 3
            ...

        e.g
            # yields a tuple of mappings for the second viewing angle/state
            region_tuple_two = region_mappings[2]
            # true knife region for second viewing angle/state
            region_tuple_two[0]
            # mapping for second viewing angle/state region
            region_tuple_two[3]
    """
    #print "Data Source: ", fname
    #print "Loading view region mappings..."

    region_mappings = {}
    with open(fname, "r") as f:
        for line in f:
            if line.startswith("#"):
                # comment line
                continue
            else:
                split_line = map(lambda x: int(x), line.split(","))
                region_mappings[split_line[0]] = \
                   dict((view_state, view_region)
                        for view_state, view_region in
                        enumerate(split_line[1:],1))

    return region_mappings

def load_state_view_results(fname):
    """ Returns dictionary bools

            File name expected to have name:
            view_*_results.csv : where the wildcard represents
                                      the given view number

            File in format:
                View Region 1, Defect [y/n]
                View Region 2, Defect [y/n]

                To get classifier for region 1, you would use key 1.
    """
    #print "Data Source: ", fname
    #print "Loading view results..."

    view_results = {}

    with open(fname, "r") as f:
        for line in f:
            if line.startswith("#"):
                continue
            else:
                split_line = line.split(",")

                region = int(split_line[0])
                classifier = bool(int(split_line[1]))

                view_results[region] = classifier

    return view_results

def load_state_view_confidence(fname):
    """ Returns dictionary of floats

            File name expected to have name:
            view_*_results.csv : where the wildcard represents
                                      the given view number

            File in format:
                View Region 1, Confidence Level [0 - 1]
                View Region 2, Confidence Level [0 - 1]

                To get confidence for region 1, you would use key 1.
    """
    #print "Data Source: ", fname
    #print "Loading view confidence..."

    view_confidence = {}

    with open(fname, "r") as f:
        for line in f:
            if line.startswith("#"):
                continue
            else:
                split_line = line.split(",")

                region = int(split_line[0])
                confidence = float(split_line[1])

                view_confidence[region] = confidence

    return view_confidence

class State:
    """
    List of states.

    Associated region int / angle

    1,30,90
    2,60,90
    3,90,90
    4,120,90
    5,150,90
    6,90,30
    7,90,60
    8,90,120
    9,90,150
    """
    X_30 = 1
    X_60 = 2
    X_90 = 3
    X_120 = 4
    X_150 = 5
    Y_30 = 6
    Y_60 = 7
    Y_120 = 8
    Y_150 = 9
    NOSTATE = 10

    STATES_LIST = [x for x in xrange(1,10)]
    XYZ = {}
    XYZ[1] = (8,  10,  0.5)
    XYZ[2] = (8,  10,  0.71)
    XYZ[3] = (4,  10, 1)
    XYZ[4] = (0,  10,  0.5)
    XYZ[5] = (0,  10,  0.71)
    XYZ[6] = (4,  0,  0.5)
    XYZ[7] = (4,  0,  0.71)
    XYZ[8] = (4,  20,  0.5)
    XYZ[9] = (4,  20,  0.71)
                                

class GreedyAlgorithm(object):

    def __init__(self,
                 mappings_fname,
                 view_region_fname,
                 state_view_results_dir,
                 state_view_confidence_dir):

        self._current_state = State.NOSTATE

        # load view states
        self._view_states = \
            load_view_state_definitions(view_region_fname)
        self._unvisited_view_states = sorted(self._view_states.keys())
        self._visited_view_states = []

        # make sure states are unique
        assert len(set(self._unvisited_view_states)) == \
            len(self._unvisited_view_states)
        # load mapping from view state to image view regions
        # and what knife regions they map to
        # dict[knife_region][view_state] -> view_region
        self._view_state_region_mappings = \
            load_view_state_region_mappings(mappings_fname)
        self._knife_regions = sorted(self._view_state_region_mappings.keys())
        assert all(sorted(subdict.keys()) == self._unvisited_view_states
                   for subdict in self._view_state_region_mappings.values())

        # maps state to a list of view regions
        self._state_view_regions = {}
        # maps a view state to the results for each image
        # region associated with it
        self._view_results = {}
        # maps a view state to the confidence it places on
        # each image view region
        self._view_confidence = {}
        for state in self._view_states:

            self._view_results[state] = \
                load_state_view_results(join(state_view_results_dir,
                                             "view_"+str(state)+"_results.csv"))
            self._view_confidence[state] = \
               load_state_view_confidence(join(state_view_confidence_dir,
                                               "view_"+str(state)+"_confidence.csv"))
            assert sorted(self._view_results[state].keys()) == sorted(self._view_confidence[state].keys())

            self._state_view_regions[state] = sorted(self._view_results.keys())

        # initialize probability distribution
        self._knife_region_probabilities = \
            dict((knife_region, 0.5) for knife_region in self._knife_regions)

    def get_entropy(self, P):
        """ Gabe to implement.

        Gets entropy for a given state.
        """
        entropy = 0.0
        for knife_region in self._knife_regions:
            p = P[knife_region]
            entropy -= p * math.log(p, 2)
            entropy -= (1-p) * math.log(1-p, 2)
        return entropy

    def getXY(self, state):
        """ Translates a state to an x,y coordinate value.
            Returns (x,y) tuple.
        """
        if state == State.X_30:
           return (-2, 0)
        elif state == State.X_60:
            return (-1, 0)
        elif state == State.X_90:
            return (0, 0)
        elif state == State.X_120:
            return (1, 0)
        elif state == State.X_150:
            return (2, 0)
        elif state == State.Y_30:
            return (0, -2)
        elif state == State.Y_60:
            return (0, -1)
        elif state == State.Y_120:
            return (0, 1)
        elif state == State.Y_150:
            return (0, 2)
        else:
            print "Error! Don't recognize state"
            sys.exit(-1)

    def update_current_state(self, next_state):
        assert next_state is not None
        assert next_state in self._view_states
        self._visited_view_states.append(next_state)
        self._unvisited_view_states.remove(next_state)
        self._current_state = next_state

    def get_state_value(self, next_state):
        """ Returns value of state
            entropy over state
        """
        current_entropy = self.get_entropy(self._knife_region_probabilities)

        # upate prob
        new_probabilities = self.update_prob_dist(next_state)
        new_entropy = self.get_entropy(new_probabilities)
        information_gain = current_entropy - new_entropy

        movement_cost = self.calc_movement_cost(self._current_state, next_state)

        return (information_gain, movement_cost)

    def manhattan_distance(self, location_a, location_b):
            """ returns manhattan_distance of (x,y) to (x2,y2)

                Does not take into account heading...
            """
            return abs(location_a[0] - location_b[0]) + \
                   abs(location_a[1] - location_b[1])

    def get_possible_states(self, accum_cost, budget):
        """ Returns a list of the states you can visit given the current
            state and current cost that fits within the overall budget.
        """

        remaining_budget = budget - accum_cost

        possible_states = []

        for state in self._unvisited_view_states:
            movement_cost = self.calc_movement_cost(self._current_state, state)
            if movement_cost < remaining_budget:
                possible_states.append(state)

        return possible_states

    def calc_movement_cost(self, state_x, state_y):
        """ Returns cost from one state to another.

            Base cost for any move is 100 (1/10 of second).
        """
        #print "Calculating cost..."

        base_cost = 100
        # cost to go from one view angle to other (motor time)
        angle_to_angle_cost = 200
        if (state_x == State.NOSTATE) or \
           (state_y == State.NOSTATE):
            return 0.0

        if state_x == state_y:
            return base_cost
        else:
            x1,y1 = self.getXY(state_x)
            x2,y2 = self.getXY(state_y)
            dist = self.manhattan_distance([x1, y1], [x2, y2])
            return base_cost + dist * angle_to_angle_cost

    def get_highest_value_move(self, states):
        max_state = None
        max_value = float('-inf')

        for next_state in states:
            entropy_gain, cost = self.get_state_value(next_state)
            if entropy_gain / cost > max_value:
                max_state = next_state
                max_value = (entropy_gain, cost)

        return (max_state, max_value)

    def update_prob_dist(self, next_state):
        """ Gabe to implement
        """
        new_probabilities = {}
        confidence = self._view_confidence[next_state]
        results = self._view_results[next_state]

        for knife_region in self._knife_regions:

            current_probability = self._knife_region_probabilities[knife_region]

            image_region  = self._view_state_region_mappings[knife_region][next_state]

            region_result = 0.8 if (results[image_region]) else 0.2
            region_confidence = confidence[image_region]

            updated_probability = region_result * current_probability
            updated_probability /= region_result * current_probability + \
                                   (1 - region_result) * (1 - current_probability)

            new_probabilities[knife_region] = \
                region_confidence * updated_probability + \
                (1 - region_confidence) * current_probability

        return new_probabilities

    def select_next_view(self, accum_cost, budget):
        """ Gets the next view from cur state (view) using 1-step lookahead

            Returns next_state, value
        """
        # select the next view based on just a greedy- 1 step lookahead
        #print "Selecting next state//view..."
        next_state = None
        next_state_val = None

        # get the possible states you can go to that fits within the budget
        possible_states = self.get_possible_states(accum_cost, budget)

        if len(possible_states) != 0:
            # get highest value move
            next_state, next_state_val = \
                self.get_highest_value_move(possible_states)

        return next_state, next_state_val

    def select_next_random_view(self, accum_cost, budget):
        """ Gets the next view from cur state (view) randomly

            Returns next_state, value
        """
        next_state = None
        next_state_val = None

        # get the possible states you can go to that fits within the budget
        possible_states = self.get_possible_states(accum_cost, budget)

        if len(possible_states) != 0:

            next_state = random.choice(possible_states)
            next_state_val = self.get_state_value(next_state)

        return next_state, next_state_val

    def get_views(self, init_state, budget, do_random=False):
        """ Decides the states to view to categorize as an object in an
            adaptive manner.

            Returns a list of (states, entropy, time) that were visited as
            things executed.
        """
        #print "Planning views..."

        # states are view angles/locations
        path = []
        # the accumulated costs of going from one view to another in path
        # relates to time in this instance
        accum_cost = 0.0
        # the overall certainty we have about object at given point
        #total_certainty = 0.0
        # optional, the confidence required to return answer
        #certainty_threshold = 1.0
        #total_reward = 0.0
        decided = False

        path.append((State.NOSTATE,
                     self._knife_region_probabilities,
                     self.get_entropy(self._knife_region_probabilities),
                     accum_cost,
                    (0.0,0.0)))
        # The start state requires no budget cost
        # from the initiali NOSTATE, where the initial probability
        # distributions are recorded
        init_state_val = self.get_state_value(init_state)
        self.update_current_state(init_state)
        self._knife_region_probabilities = \
                self.update_prob_dist(self._current_state)
        path.append((self._current_state,
                     self._knife_region_probabilities,
                     self.get_entropy(self._knife_region_probabilities),
                     accum_cost,
                     init_state_val))
        print init_state_val
        while (not decided):
            # given set of views from current state
            # perform 1 step look ahead w/ prob update

            if not do_random:
                # pick next state/view and get the val (eq, info gain...not to
                # be confused with cost
                new_state, new_state_val = \
                   self.select_next_view(accum_cost, budget)
            else:
                # pick next state/view and get the val (eq, info gain...not to
                # be confused with cost
                new_state, new_state_val = \
                   self.select_next_random_view(accum_cost, budget)

            if new_state is None:
                print "No states left within budget!"
                break

            # calculate movement (view-to-view) cost
            move_cost = self.calc_movement_cost(self._current_state, new_state)

            assert (accum_cost + move_cost) <= budget

            accum_cost += move_cost

            # if total_certainty > certainty_threshold:
            #     # we are confident enough to make a decision at this point
            #     decided = True
            #     break

            self.update_current_state(new_state)
            assert self._current_state == new_state
            # update probability distribution
            self._knife_region_probabilities = \
                self.update_prob_dist(self._current_state)

            # update state and add into path
            path.append((self._current_state,
                         self._knife_region_probabilities,
                         self.get_entropy(self._knife_region_probabilities),
                         accum_cost,
                         new_state_val))

            # update reward
            # total_reward += new_state_val

        return path

    def execute_planning(self,
                         budget,
                         do_random=False):

        # execute path planning algorithm
        init_state = State.X_90
        return self.get_views(init_state, budget, do_random=do_random)

if __name__ == "__main__":
    import os
    from os.path import join

    datadir = "data"
    resultsdir = "results"
    if not os.path.exists(resultsdir):
        os.mkdir(resultsdir)
    mappings_fname = join(datadir, "region_mapping.csv")
    view_state_fname = join(datadir, "view_state_definitions.csv")
    view_confidence_dir = datadir

    #num_trials = 2
    num_trials = 1

    # requirements
    budget = 4 * 1000 # 4 seconds..
    #budget = float('inf')

    results_to_plot = {}
    num_random = 1000

    savefig_settings = {
        'format':'pdf',
        'bbox_inches':'tight',
        'dpi':100}

    for trial in xrange(1, num_trials + 1):

        view_results_dir = os.path.join(datadir, "trial_{0}".format(trial))
        assert os.path.exists(view_results_dir)

        alg_results_dir = join(resultsdir, "trial_{0}".format(trial))
        if not os.path.exists(alg_results_dir):
            os.mkdir(alg_results_dir)

        alg = GreedyAlgorithm(mappings_fname,
                              view_state_fname,
                              view_results_dir,
                              view_confidence_dir)

        # execute greedy planning
        start = time.clock()
        res = alg.execute_planning(budget, do_random=False)
        stop = time.clock()
        print "Time elapsed for Greedy (secs): ", stop - start
        # write results pickle to file
        with open(join(alg_results_dir, "greedy.pickle"), "w") as f:
            pickle.dump(res, f)
        results_to_plot[trial,'greedy'] = res

        results_to_plot[trial,'random'] = []
        for i in xrange(num_random):
            alg = GreedyAlgorithm(mappings_fname,
                                  view_state_fname,
                                  view_results_dir,
                                  view_confidence_dir)
            # execute random planning
            start = time.clock()
            res = alg.execute_planning(budget, do_random=True)
            stop = time.clock()
            print "Time elapsed for Random (secs): ", stop - start
            # write results pickle to file
            with open(join(alg_results_dir, "random.pickle"), "w") as f:
                pickle.dump(res, f)
            results_to_plot[trial,'random'].append(res)

    for trial in xrange(1, num_trials + 1):

        alg_results_dir = join(resultsdir, "trial_{0}".format(trial))

        pp = PdfPages(join(alg_results_dir, "plots.pdf"))
        greedy = results_to_plot[trial,'greedy']
        rand = results_to_plot[trial,'random']

        nrows = 8
        ncols = 20

        plt.figure()
        plt.plot([r[3] for r in greedy], [r[2] for r in greedy],
                 label='Greedy',
                 marker='s', color='blue')
        plt.ylim((0, greedy[0][2]))
        plt.legend(numpoints=1)
        plt.xlabel('Time Budget (ms)')
        plt.ylabel('Total Entropy')
        plt.savefig(pp, **savefig_settings)
        plt.figure()
        plt.plot(list(xrange(len(greedy))),
                 [sum(r[4][0] for r in greedy[:s+1]) for s in xrange(len(greedy))],
                 label='Greedy',
                 marker='s', color='blue')
        #plt.ylim((0, greedy[0][2]))
        plt.legend(numpoints=1,loc=2)
        plt.xlabel('Number of Views')
        plt.ylabel('Cumulative Information')
        plt.savefig(pp, **savefig_settings)
        for r in greedy:
            fig = plt.figure()
            X = np.array([i for i in xrange(ncols)])
            Y = np.array([i for i in xrange(nrows)])
            X, Y = np.meshgrid(X, Y)
            Z = np.zeros((nrows, ncols))
            for i in xrange(8):
                for j in xrange(20):
                    Z[i,j] = r[1][i * ncols + j + 1]

            ax = fig.gca(projection='3d')
            ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,
                            rstride=1, cstride=1, antialiased=False)
            ax.set_zlim((0,1))
            plt.savefig(pp, **savefig_settings)

        plt.figure()
        plt.plot([r[3] for r in rand[0]], [r[2] for r in rand[0]],
                 label='Random',
                 marker='o', color='green')
        plt.ylim((0, rand[0][0][2]))
        plt.xlabel('Time Budget (ms)')
        plt.ylabel('Total Entropy')
        plt.legend(numpoints=1)
        plt.savefig(pp, **savefig_settings)
        plt.figure()
        plt.plot(list(xrange(len(rand[0]))),
                 [sum(r[4][0] for r in rand[0][:s+1]) for s in xrange(len(rand[0]))],
                 label='Random',
                 marker='s', color='green')
        #plt.ylim((0, rand[0][0][2]))
        plt.legend(numpoints=1,loc=2)
        plt.xlabel('Number of Views')
        plt.ylabel('Cumulative Information')
        plt.savefig(pp, **savefig_settings)
        for r in rand[0]:
            fig = plt.figure()
            X = np.array([i for i in xrange(ncols)])
            Y = np.array([i for i in xrange(nrows)])
            X, Y = np.meshgrid(X, Y)
            Z = np.zeros((nrows, ncols))
            for i in xrange(8):
                for j in xrange(20):
                    Z[i,j] = r[1][i * ncols + j + 1]

            ax = fig.gca(projection='3d')
            ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,
                            rstride=1, cstride=1, antialiased=True)
            ax.set_zlim((0,1))

            plt.savefig(pp, **savefig_settings)

        plt.figure()
        plt.plot([r[3] for r in rand[0]], [r[2] for r in rand[0]],
                 label='Random',
                 marker='o', color='green')
        plt.ylim((0, rand[0][0][2]))
        for i in xrange(1, num_random):
            plt.plot([r[3] for r in rand[i]], [r[2] for r in rand[i]],
                     label="_nolegend_", marker='o', color='green')

        plt.plot([r[3] for r in greedy], [r[2] for r in greedy],
                 label='Greedy',
                 marker='s', color='blue')
        plt.xlabel('Time Budget (ms)')
        plt.ylabel('Total Entropy')
        plt.legend(numpoints=1)
        plt.savefig(pp, **savefig_settings)

        plt.figure()
        plt.plot(list(xrange(len(rand[0]))),
                 [sum(r[4][0] for r in rand[0][:s+1]) for s in xrange(len(rand[0]))],
                 label='Random',
                 marker='o', color='green')
        #plt.ylim((0, rand[0][0][2]))
        for i in xrange(1, num_random):
            plt.plot(list(xrange(len(rand[i]))),
                     [sum(r[4][0] for r in rand[i][:s+1]) for s in xrange(len(rand[i]))],
                     label="_nolegend_", marker='o', color='green')

        plt.plot(list(xrange(len(greedy))),
                 [sum(r[4][0] for r in greedy[:s+1]) for s in xrange(len(greedy))],
                 label='Greedy',
                 marker='s', color='blue')
        plt.xlabel('Number of Views')
        plt.ylabel('Cumulative Information')
        plt.legend(numpoints=1,loc=2)
        plt.savefig(pp, **savefig_settings)


        plt.figure()
        #plt.ylim((0, rand[0][0][2]))
        data = [[] for s in xrange(len(greedy)-1)]
        for i in xrange(num_random):
            for s in xrange(len(greedy)-1):
                if s+1 < len(rand[i]):
                    data[s].append(sum(r[4][0] for r in rand[i][:s+1+1]))
                else:
                    data[s].append(data[s-1][-1])
        bp = plt.boxplot(data, labels=list(xrange(1,len(greedy))), showfliers=False)
        plt.setp(bp['boxes'], color='green')
        plt.setp(bp['whiskers'], color='green')
        plt.setp(bp['caps'], color='green')
        plt.plot(list(xrange(len(greedy))),
                 [sum(r[4][0] for r in greedy[:s+1]) for s in xrange(len(greedy))],
                 label='Greedy',
                 marker='s', color='blue')
        plt.xlabel('Number of Views')
        plt.ylabel('Cumulative Information')
        hR, = plt.plot([1,1],color='green',label='Random')
        plt.xlim([0,len(greedy)-1])
        plt.legend(numpoints=1,loc=2)
        hR.set_visible(False)
        plt.savefig(pp, **savefig_settings)

    pp.close()
    #plt.show()
