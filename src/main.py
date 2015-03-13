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

class State:
    NINETY = 1
    HUNDRED = 2
    STATES_LIST = [1, 2]
    
def getXY(state):
    """ Translates a state to an x,y coordinate value.
    
        Returns (x,y) tuple.
    """
    if state == State.NINETY:
        return (0, 0)
    elif state == State.HUNDRED:
        return (1, 0)
    else:
        print "Error! Don't recognize state"
        sys.exit(-1)
        
def get_state_value(state):
    """ Returns value of state
    *****************************
            Unimplemented!
    *****************************
    """
    print "Getting state value..."
    print "*************************"
    print "       Unimplemented!"
    print "*************************"
    
def manhattan_distance(location_a, location_b):
        """ returns manhattan_distance of (x,y) to (x2,y2)

            Does not take into account heading...
        """
        return abs(location_a[0] - location_b[0]) + abs(location_a[1] - location_b[1])
    

def get_possible_states(cur_state, accum_cost, budget):
    """ Returns a list of the states you can visit given the current state and current cost
            that fits within the overall budget.
    """
    all_states = State.STATES_LIST
    
    remaining_budget = budget - accum_cost
    
    possible_states = []
    
    for state in all_states:
        movement_cost = calc_movement_cost(cur_state, state)
        if movement_cost < remaining_budget:
            possible_states.append(state)
    
    return possible_states
    
def calc_movement_cost(state_x, state_y):
    """ Returns cost from one state to another.
    
        Base cost for any move is 100 (1/10 of second).
    """
    print "Calculating cost..."
    
    base_cost = 100
    angle_to_angle_cost = 200 # cost to go from one view angle to other (motor time)
    cost = base_cost
    
    if state_x == state_y:
        return cost
    else:
        x1,y1 = getXY(state_x)
        x2,y2 = getXY(state_y)
        dist = manhattan_distance([x1, y1], [x2, y2])
        cost += dist * angle_to_angle_cost
        return cost     
    
def get_highest_value_move(states):
    max_state = 0
    max_value = -9999
    
    for state in states:
        val = get_state_value(state)
        if val > max_value:
            max_state = state
            max_value = max_value
    
    return (max_state, max_value)


def load_view_region_mappings(fname):
    """ Returns a list of tuples.
    
        Index in region_mappings list correspond to view angle/state.
        Index into region_tuple correspond to:
            0 = True region
            1 = mapping at view from 1
            2 = mapping at view from 2
            3 = mapping at view from 3
        
        e.g region_tuple_two = region_mappings[1] yields a tuple of mappings for the second viewing angle/state
            region_tuple_two[0] = true knife region for second viewing angle/state
            region_tuple_two[3] = mapping for second viewing angle/state region
    """ 
    print "Data Source: ", fname
    print "Loading view region mappings..."
    
    region_mappings = [] # regions will be a list of tuples
    
    with open(fname, "r") as f:
        for line in f:
            if line.startswith("#"): 
                # comment line
                continue
            else:
                region_tuple = map(lambda x: int(x), line.split(","))
                region_mappings.append(tuple(region_tuple))
                    
    return region_mappings

def load_view_results(fname):
    """ Returns list of tuples.
    
            File name expected to have name:
            view_*_results_*.csv : the first wildcard represents the given view number
                                 : the second wildcard represents the trial 
        
            File in format: 
                View Region 1, Defect [y/n], Confidence Level [0 - 1]
                View Region 2, Defect [y/n], Confidence Level [0 - 1]
                
            List of tuples in form:
                (True/False, ConfidenceFloat)
                
                To get tuple for region 1, you would index list 1.
                The tuple at index 0 is an undefined placeholder used to make 
                    indexing easier.
        
    """
    print "Data Source: ", fname
    print "Loading view results..."
    
    view_results = []
    view_results.append(("NAN"))
    
    with open(fname, "r") as f:
        for line in f:
            if line.startswith("#"):
                continue
            else:
                split_line = line.split(",")
                
                if split_line[1]  == "1":
                    isDefect = True
                else:
                    isDefect = False
                
                confidence = float(split_line[2])
                view_tuple = (isDefect, confidence)
                view_results.append(view_tuple)
                
    return view_results

def load_view_region_definitions(fname):
    """ Returns list of tuples.
               
            File in format: 
                View Region 1, X Degrees, Y Degrees
                View Region 2, X Degrees, Y Degrees
                
            List of tuples in form:
                [(X degrees, y degrees),...]
                
                To get tuple for region 1, you would index list 1.
                The tuple at index 0 is an undefined placeholder used to make 
                    indexing easier.        
    """
    print "Data Source: ", fname
    print "Loading view region definitions..."
    
    view_region_defs = []
    view_region_defs.append(("NAN"))
    
    with open(fname, "r") as f:
        for line in f:
            if line.startswith("#"):
                continue
            else:
                split_line = line.split(",")
                
                xDegrees = float(split_line[1])
                yDegrees = float(split_line[2])
                
                view_def = (xDegrees, yDegrees)
                view_region_defs.append(view_def)
                
    return view_region_defs
          
    
def select_next_view(view_data, cur_state, accum_cost, budget):
    """ Gets the next view from cur state (view) using 1-step lookahead
        in view_data.
        
        Returns next_state, value
    """
    # select the next view based on just a greedy- 1 step lookahead
    print "Selecting next state//view..."
    
    # get the possible states you can go to that fits within the budget
    possible_states = get_possible_states(cur_state, accum_cost, budget)
    
    # get highest value move
    next_state, next_state_val = get_highest_value_move(possible_states)
    
    return next_state, next_state_val

        
def get_views_greedy_horizon(view_region_defs, view_results, view_region_mappings, init_state, budget):
    """ Decides the states to view to categorize as an object in an adaptive manner.
        
        Returns a list of states that were visited, along with total reward, total cost, total certainty.
    """ 
    print "Planning views..."    
    
    # states are view angles/locations
    path = []
    cur_state = init_state
    accum_cost = 0.0  # the accumulated costs of going from one view to another in path
    total_certainty = 0.0 # the overall certainty we have about object at given point 
    certainty_threshold = 1.0 # optional, the confidence required to return answer
    total_reward = 0.0
    decided = False
    
    while (not decided):    
        # given set of views from current state
        #  perform 1 step look ahead w/ prob update
    
        # pick next state/view and get the val (eq, info gain...not to be confused with cost
        new_state, new_state_val = select_next_view(*view_data, cur_state, accum_cost, budget)
    
        # calculate movement (view-to-view) cost
        move_cost = calc_movement_cost(new_state, cur_state)
        
        if (accum_cost + move_cost) > budget:
            # out of time
            break
        
        accum_cost += move_cost
        
        if total_certainty > certainty_threshold:
            # we are confident enough to make a decision at this point
            decided = True
            break
        
        # update state and add into path
        path.append(new_state)
        cur_state = new_state
        
        # update reward
        total_reward += new_state_val
        
        # update probability distribution
        update_prob_dist()
    
    return (path, accum_cost, total_certainty)

        
def execute_planning(mappings_fname, view_results_fname, view_region_fname, budget):
    
    # load view states 
    view_region_mappings = load_view_region_mappings(mappings_fname)
    view_results = load_view_results(view_results_fname)
    view_region_defs = load_view_region_definitions(view_region_fname)
    
    # execute path planning algorithm
    init_state = State.NINETY
    results = get_views_greedy_horizon(view_region_defs, view_results, view_region_mappings, init_state, budget)
    
    # nicely output results 
    output_results(results)
    
def output_results(results):
    print "Results:"

if __name__ == "__main__":
    
    # load files
    mappings_fname = "test.cvs"
    view_results_fname = "text.csv"
    view_region_fname = "test.csv"
    
    # requirements
    budget = 4 * 1000 # 4 seconds..
    
    # execute planning
    start = time.clock()
    execute_planning(mappings_fname, view_results_fname, view_region_fname, budget)
    stop = time.clock()
    
    print "Time elapsed (secs): ", stop - start
    
    
    
    