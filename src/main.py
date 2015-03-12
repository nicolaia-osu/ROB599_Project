"""
Description:
    Driver for our SDM adaptive view planning project.

Author: Kory Kraft
Date: 3-12-15
"""

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
                
                if split_line[1]  == 1:
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
        
        Returns next_state
    """
    # select the next view based on just a greedy- 1 step lookahead
    print "Selecting next view..."
    
def calc_movement_cost(state_x, state_y):
    """ Returns cost from one state to another.
    """
    print "Calculating cost..."
    
def plan_views(view_data, init_state):
    """ Plans the states to view to categorize as an object.
        
        Returns a list of states.
    """ 
    print "Planning views..."
    
    
    # states are view angles/locations
    plan = []
    cur_state = init_state
    accum_cost = 0.0  # the accumulated costs of going from one view to another in plan
    total_certainty = 0.0 # the overall certainty we have about object at given point
    budget = 100.0 
    certainty_threshold = 1.0 # optional, the confidence required to return answer
    planned = False
    
    while (not planned):    
        # given set of views from current state
        #  perform 1 step look ahead w/ prob update
    
        # pick next view
        new_state = select_next_view(view_data, cur_state, accum_cost, budget)
    
        # calculate movement (view-to-view) cost
        accum_cost += calc_movement_cost(new_state, cur_state)
                    
        if accum_cost > budget:
            # we probably need to move this to be right below the calc
            # movement cost.  if the new cost + old cost > budget, then end
            planned = True
            break
        
        if total_certainty > certainty_threshold:
            # we are confident enough to make a decision at this point
            planned = True
            break
        
        # update state
        plan.append(new_state)
        cur_state = new_state
    
    return plan
        
    
def output_results(results):
    print "Results:"

if __name__ == "__main__":
    
    # load view states
    mappings_fname = "test.cvs"
    view_results_fname = "text.csv"
    view_region_fname = "test.csv"
    
    view_region_mappings = load_view_region_mappings(mappings_fname)
    view_results = load_view_results(view_results_fname)
    view_region_defs = load_view_region_definitions(view_region_fname)
    
    # path planning algorithm
    init_state = 'blimey'
    results = plan_views(view_data)
    
    # nicely output results 
    output_results(results)