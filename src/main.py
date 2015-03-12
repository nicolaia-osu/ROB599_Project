"""
Description:
    Driver for our SDM adaptive view planning project.

Author: Kory Kraft
Date: 3-12-15
"""

def load_view_states(fname):
    print "Data Source: ", fname
    
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
        
        # update state
        cur_state = new_state
        
        if accum_cost > budget:
            # we probably need to move this to be right below the calc
            # movement cost.  if the new cost + old cost > budget, then end
            planned = True
            break
        
        if total_certainty > certainty_threshold:
            # we are confident enough to make a decision at this point
            planned = True
            break
        
    
def output_results(results):
    print "Results:"

if __name__ == "__main__":
    
    # load view states
    fname = ""
    view_data = load_view_states(fname)
    
    # path planning algorithm
    init_state = 'blimey'
    results = plan_views(view_data)
    
    # nicely output results 
    output_results(results)