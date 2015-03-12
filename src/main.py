"""
Description:
    Driver for our SDM adaptive view planning project.

Author: Kory Kraft
Date: 3-12-15
"""

def load_view_states(fname):
    print "Data Source: ", fname
    
def plan_views(view_data):
    print "Planning views..."
    
def output_results(results):
    print "Results:"

if __name__ == "__main__":
    
    # load view states
    fname = ""
    view_data = load_view_states(fname)
    
    # path planning algorithm
    results = plan_views(view_data)
    
    # nicely output results 
    output_results(results)