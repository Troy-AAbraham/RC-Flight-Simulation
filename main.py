from sim import simi
import time

"""Main Flight Simulation Module

Routine Listings
-----------------
simi():
    Flight SIM class with all necessary class functions

Notes
------
main.py imports the simi class from the sim.py module. It then calls the simi
methods to load the aircraft input file, initialize values, solve for trim,
and the run simulator for set conditions.

The input variable 'trim' specifies which system of equations to solve for.
With trim = 0, it uses the more complex SoE which includes more terms in the 
LHS matrix. trim = 1 uses the simpler version provided with more terms in the
RHS array.

The main module also times the Flight SIM run, and includes the option to plot
results.

Example
-------
arrow_sim.trim_func(trim = 0) #higher expanded implementation
arrow_sim.trim_func(trim = 1) #simpler implementation
"""

t1 = time.time()

aircraft_sim = simi()

aircraft_sim.load_file(filename = "11.24_input.json")
aircraft_sim.init_states()
aircraft_sim.trim_func(trim = 0)
aircraft_sim.run_sim()

aircraft_sim.plot_results(2)

t2 = time.time()

print("Simulation Time: ", t2-t1)
