import swarm_tasks.simulation.simulation

# visualizer (matplotlib/Qt) and sim_tests are NOT imported eagerly here:
# the backend (swarm.py) runs headless by default and must not pay
# matplotlib's import cost -- or risk a Qt platform-plugin failure in a
# frozen build -- just because it imported this package for Simulation.
# Anything that needs them imports the submodule directly, e.g.
# `from swarm_tasks.simulation import visualizer`, which works the same
# whether or not the package __init__ touched it first.
