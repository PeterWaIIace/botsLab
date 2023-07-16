import world.simulation as sim

if __name__ == "__main__":
    world = sim.World()
    simulation = sim.Simulation([],world)
    simulation.start()
    simulation.wait()