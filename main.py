import world.simulation as sim
import botGenerator as bg

if __name__ == "__main__":
    generator = bg.botGenerator()
    vectors = generator.generateVectors(2,170)
    bots = generator.generateBots(vectors)
    world = sim.World()
    simulation = sim.Simulation(bots,world)
    simulation.start()
    simulation.wait()