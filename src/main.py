
from src.simulation.simulator import Simulator


def main():
    """ the place where to run simulations and experiments. """
    sim = Simulator()    # empty constructor means that all the parameters of the simulation are taken from src.utilities.config.py
    sim.run()            # run the simulation
    sim.print_metrics()  # print the metrics at the end of the simulation
    sim.close()


if __name__ == "__main__":
    main()
