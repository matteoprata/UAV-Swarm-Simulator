# UAV Simulator

UAV-Simulator is a Python simulator for experimenting networks unmanned aerial vehicles. 
It requires Python 3 or greater.

## Quick Start

```bash
git clone https://github.com/uavnetworkslab/uavsimulator
cd uavsimulator

pip install -r requirements.txt
python -m src.main
```

The simulation will start in a new window, the parameters of the simulation are set in ``src.utilities.config``, 
 have a look at the simulation setup in the configuration file to understand what is going on in the 
 simulation. 

## Project Structure 
The project has the following structure:
```bash
.
├── README.md
├── data
│   ├── tours
│   │   ├── RANDOM_missions0.json
│   │   ├── ...
│   │   └── RANDOM_missions50.json
├── requirements.txt
└── src
    ├── drawing
    │   ├── color.py
    │   ├── dyn_plots.py
    │   ├── picture.py
    │   ├── pp_draw.py
    │   └── stddraw.py
    ├── experiments
    ├── main.py
    ├── simulation
    │   └── simulator.py
    ├── utilities
    │   ├── config.py
    │   └── utilities.py
    └── world_entities
        ├── antenna.py
        ├── base_station.py
        ├── drone.py
        ├── entity.py
        └── environment.py
```

The entry point of the project is the ``src.main`` file, from there you can run simulations and extensive
 experimental campaigns, by setting up an appropriate ``src.simulator.Simulator`` object. 
 
On a high level, the two main directories are ``data`` and ``src``. The directory ``data`` must contain all the 
data of the project, like drones tours, and other input and output files of the project. The directory ``src`` 
contains the source code, organized in several packages. 

* ``src.drawing`` it contains all the classes needed for drawing the simulation on screen. Typically you may 
want to get your hands in this directory, specifically in ``pp_draw.py``, if you want to change the aspect of the simulation, display a new 
object, or label on the simulated environment.

* ``src.world_entities`` it contains all the classes that define the behaviour and the structure of the main
 entities of the project like: Drone, Base Station and Environment classes.

* ``src.experiments`` it contains classes that handle experimental campaigns.

* ``src.simulation`` it contains all the classes to handle a simulation and its metrics. 

* ``src.utilities`` it contains all the utilities and the configuration parameters. In particular use ``src.utilities.config`` file to 
specify all the constants and parameters for a one-shot simulation, ideal when one wants to evaluate
the quality of a routing algorithm making frequent executions. Constants and parameters should **always** be added here
and never be hard-coded.
  
## Contacts
For further information contact Matteo Prata at [prata@di.uniroma1.it](mailto:prata@di.uniroma1.it) and 
Andrea Coletta at [coletta@di.uniroma1.it](mailto:coletta@di.uniroma1.it).
