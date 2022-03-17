from src.world_entities.entity import SimulatedEntity
from src.world_entities.antenna import AntennaEquippedDevice


class BaseStation(SimulatedEntity, AntennaEquippedDevice):

    def __init__(self, coords, com_range, simulator):
        SimulatedEntity.__init__(self, id(self), coords, simulator)
        AntennaEquippedDevice.__init__(self)

        self.com_range = com_range
        self.buffer = list()
