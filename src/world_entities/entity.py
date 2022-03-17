
class Entity:
    """ An entity has an ID. No class of this type is directly instantiable. """

    def __init__(self, identifier):
        self.identifier = identifier

    def __eq__(self, other):
        """ Entity objects are identified by their id. """
        return isinstance(other, type(self)) and other.identifier == self.identifier

    def __hash__(self):
        """ Returns an integer identifying the object. """
        return hash(self.identifier)


class SimulatedEntity(Entity):
    """ An entity in the environment, e.g. Drone, Event, Packet. """

    def __init__(self, identifier: int, coords: tuple, simulator):
        super().__init__(identifier)
        self.simulator = simulator    # the simulator
        self.coords = coords          # the coordinates of the entity on the map

    def __to_json(self):
        """ return the json repr of the Simulated Entity """
        pass