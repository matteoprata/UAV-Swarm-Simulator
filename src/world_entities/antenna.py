
class AntennaEquippedDevice:
    """ To implement """

    def __init__(self, can_receive=True, can_transmit=True):
        self.can_receive = can_receive
        self.can_transmit = can_transmit

    def transmit_data(self, pk):
        pass

    def receive_data(self, pks: list):
        pass