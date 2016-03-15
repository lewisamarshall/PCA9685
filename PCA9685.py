from __future__ import print_function
import smbus


class PCA9685(object):

    _bus_location = 1
    _address = 0x40
    _motor = [True]*16
    _clock = 25e6
    _items = [None]*16

    def __init__(self, location=None, address=None):
        self._bus_location = location or self._bus_location
        self._address = address or self._address
        self._bus = smbus.SMBus(self._bus_location)

    def __repr__(self):
        base = 'PCA9685(location={}, address={})'
        return base.format(self._bus_location, self._address)

    def __str__(self):
        lines = []
        lines.append('PCA9685')
        lines.append('-------')
        lines.append('Address: {0:x}'.format(self._address))
        lines.append('Frequency: {}'.format(self.frequency()))
        lines.append('Mode: {:b}'.format(self.mode()))
        return '\n'.join(lines)

    def __getitem__(self, idx):
        assert idx in range(16), 'Invalid channel.'
        return self._itemp[idx]

    def servo(self, channel):
        self._items[channel] = Servo(self, channel)
        return self[channel]

    def led(self, channel):
        return LED(self, channel)

    def frequency(self, freq=None):
        if freq is None:
            pre_scale = self._read(254)
            frequency = self._clock/4096/(pre_scale+1)
            return frequency
        else:
            pre_scale = int(25e6/4096/freq)-1
            assert 0x3<=pre_scale<=0xFF, 'Frequency outside valid range.'
            self._write(254, pre_scale)

    def _write(self, register, value):
        assert 0<=value<2**8, 'Invalid byte value.'
        self._bus.write_byte_data(self._address, register, value)

    def _read(self, register):
        return self._bus.read_byte_data(self._address, register)

    def _long_write(self, register, value):
        self._write(register, value&0xFF)
        self._write(register+1, value>>8)

    def _long_read(self, register):
        value1 = self._read(register)
        value2 = self._read(register+1)
        return ((value2 & 0b111)<<8) + value1

    def mode(self, bit=None, state=None):
        mode_0 = self._read(0)
        mode_1 = self._read(1)
        if bit is None:
            return (mode_0<<8) + mode_1

    def write_bit(self, register, bit, state):
        initial = self._read(register)
        mask = 1<<bit
        if state is True:
            final = initial | mask
        else:
            final = initial & ~mask
        self._write(register, final)

    def active(self, state=True):
        if state is True:
            self._write_bit(0, 0b1)
        else:
            self._write(0, 0b10001)

class Servo(object):
    def __init__(self, controller, channel):
        self._controller = controller
        self._channel = channel

    def position(self, angle=None):
        if angle is None:
            on = self._controller._long_read(6 + self._channel*4)
            off = self._controller._long_read(6 + self._channel*4+2)
            angle = ((off-on)/4095.*20.-1.)*180.
            return angle, on, off
        else:
            assert 0<=angle<=180
            on = 0
            off = int(4095/20*(1.+(angle/180.)))
            self._controller._long_write(6+self._channel*4, on)
            self._controller._long_write(6+self._channel*4+2, off)

class LED(object):
     def __init__(self, bus, channel):
        self._bus = bus
        self._channel = channel

if __name__ == '__main__':
    import time
    controller = PCA9685()
    controller.mode()
    controller.write_bit(0, 4, False)
    controller.mode()
    controller.write_bit(0, 4, True)
    controller.mode()
