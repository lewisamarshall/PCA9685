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

    def pwm(self, channel):
        return PWM(self, channel)

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
            self._write_bit(0, 4, True)
        else:
            self._write_bit(0, 4, False)


class Channel(object):
    _on_register = None
    _off_register = None

    def __init__(self, controller, channel):
        assert channel in range(16), 'invalid channel'
        self._controller = controller
        self._channel = channel
        self._on_register = 6 + channel * 4
        self._off_register = self._on_register + 2

    def __repr__(self):
        base = 'Channel(controller={}, channel={})'
        return base.format(self._controller, self._channel)

    def __str__(self):
        lines = []
        lines.append('Channel')
        lines.append('-------')
        lines.append('channel: {}'.format(self._channel))
        lines.append('on: {}'.format(self.on()))
        lines.append('off: {}'.format(self.off()))
        return '\n'.join(lines)

    def on(self, state=None):
        if state is None:
            return self._controller._long_read(self._on_register)
        else:
            self._controller._long_write(self._on_register, state)

    def off(self, state=None):
        if state is None:
            return self._controller._long_read(self._off_register)
        else:
            self._controller._long_write(self._off_register, state)


class Servo(Channel):
    def __init__(self, controller, channel):
        self._controller = controller
        self._channel = channel

    def position(self, angle=None):
        if angle is None:
            angle = ((self.off()-self.on())/4095.*20.-1.)*180.
            return angle, on, off
        else:
            assert 0<=angle<=180
            on = 0
            off = int(4095/20*(1.+(angle/180.)))
            self.on(on)
            self.off(off)


class PWM(Channel):
    def duty(self, state=None):
        if state is None:
            state = (self.on()-self.off())%4095
            return state/4095.
        elif 0.<=state<=1.:
            state = state * 4095
            self.off((self.on()+state) % 4095)
        else:
            raise RuntimeError('Duty cycle outside range.')

    def phase(self, state=None):
        if state is None:
            return self.on()/4095.
        elif 0.<=state<=1.:
            state = state * 4095
            self.off((self.on()+state) % 4095)
        else:
            raise RuntimeError('Phase outside range.')


if __name__ == '__main__':
    import time
    controller = PCA9685()
    controller.mode()
    controller.write_bit(0, 4, False)
    controller.mode()
    controller.write_bit(0, 4, True)
    controller.mode()
