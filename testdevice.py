from functools import partial
from itertools import chain
from threading import Thread


def _setup_string(default_address, default_handler_id, permitted_handler_ids=None):
  '''Yields bytes of a string to send to the ADB test device to set up the parameters of an emulated device.
  
  See AdbDevice for args.
  '''
  if not 1 <= default_address <= 255: raise ValueError('default address %s is not between 1 and 255' % default_address)
  if not 0 <= default_handler_id <= 255: raise ValueError('default handler ID %s is not between 0 and 255' % handler_id)
  yield default_address
  used_handler_ids = set()
  for handler_id in permitted_handler_ids or ():
    if handler_id == default_handler_id: continue
    if not 0 <= handler_id <= 255: raise ValueError('handler ID %s is not between 0 and 255' % handler_id)
    if handler_id in used_handler_ids: continue
    used_handler_ids.add(handler_id)
    if len(used_handler_ids) > 13: raise ValueError('too many handler IDs; maximum is 14')
    yield handler_id
  yield default_handler_id


class AdbDevice:
  '''Base class for a virtual device emulated by connection to an AdbTestDevice.
  
  Args:
      default_address: The address at which the device starts out and returns to after a reset.
      default_handler_id: The handler ID with which the device starts out and returns to after a reset.
      permitted_handler_ids: An iterable of handler IDs that the Mac may change to on the device.
  '''
  
  def __init__(self, default_address, default_handler_id, permitted_handler_ids=None):
    self.setup_string = bytes(_setup_string(default_address, default_handler_id, permitted_handler_ids))
  
  def init(self):
    '''Called by AdbTestDevice to initialize the subclass.'''
    raise NotImplementedError('subclass must override init method')
  
  def listen(self, register, data):
    '''Called by AdbTestDevice when the Mac sends an ADB Listen command to register <register> with data <data>.'''
    raise NotImplementedError('subclass must override listen method')
  
  def handler_change(self, handler_id):
    '''Called by AdbTestDevice when the Mac changes the handler ID for the device to <handler_id>.'''
    raise NotImplementedError('subclass must override handler_change method')
  
  def talk(self, register, data):
    '''Called by the subclass to send data <data> to be answered to an ADB Talk command for register <register>.'''
    raise NotImplementedError('AdbTestDevice must override talk method')
  
  def exceptional_event(self):
    '''Called by the subclass to raise an exceptional event.'''
    raise NotImplementedError('AdbTestDevice must override exceptional_event method')


class AdbTestDevice(Thread):
  '''Represents an ADB test device.
  
  The ADB test device emulates exactly two devices.  Users wishing to emulate only one should use a DummyDevice as the second.
  
  Args:
    serial_obj: serial.Serial object used for communication with the ADB test device.
    device_0: The first of two devices emulated by the ADB test device.
    device_1: The second of two devices emulated by the ADB test device.
  '''
  
  def __init__(self, serial_obj, device_0, device_1):
    super().__init__()
    self.serial_obj = serial_obj
    self.device_0 = device_0
    self.device_1 = device_1
    device_0.talk = partial(self.talk, 0)
    device_1.talk = partial(self.talk, 1)
    device_0.exceptional_event = partial(self.exceptional_event, 0)
    device_1.exceptional_event = partial(self.exceptional_event, 1)
    self._bail_out = False
  
  def talk(self, device, register, data):
    '''Sends data <data> to be answered to an ADB Talk command for register <register> of device <device>.'''
    if not 0 <= register <= 2: raise ValueError('talk register %s is not between 0 and 2' % register)
    if not 2 <= len(data) <= 8: raise ValueError('data length %s is not between 2 and 8' % len(data))
    self.serial_obj.write(bytes(chain(((0x80 if device else 0x00) | (register << 4) | len(data),), data)))
  
  def exceptional_event(self, device):
    '''Raises an exceptional event on device <device>.'''
    self.serial_obj.write(bytes((0xB0 if device else 0x30),))
  
  def run(self):
    '''Thread that serves ADB Listen and header change commands to the virtual devices emulated by this test device.
    
    The thread sends the setup strings for both devices when it starts, therefore it is recommended that the thread not be started
    until the ADB test device is powered.
    '''
    device_0_inited = False
    device_1_inited = False
    self.serial_obj.write(bytes(chain((0x30 | len(self.device_0.setup_string),), self.device_0.setup_string)))
    self.serial_obj.write(bytes(chain((0xB0 | len(self.device_1.setup_string),), self.device_1.setup_string)))
    while True:
      if self._bail_out: break
      header_byte = self.serial_obj.read(1)
      if not header_byte: continue
      header_byte = header_byte[0]
      if not header_byte & 0x80 and not device_0_inited:
        self.device_0.init()
        device_0_inited = True
      elif header_byte & 0x80 and not device_1_inited:
        self.device_1.init()
        device_1_inited = True
      data = self.serial_obj.read(header_byte & 0xF)
      if len(data) != header_byte & 0xF: raise ValueError('timeout waiting for payload with header byte 0x%02X' % header_byte)
      if header_byte & 0x70 in (0x00, 0x10, 0x20):
        func = self.device_1.listen if header_byte & 0x80 else self.device_0.listen
        func((header_byte & 0x30) >> 4, data)
      elif header_byte & 0x70 == 0x30:
        func = self.device_1.handler_change if header_byte & 0x80 else self.device_0.handler_change
        if len(data) != 1: raise ValueError('invalid data length for handler change')
        func(data[0])
  
  def stop(self):
    '''Stops the thread.'''
    self._bail_out = True


class DummyDevice(AdbDevice):
  '''Dummy ADB device.  Does nothing.'''
  def __init__(self): super().__init__(6, 0xFC)
  def init(self): pass
  def listen(self, register, data): pass
  def handler_change(self, handler_id): pass


class DemoDevice(AdbDevice):
  '''Demo ADB device.  Echoes init calls, ADB Listen commands, and handler changes but does nothing else.'''
  
  def __init__(self):
    super().__init__(6, 0xFC)
  
  def init(self):
    print('Dummy device, init')
  
  def listen(self, register, data):
    print('Dummy device, listen %X: %s' % (register, repr(data)))
  
  def handler_change(self, handler_id):
    print('Dummy device, handler change to %X' % handler_id)
