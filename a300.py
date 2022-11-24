from collections import deque
from itertools import chain
from time import sleep

from testdevice import AdbTestDevice, AdbDevice, DummyDevice


class A300Writer:
  '''Translator from raw bytes to Global Village A300 modem ADB packets.'''
  
  def __init__(self, talk):
    self.talk = talk
    self.deque = deque()
  
  def byte(self, byte=None):
    '''Queue <byte> to be sent, or flush the queue if <byte> is None.'''
    if byte is None:
      if len(self.deque) > 0:
        self.talk(0, bytes(chain(
          self.deque,
          (0x00 for i in range(7 - len(self.deque))),
          (0x80 | len(self.deque),),
        )))
        self.deque = deque()
    elif len(self.deque) == 7 and (byte < 0x80 or byte > 0x9F):
      self.deque.append(byte)
      self.talk(0, bytes(self.deque.popleft() for i in range(8)))
    elif len(self.deque) == 7 and 0x80 <= byte <= 0x9F:
      self.talk(0, bytes(chain((self.deque.popleft() for i in range(7)), (0x87,))))
      self.deque.append(byte)
    else:
      self.deque.append(byte)


class GlobalVillageA300(AdbDevice):
  '''AdbDevice subclass that provides bare bones emulation of a Global Village A300 modem.
  
  Args:
    write_func: This function is called with two args: data read from the modem and a function to call to write a reply.  If None,
      it sets itself up as a simple echo device.
  '''
  
  def __init__(self, write_func=None):
    super().__init__(5, 0x36)
    self.write_func = write_func or (lambda data, func: self.write(data))
  
  def init(self):
    '''Initialize, set up responses for Talk 1 and 2 and a status on Talk 0.'''
    self.talk(1, b'\xF0\x00\x00\x00')
    self.talk(2, b'\x14\x01\x9F\x8E\x64\x3A')
    self.talk(0, b'\xF0\x00\x00\x00\x00\x88\x88\x88')
  
  def listen(self, register, data):
    '''Handle an ADB Listen command.
    
    Note that registers are not part of this emulation, only data.
    '''
    if register != 0: return  #TODO
    if len(data) != 8: raise ValueError("GV A300 doesn't know how to deal with listen 0s that aren't 8 bytes long")
    if 0x80 <= data[7] <= 0x87:
      self.write_func(data[:data[7] & 0x7], self.write)
    elif 0x88 <= data[7] <= 0x9F:
      raise ValueError('unrecognized listen 0 payload: %s' % repr(data))
    else:
      self.write_func(data, self.write)
  
  def write(self, data):
    '''Queue data to be given as Talk 0 responses.
    
    Data is flushed after write, so short writes will be inefficient.
    '''
    writer = A300Writer(self.talk)
    for byte in chain(data, (None,)):
      if byte == 0x95: writer.byte(0x95)
      writer.byte(byte)
  
  def handler_change(self, handler_id):
    '''Handle handler change.  This should never be called with any number other than 0x36.'''
    if handler_id != 0x36: raise ValueError('A300 was asked to change to handler ID 0x%02X, only knows 0x36' % handler_id)


class XModemServer:
  '''An XMODEM server to use with GlobalVillageA300 to send a file.
  
  Args:
    filepath: Path to file to serve.
  '''
  
  def __init__(self, filepath):
    self.filepath = filepath
    self.fp = None
    self._last_block = None
    self.last_block_number = None
  
  @property
  def last_block(self):
    if self._last_block is None:
      self.fp = open(self.filepath, 'rb')
      data = self.fp.read(128)
      checksum = sum(data) & 0xFF
      self._last_block = bytes(chain((0x01, 0x01, 0xFE), data, (checksum,)))
      self.last_block_number = 1
    return self._last_block
  
  @last_block.setter
  def last_block(self, value):
    self._last_block = value
  
  def write(self, data, reply):
    '''Used to receive data from the modem.
    
    Args:
      data: Inbound data.
      reply: Function to call with outbound data.
    '''
    if 0x15 in data or 0x06 in data:
      for byte in data:
        if byte == 0x15:  # NAK (send last block again or start transmission)
          reply(self.last_block)
        elif byte == 0x06:  # ACK (send next block)
          if self.fp:
            data = self.fp.read(128)
            if len(data) == 0:
              self.last_block = b'\x04'  # EOT (end of file)
              self.fp.close()
              self.fp = None
            else:
              if len(data) != 128: data = bytes(chain(data, (0x00 for i in range(128 - len(data)))))
              checksum = sum(data) & 0xFF
              self.last_block_number = (self.last_block_number + 1) % 0x100
              self.last_block = bytes(chain((0x01, self.last_block_number, 0xFF - self.last_block_number), data, (checksum,)))
            reply(self.last_block)
          elif self.last_block == b'\x04':  # EOT (end of file)
            self.last_block = None
    else:
      reply(data)
