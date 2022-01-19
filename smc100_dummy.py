# -*- coding: utf-8 -*-
#!/usr/bin/env python
import serial
import time

from math import floor

# never wait for more than this e.g. during wait_states
MAX_WAIT_TIME_SEC = 600

# time to wait after sending a command. This number has been arrived at by
# trial and error
COMMAND_WAIT_TIME_SEC = 0.06

# States from page 65 of the manual
STATE_NOT_REFERENCED_FROM_RESET = '0A'
STATE_NOT_REFERENCED_FROM_CONFIGURATION = '0C'
STATE_READY_FROM_HOMING = '32'
STATE_READY_FROM_MOVING = '33'

STATE_CONFIGURATION = '14'

STATE_DISABLE_FROM_READY = '3C'
STATE_DISABLE_FROM_MOVING = '3D'
STATE_DISABLE_FROM_JOGGING = '3E'

class SMC100ReadTimeOutException(Exception):
  def __init__(self):
    super(SMC100ReadTimeOutException, self).__init__('Read timed out')

class SMC100WaitTimedOutException(Exception):
  def __init__(self):
    super(SMC100WaitTimedOutException, self).__init__('Wait timed out')

class SMC100DisabledStateException(Exception):
  def __init__(self, state):
    super(SMC100DisabledStateException, self).__init__('Disabled state encountered: '+state)

class SMC100RS232CorruptionException(Exception):
  def __init__(self, c):
    super(SMC100RS232CorruptionException, self).__init__('RS232 corruption detected: %s'%(hex(ord(c))))

class SMC100InvalidResponseException(Exception):
  def __init__(self, cmd, resp):
    s = 'Invalid response to %s: %s'%(cmd, resp)
    super(SMC100InvalidResponseException, self).__init__(s)

class SMC100(object):
  """
  Class to interface with Newport's SMC100 controller.
  The SMC100 accepts commands in the form of:
    <ID><command><arguments><CR><LF>
  Reply, if any, will be in the form
    <ID><command><result><CR><LF>
  There is minimal support for manually setting stage parameter as Newport's
  ESP stages can supply the SMC100 with the correct configuration parameters.
  Some effort is made to take up backlash, but this should not be trusted too
  much.
  The move commands must be used with care, because they make assumptions
  about the units which is dependent on the STAGE. I only have TRB25CC, which
  has native units of mm. A more general implementation will move the move
  methods into a stage class.
  """

  _port = None
  _smcID = None

  _silent = True

  _sleepfunc = time.sleep

  def __init__(self, smcID, port, backlash_compensation=True, silent=True, sleepfunc=None):
    """
    If backlash_compensation is False, no backlash compensation will be done.
    If silent is False, then additional output will be emitted to aid in
    debugging.
    If sleepfunc is not None, then it will be used instead of time.sleep. It
    will be given the number of seconds (float) to sleep for, and is provided
    for ease integration with single threaded GUIs.
    Note that this method only connects to the controller, it otherwise makes
    no attempt to home or configure the controller for the attached stage. This
    delibrate to minimise realworld side effects.
    If the controller has previously been configured, it will suffice to simply
    call home() to take the controller out of not referenced mode. For a brand
    new controller, call reset_and_configure().
    """

    super(SMC100, self).__init__()

    assert smcID is not None
    assert port is not None

    if sleepfunc is not None:
      self._sleepfunc = sleepfunc

    self._silent = silent

    self._last_sendcmd_time = 0

    print('Connecting to SMC100 on %s', str(port))

    self._positions_um = {}
    for key in smcID:
      self._positions_um[key] = 0

    self._smcID = smcID

  def home(self, smcid, position, waitStop=True):
    """
    Homes the controller. If waitStop is True, then this method returns when
    homing is complete.
    Note that because calling home when the stage is already homed has no
    effect, and homing is generally expected to place the stage at the
    origin, an absolute move to 0 um is executed after homing. This ensures
    that the stage is at origin after calling this method.
    Calling this method is necessary to take the controller out of not referenced
    state after a restart.
    """
    self.sendcmd('OR', smc=smcid)
    """The command BH enables to activate the hysteresis compensation."""
    self.sendcmd('BH', smc=smcid)
    
    if waitStop:
      st = self.wait_states((STATE_READY_FROM_HOMING, STATE_READY_FROM_MOVING), smcid)
      if st == STATE_READY_FROM_MOVING:
        self.move_absolute_um(position, smcid, waitStop=True)
    else:
      self.move_absolute_um(position, smcid, waitStop=False)
      
  def stop(self, smcid):
    self.sendcmd('ST', smc=smcid)

  def get_status(self, smcid):
    """
    Executes TS? and returns the the error code as integer and state as string
    as specified on pages 64 - 65 of the manual.
    """

    resp = self.sendcmd('TS', '?', smc=smcid, expect_response=True, retry=10)
    errors = int(resp[0:4], 16)
    state = resp[4:]

    assert len(state) == 2

    return errors, state

  def get_position_um(self, smcid):
    return self._positions_um[smcid]

  def move_relative_mm(self, dist_mm, smcid, waitStop=True):
    """
    Moves the stage relatively to the current position by the given distance given in mm
    """



  def move_relative_um(self, dist_um, smcid, **kwargs):
    """
    Moves the stage relatively to the current position by the given distance given in um. The
    given distance is first converted to an integer.
    If waitStop is True then this method returns when the move is completed.
    """

    self._positions_um[smcid] += dist_um
    time.sleep(abs(dist_um/1000))

  def get_speed_um(self,smcid):
      '''We get the speed in mm/s and we convert it in um/s'''
      speed=float(self.sendcmd('VA','?',smcid,expect_response=True,retry=10))
      speed=speed*1000.0
      return speed
  
  def set_speed_um(self,vitesse,smcid):
      '''We have a speed in um/s that we must divide by 1000 to obtain mm/s'''
      vitesse=vitesse/1000.0
      self.sendcmd('VA',vitesse,smcid)

  def wait_states(self, targetstates, smcid, ignore_disabled_states=False):
    """
    Waits for the controller to enter one of the the specified target state.
    Controller state is determined via the TS command.
    If ignore_disabled_states is True, disable states are ignored. The normal
    behaviour when encountering a disabled state when not looking for one is
    for an exception to be raised.
    Note that this method will ignore read timeouts and keep trying until the
    controller responds.  Because of this it can be used to determine when the
    controller is ready again after a command like PW0 which can take up to 10
    seconds to execute.
    If any disable state is encountered, the method will raise an error,
    UNLESS you were waiting for that state. This is because if we wait for
    READY_FROM_MOVING, and the stage gets stuck we transition into
    DISABLE_FROM_MOVING and then STAY THERE FOREVER.
    The state encountered is returned.
    """
    return

  def disable(self,smcid):
      
      self.sendcmd('MM',0,smcid)
      
  def sendcmd(self, command, argument=None, smc=0, expect_response=False, retry=False):
    """
    Send the specified command along with the argument, if any. The response
    is checked to ensure it has the correct prefix, and is returned WITHOUT
    the prefix.
    It is important that for GET commands, e.g. 1ID?, the ? is specified as an
    ARGUMENT, not as part of the command. Doing so will result in assertion
    failure.
    If expect_response is True, a response is expected from the controller
    which will be verified and returned without the prefix.
    If expect_response is True, and retry is True or an integer, then when the
    response does not pass verification, the command will be sent again for
    retry number of times, or until success if retry is True.
    The retry option MUST BE USED CAREFULLY. It should ONLY be used read-only
    commands, because otherwise REPEATED MOTION MIGHT RESULT. In fact some
    commands are EXPLICITLY REJECTED to prevent this, such as relative move.
    """
    assert command[-1] != '?'

    if self._port is None:
      return

    if argument is None:
      argument = ''

    prefix = self._smcID[smc-1] + command
    tosend = prefix + str(argument)

    # prevent certain commands from being retried automatically
    no_retry_commands = ['PR', 'OR']
    if command in no_retry_commands:
      retry = False

    while self._port is not None:

      print(tosend.encode('ascii'))

      # we only need to delay when we are not waiting for a response
      now = time.time()
      dt = now - self._last_sendcmd_time
      dt = COMMAND_WAIT_TIME_SEC - dt
      if dt > 0:
        self._sleepfunc(dt)

      self._last_sendcmd_time = now
      return None

  def _readline(self):
    """
    Returns a line, that is reads until \r\n.
    OK, so you are probably wondering why I wrote this. Why not just use
    self._port.readline()?
    I am glad you asked.
    With python < 2.6, pySerial uses serial.FileLike, that provides a readline
    that accepts the max number of chars to read, and the end of line
    character.
    With python >= 2.6, pySerial uses io.RawIOBase, whose readline only
    accepts the max number of chars to read. io.RawIOBase does support the
    idea of a end of line character, but it is an attribute on the instance,
    which makes sense... except pySerial doesn't pass the newline= keyword
    argument along to the underlying class, and so you can't actually change
    it.
    """
    return ''

  def _emit(self, *args):
    if len(args) == 1:
      prefix = ''
      message = args[0]
    else:
      prefix = ' ' + args[0]
      message = args[1]

    if not self._silent:
      print ('[SMC100' + prefix + '] ' + message)

  def close(self):
    pass

  def __del__(self):
    self.close()

