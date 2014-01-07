#! usr/bin/python

### Firmata implementation for Micro Python
### Protocol defined here: http://firmata.org/wiki/Protocol

### Custom for each hardware port
### Define the PINS and COMMS dictionaries below for each hardware device



VERSION_BLINK_PIN = 24 # blink this pin to show version info.

#------------------------------------------------
# micro python is likely to be running on chips with many more pins than originally envisioned by this protocol.
#   (The protocol maximum pin is 127. Special analog commands have a maximum of 15 pins. Special commands  deal with indices > 15.
# To solve this problem and make implementation and control easier - the physical pins are remapped to virtual pins.
# - virtual pins are all numbered from 0
# - analog pins are grouped into their own subset and labelled from 1
#   - They will be masked out of operations on digital pins
# Digital Pins:
#  - a pin should appear only once in a single category - declaring your intention for its use.
#  - Alter this list so that the pins you want to control appear in it.
#    - Remove pins you do not want to make available.
# Set the config up for the chip using !! pyb.??
# !!Could be an import?

# potentially all defined pins can be configured as digital inputs or outputs

# Indices used in PINS to keep structure small.
# Using same values as Firmata pin mode values to make mode discovery easier
INPUT    = 0x00 # digital pin in input mode
OUTPUT   = 0x01 # digital pin in output mode
ANALOG   = 0x02 # analog pin in input mode
PWM      = 0x03 # digital pin in PWM output mode
SERVO    = 0x04 # digital pin in Servo output mode
# to these we add shortcuts to make life simpler and
# we use D=O to make Digital correspond with Output in Firmata
#D,A,P,S,I = 1,2,3,4,5   # Digital, Analog, Pwm, Servo, I2c
D,A,P,S = OUTPUT, ANALOG, PWM, SERVO
# add extras for defining the Firmata PORTs
PORTS, I2C, SPI = 252,253,254 # ...

### Define for specific hardware chip and project
# The key is the virtual pin and the first item in the list is the physical pin.
# followed by a list of capabilities.
PINS = {1 : (1,  D,A,P,S),   2 : (15, D,A,P,S),   # PA0,1
		3 : (16, D,A,P,S),   4 : (17, D,A,P,S),   # PA2,3
		5 : (20, D,A),       6 : (21, D,A,P),     # PA4,5
		7 : (22, D,A,P),     8 : (23, D,A,P),     # PA6,7
		9 : (49, D),         10: (50, D,P),       # PA14,15
		#
		11: (26, D,A,P),     12: (27, D,A,P),     # PB0,1
		13: (55, D,P),       14: (56, D,P),       # PB3,4
		15: (58, D,P),       16: (59, D,P),       # PB6,7
		17: (61, D,P),       18: (62, D,P),       # PB8,9
		19: (29, D,P),       20: (30, D,P),       # PB10,11
		21: (33, D),         22: (34, D,P),       # PB12,13
		23: (35, D,P),       24: (36, D,P),       # PB14,15
		#
		25: (8,  D,A),       26: (9,  D,A),       # PC0,1
		27: (10, D,A),       28: (11, D,A),       # PC2,3
		29: (37, D,P),       30: (38, D,P)        # PC6,7
		}
# for ports map contiguous ports on chip. Use 0 for unused pins (they will be masked)
COMMS = {PORTS: ((1,15,16,17,20,21,22,23),   # PA0-7
				 (26,27,55,56,0,58,59),      # PB0-7
				 (61,62,29,30,33,34,35),     # PB8-15
				 (6,9,10,11,0,0,37,38)    ), # PC0-7
		 I2C: (61,62),   # SCL, SDA (PB8,9)
		 SPI: (34,35,36) # SCLK, MOSI, MISO
		 }
# helper for number of ports needed for DIGITAL_MESSAGE responses
NUM_PORTS = len(PINS)//8 if len(PINS) % 8 == 0 else len(PINS+8)//8
# max number of data bytes in non-Sysex messages
MAX_DATA_BYTES 32

#------------------------------------------------
### Firmata protocol
# The host may send a request capability message which will tell it which pins are exposed
#  and available and for what purpose.
# The host may also request firmware and version info
# V1 enables...
# V2 adds ...
# Then it will send read or write messages (in its protocol) for one pin or port at a time 
# At this time the code will create a pin instance and link it to that pin.
# Also setting up buffers if required.
#  on subsequent calls a simple test confirms this pin is already allocated and 
#  messages pass direct to each pins' class for action.
# Pins/ports set as inputs have a simple interface to read and send data to server if it has changed.

#  - The extended analog message is used for all ANALOG, PWM and SERVO pins over 15

# Possible commands
# host can define:
## two data messages:
# ANALOG_MESSAGE:
#  - uses analog pin 0-15 plus twobytes(14bits) value
#  - set the pin to that value
#  - only arrives for Analog out (PWM, SERVO) ?
# DIGITAL_MESSAGE:
#  - uses port number (0-15)x8pins plus two bytes to give 1byte mask(bits 0-6,7)
#  - arrives when setting outputs
#  - send when sending digital pin data back (as a port)

## four control messages:
# REPORT_ANALOG:
#  - uses an analog pin ref 0-15 plus 0=disable, 1=enable
#  - for each analog pin sets whether to report back or not
#  - not clear if this demands every sampleframe or only when changes or only on demand
# REPORT_DIGITAL:
#  - uses a digital port ref (0-15) plus 0=disable, 1=enable
#  - ports are consecutive sets of 8 pins (0-127)
#  - all pins in port send result (pins set to analog are masked out and report 0)
# SET_PIN_MODE:
#  - uses a digital pin (0-127) and I/O/A/P/S
# REPORT_VERSION:
#  - asks to send back major and minor version to determine level of support 
# SYSTEM_RESET:
#  - asks client to do reset

## numerous sysex messages
# ANALOG_MAPPING_QUERY
#  - ask for mapping of analog to pin numbers
#  - Analog messages are numbered 0 to 15, which traditionally refer to the Arduino pins labeled A0,A1,A2,...
#  - However, these pins are actually configured using "normal" pin numbers in the pin mode message,
#  -  so some are used for non-analog functions. 
#  - The analog mapping query provides information about which pins (as used with Firmata's pin mode message) correspond to the analog channels.
#  - send analog channel corresponding to pin 0, or 127 if pin 0 does not support analog
#         analog channel corresponding to pin 1, or 127 if pin 1 does not support analog,...
# CAPABILITY_QUERY
#  - ask for supported modes and resolution of all pins
#  - requests a list of all modes supported by all pins, and the resolution used by each mode.
#  - send for each pin (start at 0) - mode plus resolution,..., end with 127, next pin...
#  - Each pin has 2 bytes for each supported mode, and a single 127 to mark the end of that pin's data.
# PIN_STATE_QUERY
#  - ask for a pin's current mode and value
#  - allows host to read the current configuration of any pin.
#  - can also be used to verify pin mode settings are received properly.
#  - send pin number(0-127) plus current pin mode plus pins state (0-6) plus (7-13) plus...
#  - The pin "state" is any data written to the pin.
#  - For output modes (digital output, PWM, and Servo), this is the last value written to the pin.
#  - For input modes, typically the state is zero or the status of the pullup resistor. 
# EXTENDED_ANALOG
#  - analog write (PWM, Servo, etc) to any pin
#  - an alternative to the normal analog message, extended version allows addressing beyond pin 15,
#  -  supports sending analog values with any number of bits. The number of data bits is inferred by the length of the message.
#  - receive pin + bits 0-6, bits7-13, ...
# SERVO CONFIG
#  - set minPulse, maxPulse, (i.e. freq)
#  - can be changed at any time LSB + MSB = 14 bits (0-6), (7-13)
#  - receive pin number + minpulse LSB + MSB + maxpulse LSB + MSB
# STRING_DATA a string message with 14-bits per char
# SHIFT_DATA shiftOut config/data message (34 bits)
# I2C_REQUEST
#  - I2C request messages from a host to an I/O board
#  - receive slave addr LSB + MSB and r/w,mode + data0 LSB + MSB + data1...
#  - where read/write and address mode bits are:
#    {7: always 0} + {6: reserved} + {5: address mode, 1 means 10-bit mode} +
#    {4-3: read/write, 00 => write, 01 => read once, 10 => read continuously, 11 => stop reading} +
#    {2-0: slave address MSB in 10-bit mode, not used in 7-bit mode}
#  - Then send back
#  - send slave addr LSB + MSB + register LSB + MSB + data 0 LSB + MSB + data1...
# I2C_CONFIG
#  - Configure special I2C settings such as power pins and delay times
#  - receive Delay in ms LSB + MSB + extra info user defined
#  - needs special implementation possibly for each host
# REPORT_FIRMWARE
#  - report name and version of the firmware
#  - send back major,minor version plus 7bit quantities of file name (firmata.py)
# SAMPLING_INTERVAL 
#  - sets how often analog data and i2c data is reported to the client. The default value is 19 milliseconds. 
#  - receive LSB, MSB of sampling interval in ms
#  - implies there is a maximum sampling interval 65535 ms ?

#
MAX_DATA_BYTES = 32 # max number of data bytes in non-Sysex messages


### helper functions

# used for flashing the pin to display the version number
def strobeLED (count, onInterval, offInterval):
	""" Turn the LED on and off count times,
		- where each on and off interval is also set.
		Intervals in ms.
	"""
	led = pyb.pinMode(VERSION_BLINK_PIN, OUTPUT) # indicate pin is output
	for i in range(count):
		pyb.delay(offInterval);
		#!! how does pyb.LED work ?
		pyb.LED(VERSION_BLINK_PIN, True) # turn it on
		pyb.delay(onInterval)
		pyb.LED(VERSION_BLINK_PIN, False)  # turn it off

def strobe_version():
	" flash the major, minor version number "
	strobeLED(FIRMATA_MAJOR_VERSION, 200, 400)
	pyb.delay(300)
	strobeLED(2,1,4) # separator - quick burst
	pyb.delay(300)
	strobeLED(FIRMATA_MINOR_VERSION, 200, 400)

def compose_two_byte(value):
	""" split an 8 bit value into two seven bit values
		as per MIDI requirements. (high bit denotes a command)
	"""
	return (value & 0b01111111,         # LSB
			(value >> 7) & 0b01111111)  # MSB

def validate_pins_p():
	" check that vpins and physical pins are unique "
	pins = [a[0] for a in PINS.values()]
	pin_counts = [pins.count(i) for i in pins]
	vpins = PINS.keys()
	vpin_counts = [vpins.count(i) for i in vpins]
	#
	if sum(pin_counts) > len(pin_counts) or sum(vpin_counts) > len(vpin_counts):
		# duplicates
		return (False)
	else: return(True)

# !!set of default callbacks if none defined
# I,A - call read, O call write, PWM, SERVO  - call analogwrite
def default_input(firmata, pin, value):
	firmata.active_vpins[pin].report()

###
class Firmata(object):
	""" This class contains the pin classes for each type and manages sampling rate etc """
	## Class constants
	#pin mode values
	# INPUT    = 0x00 # digital pin in input mode
	# OUTPUT   = 0x01 # digital pin in output mode
	# ANALOG   = 0x02 # analog pin in input mode
	# PWM      = 0x03 # digital pin in PWM output mode
	# SERVO    = 0x04 # digital pin in Servo output mode
	# Version info - what we support
	FIRMATA_MAJOR_VERSION = 2
	FIRMATA_MINOR_VERSION = 0
	# Protocol enumerated values
	# message command bytes (128-255/0x80-0xFF)
	DIGITAL_MESSAGE  = 0x90 # send data for a digital pin
	ANALOG_MESSAGE   = 0xE0 # send data for an analog or PWM pin
	REPORT_ANALOG    = 0xC0 # enable analog input by pin #
	REPORT_DIGITAL   = 0xD0 # enable digital input by port pair
	SET_PIN_MODE     = 0xF4 # set a pin to INPUT/OUTPUT/PWM/etc
	REPORT_VERSION   = 0xF9 # report protocol version
	SYSTEM_RESET     = 0xFF # reset from MIDI
	START_SYSEX      = 0xF0 # start a MIDI Sysex message
	END_SYSEX        = 0xF7 # end a MIDI Sysex message
	# extended command set using sysex (0-127/0x00-0x7F)
	# 0x00-0x0F reserved for custom commands
	#RESERVED_COMMAND         = 0x00 # 2nd SysEx data byte is a chip-specific command (AVR, PIC, TI, etc).
	ANALOG_MAPPING_QUERY     = 0x69 # ask for mapping of analog to pin numbers
	#ANALOG_MAPPING_RESPONSE  = 0x6A # reply with mapping info
	CAPABILITY_QUERY         = 0x6B # ask for supported modes and resolution of all pins
	#CAPABILITY_RESPONSE      = 0x6C # reply with supported modes and resolution
	PIN_STATE_QUERY          = 0x6D # ask for a pin's current mode and value
	PIN_STATE_RESPONSE       = 0x6E # reply with a pin's current mode and value
	EXTENDED_ANALOG          = 0x6F # analog write (PWM, Servo, etc) to any pin beyond 15
	SERVO_CONFIG             = 0x70 # set max angle, minPulse, maxPulse, freq
	#STRING_DATA              = 0x71 # a string message with 14-bits per char
	#SHIFT_DATA               = 0x75 # shiftOut config/data message (34 bits)
	#I2C_REQUEST              = 0x76 # I2C request messages from a host to an I/O board
	#I2C_REPLY                = 0x77 # I2C reply messages from an I/O board to a host
	#I2C_CONFIG               = 0x78 # Configure special I2C settings such as power pins and delay times
	REPORT_FIRMWARE          = 0x79 # report name and version of the firmware
	#SAMPLING_INTERVAL        = 0x7A # sampling interval
	#SYSEX_NON_REALTIME       = 0x7E # MIDI Reserved for non-realtime messages
	#SYSEX_REALTIME           = 0x7F # MIDI Reserved for realtime messages
	#
	flattened_pins = PINS.keys() # just the vpins in a list
	
	def __init__(self, serial_port, baudrate=57600, auto_start=True, delay=500):
		self.port = pyb.Serial(serial_port, baudrate, '8N1') # !!we're guessing
		self.analog_apins = {} # grows as analog pins defined.
		self.active_vpins = {} # hold all pin class instances once created
		# shadows not strictly required - could use .keys() but quicker at cost of array
		self.active_vpin_shadow = [] # holds list of active vpins
		self.active_apin_shadow = [] # holds list of active analog pins
		self.system_reset()
		self.system_reset_function = None
		# ivs for parsing state machine
		self.command = 0
		self.parsing_sysex = False #
		self.sysex_bytes_read = 0 #
		self.stored_input_data = [0]*MAX_DATA_BYTES # maximum non-sysex message length buffer
		self.multiByteChannel = 0 # channel data for multiByteCommands
		self.wait_for_data = 0    # this flag says the next serial input will be data
		self.execute_multibyte_command = 0 # execute this after getting multi-byte data
		#
		if auto_start:
			pyb.delay(delay)
			self.port.open() #!! guessing
		# Validate PINS setup
		if not validate_pins_p():
			#!! raise exception 'pins not unique'
			pass

	def system_reset(self):
		""" resets the system state when receiving a 
			SYSTEM_RESET message from the host
		"""
		# discard all pins
		self.active_vpins = {}
		self.active_vpin_shadow = []
		# reset parsing state machine
		self.parsing_sysex = False
		self.sysex_bytes_read = 0
		self.multiByteChannel = 0
		self.wait_for_data = 0
		self.execute_multibyte_command = 0
		for i in range(len(self.stored_input_data)):
			self.stored_input_data[i] = 0
		# also execute users system reset callback function if defined
		if self.system_reset_function:
			self.system_reset_function()

	def set_system_reset_callback(self, function):
		" user can register a function to be called when system is reset "
		self.system_reset_function = function

	def send_message(self, bytes=[]):
		" sends stream of bytes back to host "
		for b in bytes:
			self.port.write(b)

	def send_sysex_message(self, bytes=[]):
		" sysex messages encode all 8 bit values as two 7bit values "
		self.port.write(START_SYSEX)
		for b in bytes:
			self.port.write(b)
		self.port.write(END_SYSEX)
		
##!! read ports - host cares about virtual ports. so can see/set pins en-masse.
# - client cares about local ports so can set pins in single op using bit mask.
# - cannot reconcile these unless we place physical pins in same order as virtual pins for a port.
# what matters in protocol is virtual ports.
# - report using port and set using port are functional interfaces


### child classes for pin modes
	class Firmata_pin(object):
		""" This class is base for 4 pin classes but init does job for all of them
			The base class also handles INPUT, OUTPUT, PWM directly.
			- SERVO has extra instance variables.
			- I2C handled ? !!
		"""
		def __init__(self, parent, vpin, mode, callback=None):
			self.parent = parent # to get Serial port
			self.vpin = vpin     # the virtual pin
			if mode in PINS[vpin][1:]:
				self.last_sent = 0       # to compare current with last sent sample to see if changed
				self.pin = PINS[vpin][0] # the physical pin
				parent.active_pins[vpin] = self
				parent.active_vpin_shadow.append(vpin)
				# add analog pins 
				if mode == ANALOG:
					# calc apin as next highest and add to analog pins
					apin = max(parent.active_apin_shadow) + 1
					self.analog_apins[apin] = self
					self.active_apin_shadow.append(apin)
					self.apin = apin
				self.mode = mode
				self.set_mode(mode)
				# action to perform
				if callback:
					self.callback = callback # will be called on action()
				#
				self.report_fun = foo # !!
			else: # raise 'mode not available' exception
				pass

		def set_callback_name(self, function):
			self.callback = function

		def id(self):
			return self.vpin

		def set_mode(self, mode):
			""" set mode and params to support it
				if existing mode changing then optimise
			"""
			if mode != self.mode:
				# called from outside constructor
				# remove self from list and make new pin object
				self._remove_pin(vpin)
				self._create_new_pin(vpin, mode, callback_name)
			else:
				# set I/O based on mode
				if mode in [INPUT, ANALOG]:
					pyb.set_pin_mode(self.pin, INPUT) # !! check pyb
				else:
					pyb.set_pin_mode(self.pin, OUTPUT) # !! check pyb

		def read(self, only_if_changed=False):
			""" return read value
				- if flag set then only return value if it has changed
			"""
			if self.mode == INPUT or self.mode == ANALOG:
				now = self.pin.read()
				if only_if_changed and now == self.last_sent:
						now = None # !! or False
				return now
			else: # its not readable (SERVO, PWM, OUTPUT) so return currently set value
				return self.last_sent

		# only have pin reports for REPORT_ANALOG
		# digital output pins report entire port of 8 pins ()
		# for all pins, firmata reads back by using PIN_STATE_QUERY (a SYSEX command)
		def report(self, only_if_changed=False, sysex=False):
			" can only respond to a PIN_STATE_QUERY message "
			# The flags only_if_changed and sysex are unused
			value = self.read(only_if_changed)
			if value:
				msg = [f.PIN_STATE_RESPONSE, self.vpin, self.mode]
				msg.extend(compose_two_byte(value))
				self.parent.send_sysex_message(msg)
				self.last_sent = value

		def action(self, *args):
			""" The action to perform for this pin.
				user sets action by adding callback
			"""
			if self.callback:
				if type(self.callback) == type(""):
					# call the callback string as a function:
					# - passing any args to it.
					# - return its return value
						return globals()[self.callback](*args)
				else: # its a function
					return(self.callback(*args))

	class Firmata_analog_pin(Firmata_pin):
		def __init__(self, *args):
			super().__init__(*args) # super(Firmata_analog_pin, self).__init__(*args)
			#!! assign def callback if not supplied

		def report(self, only_if_changed=False, sysex=False):
			""" send host the analog value
				- if flag set then only return value if it has changed
				- use regular or SYSEX message format
			"""
			pin = self.apin
			value = self.read(only_if_changed)
			if value:
				if not(sysex) and pin < 16:
					# regular REPORT_ANALOG response
					msg = [f.ANALOG_MESSAGE | (pin & 0xF)]
					msg.extend(compose_two_byte(value))
					self.parent.send_message(msg)
				elif sysex:
					# send sysex response
					msg = [f.PIN_STATE_RESPONSE, self.vpin, self.mode]
					msg.extend(compose_two_byte(value))
					self.parent.send_sysex_message(msg)

	class Firmata_servo_pin(Firmata_pin):
		def __init__(self, *args):
			super().__init__(*args) # super(Firmata_servo_pin, self).__init__(*args)
			#!! assign def callback if not supplied

		def set_mode(self, mode):
			pyb.set_pin_mode(self.pin, OUTPUT) # !! check pyb
			# add min/max pulse ivs
			self.min_pulse = 0 # ms
			self.max_pulse = 2.5 # ms !!


	# class Firmata_PWM_pin(Firmata_pin):
		# def __init__(self, *args):
			# super().__init__(*args) # super(Firmata_PWM_pin, self).__init__(*args)
			# #!! assign def callback if not supplied
 
 #!! when error (e.g. mode unknown, ) then send sysex string message error


###
	def set_mode_pin(self, vpin, mode, callback=False):
		""" set the virtual pin to the mode defined.
			mode is one of the pin mode values.
		"""
		# check if pin legal
		# check if instance made already and correct mode
		#  - change it if it is
		# else create it and add to active_pins
		create = False
		if vpin in self.flattened_pins:
			# pin is viable
			if vpin in self.active_vpin_shadow:
				# class already defined
				pin_object = self.active_pins[vpin]
				# do we need to change mode
				if pin_object.mode != mode and mode in PINS[vpin][1:]:
					# remove old instance and create new one
					self._remove_pin(vpin)
					create = True
				elif callback and callback != pin_object.callback:
					pin_object.set_callback(callback)
				# reset current value
				pin_object.current = 0
			else: # need new instance
			create = True
		#
		if create:
			self._create_new_pin(vpin, mode, callback_name)

	def _create_new_pin(self, vpin, mode, callback_name):
		" create specific class required "
		if mode == ANALOG:
			Firmata_analog_pin(self, vpin, mode, callback_name)
		# elif mode == PWM:
			# Firmata_PWM_pin(self, vpin, mode, callback_name)
		elif mode == SERVO:
			Firmata_servo_pin(self, vpin, mode, callback_name)
		else:
			Firmata_pin(self, vpin, mode, callback_name)

	def _remove_pin(self, vpin):
		" remove vpin from lists and apin if analog "
		if self.active_pins.count(vpin):
			# vpin in list so remove it
			pin_object = self.active_pins[vpin]
			if hasattr(pin_object, 'apin'): # or check classname ?
				self.analog_apins.remove(apin)
				self.active_apin_shadow.remove(apin)
			self.active_pins.remove(vpin)
			self.active_vpin_shadow.remove(vpin)

	def set_port_pins(self, port_number, mask):
		""" do set_pin_mode(vpin, INPUT) for pins in mask
		"""
		pstart = port_number * 8
		for i in range(0, 7):  
			vpin = pstart + i # !! count down ? pstart + 8 - i
			if mask & (1 << i):
				# that pin is set
				# remove it if already defined as other than an input
				if self.active_pins.count(vpin):
					# vpin in list
					if self.active_pins[vpin].mode != INPUT:
						self._remove_pin(vpin)
				# add it as an input
				self.set_mode_pin(vpin, INPUT) # !! add callback ?
			else: # pin is clear so remove pin if defined as input
				if self.active_pins[vpin].mode == INPUT:
						self._remove_pin(vpin)

###
	def begin(self):
		""" 1. blink version
			2. open serial
			3. send version and firmware version back
		"""
		strobeLED()
		if self.port.status() != 'open': #!! guessing
			self.port.open()
		# report version
		self.report_version()
		# report firmware
		self.report_firmware_version()

	def available(self):
		return self.port.available()

### send functions
	def report_version(self):
		self.send_message([f.REPORT_VERSION, f.FIRMATA_MAJOR_VERSION, f.FIRMATA_MINOR_VERSION])

	def report_firmware_version(self):
		msg = [f.REPORT_FIRMWARE, f.FIRMATA_MAJOR_VERSION, f.FIRMATA_MINOR_VERSION]
		filename = 'firmata.py'
		for c in filename:
			msg.append(c)
		self.send_sysex_message(msg)

	def send_digital_port(self, port_number):
		""" Firmata ports are all based on the virtual pin index and 8 bit ports.
			- there are 15 ports (pins 0-127) in 8 bit chunks
			So we use the vpin to relate to a port.
		"""
		# given a port number - assemble those pins into a byte for sending
		# if none have changed then do not send the byte
		pstart = port_number * 8
		byte = 0
		changed = False
		for i in range(pstart, pstart + 8): # count down =(pstart+7, pstart-1, -1)
			value = 0
			if i in self.active_vpins_shadow:
				pin = self.active_vpins[i]
				value = pin.read(True) # only get result if changed from last sent
				if value:
					changed = True
			# either have a value or not (no change)
			byte |= (1<<i) if value else 0
		# byte is now a packed rep of pins
		# if changed False then NONE of the pins changed and message does not need to be sent
		if changed:
			# DIGITAL_MESSAGE is sent as two bytes (0-6),7 (could use compose_two_byte but unnecessary)
			self.send_message([f.REPORT_DIGITAL, port_number & 0xF, byte & 127, byte >> 7])

	def send_string(self, string):
		""" strings are all sent as two byte sysex messages """
		self.send_sysex_message(f.FIRMATA_STRING, string)

### callbacks
	def attach(self, command, callback_function):
		""" command is one of:
			ANALOG_MESSAGE, DIGITAL_MESSAGE,
			REPORT_ANALOG, REPORT_DIGITAL,
			SET_PIN_MODE, SYSTEM_RESET,
			FIRMATA_STRING, START_SYSEX
			The fallback is START_SYSEX
		"""
		self.callback_functions[command] = callback_function

### process incoming
	def process_sysex_message(self):
		""" sysex message has been received.
			process it here
		"""
		command = self.stored_input_data[0] # first byte in buffer is command
		if command == REPORT_FIRMWARE:
			self.report_firmware_version()
		elif command == STRING_DATA:
			if currentStringCallback:
			buffer = ""
				for i in range(1, (sysex_bytes_read - 1) / 2, 2):
					buffer += self.stored_input_data[i] +  self.stored_input_data[i+1] << 7
				*currentStringCallback)(buffer)
		else:
			if currentSysexCallback:
				*currentSysexCallback(storedInputData[0], sysexBytesRead - 1, storedInputData + 1)
 

	def attachpin(self,):
		""
		pass

	def process_input(self, ):
		""" Listen for incoming messages from host.
			this needs to be non blocking and may not receive a byte each time through.
			Gather info until entire command is collected - then dispatch:
			ANALOG_MESSAGE, DIGITAL_MESSAGE, REPORT_ANALOG, REPORT_DIGITAL, SET_PIN_MODE, REPORT_VERSION, SYSTEM_RESET
			Sysex messages:
			PIN_STATE_QUERY, REPORT_FIRMWARE, SAMPLING_INTERVAL, EXTENDED_ANALOG,
			ANALOG_MAPPING_QUERY, CAPABILITY_QUERY
		"""
		# state machine uses: command, parsing_sysex, sysex_bytes_read, stored_input_data, multiByteChannel, wait_for_data, execute_multibyte_command
		indata = self.port.read() # must be non-bocking
		if indata:
			# process it - else drop out
			if self.parsing_sysex:
				if indata == f.END_SYSEX:
					#stop sysex
					self.parsing_sysex = False
					#fire off handler function
					f.process_sysex_message()
				else:
					#normal data byte - add to buffer
					self.stored_input_data[self.sysex_bytes_read] = indata
					self.sysex_bytes_read += 1
			elif self.wait_for_data > 0 and indata < 128:
				self.wait_for_data -= 1
				self.stored_input_data[self.wait_for_data] = indata
				# do we have complete multibyte command ?
				if self.wait_for_data == 0 and self.execute_multibyte_command: # got the whole message
					if self.execute_multibyte_command == ANALOG_MESSAGE:
						# indicates an analog write message to SERVO or PWM port - two bytes - pin, value
						if currentAnalogCallback:
							(*currentAnalogCallback)(self.multiByteChannel,
												   (self.stored_input_data[0] << 7)
												   + self.stored_input_data[1])
					elif self.execute_multibyte_command == DIGITAL_MESSAGE:
						# indicates to write to output pins defined in port - two bytes - port, pin mask
						if currentDigitalCallback:
							(*currentDigitalCallback)(self.multiByteChannel,
													(self.stored_input_data[0] << 7)
													+ self.stored_input_data[1])
					elif self.execute_multibyte_command == SET_PIN_MODE:
						# indicates to set the pin to that mode. two bytes - pin, mode
						self.set_mode_pin(self.stored_input_data[1], self.stored_input_data[0]) # !! should we add callback too? 
					elif self.execute_multibyte_command == REPORT_ANALOG:
						# indicate that this analog pin is to report back - single byte - value 0-15
						if self.stored_input_data[0]:
							# tell pin to report
							self.set_mode_pin(self.multiByteChannel, ANALOG) # !! should we add callback too? 
						else: # remove it if already registered
							self._remove_pin(self.multiByteChannel)
					elif self.execute_multibyte_command == REPORT_DIGITAL:
						# indicates this port of pins is to report back - single byte - pins in port mask
						self.set_port_pins(self.multiByteChannel, self.stored_input_data[0])
					else: # unimplemented  - don't know what to do with it - skip it :)
						self.execute_multibyte_command = 0
			else:
				#remove channel info from command byte if less than 0xF0
				# command could be local to this block !! test and remove self. if possible
				if indata < 0xF0:
					self.command = indata & 0xF0
					self.multiByteChannel = indata & 0x0F
				else: #commands in the 0xF* range don't use channel data
					self.command = indata
				# update progress or execute one byte commands
				if self.command == SET_PIN_MODE:
					self.wait_for_data = 2 # two data bytes needed
					self.execute_multibyte_command = self.command
				elif self.command == REPORT_DIGITAL:
					self.wait_for_data = 1 # one data byte needed
					self.execute_multibyte_command = self.command
				elif self.command == START_SYSEX:
					self.parsing_sysex = True
					self.sysex_bytes_read = 0
				elif self.command == SYSTEM_RESET:
					self.system_reset()
				elif self.command == REPORT_VERSION:
					self.report_version()




###-----------------------------
### Usage:

### Define main class
# f = Firmata(port, baud) # port is a pyb.Serial instance, baud defaults to 57600
# f = Firmata(port, baud, False, 3000) # do not autostart, set 3sec timeout

### Define new pins, modes and optionally callbacks
### - callbacks can be added later. They are existing functions
# f.set_mode_pin(1,  ANALOG)  # Analog input pin (no callback)
# f.set_mode_pin(1,  ANALOG, read_analog)  # Analog input pin (read_analog is a function)
# f.set_mode_pin(2,  INPUT,  read_digital) # Digital input pin (read_digital is a function)
# f.set_mode_pin(3,  OUTPUT, write_output) # Digital output pin (write_output is a function)
# f.set_mode_pin(12, PWM,    drive_PWM)    # PWM output pin (drive_PWM is a function)
# f.set_mode_pin(4,  SERVO,  servo_device) # Servo output pin (servo_device is a function)
### Ports - virtual port of pins used as Input or Output
# ports are only read and defined in COMMS variable
# f.read_port(0) # returns 1 byte of packed input pins (masked for non-INPUT pins)

### Change mode of existing pin
# f.active_vpins[4].set_mode(PWM)
### Change/Set callback for existing pin
# f.active_vpins[2].set_callback_name(input_action) # where input_action is a function or string
### Redefine an existing pin to a new mode or callback
# f.set_mode_pin(1, ANALOG, read_analog) # where read_analog is a function
### Read value from a pin (if I,A then get new value - else get value previously set)
# f.active_vpins[1].read() # for pin 1 (ANALOG)
# f.active_vpins[2].read() # for pin 2 (DIGITAL INPUT)
### Read but only if changed - returns None if unchanged
# f.active_vpins[1].read(True) # for pin 1 (ANALOG)
### Report read values to Host
# f.active_vpins[1].report() # for pin 1 (ANALOG)
### Report value to host only if changed
# f.active_vpins[1].report(True) # for pin 1 (ANALOG)
### Report only changed values of all in-use pins to host
# for p in f.active_vpins.values():
#	p.report(True)
### Report only changed values of all DIGITAL pins
# for pid in range(NUM_PORTS):
#	f.send_digital_port(pid)
## Report changed values of 1st 15 Analog pins
# for apin in f.analog_apins():
#	apin.report(True)
## Report changed values of all Analog pins (using sysex msg)
# for apin in f.analog_apins():
#	apin.report(True, True)
### or one at a time inside a loop
# apin = 0
#	f.analog_apins[apin].report(True)
#	apin += 1
#	if apin > len(f.analog_apins):
#		apin = 0
### Perform callback function directly
# f.active_vpins[4].action() # for pin 4 (SERVO)

## if an analog or digital out msg arrives before pin mode is set then a virtual pin mode command is performed first



