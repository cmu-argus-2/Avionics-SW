from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_register.i2c_bit import ROBit, RWBit
from adafruit_register.i2c_bits import RWBits
from hal.drivers.middleware.errors import Errors
from hal.drivers.middleware.generic_driver import Driver
from micropython import const

# DEVICE REGISTER MAP
_FAULT_STATUS = const(0x00)   # Fault Status Register R-
_RC_STATUS1   = const(0x01)   # Motor Speed status R-
_REG_STATUS1  = const(0x04)   # Output voltage across motor R-
_REG_STATUS2  = const(0x05)   # Current flowing through motor R-
_REG_STATUS3  = const(0x06)   # PWM Duty Cycle R-
_CONFIG0      = const(0x09)   # RW
_CONFIG1      = const(0x0A)   # RW
_CONFIG2      = const(0x0B)   # RW
_CONFIG3      = const(0x0C)   # RW
_CONFIG4      = const(0x0D)   # RW
_REG_CTRL0    = const(0x0E)   # RW
_REG_CTRL1    = const(0x0F)   # RW
_REG_CTRL2    = const(0x10)   # RW
_RC_CTRL2     = const(0x13)   # RW
_RC_CTRL3     = const(0x14)   # RW
_RC_CTRL4     = const(0x15)   # RW
_RC_CTRL7     = const(0x18)   # RW
_RC_CTRL8     = const(0x19)   # RW



class VoltageAdapter:
    """Output voltage calculator."""

    def index_to_voltage(self, index):
        """Convert an index value to nearest voltage value."""
        return index * (42.67/255)

    def voltage_to_index(self, volts):
        """Convert a voltage to nearest index value."""
        return volts * (255/42.67)


class BridgeControl:
    """H-bridge PWM control states and descriptors. Bit order: IN2 IN1"""

    COAST   = 0b00  # Standby/Coast function (Hi-Z)
    REVERSE = 0b01  # Reverse function
    FORWARD = 0b10  # Forward function
    BRAKE   = 0b11  # Brake function

    DESCRIPTOR = ["COAST", "REVERSE", "FORWARD", "BRAKE"]


class Faults:
    """Fault Register Flag Descriptors
    FAULT  Any fault condition
    STALL  Stall event;
           device disabled, clear fault to reactivate
    OCP    Overcurrent event;
           device disabled, clear fault to reactivate
    OVP    Overvoltage event
    TSD    Overtemperature condition;
           device disabled, resumes with lower temperature
    NPOR   Undervoltage lockout; device disabled,
           resumes with voltage restoration
    """

    DESCRIPTOR = ["FAULT", "OCP", "OVP", "UVLO", "TSD"]


class DRV8235(Driver):
    """DC motor driver with I2C interface. 

    :param i2c_bus: The microcontroller I2C interface bus pins.
    :param address: The I2C address of the DRV8235 motor controller."""

    def __init__(self, i2c_bus, address=0x60):
        """Instantiate DRV8235. Set output voltage to 0.0, place into STANDBY
        mode, and reset all fault status flags."""
        self.i2c_device = I2CDevice(i2c_bus, address)
        self._i2c_bc = True
        self._pmode = True
        self._set_dir = BridgeControl.COAST
        self._reg_ctrl = 0x2 # Sets to voltage regulation
        # Clear all fault status flags
        self.clear_faults()

        super().__init__()

    # DEFINE I2C DEVICE BITS, NYBBLES, BYTES, AND REGISTERS
    _clear = RWBit(_CONFIG0, 1, 1, False)  # Clears fault status flag bits
    _i2c_bc = RWBit(_CONFIG4, 2, 1, False) # Sets Bridge Control to I2C
    _pmode = RWBit(_CONFIG4, 3, 1, False) # Sets programming mode to PWM
    _set_dir = RWBits (2, _CONFIG4, 0, 1, False) # Sets direction of h-bridge IN1, IN2
    _reg_ctrl = RWBits (2, _REG_CTRL0, 3, 1, False) # Sets current/voltage regulation scheme

    # _vset = RWBits(6, _CONTROL, 2, 1, False)  # DAC output voltage (raw)
    # _fault = ROBit(_FAULT, 0, 1, False)  # Any fault condition
    # _ocp = ROBit(_FAULT, 1, 1, False)  # Overcurrent event
    # _uvlo = ROBit(_FAULT, 2, 1, False)  # Undervoltage lockout
    # _ots = ROBit(_FAULT, 3, 1, False)  # Overtemperature condition
    # _ilimit = ROBit(_FAULT, 4, 1, False)  # Extended current limit event
    
    def clear_faults(self):
        """Clears all fault conditions."""
        self._clear = True  # Clear all fault status flags

    # def throttle(self):
    #     """Current motor speed, ranging from -1.0 (full speed reverse) to
    #     +1.0 (full speed forward), or ``None`` (controller off). If ``None``,
    #     the H-bridge is set to high-impedance (coasting). If ``0.0``, the
    #     H-bridge is set to cause braking."""
    #     if self.bridge_control[0] == BridgeControl.COAST:
    #         return None
    #     if self.bridge_control[0] == BridgeControl.BRAKE:
    #         return 0.0
    #     if self.bridge_control[0] == BridgeControl.REVERSE:
    #         return -1 * round(self._vset / 0x3F, 3)
    #     return round(self._vset / 0x3F, 3)

    # def set_throttle(self, new_throttle):
    #     if new_throttle is None:
    #         self._vset = 0
    #         self._in_x = BridgeControl.COAST
    #         return
    #     # Constrain throttle value
    #     self._throttle_normalized = min(max(new_throttle, -1.0), +1.0)
    #     if new_throttle < 0:
    #         self._vset = int(abs(new_throttle * 0x3F))
    #         self._in_x = BridgeControl.REVERSE
    #     elif new_throttle > 0:
    #         self._vset = int(new_throttle * 0x3F)
    #         self._in_x = BridgeControl.FORWARD
    #     else:
    #         self._vset = 0
    #         self._in_x = BridgeControl.BRAKE
    #     return

    # def throttle_volts(self):
    #     """Current motor speed, ranging from -5.06 volts (full speed reverse) to
    #     +5.06 volts (full speed forward), or ``None`` (controller off). If ``None``,
    #     the H-bridge is set to high-impedance (coasting). If ``0.0``, the
    #     H-bridge is set to cause braking."""
    #     if self.bridge_control[0] == BridgeControl.COAST:
    #         return None
    #     if self.bridge_control[0] == BridgeControl.BRAKE:
    #         return 0.0
    #     if self.bridge_control[0] == BridgeControl.REVERSE:
    #         return -1 * VoltageAdapter.index_to_voltage(self, self._vset)
    #     return VoltageAdapter.index_to_voltage(self, self._vset)

    # def set_throttle_volts(self, new_throttle_volts):
    #     if new_throttle_volts is None:
    #         self._vset = 0
    #         self._in_x = BridgeControl.COAST
    #         return
    #     # Constrain throttle voltage value
    #     new_throttle_volts = min(max(new_throttle_volts, -5.1), +5.1)
    #     if new_throttle_volts < 0:
    #         self._vset = VoltageAdapter.voltage_to_index(self, abs(new_throttle_volts))
    #         self._in_x = BridgeControl.REVERSE
    #     elif new_throttle_volts > 0:
    #         self._vset = VoltageAdapter.voltage_to_index(self, new_throttle_volts)
    #         self._in_x = BridgeControl.FORWARD
    #     else:
    #         self._vset = 0
    #         self._in_x = BridgeControl.BRAKE
    #     return

    # def throttle_raw(self):
    #     """Current motor speed, 6-bit VSET byte value, ranging from -63 (full speed reverse) to
    #     63 (full speed forward), or ``None`` (controller off). If ``None``,
    #     the H-bridge is set to high-impedance (coasting). If ``0``, the
    #     H-bridge is set to cause braking."""
    #     if self.bridge_control[0] == BridgeControl.COAST:
    #         return None
    #     if self.bridge_control[0] == BridgeControl.BRAKE:
    #         return 0
    #     if self.bridge_control[0] == BridgeControl.REVERSE:
    #         return -1 * self._vset
    #     return self._vset

    # def set_throttle_raw(self, new_throttle_raw):
    #     if new_throttle_raw is None:
    #         self._vset = 0
    #         self._in_x = BridgeControl.COAST
    #         return
    #     # Constrain raw throttle value
    #     new_throttle_raw = min(max(new_throttle_raw, -63), 63)
    #     if new_throttle_raw < 0:
    #         self._vset = new_throttle_raw
    #         self._in_x = BridgeControl.REVERSE
    #     elif new_throttle_raw > 0:
    #         self._vset = new_throttle_raw
    #         self._in_x = BridgeControl.FORWARD
    #     else:
    #         self._vset = 0
    #         self._in_x = BridgeControl.BRAKE
    #     return

    # @property
    # def bridge_control(self):
    #     """Motor driver bridge status. Returns the 2-bit bridge control integer
    #     value and corresponding description string."""
    #     return self._in_x, BridgeControl.DESCRIPTOR[self._in_x]

    # @property
    # def fault(self):
    #     """Motor driver fault register status. Returns state of FAULT flag and
    #     a list of activated fault flag descriptors. FAULT flag is ``True`` if
    #     one or more fault register flags are ``True``."""
    #     faults = []
    #     if self._fault:
    #         faults.append(Faults.DESCRIPTOR[0])
    #         if self._ocp:
    #             faults.append(Faults.DESCRIPTOR[1])
    #         if self._uvlo:
    #             faults.append(Faults.DESCRIPTOR[2])
    #         if self._ots:
    #             faults.append(Faults.DESCRIPTOR[3])
    #         if self._ilimit:
    #             faults.append(Faults.DESCRIPTOR[4])
    #     return self._fault, faults

    # def __enter__(self):
    #     return self

    # def __exit__(self, exception_type, exception_value, traceback):
    #     self._vset = 0
    #     self._in_x = BridgeControl.STANDBY

    # """
    # ----------------------- HANDLER METHODS -----------------------
    # """

    # @property
    # def get_flags(self):
    #     flags = {}
    #     if self._fault:
    #         if self._ocp:
    #             flags["ocp"] = None
    #         if self._uvlo:
    #             flags["uvlo"] = None
    #         if self._ots:
    #             flags["ots"] = None
    #         if self._ilimit:
    #             flags["ilimit"] = None
    #     return flags

    # ######################### DIAGNOSTICS #########################
    # def __check_for_faults(self) -> list[int]:
    #     """_check_for_faults: Checks for any device faluts returned by fault function in DRV8830

    #     :return: List of errors that exist in the fault register
    #     """
    #     faults_flag, faults = self.fault

    #     if not faults_flag:
    #         return [Errors.NOERROR]

    #     errors: list[int] = []

    #     if "OCP" in faults:
    #         errors.append(Errors.DRV8830_OVERCURRENT_EVENT)
    #     if "UVLO" in faults:
    #         errors.append(Errors.DRV8830_UNDERVOLTAGE_LOCKOUT)
    #     if "OTS" in faults:
    #         errors.append(Errors.DRV8830_OVERTEMPERATURE_CONDITION)
    #     if "ILIMIT" in faults:
    #         errors.append(Errors.DRV8830_EXTENDED_CURRENT_LIMIT_EVENT)

    #     self.clear_faults()

    #     return errors

    # def __throttle_tests(self) -> int:
    #     """_throttle_tests: Checks for any throttle errors in DRV8830, whether the returned reading is
    #     outside of the set range indicated in the driver file

    #     :return: true if test passes, false if fails
    #     """
    #     throttle_volts_val = self.throttle_volts()
    #     if throttle_volts_val is not None:
    #         if (throttle_volts_val < -5.06) or (throttle_volts_val > 5.06):
    #             return Errors.DRV8830_THROTTLE_OUTSIDE_RANGE

    #     throttle_raw_val = self.throttle_raw()
    #     if throttle_raw_val is not None:
    #         if (throttle_raw_val < -63) or (throttle_raw_val > 63):
    #             return Errors.DRV8830_THROTTLE_OUTSIDE_RANGE

    #     return Errors.NOERROR

    # def run_diagnostics(self) -> list[int] | None:
    #     """run_diagnostic_test: Run all tests for the component"""
    #     error_list: list[int] = []

    #     error_list = self.__check_for_faults()
    #     error_list.append(self.__throttle_tests())

    #     error_list = list(set(error_list))

    #     if Errors.NOERROR not in error_list:
    #         self.errors_present = True

    #     return error_list 
