#!/usr/env python3
# -*- encoding: utf-8 -*-

# ======================================================================
# LSST-VRO / StarDICE
#
# Low level control for the Primalucelab Sesto Senso 2 focuser
# Python3 minimal driver based on indilib 3rd party drivers available at :
# https://github.com/indilib/indi/blob/master/drivers/focuser/sestosenso2.cpp
# ======================================================================

# ======================================================================
# NOTES :
# Je ne sais pas ce que fait initCalibrations, mais la calibration est censée suivre ces étapes :
# 1/ s'assurer que le motor est en HOLD: set_motorHold(True); cela permet de manipuler le focus à la main
# 2/ placer le focus à son minimum (complètement rentré), et exécuter storeAsMinPosition()
# 3/ placer le focus à son maximum (complètement sorti), et exécuter storeAsMaxPosition()

# def get_hallSensor(self):
#     return self.send('get', 'MOT1')['HSENDET']
# ======================================================================
# Authors: J. Cohen-Tanugi, K. Sommer
# Email: <johann.cohen-tanugi@umontpellier.fr>, <kelian.sommer@umontpellier.fr>
# ======================================================================

import serial
import json
import time

class SS2_Focuser(object):
    """
    Class to control the Primalucelab Sesto Senso 2 focuser
    Interface implemented based on INDI driver :
    https://github.com/indilib/indi/blob/master/drivers/focuser/sestosenso2.cpp
    """
    
    def __init__(self, port = '/dev/ttyUSB0', baudrate=115200, timeout=1, debug = False):
        """
        Parameters
        ----------
        port : str
            Port to connect to the focuser (default = '/dev/ttyUSB0')
        baudrate : int
            Baudrate speed for communication in serial (default = 115200)
        timeout : int
            Timeout for response in seconds (default = 1)
        debug : bool
            If True, print additional information (get and set commands in raw format)
        """
        
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        self.debug = debug
        
        if not self.ser.is_open:
            raise Exception("Device not open, check port!")
        else:
            self.print_info()

    def print_info(self):
        """Get and display information about the focuser model"""
        
        SN = self.get_serialNumber()
        SW = self.get_firmwareVersion()
        MOT1 = self.get_motorInfo()
        T = MOT1['NTC_T']

        print("============SESTO SENSO2 FOCUSER============")
        print("Serial Number : %s"%SN)
        print("Firmware Version : %s"%SW)
        print("Motor Temperature : %s °C"%T)
        print("Status Motor HOLD : %s"%MOT1['HOLDCURR_STATUS'])#0 means not on HOLD, so ACTIVE
        print("")
        print("Status : ")
        for key, val in MOT1['STATUS'].items():
            print("%s : %s"%(key, val))
        print("")
        print("Calibration Settings : ")
        for key, val in MOT1.items():
            if 'CAL' in key:
                print("%s : %s"%(key, val))
        print("")
        print("Operating currents (values from 0 to 10):")
        print("--- Acceleration : %s"%MOT1['FnRUN_CURR_ACC'])
        print("--- Speed        : %s"%MOT1['FnRUN_CURR_SPD'])
        print("--- Deceleration : %s"%MOT1['FnRUN_CURR_DEC'])
        print("--- HOLD         : %s"%MOT1['FnRUN_CURR_HOLD'])
        print("Speed parameters (values from 0 to 10)")
        print("--- Acceleration : %s"%MOT1['FnRUN_ACC'])
        print("--- Speed        : %s"%MOT1['FnRUN_SPD'])
        print("--- Deceleration : %s"%MOT1['FnRUN_DEC'])
        
        self.INFO = MOT1
        
    def close(self):
        """Close serial communication with the device"""
        self.ser.close()

    def _write_to_dev(self, req_str: str):
        """Low-level method to write to the device
        
        Parameters
        ----------
        req_str : str
            Command to write to the device

        Returns
        -------
        retval : str
            Device response
        """
        
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        retval = self.ser.write(str.encode(req_str))
        if retval != len(req_str):
            raise Exception("wrong returned length %d!=%d. Write to device likely failed"%(retval,len(req_str)))
        return retval
    
    def _read_from_dev(self, req_typ, req_cmd, cmd_action=''):
        """Low-level method to read from the device

        Parameters
        ----------
        req_typ : str
            Command to write to the device
        req_cmd : str
            Sub-command to write to the device
        cmd_action : str
            Action command to send to the device

        Returns
        -------
        dict or int or float
            
        """
        
        res = b''
        while res == b'':
            res = self.ser.read(1024)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        try:
            if cmd_action == '':
                return json.loads(res.decode('ascii'))['res'][req_typ][req_cmd]
            else:
                return json.loads(res.decode('ascii'))['res'][req_typ][req_cmd][cmd_action]
        except:
           raise Exception('Parsing read string failed:%s'%res.decode('utf-8'))
        
    def send(self, req_typ, req_cmd, cmd_action='', cmd_attr='', blocking=True, wait=0.001):
        """High-level method to send instruction to the device

        Parameters
        ----------
        req_typ : str
            Command to write to the device
        req_cmd : str
            Sub-command to write to the device
        cmd_action : str
            Action command
        cmd_attr : str
            Additional parameter to send (e.g steps to go)
        blocking : bool
            If True, block the focuser after sending command
        wait : float
            Wait for this time after sending the command

        Returns
        -------
        res : str
            Device response

        """
        
        if req_typ == 'get':
            req_str = '{"req":{"%s":{"%s":""}}}'%(req_typ, req_cmd)
        if req_typ == 'cmd':
            req_str = '{"req":{"%s":{"%s":{"%s":%s}}}}'%(req_typ, req_cmd, cmd_action, cmd_attr)
        
        self._write_to_dev(req_str)
        if self.debug == True:
            print('SS2 FOCUSER : SEND : [' + req_str + ']')
        time.sleep(wait)
        if blocking:
            res = self._read_from_dev(req_typ, req_cmd, cmd_action)
            if self.debug == True:
                print('SS2 FOCUSER : RECEIVED : [' + str(res) + ']')
            return res
    
    def get_serialNumber(self):
        """Get SN number of the focuser"""
        return self.send('get', 'SN')

    def get_firmwareVersion(self):
        """Get firmware version of the focuser"""
        return self.send('get', 'SWVERS')['SWAPP']

    def get_maxPosition(self):
        """Get maximum achievable position (extended = focuser out) defined by user calibration process"""
        return self.send('get', 'MOT1')['CAL_MAXPOS']

    def get_motorInfo(self):
        """Get all information on the focuser in a dict"""
        return self.send('get', 'MOT1')

    def get_externalTemp(self):
        """Get external temperature from sensor if connected (unavailable for now)"""
        return self.send('get', 'EXT_T')

    def get_absPosition(self):
        """Get current absolute position in step"""
        return self.send('get', 'MOT1')['ABS_POS']
    
    def get_currentSpeed(self):
        """Get current moving speed; returns 0 if the motor isn't moving"""
        return self.send('get', 'MOT1')['SPEED']
    
    def get_motorTemp(self):
        """Get motor temperature from embedded sensor"""
        return self.send('get', 'MOT1')['NTC_T']
    
    def get_voltageIn(self):
        """Get current voltage for powering the focuser"""
        return self.send('get', 'VIN_12V')

    def get_motorUserPreset(self, user_index: int):
        """Get user preset in a dict"""
        return self.send('get', 'RUNPRESET_%u'%user_index)

    def set_motorUserPreset(self, user_index, accRate, decRate, runSpeed, accCurr, decCurr, runCurr, holdCurr):
        """Set custom user preset in a dict for acceleration, speed and hold mode. Storing multiple presets is possible.
        According to the INDI driver, expected values are in the 1-10 range, except the CURR_HOLD in the 1-5 range.

        Parameters
        ----------
        user_index : int
            Index of user setting (1-3)
        accRate : int
            Level of acceleration (1-10)
        decRate : int
            Level of deceleration (1-10)
        runSpeed : int
            Speed for movement (1-10)
        accCurr : int
            Power level for acceleration (1-10)
        decCurr : int
            Power level for deceleration (1-10)
        runCurr : int
            Power level for constant speed movement (1-10)
        holdCurr : int
            Power level for holding the focuser at a stable position (1-5)
        """
        
        req_str = '{"req":{"set":{"RUNPRESET_%u":{"RP_NAME":"User %u", "M1ACC":%u, "M1DEC":%u,"M1SPD":%u, "M1CACC":%u, "M1CDEC":%u, "M1CSPD":%u, "M1HOLD":%u""}}}}'% \
                                                  (user_index, user_index, accRate, decRate, runSpeed, accCurr, decCurr, runCurr, holdCurr)
        retval = self._write_to_dev(req_str)
        time.sleep(1)
        ret_dict = self._read_from_dev('set', 'RUNPRESET_%u'%user_index)
        for key,val in retstr['res']['set']['RUNPRESET_2'].items():
            if val != 'done':
                raise Exception('set action not completed : %s status is %s'%(key, val))

    def set_motorRates(self, accRate, runSpeed, decRate):
        """Set motor rates : acceleration, speed and deceleration.

        Parameters
        ----------
        accRate : int
            Level of acceleration (1-10)
        decRate : int
            Level of deceleration (1-10)
        runSpeed : int
        """

        req_str = '{"req":{"set":{"MOT1":{"FnRUN_ACC":%u,"FnRUN_SPD":%u,"FnRUN_DEC":%u}}}}'%\
            (accRate, runSpeed, decRate)
        self._write_to_dev(req_str)
        for key in ["FnRUN_ACC", "FnRUN_SPD", "FnRUN_DEC"]:
            if retdict[key] != 'done':
                raise Exception('set action not completed: %s'%retval)
        
    def set_motorCurrents(self, accCurrent, runCurrent, decCurrent, holdCurrent):
        """Set power level for acceleration, speed, deceleration and hold.
        According to the INDI driver, expected values are in the 1-10 range, except the CURR_HOLD in the 1-5 range

        Parameters
        ----------
        accCurr : int
            Power level for acceleration (1-10)
        decCurr : int
            Power level for deceleration (1-10)
        runCurr : int
            Power level for constant speed movement (1-10)
        holdCurr : int
            Power level for holding the focuser at a stable position (1-10)
        """
        req_str = '{"req":{"set":{"MOT1":{"FnRUN_CURR_ACC":%u,"FnRUN_CURR_SPD":%u,"FnRUN_CURR_DEC":%u,"FnRUN_CURR_HOLD":%u}}}}'% \
            (accCurrent, runCurrent, decCurrent, holdCurrent)
        self._write_to_dev(req_str)
        retdict = self._read_from_dev('set', 'MOT1')
        for key in ["FnRUN_CURR_ACC", "FnRUN_CURR_SPD", "FnRUN_CURR_DEC", "FnRUN_CURR_HOLD"]:
            if retdict[key] != 'done':
                raise Exception('set action not completed: %s'%retval)
        
    def set_motorHold(self, hold: bool):
        """Set motor holding mode (if True : draws current to maintain position)

        Parameters
        ----------
        hold : bool
            If True, draws current to maintain position
        """
        req_str = '{\"req\":{\"set\":{\"MOT1\":{\"HOLDCURR_STATUS\":%u}}}}'%int(hold)
        retval = self._write_to_dev(req_str)
        time.sleep(0.001)
        retval = self._read_from_dev('set', 'MOT1', 'HOLDCURR_STATUS')
        if not  retval == 'done':
            raise Exception('set action not completed: %s'%retval)
            
    def apply_motorPreset(self, preset: str):
        """Apply default preset to the focuser
        
        Parameters
        ----------
        preset : str
            Default preset for speed and acceleration
        """
        
        if preset not in ['light', 'medium', 'slow']:
            raise Exception('user parameter must be "light", "medium", or "slow"')
        req_str = '{"req":{"cmd":{"RUNPRESET":"%s"}}}'%preset
        self._write_to_dev(req_str)
        time.sleep(0.01)
        retval = self._read_from_dev('cmd', 'RUN_PRESET', 'CAL_FOCUSER')
        if retval != 'done':
            raise Exception("Applying user preset values failed: %s"%retval)
        
    def abort(self, blocking=True):
        """Abort current operation
        
        Parameters
        ----------
        blocking : bool
            If True, block the focuser after sending command
        """
        print("SS2 FOCUSER : ABORT")
        return self.send('cmd', 'MOT1', cmd_action='MOT_ABORT', cmd_attr='""', blocking=blocking)

    def go(self, targetTicks, blocking=False):
        """Move the motor to the defined step position
        
        Parameters
        ----------
        targetTicks : int
            Position to go to
        blocking : bool
            If True, block the focuser after sending command
        """
        print("SS2 FOCUSER : GO TO {}".format(targetTicks))
        return self.send('cmd', 'MOT1', cmd_action='GOTO', cmd_attr=str(targetTicks), blocking=blocking)
    
    def stop(self, blocking=True):
        """Stop the movement of the motor
        
        Parameters
        ----------
        blocking : bool
            If True, block the focuser after sending command
        """
        print("SS2 FOCUSER : STOP")
        return self.send('cmd', 'MOT1', cmd_action='MOT_STOP', cmd_attr='""', blocking=blocking)
    
    def gohome(self, blocking=False):
        """Move the motor to the home position (retracted)
        
        Parameters
        ----------
        blocking : bool
            If True, block the focuser after sending command
        """
        self.send('cmd', 'MOT1', cmd_action='GOHOME', cmd_attr='""', blocking=blocking)
        print("SS2 FOCUSER : GO HOME...")
    
    def fastMoveIn(self, blocking=False):
        """Retract the motor at high speed
        
        Parameters
        ----------
        blocking : bool
            If True, block the focuser after sending command
        """
        return self.send('cmd', 'MOT1', cmd_action='F_INW', cmd_attr='""', blocking=blocking)

    def initCalibration(self):
        """Init calibration process"""
        return self.send('cmd', 'MOT1', 'CAL_FOCUSER', '"Init"')

    def storeAsMinPosition(self):
        """Store current position as fully retracted"""
        return self.send('cmd', 'MOT1', 'CAL_FOCUSER', '"StoreAsMinPos"')

    def storeAsMaxPosition(self):
        """Store current position as fully extended"""
        return self.send('cmd', 'MOT1', 'CAL_FOCUSER', '"StoreAsMaxPos"')

    def goOutToFindMaxPos(self):
        """Fast move out to reach maximum position with the motor. NEED to be stopped by hand (executing stop()). Not recommended!"""
        return self.send('cmd', 'MOT1', 'CAL_FOCUSER', '"GoOutToFindMaxPos"')

    def fastMoveOut(self, blocking=False):
        """Fast move out to reach maximum position with the motor. NEED to be stopped by hand (executing stop()). Not recommended!
        INDI uses goOutToFindMaxPos rather than fastMoveOut"""
        return self.send('cmd', 'MOT1', cmd_action='F_OUTW', cmd_attr='""', blocking=blocking)
