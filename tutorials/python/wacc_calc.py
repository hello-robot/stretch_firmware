#! /usr/bin/env python

from stretch_body.wacc import *
from stretch_body.transport import *


class WaccCalc(Wacc):
    """
    This class demonstrates how to extend the Wacc class with custom data
    See the corresponding tutorial for more information.
    """
    def __init__(self,verbose=False):
        Wacc.__init__(self,verbose=verbose,
                      ext_status_cb=self.ext_unpack_status, #Set callback to unpack status
                      ext_command_cb=self.ext_pack_command) #Set callback to pack command

        self._command['calc']={'op':0,'var1':0,'var2':0} #Extend command dictionary with custom fields
        self.status['calc'] =0.0                         #Extend status dictionary with custom fields
        self.valid_firmware_protocol = ‘p99’

    def calculate(self,op,var1,var2):
        """
        0: addition
        1: multiplication
        2: division
        """
        self._command['calc']['op'] =int(op)
        self._command['calc']['var1']=float(var1)
        self._command['calc']['var2']=float(var2)
        self._dirty_command=True

    def pretty_print(self):
        Wacc.pretty_print(self)
        print 'Calc:',self.status['calc']

    def ext_unpack_status(self,s):
        """
        s: byte array to unpack
        return: number of bytes unpacked
        """
        sidx=0
        self.status['calc'] = unpack_float_t(s[sidx:])
        return 4

    def ext_pack_command(self,s,sidx):
        """
        s: byte array to pack in to
        sidx: index to start packing at
        return: new sidx
        """
        pack_float_t(s, sidx, self._command['calc']['var1'])
        sidx += 4
        pack_float_t(s, sidx, self._command['calc']['var2'])
        sidx += 4
        pack_uint8_t(s, sidx, self._command['calc']['op'])
        sidx += 1
        return sidx






