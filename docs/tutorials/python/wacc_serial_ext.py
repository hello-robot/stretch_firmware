#! /usr/bin/env python

from stretch_body.wacc import *
from stretch_body.transport import *


class WaccSerialExt(Wacc):
    """
    This class demonstrates how to extend the Wacc class with custom data
    See the corresponding tutorial for more information.
    """
    def __init__(self,verbose=False):
        Wacc.__init__(self,verbose=verbose,
                      ext_status_cb=self.ext_unpack_status, #Set callback to unpack status
                      ext_command_cb=self.ext_pack_command) #Set callback to pack command
        self.valid_firmware_protocol = 'p99' #Ensure the custom protocol identifier matches that in the Common.h of the custom Wacc Serial firmware
        self.n_float=10
        self._command['serial_ext']=range(self.n_float)   #Extend command dictionary with custom fields
        self.status['serial_ext'] =[0.0]*self.n_float     #Extend status dictionary with custom fields

    def serial_data_increment(self):
        for i in range(self.n_float):
            self._command['serial_ext'][i] =float(self._command['serial_ext'][i]+1)
        self._dirty_command=True

    def pretty_print(self):
        Wacc.pretty_print(self)
        print('Serial Ext:',self.status['serial_ext'])

    def ext_unpack_status(self,s):
        """
        s: byte array to unpack
        return: number of bytes unpacked
        """
        sidx=0
        for i in range(self.n_float):
            self.status['serial_ext'][i] = unpack_float_t(s[sidx:])
            sidx+=4
        return sidx

    def ext_pack_command(self,s,sidx):
        """
        s: byte array to pack in to
        sidx: index to start packing at
        return: new sidx
        """
        for i in range(self.n_float):
            pack_float_t(s, sidx, self._command['serial_ext'][i])
            sidx += 4
        return sidx

