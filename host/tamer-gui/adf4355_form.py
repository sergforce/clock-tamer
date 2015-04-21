#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import time
import re

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtXml import *

from adf4355_ui import *
from adf4355_regs import *

class Adf4355(QtGui.QDialog):
    def __init__(self):
        QtGui.QWidget.__init__(self)
        self.obj = Ui_Dialog()
        self.obj.setupUi(self)
        
        self.obj.r3_uis = [ self.obj.r3_sd_load, self.obj.r3_phase_rsync, self.obj.r3_phase_adj, self.obj.r3_phase ]
        for i in self.obj.r3_uis:
            try:    i.currentIndexChanged['QString'].connect(self.r3_changed)
            except: i.valueChanged['QString'].connect(self.r3_changed)
        

    def r3_changed(self):
        reg = (((self.obj.r3_sd_load.currentIndex() & 1) << REG3_SD_LOAD_SHIFT) |
               ((self.obj.r3_phase_rsync.currentIndex() & 1) << REG3_PHASE_RSYNC_SHIFT) |
               ((self.obj.r3_phase_adj.currentIndex() & 1) << REG3_PHASE_ADJ_SHIFT) |
               ((self.obj.r3_phase.value() & REG3_PHASE_MASK) << REG3_PHASE_SHIFT) | 3)
        self.obj.reg3.setText("x%08x" % reg)
        #print ("r3 changed: %08x!" % reg)

if __name__ == '__main__':
      app = QtGui.QApplication(sys.argv)
      qb = Adf4355()
      qb.show()

      res = app.exec_()
      sys.exit(res)


