#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import time
import re

from PyQt4 import QtCore, QtGui, uic

#from adf4355_ui import *
from adf4355_regs import *

class Adf4355(QtGui.QWidget):
    def __init__(self, func = None):
        QtGui.QWidget.__init__(self)
        #self.obj = Ui_Dialog()
        #self.obj.setupUi(self)
        self.obj = uic.loadUi("adf4355.ui")
        self.obj.show()
        
        self.func = func
        
        self.N = 1.0
        self.pfd = 1.0

        self.reg = [0] * 13
        self.reg[0] = 0
        self.reg[1] = 1
        self.reg[2] = 2
        self.reg[3] = 3
        self.reg[4] = 4
        self.reg[5] = REG5_RESERVED
        self.reg[6] = 6
        self.reg[7] = 7
        self.reg[8] = REG8_RESERVED
        self.reg[9] = 9
        self.reg[10] = 10
        self.reg[11] = REG11_RESERVED
        self.reg[12] = 12

        self.all_changed()        
        def ui_change(func, uilist):
            for i in uilist:
                try:    i.currentIndexChanged['QString'].connect(func)
                except: i.valueChanged['QString'].connect(func)

        self.obj.r0_uis = [ self.obj.f_int, self.obj.r0_autocal, self.obj.r0_prescaler ]
        ui_change(self.r0_changed, self.obj.r0_uis)

        self.obj.r1_uis = [ self.obj.f_frac1 ]
        ui_change(self.r1_changed, self.obj.r1_uis)

        self.obj.r2_uis = [ self.obj.f_frac2, self.obj.f_mod2 ]
        ui_change(self.r2_changed, self.obj.r2_uis)

        self.obj.r3_uis = [ self.obj.r3_sd_load, self.obj.r3_phase_rsync, self.obj.r3_phase_adj, self.obj.r3_phase ]
        ui_change(self.r3_changed, self.obj.r3_uis)
        
        self.obj.r4_uis = [ self.obj.r4_muxout, self.obj.r4_double_buf, self.obj.r4_cp_curr, self.obj.r4_refin, self.obj.r4_mux_lvl, self.obj.r4_pd_pol, self.obj.r4_powerdown, self.obj.r4_cp_state, self.obj.r4_counter_rst, self.obj.f_r_cnt ]
        ui_change(self.r4_changed, self.obj.r4_uis)
        # Special tricks with check boxes
        self.obj.f_x2.stateChanged['int'].connect(self.r4_changed)
        self.obj.f_div2.stateChanged['int'].connect(self.r4_changed)

        self.obj.r6_uis = [ self.obj.r6_feedback, self.obj.r6_mtld, self.obj.r6_aux_out, self.obj.r6_aux_out_pwr, self.obj.r6_out, self.obj.r6_out_pwr, self.obj.r6_neg_bleed, self.obj.r6_gated_bleed, self.obj.r6_bleed_curr, self.obj.f_div_out ] #self.obj.r6_bleed
        ui_change(self.r6_changed, self.obj.r6_uis)

        self.obj.r7_uis = [ self.obj.r7_le_sync, self.obj.r7_ld_cycles, self.obj.r7_lol_mode, self.obj.r7_frac_n, self.obj.r7_ld ]
        ui_change(self.r7_changed, self.obj.r7_uis)

        self.obj.r9_uis = [ self.obj.r9_vco_div, self.obj.r9_to, self.obj.r9_alc_to, self.obj.r9_lock_to ]
        ui_change(self.r9_changed, self.obj.r9_uis)

        self.obj.r10_uis = [ self.obj.r10_adc_div, self.obj.r10_adc_conv, self.obj.r10_adc_en ]
        ui_change(self.r10_changed, self.obj.r10_uis)

        self.obj.r12_uis = [ self.obj.r12_phase_rsync ]
        ui_change(self.r12_changed, self.obj.r12_uis)
        
        #Tune Logic
        self.obj.f_ref.valueChanged['QString'].connect(self.pfd_changed)
        btns = [ self.obj.b_reg0_set, self.obj.b_reg1_set, self.obj.b_reg2_set,
                 self.obj.b_reg3_set, self.obj.b_reg4_set, self.obj.b_reg5_set,
                 self.obj.b_reg6_set, self.obj.b_reg7_set, self.obj.b_reg8_set,
                 self.obj.b_reg9_set, self.obj.b_reg10_set, self.obj.b_reg11_set,
                 self.obj.b_reg12_set ]

        if self.func is not None:
            self.obj.b_set.clicked.connect(self.set_regs)
            self.obj.b_set_freq.clicked.connect(self.set_freq_regs)

            for i,v in enumerate(btns):
                v.clicked.connect(lambda: self.set_reg(int(i)))

    def set_freq_regs(self):
        self.func([self.reg[4] | (1<<REG4_CNTR_RESET),
                   self.reg[2],
                   self.reg[1],
                   self.reg[0] & (~(1<<REG0_AUTOCAL_SHIFT)),
                   self.reg[4] & (~(1<<REG4_CNTR_RESET))])
        # Wait > 16 ADC_CLK_DIV
        self.func([self.reg[0] | (1<<REG0_AUTOCAL_SHIFT)])

    def set_regs(self):
        self.func([ i for i in reversed(self.reg) ])

    @QtCore.pyqtSlot(int)
    def set_reg(self, regno):
        self.func([self.reg[regno]])
        
    def all_changed(self):
        self.r0_changed()
        self.r1_changed()
        self.r2_changed()
        self.r3_changed()
        self.r4_changed()
        self.obj.reg5.setText("x%08x" % self.reg[5])
        self.r6_changed()
        self.r7_changed()
        self.obj.reg8.setText("x%08x" % self.reg[8])
        self.r9_changed()
        self.r10_changed()
        self.obj.reg11.setText("x%08x" % self.reg[11])
        self.r12_changed()
        
    def mul_changed(self):
        self.N = (((float(self.obj.f_frac2.value()) / float(self.obj.f_mod2.value()) +
                    self.obj.f_frac1.value()) / 16777216.0) + self.obj.f_int.value())
        self.obj.f_vco_out.setText("%.9f" % (self.N * self.pfd))
        self.obj.f_n.setText("N = %.12f" % self.N)
	  
    def pfd_changed(self):
	    self.pfd = (self.obj.f_ref.value() * 
	                (2 if self.obj.f_x2.isChecked() else 1) /
	                self.obj.f_r_cnt.value() /
	                (2 if self.obj.f_div2.isChecked() else 1))
	    self.obj.f_pfd.setText("%.3f" % self.pfd)
	    self.mul_changed()

    def r0_changed(self):
        self.obj.f_int.setMinimum(75) if self.obj.r0_prescaler.currentIndex() == 1 else self.obj.f_int.setMinimum(23)
        reg = (((self.obj.r0_autocal.currentIndex() & 1) << REG0_AUTOCAL_SHIFT) |
               ((self.obj.r0_prescaler.currentIndex() & 1) << REG0_PRESCALER_SHIFT) |
               ((self.obj.f_int.value() & REG0_NVALUE_MASK) << REG0_NVALUE_SHIFT) | 0)
        self.reg[0] = reg
        self.obj.reg0.setText("x%08x" % reg)
        self.mul_changed()

    def r1_changed(self):
        reg = (((self.obj.f_frac1.value() & REG1_MFRAC_MASK) << REG1_MFRAC_SHIFT) | 1)
        self.reg[1] = reg
        self.obj.reg1.setText("x%08x" % reg)
        self.mul_changed()

    def r2_changed(self):
        reg = (((self.obj.f_frac2.value() & REG2_AUX_FRAC_MASK) << REG2_AUX_FRAC_SHIFT) |
               ((self.obj.f_mod2.value() & REG2_AUX_MOD_MASK) << REG2_AUX_MOD_SHIFT) | 2)
        self.reg[2] = reg
        self.obj.reg2.setText("x%08x" % reg)
        self.mul_changed()

    def r3_changed(self):
        reg = (((self.obj.r3_sd_load.currentIndex() & 1) << REG3_SD_LOAD_SHIFT) |
               ((self.obj.r3_phase_rsync.currentIndex() & 1) << REG3_PHASE_RSYNC_SHIFT) |
               ((self.obj.r3_phase_adj.currentIndex() & 1) << REG3_PHASE_ADJ_SHIFT) |
               ((self.obj.r3_phase.value() & REG3_PHASE_MASK) << REG3_PHASE_SHIFT) | 3)
        self.reg[3] = reg
        self.obj.reg3.setText("x%08x" % reg)

    def r4_changed(self):
        reg = (((self.obj.r4_muxout.currentIndex() & REG4_MUXOUT_MASK) << REG4_MUXOUT_SHIFT) |
	       ((1 if self.obj.f_x2.isChecked() else 0) << REG4_REF_DBL_SHIFT) |
	       ((1 if self.obj.f_div2.isChecked() else 0) << REG4_REF_DIV_SHIFT) |
	       ((self.obj.f_r_cnt.value() & REG4_R_MASK) << REG4_R_SHIFT) |
               ((self.obj.r4_double_buf.currentIndex() & 1) << REG4_DBL_BUFF_SHIFT) |
               ((self.obj.r4_cp_curr.currentIndex() & REG4_CURRENT_MASK) << REG4_CURRENT_SHIFT) |
               ((self.obj.r4_refin.currentIndex() & 1) << REG4_REF_MODE_SHIFT) |
               ((self.obj.r4_mux_lvl.currentIndex() & 1) << REG4_MUX_LOGIC_SHIFT) |
               ((self.obj.r4_pd_pol.currentIndex() & 1) << REG4_PD_POL_SHIFT) |
               ((self.obj.r4_powerdown.currentIndex() & 1) << REG4_PWR_DOWN_SHIFT) |
               ((self.obj.r4_cp_state.currentIndex() & 1) << REG4_CP_3STATE_SHIFT) |
               ((self.obj.r4_counter_rst.currentIndex() & 1) << REG4_CNTR_RESET ) | 4)
        self.reg[4] = reg
        self.obj.reg4.setText("x%08x" % reg)
        self.pfd_changed()

    def r6_changed(self):
        reg = (((self.obj.r6_feedback.currentIndex() & 1) << REG6_FB_SEL_SHIFT) |
               ((self.obj.r6_mtld.currentIndex() & 1) << REG6_MLTD_SHIFT) |
               ((self.obj.r6_aux_out.currentIndex() & 1) << REG6_AUX_PWR_EN_SHIFT) |
               ((self.obj.r6_aux_out_pwr.currentIndex() & REG6_AUX_PWR_MASK) << REG6_AUX_PWR_SHIFT) |
               ((self.obj.r6_out.currentIndex() & 1) << REG6_PWR_EN_SHIFT) |
               ((self.obj.r6_out_pwr.currentIndex() & REG6_PWR_MASK) << REG6_PWR_SHIFT) |
               ((self.obj.r6_neg_bleed.currentIndex() & 1) << REG6_NEG_BLEED_SHIFT) |
               ((self.obj.r6_gated_bleed.currentIndex() & 1) << REG6_GATED_BLEED_SHIFT) |
               ((self.obj.r6_bleed_curr.value() & REG6_CP_BLEED_CURR_MASK) << REG6_CP_BLEED_CURR_SHIFT) |
               ((self.obj.f_div_out.currentIndex() & REG6_RF_DIV_MASK) << REG6_RF_DIV_SHIFT) |
               (REG6_RESERVED_VALUE << REG6_RESERVED_SHIFT) | 6)
        self.reg[6] = reg
        self.obj.reg6.setText("x%08x" % reg)
        self.obj.r6_bleed_value.setText("%.2f uA" % (self.obj.r6_bleed_curr.value() * 3.75))

    def r7_changed(self):
        reg = (((self.obj.r7_le_sync.currentIndex() & 1) << REG7_LE_SYNC_SHIFT) |
               ((self.obj.r7_ld_cycles.currentIndex() & REG7_LD_CYCLE_CNT_MASK) << REG7_LD_CYCLE_CNT_SHIFT) |
               ((self.obj.r7_lol_mode.currentIndex() & 1) << REG7_LOL_MODE_SHIFT) |
               ((self.obj.r7_frac_n.currentIndex() & REG7_FRAC_N_PREC_MASK) << REG7_FRAC_N_PREC_SHIFT) |
               ((self.obj.r7_ld.currentIndex() & 1) << REG7_LD_MODE_SHIFT) |
               (REG7_RESERVED_VALUE << REG7_RESERVED_SHIFT) | 7)
        self.reg[7] = reg
        self.obj.reg7.setText("x%08x" % reg)

    def r9_changed(self): 
        reg = (((self.obj.r9_vco_div.value() & REG9_VCO_BAND_MASK) << REG9_VCO_BAND_SHIFT) |
               ((self.obj.r9_to.value() & REG9_TIMEOUT_MASK) << REG9_TIMEOUT_SHIFT) |
               ((self.obj.r9_alc_to.value() & REG9_AUTO_LVL_TO_MASK) << REG9_AUTO_LVL_TO_SHIFT) |
               ((self.obj.r9_lock_to.value() & REG9_SYNT_LOCK_TO_MASK) << REG9_SYNT_LOCK_TO_SHIFT) | 9)
        self.reg[9] = reg
        self.obj.reg9.setText("x%08x" % reg)

    def r10_changed(self): 
        reg = (((self.obj.r10_adc_div.value() & REG10_ADC_CLK_DIV_MASK) << REG10_ADC_CLK_DIV_SHIFT) |
               ((self.obj.r10_adc_conv.currentIndex() & 1) << REG10_ADC_CONV_SHIFT) |
               ((self.obj.r10_adc_en.currentIndex() & 1) << REG10_ADC_EN_SHIFT) |
               (REG10_RESERVED_VALUE << REG10_RESERVED_SHIFT) | 10)
        self.reg[10] = reg
        self.obj.reg10.setText("x%08x" % reg)

    def r12_changed(self): 
        reg = (((self.obj.r12_phase_rsync.value() & REG12_RESYNC_CLOCK_MASK) << REG12_RESYNC_CLOCK_SHIFT) |
               (REG12_RESERVED_VALUE << REG12_RESERVED_SHIFT) | 12)
        self.reg[12] = reg
        self.obj.reg12.setText("x%08x" % reg)


if __name__ == '__main__':
      app = QtGui.QApplication(sys.argv)
      qb = Adf4355()
      #qb.show()

      res = app.exec_()
      sys.exit(res)


