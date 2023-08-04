#!/usr/bin/python3
# -*- coding: utf-8 -*-

# tb.py

import sys
import time
import re

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtXml import *

from tamer_basic_ui import *
from tamerdevice import *

tamer10_lmk1000  = ("LVDS", None, "LVDS",   None,        "LVPECL/SYNC", "LVPECL/LVCMOS", "LVCMOS", "LVPECL")
tamer11_lmk1010  = ("LVDS", "CMOS", "LVDS", None,        "LVDS",   "LVDS",   "CMOS", "LVDS")
tamer12_lmk1000  = ("LVDS", "CMOS", None,   "LVPECL",    "LVPECL", "LVPECL", None,   "LVPECL")
tamer12_lmk1010  = ("LVDS", "CMOS", None,   "LVDS",      "LVDS",   "LVDS",   "CMOS", "LVDS")
tamer121_lmk1010 = ("LVDS/P+", "CMOS", None,   "LVDS/P+",      "LVDS",   "LVDS",   "CMOS", "LVDS")
tamer_unknown    = ("unknown", "unknown", "unknown", "unknown", "unknown", "unknown", "unknown", "unknown")

from adf4355_form import *

class MainWindow(QtWidgets.QWidget):
    def __init__(self, device=None):
        QtWidgets.QWidget.__init__(self)
        self.obj = Ui_Basic()
        self.obj.setupUi(self)
        self.dev = device

        self.outCb = (self.obj.cbOut0, self.obj.cbOut1, self.obj.cbOut2, self.obj.cbOut3,
                      self.obj.cbOut4, self.obj.cbOut5, self.obj.cbOut6, self.obj.cbOut7)

        spl = self.ParseHWI()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.onTimer)
        #self.connect(self.timer,        QtCore.SIGNAL('timeout()'), self.onTimer)

        self.ReadAll()

        self.obj.btStore.clicked.connect(self.onStoreEeprom)
        self.obj.btLoad.clicked.connect(self.onLoadEeprom)
        self.obj.btSet.clicked.connect(self.onSet)
        self.obj.btGet.clicked.connect(self.onGet)
        self.obj.btCPLD.clicked.connect(self.onCPLD)

        #self.connect(self.obj.btStore,  QtCore.SIGNAL('clicked()'), self.onStoreEeprom)
        #self.connect(self.obj.btLoad,   QtCore.SIGNAL('clicked()'), self.onLoadEeprom)
        #self.connect(self.obj.btSet,    QtCore.SIGNAL('clicked()'), self.onSet)
        #self.connect(self.obj.btGet,    QtCore.SIGNAL('clicked()'), self.onGet)
        #self.connect(self.obj.btCPLD,   QtCore.SIGNAL('clicked()'), self.onCPLD)

        if spl != None:
            spl.finish(self)

        self.adf = Adf4355(self.write_adf)
        self.adf.obj.b_save_ee.setEnabled(True)
        self.adf.obj.b_load_ee.setEnabled(True)
        self.adf.obj.b_save_ee.clicked.connect(self.write_ee_adf)
        self.adf.obj.b_load_ee.clicked.connect(self.load_ee_adf)

    def load_ee_adf(self):
        regs = [ self.dev.loadEepromADF4355Reg(i) for i in range(13) ]
        self.adf.process_load(regs)
        # Load OSC
        self.adf.obj.f_ref.setValue(float(self.dev.getOsc()) / 1000000)

    def write_ee_adf(self):
        for i in self.adf.reg:
            self.dev.storeEepromADF4355Reg(i)

    def write_adf(self, d):
        for i in d:
            data = self.dev.setAdf(i)
            print("Reg: %x => %s" % (i, data))


    def onTimer(self):
        self.onGet()

    def onCPLD(self):
        import svf_to_tamer
        from time import sleep

        filename = QtWidgets.QFileDialog.getOpenFileName(self, "Load CPLD firmware", "*.svf", "SVF *.svf (*.svf)")
        if len(filename) > 0:
                pixmap = QtGui.QPixmap(480, 320)
                pixmap.fill(QtGui.QColor(20,200,20))
                splash = QtGui.QSplashScreen(pixmap)
                progressBar = QtGui.QProgressBar(splash)
                progressBar.setGeometry(splash.width()/10, 8*splash.height()/10,
                                        8*splash.width()/10, splash.height()/10)
                splash.show()
            #try:
                prs = svf_to_tamer.svf_to_tamer(filename)
                ret = self.dev.jtagCmd("RST", 0)
                cmds = prs.commands()
                i = 0
                for cmd,val,exp in cmds:
                    app.processEvents()
                    i += 1
                    #time.sleep(0.005)
                    #self.dev.flush()
                    p = float(100*i)/len(cmds)
                    progressBar.setValue(p)
                    print("%02.1f%%: CMD: %s => %d; EXPECT: %s" % (p, cmd, val, exp))
                    # usb transaction fails on big delays
                    cnt = 1
                    if cmd == "RUN":
                        if int(val) > 30000:
                            cnt = val / 30000
                            val = 30000
                        ##elif int(val) < 100:
                        ##    val = 100
                    for j in range(cnt):
                        ret = self.dev.jtagCmd(cmd, val)
                    if cmd != "RUN":
                        rval = (ret & 0xffff)
                        if exp is not None and rval != exp:
                            raise ValueError("Expected %x, got %x" % (exp, ret))
                    elif ret != "OK":
                        raise ValueError("FAILED RUN")
            #except Exception, e:
            #    print("Can't parse CPLD image: %s", e)
                print ("Done!")

    def ParseHWI(self):
        data = self.dev.getHwi()
        self.obj.leHWI.setText(data)

        if "GPS" in str(data):
            self.gps = 1
        else:
            self.gps = 0

        if "VCTCXO" in str(data):
            self.vctcxo = True
        else:
            self.vctcxo = False

        dataVer = self.dev.getVer()
        self.obj.leVER.setText(dataVer)

        vals = str(dataVer).split(" ")
        self.ver = vals[1].split("=")[1]
        self.vernum = float(self.ver)
        print("Tamer version: %s" % self.ver)

        vals = str(data).split(" ")

        self.obj.lbDac.setVisible(self.vctcxo)
        self.obj.sbDacValue.setVisible(self.vctcxo)
        self.obj.cbOutMode.setVisible(self.vctcxo)

        if self.vernum >= 2:
            self.obj.groupBox.setVisible(False)
            self.obj.cbOscDisable.setEnabled(False)
        else:
            self.obj.btCPLD.setVisible(False)
            try:
                self.lmx = int(vals[0].split("=")[1])
                self.lmk = int(vals[1].split("=")[1])
            except:
                self.lmx=0
                self.lmk=0
            if self.lmk == 1010 and (self.ver == "1.21" or self.ver == "1.22" or self.ver == "1.23" or self.ver == "1.30"):
                self.outputsConfig = tamer121_lmk1010
            elif self.lmk == 1010 and self.ver == "1.2":
                self.outputsConfig = tamer12_lmk1010
            elif self.lmk == 1000 and self.ver == "1.2":
                self.outputsConfig = tamer12_lmk1010
            elif self.lmk == 1010 and self.ver == "1.1pre":
                self.outputsConfig = tamer11_lmk1010
            elif self.ver == "1.0":
                self.outputsConfig = tamer10_lmk1000
            else:
                self.outputsConfig = tamer_unknown

            for i,v in enumerate(self.outputsConfig):
                if v == None:
                    self.outCb[i].setVisible(False)
                else:
                    self.outCb[i].setText("%d: %s" % (i, v))

        if self.gps == 0:
            self.obj.gGps.setVisible(False)
        elif (self.ver == "1.22" or self.ver == "1.23" or self.ver == "1.30"):
            pxm = QPixmap(":/Tamer/P-img-small.png")
            spl = QSplashScreen(pxm)
            spl.show();
            spl.showMessage("Probing GPS...");
            app.processEvents();
            gpsid = False
            cnt = 3
            for j in xrange(cnt-1):
                try:
                    gpsid = self.dev.checkGps()
                    if gpsid == True:
                        break
                    else:
                        print("GPS NOT PRESENT")
                except:
                    pass

                self.dev.flush()
                time.sleep(1)
                spl.showMessage("Probing GPS: try %d of %d maximum..." % (j + 2, cnt));
                app.processEvents();


            if not gpsid:
                self.obj.gGps.setTitle("GPS FAILED!")
                self.gps = 0
                #spl.finish(self)
                return spl

            spl.showMessage("Probing GPS module...");
            app.processEvents();

            cnt = 3
            v = ["[failed]","[failed]"]
            for k in xrange(2):
              for j in xrange(cnt):
                try:
                    if k == 0:
                        ver = self.dev.getGpsVer()
                    else:
                        ver = self.dev.getGpsFw()
                    ver = ver[ver.find(',')+1:]
                    v[k] = ver
                    break
                except:
                    time.sleep(1)
                    self.dev.flush()
                    continue

            self.obj.gGps.setTitle("GPS VER='%s' HW='%s'" % (v[0], v[1]))
            #spl.finish(self)
            return spl

    def ReadOutputs(self):
        data = self.dev.getOutputsMask()
        if isinstance(data, (int,long)):
            for i in range(0,8):
                state = (2**i) & data
                self.outCb[i].setChecked(state)

    def GetOututsMask(self):
        data = 0
        for i in range(0,8):
            data = data + self.outCb[i].isChecked() * (2**i)
        return data

    def ReadFosc(self):
        data = self.dev.getOsc()
        self.obj.leFosc.setText(str(data))

    def ReadFout(self):
        data = self.dev.getOut()
        self.obj.leFout.setText(str(data))

    def GetFosc(self):
        data = self.obj.leFosc.text()
        data = re.sub("[^\d]", "", str(data))
        return data

    def GetFout(self):
        data = self.obj.leFout.text()
        data = re.sub("[^\d]", "", str(data))
        return data

    def ReadAll(self):
        data = self.dev.flush() ### Using this to reset buffer in buggy firmware

        self.onGet()

    def onStoreEeprom(self):
        self.dev.storeEeprom()

    def onLoadEeprom(self):
        self.dev.loadEeprom()
        self.onGet()


    def onGet(self):
        data = self.dev.getAut()
        self.obj.cbAutoStart.setChecked(data)

        if self.vernum < 2:
            data = self.dev.getIntOscState()
            if data is None:
                self.obj.cbOscDisable.setEnabled(False)
                self.oscState = False
            else:
                self.obj.cbOscDisable.setChecked(not data)
                self.oscState = True
            self.ReadOutputs()
        else:
            data = self.dev.getDac()
            if data is None:
                self.obj.sbDacValue.setEnabled(False)
                self.dacValue = 2048
            else:
                self.dacValue = int(data)
                self.obj.sbDacValue.setValue(self.dacValue)

        self.ReadFosc()
        self.ReadFout()
        if self.gps:
            self.ReadGps()

    def ReadGps(self):
        data = self.dev.getGpsAut()
        self.obj.cbGPSSync.setChecked(data)

        if data and not self.timer.isActive():
            self.timer.start(3000)
        elif not data and self.timer.isActive():
            self.timer.stop();

        data = self.dev.getGps("R01")

        try:
            di = data - self.prev1pps
            if di > 0:
                self.obj.lbGpsInfo.setText("In sync")
                self.obj.lbGpsInfo.setStyleSheet("QLabel { background-color: green; }");
            else:
                self.obj.lbGpsInfo.setText("Out of sync!")
                self.obj.lbGpsInfo.setStyleSheet("QLabel { background-color: red; }");
        except:
            pass

        self.prev1pps = data
        self.obj.le1PPS.setText(str(data))

        data = float(self.dev.getGps("R03")) / 32
        self.obj.leCalcFout.setText(str(data))

    def onSet(self):
        if self.vernum < 2:
            data = self.GetOututsMask()
            self.dev.setOutputsMask(data)
        else:
            idx = self.obj.cbOutMode.currentIndex()
            self.dev.pinDivOut(0 if idx == 1 else 1)

        if self.vctcxo:
            self.dev.setDac(self.obj.sbDacValue.value())
        #print data
        data = self.GetFosc()
        res1 = self.dev.setOsc(data)
        #print data
        data = self.GetFout()
        res2 = self.dev.setOut(data)

        if "Bad" in res1 or "Bad" in res2:
            QMessageBox.critical(self, "Can't set frequency", "OSC: %s\r\nOUT: %s" %(res1, res2))
            return
        #print data

        if self.oscState:
            data = int(not self.obj.cbOscDisable.isChecked())
            self.dev.setIntOscState(data)
            #print data

        data = int(self.obj.cbAutoStart.isChecked())
        self.dev.setAut(data)
        #print data

        if self.gps:
            data = self.obj.cbGPSSync.isChecked()
            self.dev.setGpsAut(int(data))


if __name__ == '__main__':
      noCloseMainForm = False
      app = QtWidgets.QApplication(sys.argv)
      dev = TamerDevice()
      qb = MainWindow(dev)
#      if qb.error == 1:
#          sys.exit(-1)

      qb.show()

#      qb.setGeometry(10, 20, 1440, 900)
#      qb.setFocus()
      res = app.exec_()
      if qb.gps:
           qb.dev.enterGpsMode()
           print("Now you can attach gpsd to you ClockTamer")
      sys.exit(res)


