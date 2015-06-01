#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import os
import getopt
import re

class svf_to_tamer:
    def commands(self):
        return self.cmds

    def __init__(self, input_filename, verbose):
        self.cmds = []
        self.freq = 0.0
        
        # RE to parse
        commet = re.compile('^\!.*')
        freq   = re.compile('^FREQUENCY\s+([0-9E+.]+)\s+HZ\s*;')
        trst   = re.compile('^TRST\s*(\w*)\s*;')
        enddr  = re.compile('^ENDDR\s*(\w*)\s*;')
        endir  = re.compile('^ENDIR\s*(\w*)\s*;')
        state  = re.compile('^STATE\s+IDLE\s*;')
        sir    = re.compile('^SIR\s+(\d+)\s+TDI\s+\(([0-9a-fA-F]+)\)\s*;')
        runtest= re.compile('^RUNTEST(?:\s+IDLE)?\s+(\d+)\s+TCK(?:\s+ENDSTATE\s+IDLE)?\s*;')
        sdr    = re.compile('^SDR\s+(\d+)\s+TDI\s+\(([0-9a-fA-F]+)\)(?:\s+TDO\s+\(([0-9a-fA-F]+)\)(?:\s+MASK\s+\(FFFF\))?)?\s*;')
        
        with open(input_filename, 'r') as f:
            for no,l in enumerate(f.readlines()):
                # remove newline
                line = l.rstrip()
                
                #parsing file by line
                if commet.match(line):
                    if verbose: print 'Line %d is comment (%s)' % (no, line)
                    continue
                  
                matchObj = freq.match(line)
                if matchObj:
                    self.freq = float(matchObj.group(1))
                    if verbose: print 'Line %d is freq %.0f HZ' % (no, self.freq)
                    continue
                
                matchObj = trst.match(line)
                if matchObj:
                    self.trst = matchObj.group(1)
                    if verbose: print 'Line %d is trst %s' % (no, self.trst)
                    continue
                
                matchObj = enddr.match(line)
                if matchObj:
                    self.enddr = matchObj.group(1)
                    if verbose: print 'Line %d is enddr %s' % (no, self.enddr)
                    continue
                
                matchObj = endir.match(line)
                if matchObj:
                    self.endir = matchObj.group(1)
                    if verbose: print 'Line %d is endir %s' % (no, self.endir)
                    continue
                  
                matchObj = state.match(line)
                if matchObj:
                    if verbose: print 'Line %d is state idle' % (no)
                    continue

                matchObj = sir.match(line)
                if matchObj:
                    d_sir = int(matchObj.group(1))
                    d_tdi = int(matchObj.group(2), 16)
                    if d_sir > 16: raise ValueError("SIR bit counter can't be more 16 bits!")
                    if verbose: print 'Line %d is sir (%d) tdi %x' % (no, d_sir, d_tdi)
                    ncmd = ( "REG,JTG,SIR,%08x" % (d_sir << 24 | d_tdi & 0xffff), None)
                    self.cmds.append(ncmd)
                    continue
                
                matchObj = runtest.match(line)
                if matchObj:
                    d_tst = int(matchObj.group(1))
                    if d_tst > 16777216: raise ValueError("RUNTEST counter can't be more 16777216 cycles!")
                    if verbose: print 'Line %d runtest (%d) tck' % (no, d_tst)
                    ncmd = ( "REG,JTG,RUN,%d" % (d_tst), None)
                    self.cmds.append(ncmd)
                    continue
                  
                matchObj = sdr.match(line)
                if matchObj:
                    d_sdr = int(matchObj.group(1))
                    d_tdi = int(matchObj.group(2), 16)
                    try:
                        d_tdo = int(matchObj.group(3), 16)
                    except:
                        d_tdo = None
                    if d_sdr > 16: raise ValueError("SDR bit counter can't be more 16 bits!")
                    if verbose: print 'Line %d is sdr (%d) tdi %x tdo %s' % (no, d_sdr, d_tdi, d_tdo)
                    ncmd = ( "REG,JTG,SDR,%08x" % (d_sdr << 24 | d_tdi & 0xffff), d_tdo)
                    self.cmds.append(ncmd)
                    continue

                if verbose: print 'Error in line %d: (%s)' % (no, line)
                raise SyntaxError('SVF error in line %d' % no)


                