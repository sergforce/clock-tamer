#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import os
import getopt

import svf_to_tamer


class Usage(Exception):
    def __init__(self, msg):
        self.msg = msg

def main(argv=None):
    if argv is None:
        argv = sys.argv
    try:
        try:
            verbose = False
            input_filename = '/home/serg/prjs/clock-tamer/board/FW/clocktamer.svf'
            opts, args = getopt.getopt(argv[1:], "hf:v", ["help", "file=", "verbose"])
            
            for opt, arg in opts:
                if opt in ('-f', '--file'):
                    input_filename = arg
                elif opt in ('-v', '--verbose'):
                    verbose = True
                    
            parser = svf_to_tamer.svf_to_tamer(input_filename, verbose)
            print parser.commands()
            
        except getopt.error, msg:
             raise Usage(msg)
        # more code, unchanged
    except Usage, err:
        print >>sys.stderr, err.msg
        print >>sys.stderr, "for help use --help"
        return 2

if __name__ == "__main__":
    sys.exit(main())
