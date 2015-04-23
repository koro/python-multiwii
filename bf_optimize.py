#!/usr/bin/env python
"""optimize baseflight parameters"""

__author__ = "Oswald Berthold"

import argparse, sys, time
import numpy

from multiwii_serial import MultiwiiCopter

if __name__ == "__main__":
    copter = MultiwiiCopter()
    print copter.get_pid()
    time.sleep(1.)
    
    while True:
        # print "blub"
        # print copter.get_status()
        print copter.get_attitude()
        time.sleep(0.02)

