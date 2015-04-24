#!/usr/bin/env python
"""optimize baseflight parameters"""

__author__ = "Oswald Berthold"

import argparse, sys, time
import numpy as np
import pylab as pl

from multiwii_serial import MultiwiiCopter, BF_PID

def copter(params):
    copter = MultiwiiCopter()
    time.sleep(1)
    # get current PID values
    (status, ret) = copter.get_pid()
    time.sleep(1.)
    piddata = list(ret[1])
    print "piddata cur", piddata
    # # test setting new PID values
    # # modify values
    # piddata[0] += 1
    # piddata[3] += 1
    # set new values
    alt_offset = BF_PID.PIDALT*3
    piddata[alt_offset] = params[0]
    piddata[alt_offset+1] = params[1]
    piddata[alt_offset+2] = params[2]
    vel_offset = BF_PID.PIDVEL*3
    piddata[vel_offset] = params[3]
    piddata[vel_offset+1] = params[4]
    piddata[vel_offset+2] = params[5]
    print "piddata new", piddata
    # transmit new values
    (status, ret) = copter.set_pid(piddata)


    done = False
    armed = False
    maxsamp = 1000
    logdata = np.zeros((maxsamp, 3))
    i = 0

    # start and init
    while not done:
        # print "blub"
        # print copter.get_status()
        # get telemetry/data
        # MSP_STATUS
        status = copter.get_status()
        # # MSP_RAW_IMU
        # raw_imu =  copter.get_raw_imu()
        # print "raw imu", 
        # # MSP_ATTITUDE
        # attitude = copter.get_attitude()
        # # print "attitude", 
        # MSP_ALTITUDE
        altitude = copter.get_altitude()
        # print "altitude", 
        # # MSP_RAW_GPS
        # raw_gps = copter.get_raw_gps()
        # # print "raw gps",  
        # MSP_RC
        # rc = copter.get_rc()
        # print "rc",       
        # MSP_MOTORS
        # time.sleep(0.01)
        print status, altitude # , rc
        # print status[1]

        # check data
        if status[0] != 0 or altitude[0] != 0:
            print("got serial error, skipping datum")
            continue

        logdata[i,0] = status[1][1][3] # mode
        logdata[i,1] = altitude[1][1][0] # alt est
        logdata[i,2] = altitude[1][1][1] # vario

        if status[1][1][3] == 3 and not armed:
            armed = True
        elif status[1][1][3] == 2 and armed:
            armed = False
            done = True
            
        # max samples recorded
        if i == (maxsamp-1):
            done = True

        i += 1

    # save data
    ts = time.strftime("%Y%m%d-%H%M%S")
    np.save("bf_optimize_log_%s" % ts, logdata)

    # in place inspection
    pl.subplot(311)
    pl.plot(logdata[:,0])
    pl.subplot(312)
    pl.plot(logdata[:,1])
    pl.subplot(313)
    pl.plot(logdata[:,2])
    pl.show()

    alt_active_idx = logdata[:,0] == 27
    alt_data = logdata[alt_active_idx,1]
    alt_target = np.mean(alt_data[0:10])
    alt_mse = np.mean(np.square(alt_target - alt_data))
        
    # generate params, set and run
    # read data
    #  - min: altitude, RC5
    #  - max: record all
    # compute performance
    # return np.random.uniform(0, 1)
    return alt_mse
         
def objective(params):
    """objective function: params is a tuple passed by fmin)"""
    print "params", params
    # return np.random.uniform(0, 1.)
    ret = copter(params)
    print "mse", ret
    return ret
    
def main(args):
    from hyperopt import hp, fmin, tpe, Trials
    space = [
        hp.quniform("alt_p", 5, 120, 1),
        hp.quniform("alt_i", 0, 100, 1),
        hp.quniform("alt_d", 0, 50, 1),
        hp.quniform("vel_p", 10, 120, 1),
        hp.quniform("vel_i", 0, 100, 1),
        hp.quniform("vel_d", 0, 50, 1),
        ]
        
    trials = Trials()
    best = fmin(objective, space, algo=tpe.suggest, max_evals=100, trials=trials)
    print "best", best

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    args = parser.parse_args()
    main(args)
