#!/usr/bin/env python
"""optimize baseflight parameters"""

__author__ = "Oswald Berthold"

# - use Trials, save it each round
# - use record to learn model
# - use ITAE: integral of time weighted absolute error
# - make mse pos and mse vel same OoM
# - log: batt, throttle, acc

import argparse, sys, time
import numpy as np
import pylab as pl

from multiwii_serial import MultiwiiCopter, BF_PID

def copter(params):
    copter = MultiwiiCopter()
    time.sleep(1)
    # get current PID values
    gotpid = False
    while not gotpid:
        (status, ret) = copter.get_pid()
        if status == 0:
            gotpid = True
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
    numdata = 5 # mode, alt, vel, throttle, acc
    logdata = np.zeros((maxsamp, numdata))
    i = 0

    # start and init
    while not done:
        # print "blub"
        # print copter.get_status()
        # get telemetry/data
        # MSP_STATUS
        status = copter.get_status()
        # MSP_RAW_IMU
        raw_imu =  copter.get_raw_imu()
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
        rc = copter.get_rc()
        # print "rc",       
        # MSP_MOTORS
        # time.sleep(0.01)
        # print status, altitude # , rc
        # print status[1]

        # check data
        if status[0] != 0 or altitude[0] != 0:
            print("got serial error, skipping datum")
            continue
        if status[1][0] != MultiwiiCopter.MSP_STATUS:
            print("STATUS msg got mixed up, skipping datum")
            print copter.recv_serial()
            # status = copter.get_status()
            continue
        if altitude[1][0] != MultiwiiCopter.MSP_ALTITUDE:
            print("ALT msg got mixed up, skipping datum")
            print copter.recv_serial()
            continue
        if rc[1][0] != MultiwiiCopter.MSP_RC:
            print("RC msg got mixed up, skipping datum")
            print copter.recv_serial()
            continue
        if raw_imu[1][0] != MultiwiiCopter.MSP_RAW_IMU:
            print("RAW_IMU msg got mixed up, skipping datum")
            print copter.recv_serial()
            continue

        try:
            logdata[i,0] = status[1][1][3] # mode
        except IndexError:
            print "invalid index", status

        try:
            logdata[i,1] = altitude[1][1][0] # alt est
            logdata[i,2] = altitude[1][1][1] # vario
            logdata[i,3] = rc[1][1][3] # throttle
            logdata[i,4] = raw_imu[1][1][2] # acc-z
        except IndexError:
            print "invalid index", altitude

        try:
            if status[1][1][3] == 3 and not armed:
                armed = True
            elif status[1][1][3] == 2 and armed:
                armed = False
                done = True
        except:
            print "Unexpected error:", sys.exc_info()[0]
            # print "arm check failed", e
            
        # max samples recorded
        if i == (maxsamp-1):
            done = True

        i += 1

    # save data
    ts = time.strftime("%Y%m%d-%H%M%S")
    np.save("bf_optimize_%s_log" % ts, logdata)
    params_array = np.array(params)
    np.save("bf_optimize_%s_params" % ts, params_array)

    # compute performance
    # alt_active_idx = logdata[:,0] == 27 # with mag lock
    alt_active_idx = logdata[:,0] == 11
    alt_data = logdata[alt_active_idx,1]
    alt_target = np.mean(alt_data[0:10])
    alt_mse = np.mean(np.square(alt_target - alt_data))
    alt_target_mse_array = np.array((alt_target, alt_mse))
    np.save("bf_optimize_%s_alt_target_mse" % ts, alt_target_mse_array)
    
    vel_data = logdata[alt_active_idx,2]
    vel_mse  = np.mean(np.square(vel_data))
    vel_target_mse_array = np.array((0., vel_mse))
    np.save("bf_optimize_%s_vel_target_mse" % ts, vel_target_mse_array)
        
    # in place inspection
    pl.subplot(411)
    pl.title("mode")
    pl.plot(logdata[:,0])
    pl.subplot(412)
    pl.title("alt")
    pl.plot(logdata[:,1])
    pl.plot(np.ones_like(logdata[:,1]) * alt_target)
    pl.subplot(413)
    pl.title("alt vel")
    pl.plot(logdata[:,2])
    pl.subplot(414)
    pl.title("thr + acc z")
    pl.plot(logdata[:,3])
    pl.plot(logdata[:,4])
    pl.show()

    # it was so bad, alt hold was disabled immediatly
    if np.sum(alt_active_idx) < 30:
        alt_mse = 1000.
            
    # generate params, set and run
    # read data
    #  - min: altitude, RC5
    #  - max: record all
    # compute performance
    # return np.random.uniform(0, 1)
    print "alt_mse = %f, vel_mse = %f" % (alt_mse, vel_mse)
    return (alt_mse + vel_mse)
         
def objective(params):
    """objective function: params is a tuple passed by fmin)"""
    print "params", params
    # return np.random.uniform(0, 1.)
    ret = copter(params)
    print "params", params
    print "mse", ret
    return ret
    
def main(args):
    from hyperopt import hp, fmin, tpe, Trials
    space = [
        # hp.quniform("alt_p", 5, 120, 1),
        hp.quniform("alt_p", 5, 60, 1),
        hp.quniform("alt_i", 0, 100, 1),
        # hp.quniform("alt_d", 0, 50, 1),
        hp.quniform("alt_d", 20, 70, 1),
        # hp.quniform("vel_p", 10, 120, 1),
        hp.quniform("vel_p", 20, 90, 1),
        # hp.quniform("vel_i", 0, 100, 1),
        hp.quniform("vel_i", 0, 60, 1),
        hp.quniform("vel_d", 0, 50, 1),
        ]
        
    trials = Trials()
    best = fmin(objective, space, algo=tpe.suggest, max_evals=50, trials=trials)
    print "best", best

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    args = parser.parse_args()
    main(args)
