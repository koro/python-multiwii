#!/usr/bin/env python

import Queue

import time, sys
import numpy as np
import pylab as pl

# sys.path.insert(0, "/home/src/QK/smp/neural")
sys.path.insert(0, "/home/src/QK/mavlink/pymavlink/")
# sys.path.insert(0, "/home/x75/tmp")

# import pymavlink
import mavutil
import mavlinkv10 as mavlink

from DataSource import DataSource
from multiwii_serial import BF_PID

class BaseflightOptimize(object):
    def __init__(self):
        self.dq = Queue.Queue
        self.ds = DataSource(self, self.dq, 21)
        self.ds.start()
        self.x_flag = False
        self.running = True

        self.maxsamp = 4000
        self.run_cnt = 0

        self.armed = False
        self.mode = 0
        self.alt = 0
        self.vz = 0
        self.throttle = 0
        self.accz = 0

    def mavlink_sys_status_handler(self, msg):
        self.mode = msg.onboard_control_sensors_health # ugly hack
        # print "sys status", msg
    def mavlink_raw_imu_handler(self, msg):
        self.zacc = msg.zacc
        # print "raw imu", msg
        pass
    def mavlink_attitude_handler(self, msg):
        # print "attitude", msg
        pass
    def mavlink_global_position_int_handler(self, msg):
        self.alt = msg.alt
        self.vz = msg.vz
        # print "global position int", msg
        # pass
    def mavlink_rc_channels_raw_handler(self, msg):
        self.throttle = msg.chan4_raw
        # print "rc channels raw", msg
        # pass
    def mavlink_servo_output_raw_handler(self, msg):
        # print "servo output raw", msg
        pass

    def objective(self, params):
        print "params", params
        # reset stuff
        self.running = True
        self.run_cnt = 0
        self.armed = False

        # set PID values
        # first request the values
        for i in range(3):
            self.ds.mavo.mav.param_request_list_send(21, 46)
            time.sleep(0.5)
        
        # alt_offset = BF_PID.PIDALT*3
        for i in range(3):
            self.ds.mavo.mav.param_set_send(21, 46, "P_ALT", params[0], mavlink.MAVLINK_TYPE_UINT8_T)
            self.ds.mavo.mav.param_set_send(21, 46, "I_ALT", params[1], mavlink.MAVLINK_TYPE_UINT8_T)
            self.ds.mavo.mav.param_set_send(21, 46, "D_ALT", params[2], mavlink.MAVLINK_TYPE_UINT8_T)
            self.ds.mavo.mav.param_set_send(21, 46, "P_VEL", params[3], mavlink.MAVLINK_TYPE_UINT8_T)
            self.ds.mavo.mav.param_set_send(21, 46, "I_VEL", params[4], mavlink.MAVLINK_TYPE_UINT8_T)
            self.ds.mavo.mav.param_set_send(21, 46, "D_VEL", params[5], mavlink.MAVLINK_TYPE_UINT8_T)
        # vel_offset = BF_PID.PIDVEL*3
        

        # init logdata
        numdata = 5 # mode, alt, vel, throttle, acc
        logdata = np.zeros((self.maxsamp, numdata))
        
        while self.running:
            # print self.mode

            logdata[self.run_cnt,0] = self.mode
            logdata[self.run_cnt,1] = np.clip(self.alt, -500., 1000.)
            logdata[self.run_cnt,2] = np.clip(self.vz, -1000., 1000.)
            logdata[self.run_cnt,3] = np.clip(self.throttle, 0., 2000.)
            logdata[self.run_cnt,4] = np.clip(self.zacc, -1000., 1000.)
            
            if self.mode == 3 and not self.armed:
                self.armed = True
            elif self.mode == 2 and self.armed:
                self.armed = False
                self.running = False
            if self.run_cnt > self.maxsamp:
                self.running = False
            self.run_cnt += 1
            time.sleep(0.02)


        # save data
        ts = time.strftime("%Y%m%d-%H%M%S")
        np.save("bf_optimize_mavlink_%s_log" % ts, logdata)
        params_array = np.array(params)
        np.save("bf_optimize_mavlink_%s_params" % ts, params_array)

        # compute performance
        # alt_active_idx = logdata[:,0] == 27 # with mag lock
        alt_active_idx = logdata[:,0] == 11
        alt_data = logdata[alt_active_idx,1]
        alt_target = np.mean(alt_data[0:10])
        alt_mse = np.mean(np.square(alt_target - alt_data))
        alt_target_mse_array = np.array((alt_target, alt_mse))
        np.save("bf_optimize_mavlink_%s_alt_target_mse" % ts, alt_target_mse_array)
    
        vel_data = logdata[alt_active_idx,2]
        vel_mse  = np.mean(np.square(vel_data))
        vel_target_mse_array = np.array((0., vel_mse))
        np.save("bf_optimize_mavlink_%s_vel_target_mse" % ts, vel_target_mse_array)
        
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
        if np.sum(alt_active_idx) < 500:
            alt_mse = 1e6
            
        # generate params, set and run
        # read data
        #  - min: altitude, RC5
        #  - max: record all
        # compute performance
        # return np.random.uniform(0, 1)
        print "alt_mse = %f, vel_mse = %f" % (alt_mse, vel_mse)
        print "mse total", alt_mse + vel_mse
        return (alt_mse + vel_mse)
        # return self.run_cnt
        
def main():
    bo = BaseflightOptimize()
    
    # for i in range(3):
    #     params = np.random.uniform(0, 20, (6,))
    #     # params.dtype = int
    #     print bo.objective(params.astype(int))
    # # while bo.running:
    # #     time.sleep(1.0)

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
    best = fmin(bo.objective, space, algo=tpe.suggest, max_evals=50, trials=trials)
    print "best", best

    
if __name__ == "__main__":
    main()
