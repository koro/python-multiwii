#!/usr/bin/env python

import Queue

import time, sys, argparse
import numpy as np
import pylab as pl
import tables as tb

# sys.path.insert(0, "/home/src/QK/smp/neural")
sys.path.insert(0, "/home/src/QK/mavlink/pymavlink/")
# sys.path.insert(0, "/home/x75/tmp")

# import pymavlink
import mavutil
import mavlinkv10 as mavlink

from DataSource import DataSource
from multiwii_serial import BF_PID

from bf_optimize_data import BFOptML, BFOptML2

class BaseflightOptimize(object):
    def __init__(self):
        self.dq = Queue.Queue
        self.ds = DataSource(self, self.dq, 21)
        self.ds.start()
        self.x_flag = False
        self.running = True

        self.maxsamp        = 4000
        self.run_cnt        = 0
        self.run_cnt_active = 0
        self.lim_run_cnt_active = 1500
        self.eval_cnt       = 0

        self.armed = False
        self.numdata = 6 # mode, alt, vel, throttle, zacc, battery
        self.mode = 0
        self.alt = 0
        self.vz = 0
        self.throttle = 0
        self.zacc = 0
        self.batt = 0
        self.storage_version = "v2"

        # data storage
        self.tblfilename = "bf_optimize_mavlink.h5"
        self.h5file = tb.open_file(self.tblfilename, mode = "a")
        # check if top group exists
        try:
            self.g1    = self.h5file.get_node("/%s" % self.storage_version)
            # self.table = self.h5file.list_nodes("/v1")[0]
            self.table = self.h5file.get_node("/%s/evaluations" % self.storage_version)
        except:
            self.g1     = self.h5file.create_group(self.h5file.root, self.storage_version,
                                               "Optimization run params, perf and logdata")
            if self.storage_version == "v1":
                self.table = self.h5file.create_table(self.g1, 'evaluations', BFOptML,
                                                      "Single optimizer evaluations")
            elif self.storage_version == "v2":
                self.table = self.h5file.create_table(self.g1, 'evaluations', BFOptML2,
                                                      "Single optimizer evaluations")
        print self.g1, self.table
        self.bfoptml = self.table.row
        # if g1 exists
        #  pass
        # else
        # g1     = h5file.create_group(h5file.root, "v1", "Optimization run params, perf and logdata")

    def mavlink_sys_status_handler(self, msg):
        self.mode = msg.onboard_control_sensors_health # ugly hack
        self.batt = msg.voltage_battery #
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

    def reset(self):
        self.running = True
        self.run_cnt = 0
        self.run_cnt_active = 0
        self.armed = False
    
    def objective(self, params):
        self.eval_cnt += 1
        print "eval#", self.eval_cnt, "params", params
        # reset stuff
        self.reset()
        
        # check if we have data for this parameterization
        # stored_params = [(x["alt_p"], x["alt_i"], x["alt_d"], x["vel_p"], x["vel_i"], x["vel_d"]) for x in table.]
        existing_run_data = \
        [ (x["alt_p"], x["alt_i"], x["alt_d"], x["vel_p"], x["vel_i"], x["vel_d"], x["mse"])
                 for x in self.table.where("""(alt_p == %d) & (alt_i == %d) & (alt_d == %d) & \
                 (vel_p == %d) & (vel_i == %d) & (vel_d == %d)""" %
                                      tuple([params[i] for i in range(6)]))]
        if len(existing_run_data) > 0:
            mse = existing_run_data[-1][-1]
            print "reusing existing run data: mse = %f" % mse
            time.sleep(0.1)
            return mse

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
        logdata = np.zeros((self.maxsamp, self.numdata))
        
        while self.running:
            # print self.mode

            logdata[self.run_cnt,0] = np.clip(self.mode, 0., 1000.)
            logdata[self.run_cnt,1] = np.clip(self.alt, -500., 1000.)
            logdata[self.run_cnt,2] = np.clip(self.vz, -1000., 1000.)
            logdata[self.run_cnt,3] = np.clip(self.throttle, 0., 2000.)
            logdata[self.run_cnt,4] = np.clip(self.zacc, -1000., 1000.)
            logdata[self.run_cnt,5] = np.clip(self.batt, 0., 20000.) #

            # detect premature termination: landing and disarm
            if self.mode == 3 and not self.armed:
                self.armed = True
            elif self.mode == 2 and self.armed:
                self.armed = False
                self.running = False
                
            self.run_cnt += 1

            # match MSP telemetry update rate
            time.sleep(0.02)

            # count number of controller active samples
            if self.armed and self.mode == 11:
                self.run_cnt_active += 1

            # terminate if sufficient number of controller active samples
            if self.run_cnt_active == self.lim_run_cnt_active:
                self.running = False

            # terminate if maximum number of samples
            if self.run_cnt >= self.maxsamp:
                self.running = False
                


        # save data
        ts = time.strftime("%Y%m%d%H%M%S")
        np.save("logs/bf_optimize_mavlink_%s_log" % ts, logdata)
        params_array = np.array(params)
        np.save("logs/bf_optimize_mavlink_%s_params" % ts, params_array)

        # compute performance
        # alt_active_idx = logdata[:,0] == 27 # with mag lock
        alt_active_idx = logdata[:,0] == 11
        # catch empty index
        if np.sum(alt_active_idx) == 0:
            # set two elements True to have at least 2 element arrays below, even if they're bogus
            alt_active_idx[0] = True 
            alt_active_idx[1] = True

        # it was so bad, alt hold was disabled prior to lim_cnt_active
        if np.sum(alt_active_idx) < (self.lim_run_cnt_active - 100):
            alt_mse = 1e5
            vel_mse = 1e5
            alt_target = 0.
            vel_target = 0.
        else:
            alt_data = logdata[alt_active_idx,1]
            alt_target = np.mean(alt_data[0:10])
            alt_mse = np.mean(np.square(alt_target - alt_data))
        
            vel_data = logdata[alt_active_idx,2]
            vel_target = 0.
            vel_mse  = np.mean(np.square(vel_target - vel_data))
            
        mse_array = np.array((alt_target, alt_mse, vel_target, vel_mse))
        np.save("logs/bf_optimize_mavlink_%s_mse" % ts, mse_array)

        # save data to pytable
        self.bfoptml["id"] = int(ts)
        self.bfoptml["alt_p"] = params[0]
        self.bfoptml["alt_i"] = params[1]
        self.bfoptml["alt_d"] = params[2]
        self.bfoptml["vel_p"] = params[3]
        self.bfoptml["vel_i"] = params[4]
        self.bfoptml["vel_d"] = params[5]
        # set run performance measure
        self.bfoptml["alt_target"] = alt_target
        self.bfoptml["alt_mse"]    = alt_mse
        self.bfoptml["vel_target"] = vel_target
        self.bfoptml["vel_mse"]    = vel_mse
        self.bfoptml["mse"]        = alt_mse + vel_mse
        # set run logdata
        self.bfoptml["timeseries"]    = logdata
        # append new row
        self.bfoptml.append()
        self.table.flush()
                
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
    parser = argparse.ArgumentParser()
    parser.add_argument("-me", "--maxeval", default=60)

    args = parser.parse_args()
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
    best = fmin(bo.objective, space, algo=tpe.suggest, max_evals=int(args.maxeval), trials=trials)
    print "best", best

    
if __name__ == "__main__":
    main()
