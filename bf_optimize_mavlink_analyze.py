#!/usr/bin/env python

import argparse, sys, os, time
import numpy as np
import matplotlib.pylab as pl
from matplotlib import gridspec
from matplotlib import cm
import pandas as pd
import tables as tb

modes = {"mse_sequential": 0, "convert_to_tables": 1}

class BFOptMLParams(tb.IsDescription):
    name = tb.StringCol(100)
    alt_p = tb.Int32Col()
    alt_i = tb.Int32Col()
    alt_d = tb.Int32Col()
    vel_p = tb.Int32Col()
    vel_i = tb.Int32Col()
    vel_d = tb.Int32Col()

class BFOptMLTimeseries(tb.IsDescription):
    name = tb.StringCol(100)
    # index = tb.Int32Col()
    mode = tb.Int16Col()
    alt = tb.Float32Col()
    vz = tb.Float32Col()
    throttle = tb.Float32Col()
    zacc = tb.Float32Col()
    
class BFOptMLPerf(tb.IsDescription):
    name = tb.StringCol(100)
    alt_target = tb.Float32Col()
    alt_mse    = tb.Float32Col()
    vel_target = tb.Float32Col()
    vel_mse    = tb.Float32Col()
    
class BFOptML(tb.IsDescription):
    name = tb.StringCol(100)
    params = BFOptMLParams()
    # timeseries = BFOptMLTimeseries()
    # timeseries = tb.CArray()
    perf = BFOptMLPerf()

def load_data_from_dir(args):
    expr_dir = args.datadir
    results = [f for f in os.listdir(expr_dir) if f.startswith("bf_optimize_mavlink_20150507-")]
    results.sort()

    # print results
    # open outfile
    print "optrun", args.optrun
    # tblfilename = "bf_optimize_mavlink_optrun_%s.h5" % args.optrun
    tblfilename = "bf_optimize_mavlink.h5"
    h5file = tb.open_file(tblfilename, mode = "w",
                          title = "Baseflight optimization runs")
    g1     = h5file.create_group(h5file.root, "%s" %args.optrun, "Optimization run params and perf")
    g_params     = h5file.create_group(g1, "params",     "Optimization run params")
    g_perf       = h5file.create_group(g1, "perf",       "Optimization run performance")
    g_timeseries = h5file.create_group(g1, "timeseries", "Optimization run timeseries")
    # table = h5file.create_table(g1, 'evaluation', BFOptML, "Single optimizer evaluation")
    # bfoptml = table.row
    # print bfoptml
    print h5file
    # h5file
    
    index2 = ["timestamp", "alt_target", "alt_mse", "vel_target", "vel_mse"]
    # extract timestamps
    tss = np.array([r.split("_")[3] for r in results])
    # tss = [r.split("-")[2] + "-" + r.split("-")[3].split(".")[0] for r in results]
    # tss = [r.split("-")[2] + "-" + r.split("-")[3] + "-" + r.split("-")[4].split(".")[0] for r in results]
    print np.unique(tss)
    # get lag and performance
    for ts in np.unique(tss):
        # get config data
        # print ts
        tsh5 = ts.split("-")[0] + ts.split("-")[1]
        params   = np.load("%s/bf_optimize_mavlink_%s_params.npy" % (expr_dir, ts))
        # c_params = h5file.create_carray(g_params, "%s" % args.optrun, params.dtype, params.shape)
        # c_params = params
        perf_alt = np.load("%s/bf_optimize_mavlink_%s_alt_target_mse.npy" % (expr_dir, ts))
        perf_vel = np.load("%s/bf_optimize_mavlink_%s_vel_target_mse.npy" % (expr_dir, ts))
        perf =  np.hstack((perf_alt, perf_vel))
        logdata  = np.load("%s/bf_optimize_mavlink_%s_log.npy" % (expr_dir, ts))
        print params, perf
        # print "log", logdata
        # # x1 = np.load("%s/bf_optimize_mavlink_%s_alt_target_mse.npy" % (expr_dir, ts))
        # # x2 = np.load("%s/bf_optimize_mavlink_%s_vel_target_mse.npy" % (expr_dir, ts))
        # # data = []
        # # data.append(str(x1[0]))
        # # data.append(str(x1[1]))
        # # data.append(str(x2[0]))
        # # data.append(str(x2[1]))
        # # print "x1", x1
        # # print "x2", x2
        # # print "data", data
        # # get dataframe
        # # df_raw = pd.read_csv(resultfile)
        # # df_raw.columns = map(lambda x: x.replace(" ", ""), df_raw.columns)
        
        # # set run name
        # bfoptml["name"] = tsh5
        # # set run params
        # bfoptml["params/name"] = tsh5 + "_params"
        # bfoptml["params/alt_p"] = params[0]
        # bfoptml["params/alt_i"] = params[1]
        # bfoptml["params/alt_d"] = params[2]
        # bfoptml["params/vel_p"] = params[3]
        # bfoptml["params/vel_i"] = params[4]
        # bfoptml["params/vel_d"] = params[5]
        # # set run perf
        # bfoptml["perf/name"] = tsh5 + "_perf"
        # bfoptml["perf/alt_target"] = perf_alt[0]
        # bfoptml["perf/alt_mse"]    = perf_alt[1]
        # bfoptml["perf/vel_target"] = perf_vel[0]
        # bfoptml["perf/vel_mse"]    = perf_vel[1]
        # # # set run timeseries
        # # shape = logdata.shape
        # # atom = tb.Float32Atom()
        # # # atomarr = tb.CArray(atom, shape)
        
        h5file.create_array(g_params, "_%s" % tsh5, params)
        h5file.create_array(g_perf, "_%s" % tsh5, perf)
        h5file.create_array(g_timeseries, "_%s" % tsh5, logdata)
        # # bfoptml["timeseries/name"] = ts + "_timeseries"
        
        # bfoptml.append()
        
    # table.flush()
    print h5file
    
def plot_mse_consecutive(args):
    load_data_from_dir(args)
    print args

def convert_to_tables(args):
    load_data_from_dir(args)
    print args

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mode", dest="mode", help="Mode, one of " + ", ".join(modes.keys()), default="mse_consecutive")
    parser.add_argument("-d", "--datadir", dest="datadir", default=".")
    parser.add_argument('-l', "--logfiles", action='append', dest='logfiles',
                        default=[], nargs = "+",
                        help='Add logfiles for analysis')
    parser.add_argument("-lg", "--legend", dest="legend", action="store_true")
    parser.add_argument('-ps', "--plotsave", action='store_true', help='Save plot to pdf?')
    parser.add_argument('-df', "--dofit", action='store_true', help='Do fit a curve to data')
    parser.add_argument('-or', "--optrun", default=time.strftime("%Y%m%d-%H%M%S"),
                        help='Name of optimization run')

    args = parser.parse_args()
    if len(args.logfiles) < 1:
        print "need to pass at least one logfile"
        sys.exit(1)
        
    if args.mode == "mse_consecutive":
        plot_mse_consecutive(args)
    elif args.mode == "convert_to_tables":
        convert_to_tables(args)
