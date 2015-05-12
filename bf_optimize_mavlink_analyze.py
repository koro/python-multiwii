#!/usr/bin/env python

import argparse, sys, os, time
import numpy as np
import matplotlib.pylab as pl
from matplotlib import gridspec
from matplotlib import cm
import pandas as pd
import tables as tb

from bf_optimize_data import BFOptML

modes = {"mse_sequential": 0, "convert_to_tables": 1,
         "read_table": 2, "test_for_params": 3}

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
    g1     = h5file.create_group(h5file.root, "v1", "Optimization run params, perf and logdata")
    # g1     = h5file.create_group(h5file.root, "%s" %args.optrun, "Optimization run params and perf")
    # g_params     = h5file.create_group(g1, "params",     "Optimization run params")
    # g_perf       = h5file.create_group(g1, "perf",       "Optimization run performance")
    # g_timeseries = h5file.create_group(g1, "timeseries", "Optimization run timeseries")
    # table = h5file.create_table(g1, 'evaluation', BFOptML2, "Single optimizer evaluation")
    table = h5file.create_table(g1, 'evaluations', BFOptML, "Single optimizer evaluations")
    bfoptml = table.row
    # print bfoptml
    print h5file
    # h5file
    
    # index2 = ["timestamp", "alt_target", "alt_mse", "vel_target", "vel_mse"]
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
    
        # set run ID
        bfoptml["id"] = int(tsh5)
        # set run params
        bfoptml["alt_p"] = params[0]
        bfoptml["alt_i"] = params[1]
        bfoptml["alt_d"] = params[2]
        bfoptml["vel_p"] = params[3]
        bfoptml["vel_i"] = params[4]
        bfoptml["vel_d"] = params[5]
        # set run performance measure
        bfoptml["alt_target"] = perf_alt[0]
        bfoptml["alt_mse"]    = perf_alt[1]
        bfoptml["vel_target"] = perf_vel[0]
        bfoptml["vel_mse"]    = perf_vel[1]
        bfoptml["mse"]        = perf_alt[1] + perf_vel[1]
        # set run logdata
        bfoptml["timeseries"]    = logdata

        # # array style version        
        # h5file.create_array(g_params, "_%s" % tsh5, params)
        # h5file.create_array(g_perf, "_%s" % tsh5, perf)
        # h5file.create_array(g_timeseries, "_%s" % tsh5, logdata)
        
        bfoptml.append()
        
    table.flush()
    # print h5file.root.v1
    
def plot_mse_consecutive(args):
    load_data_from_dir(args)
    print args

def convert_to_tables(args):
    load_data_from_dir(args)
    print args

def read_table(args):
    tblfilename = "bf_optimize_mavlink.h5"
    h5file = tb.open_file(tblfilename, mode = "r")
    # print h5file
    # a = h5file.root
    # print a
    # a = h5file.get_node(where = "/20150507-run1/params/_20150507155408")
    # print a
    table = h5file.root.v1.evaluations
    print "table", table
    # mse = [x["mse"] for x in table.iterrows() if x["alt_p"] < 20.]
    # mse = [x["mse"] for x in table.iterrows()]
    logdata = [x["timeseries"] for x in table.iterrows() if x["mse"] < 1000]
    # alt_pid = [(x["alt_p"], x["alt_i"], x["alt_d"]) for x in table.iterrows() if x["mse"] < 1000]
    alt_pid = [(x["alt_p"], x["alt_i"], x["alt_d"]) for x in table.iterrows() if x["alt_p"] == 17 and x["alt_i"] == 0.]
    print "alt_pid", alt_pid
    # print mse
    # pl.plot(mse)
    print len(logdata)
    for i in range(len(logdata)):
        pl.subplot(len(logdata), 1, i+1)
        pl.plot(logdata[i][:,1:3])
        pl.ylim((-300, 1000))
    pl.show()

def test_for_params(args):
    tblfilename = "bf_optimize_mavlink.h5"
    h5file = tb.open_file(tblfilename, mode = "r")
    table = h5file.root.v1.evaluations
    alt_p = 17
    alt_i = 0
    pids = [ (x["alt_p"], x["alt_i"], x["alt_d"], x["vel_p"], x["vel_i"], x["vel_d"])
             for x in table.where("""(alt_p == %d) & (alt_i == %d)""" % (alt_p, alt_i)) ]
    print pids

def pid_pca(args):
    import mdp
    tblfilename = "bf_optimize_mavlink.h5"
    h5file = tb.open_file(tblfilename, mode = "r")
    table = h5file.root.v1.evaluations
    pids = [ [x["alt_p"], x["alt_i"], x["alt_d"], x["vel_p"], x["vel_i"], x["vel_d"]]
             for x in table.iterrows() ]
    mses = [ [x["mse"]] for x in table.iterrows() ]
    mses_a = np.clip(np.array(mses), 0, 20000.)
    mses_a /= np.max(mses_a)
    pl.subplot(211)
    pl.plot(pids)
    pl.subplot(212)
    pid_p = mdp.pca(np.array(pids).astype(float))
    # [:,0:2]
    colors = np.zeros((100, 3))
    colors = np.hstack((colors, 1-mses_a))
    print colors.shape
    pl.scatter(pid_p[:,0], pid_p[:,1], color=colors)
    # pl.scatter(pid_p[:,0], pid_p[:,1], alpha=1.)
    pl.show()

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
    elif args.mode == "read_table":
        read_table(args)
    elif args.mode == "test_for_params":
        test_for_params(args)
    elif args.mode == "pid_pca":
        pid_pca(args)
