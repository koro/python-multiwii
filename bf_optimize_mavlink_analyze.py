#!/usr/bin/env python

import argparse, sys, os, time
import numpy as np
import matplotlib.pylab as pl
from matplotlib import gridspec
from matplotlib import cm
import pandas as pd
import tables as tb

from bf_optimize_data import BFOptML

modes = {"mse_consecutive": 0, "convert_to_tables": 1,
         "read_table": 2, "test_for_params": 3, "pid_pca": 4,
         "plot_episode": 5, "te_alt_motor": 6}

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
    # table = h5file.root.v1.evaluations
    table = h5file.root.v2.evaluations
    print "table", table
    # mse = [x["mse"] for x in table.iterrows() if x["alt_p"] < 20.]
    # mse = [x["mse"] for x in table.iterrows()]
    logdata = [x["timeseries"] for x in table.iterrows() if x["mse"] < 2000]
    alt_pid = [(x["alt_p"], x["alt_i"], x["alt_d"], x["vel_p"], x["vel_i"], x["vel_d"]) for x in table.iterrows() if x["mse"] < 1000]
    # alt_pid = [(x["alt_p"], x["alt_i"], x["alt_d"]) for x in table.iterrows() if x["alt_p"] == 17 and x["alt_i"] == 0.]
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

def plot_episode(args):
    """Plot an episode plucked from the large h5 database"""
    print "plot_episode"
    # load the data file
    tblfilename = "bf_optimize_mavlink.h5"
    h5file = tb.open_file(tblfilename, mode = "a")
    # get the table handle
    table = h5file.root.v2.evaluations

    # selected episode
    episode_row = table.read_coordinates([int(args.epinum)])
    # compare episodes
    episode_row_1 = table.read_coordinates([2, 3, 22, 46]) # bad episodes
    print "row_1", episode_row_1.shape
    # episode_row = table.read_coordinates([3, 87])
    episode_target = episode_row["alt_target"]
    episode_target_1 = [row["alt_target"] for row in episode_row_1]
    print "episode_target_1.shape", episode_target_1
    episode_timeseries = episode_row["timeseries"][0]
    episode_timeseries_1 = [row["timeseries"] for row in episode_row_1]
    print "row", episode_timeseries.shape
    print "row_1", episode_timeseries_1

    sl_start = 0
    sl_end = 2500
    sl_len = sl_end - sl_start
    sl = slice(sl_start, sl_end)
    pl.plot(episode_timeseries[sl,1], "k-", label="alt", lw=2.)
    print np.array(episode_timeseries_1)[:,:,1]
    pl.plot(np.array(episode_timeseries_1)[:,:,1].T, "k-", alpha=0.2)
    # alt_hold = episode_timeseries[:,0] > 4
    alt_hold_act = np.where(episode_timeseries[sl,0] == 11)
    print "alt_hold_act", alt_hold_act[0].shape, sl_len
    alt_hold_act_min = np.min(alt_hold_act)
    alt_hold_act_max = np.max(alt_hold_act)
    print "min, max", alt_hold_act_min, alt_hold_act_max, alt_hold_act_min/float(sl_len), alt_hold_act_max/float(sl_len),

    # pl.plot(episode_timeseries[sl,0] * 10, label="mode")
    pl.axhspan(-100., 1000,
               alt_hold_act_min/float(sl_len),
               alt_hold_act_max/float(sl_len),
               facecolor='0.5', alpha=0.25)
    pl.axhline(episode_target, label="target")
    pl.xlim((0, sl_len))
    pl.xlabel("Time steps [1/50 s]")
    pl.ylabel("Alt [cm]")
    pl.legend()
    if args.plotsave:
        pl.gcf().set_size_inches((10, 3))
        pl.gcf().savefig("%s.pdf" % (sys.argv[0][:-3]), dpi=300, bbox_inches="tight")
    pl.show()

def pid_pca(args):
    #  import modular data processing toolkit
    import mdp
    # load data file
    tblfilename = "bf_optimize_mavlink.h5"
    h5file = tb.open_file(tblfilename, mode = "a")
    # table = h5file.root.v1.evaluations
    #  get tabke handle
    table = h5file.root.v2.evaluations

    # sort rows
    if not table.cols.mse.is_indexed:
        table.cols.mse.createCSIndex()
        
    if not args.sorted:
        pids = [ [x["alt_p"], x["alt_i"], x["alt_d"], x["vel_p"], x["vel_i"], x["vel_d"]]
             for x in table.iterrows() ]
        mses = [ [x["mse"]] for x in table.iterrows() ]
    else:
        pids = [ [x["alt_p"], x["alt_i"], x["alt_d"], x["vel_p"], x["vel_i"], x["vel_d"]] for x in table.itersorted("mse")]
        mses = [ [x["mse"]] for x in table.itersorted("mse")]
    print "best two", pids
    mses_a = np.log(np.clip(np.array(mses), 0, 200000.))
    mses_a /= np.max(mses_a)
    # FIXME: try kernel pca on this
    from sklearn.decomposition import PCA, KernelPCA, SparsePCA
    kpca = KernelPCA(n_components = None,
                     kernel="rbf", degree=6, fit_inverse_transform=True,
                     gamma=1/6., alpha=1.)
    # kpca = SparsePCA(alpha=2., ridge_alpha=0.1)
    X_kpca = kpca.fit_transform(np.asarray(pids).astype(float))
    # X_back = kpca.inverse_transform(X_kpca)

    Z_kpca = kpca.transform(np.asarray(pids).astype(float))

    print Z_kpca.shape, X_kpca.shape
    print "|Z_kpca|", np.linalg.norm(Z_kpca, 2, axis=1)
    # for i in range(8):
    #     pl.subplot(8,1,i+1)
    #     pl.plot(Z_kpca[:,i])
    #     pl.legend()
    # pl.show()

    
    # fast PCA
    # pid_p = mdp.pca(np.array(pids).astype(float))
    pid_array = np.array(pids).astype(float)
    print "pid_array.shape", pid_array.shape
    pcanode = mdp.nodes.PCANode(output_dim = 6)
    # pcanode.desired_variance = 0.75
    pcanode.train(np.array(pids).astype(float))
    pcanode.stop_training()
    print "out dim", pcanode.output_dim

    pid_p = pcanode.execute(np.array(pids).astype(float))

    # pid_array_mse = np.hstack((np.array(pids).astype(float), mses_a))
    pid_ica = mdp.fastica(np.array(pids).astype(float))
    print "ica.shape", pid_ica.shape
    # pid_p = np.asarray(pids)[:,[0, 3]]
    # pid_p = pids[:,0:2]
    # [:,0:2]
    sl_start = 0
    sl_end = 100
    sl = slice(sl_start, sl_end)

    print "expl var", pcanode.explained_variance
    pl.subplot(111)
    colors = np.zeros((100, 3))
    # colors = np.hstack((colors, 1-(0.5*mses_a)))
    colors = np.hstack((colors, 1-(0.8*mses_a)))
    # print colors.shape
    # pl.scatter(pid_p[sl,0], pid_p[sl,1], color=colors)

    # ica spektrum
    pid_ica_sum = np.sum(np.square(pid_ica), axis=0)
    # pid_ica_sum_sort = np.sort(pid_ica_sum)
    pid_ica_sum_0 = np.argmax(pid_ica_sum)
    pid_ica_sum[pid_ica_sum_0] = 0
    pid_ica_sum_1 = np.argmax(pid_ica_sum)
    
    # pl.scatter(pid_p[sl,0], pid_p[sl,1], color=colors)
    pl.scatter(pid_ica[sl,pid_ica_sum_0], pid_ica[sl,pid_ica_sum_1], color=colors)
    # pl.scatter(X_kpca[:,0], X_kpca[:,1], color=colors)
    pl.gca().set_aspect(1)
    # pl.scatter(pid_p[:,0], pid_p[:,1], alpha=1.)
    # pl.show()
     
    pl.subplot(211)
    # pl.plot(pid_p, ".")
    # pl.plot(pid_p[sl], "o")
    pl.plot(pid_ica[sl] + np.random.uniform(-0.01, 0.01, size=pid_ica[sl].shape),
            "o")
    pl.xlim((sl_start - 0.2, sl_end + 0.2))
    # pl.plot(Z_kpca[:,:], "-o", label="kpca")
    # pl.plot(Z_kpca[:,:], ".", label="kpca")
    # pl.legend()
        
    pl.subplot(212)
    pl.plot(mses_a[sl], "ko")
    # pl.gca().set_yscale("log")
    pl.xlim((sl_start - 0.2, sl_end + 0.2))
    # pl.show()

    # gp fit
    x = mses_a[sl]
    x_sup = np.atleast_2d(np.arange(0, x.shape[0])).T
    x_ones = x != 1.
    print x, x_sup, x_ones.shape
    print "x[x_ones]", x[x_ones].shape
    print "x_sup[x_ones]", x_sup[x_ones].shape

    from sklearn.gaussian_process import GaussianProcess
    # gp = GaussianProcess(regr='constant', corr='absolute_exponential',
    #                  theta0=[1e-4] * 1, thetaL=[1e-12] * 1,
    #                  thetaU=[1e-2] * 1, nugget=1e-2, optimizer='Welch')
    gp = GaussianProcess(corr="squared_exponential",
                         theta0=1e-2, thetaL=1e-4, thetaU=1e-1,
                         nugget=1e-1/x[x_ones])
    gp.fit(x_sup[x_ones,np.newaxis], x[x_ones,np.newaxis])
    x_pred, sigma2_pred = gp.predict(x_sup, eval_MSE=True)
    print x_pred, sigma2_pred

    from sklearn import linear_model
    clf = linear_model.Ridge (alpha = .5)
    clf.fit(x_sup[x_ones,np.newaxis], x[x_ones,np.newaxis])
    x_pred = clf.predict(x_sup)
        
    pl.subplot(111)
    pl.plot(mses_a[sl], "ko")
    pl.plot(x_pred, "k-", alpha=0.5)
    # pl.plot(x_pred + sigma2_pred, "k-", alpha=0.5)
    # pl.plot(x_pred - sigma2_pred, "k-", alpha=0.5)
    # pl.gca().set_yscale("log")
    pl.xlim((sl_start - 0.2, sl_end + 0.2))
    pl.xlabel("Episode #")
    pl.ylabel("MSE")
    if arg.plotsave:
        pl.gcf().set_size_inches((10, 3))
        pl.gcf().savefig("%s-mse.pdf" % (sys.argv[0][:-3]), dpi=300,
                        bbox_inches="tight")
    pl.show()

################################################################################
# compute transfer entropy from altitude estimate to motor over all evaluations
def te_alt_motor(args):
    """calc sensor/motor TE"""
    tblfilename = "bf_optimize_mavlink.h5"
    h5file = tb.open_file(tblfilename, mode = "a")
    table = h5file.root.v2.evaluations
    # table.cols.mse.createCSIndex()

    from jpype import *
    # I think this is a bit of a hack, python users will do better on this:
    sys.path.append("../../infodynamics-dist/demos/python")
    import readFloatsFile

    # Add JIDT jar library to the path

    jarLocation = "../../infodynamics-dist/infodynamics.jar"
    # Start the JVM (add the "-Xmx" option with say 1024M if you get crashes due to not enough memory space)
    startJVM(getDefaultJVMPath(), "-ea", "-Djava.class.path=" + jarLocation)

    # 0. Load/prepare the data:
    dataRaw = readFloatsFile.readFloatsFile("/home/src/QK/infodynamics-dist/demos/data/2coupledBinaryColsUseK2.txt")
    # As numpy array:
    data = np.array(dataRaw)
    source = data[:,0]
    dest = data[:,1]
    print type(source), source.shape
    print source.dtype


    # mutual information
    # 1. Construct the calculator:
    calcClassMI = JPackage("infodynamics.measures.continuous.kraskov").MutualInfoCalculatorMultiVariateKraskov1
    calcMI = calcClassMI()
    # 2. Set any properties to non-default values:
    calcMI.setProperty("TIME_DIFF", "1")
    # 3. Initialise the calculator for (re-)use:
    calcMI.initialise()
    
    # transfer entropy    
    # 1. Construct the calculator:
    calcClass = JPackage("infodynamics.measures.continuous.kraskov").TransferEntropyCalculatorKraskov
    calc = calcClass()
    # 2. Set any properties to non-default values:
    calc.setProperty("k_HISTORY", "1")
    # calc.setProperty("k_TAU", "2")
    calc.setProperty("l_HISTORY", "100")
    # calc.setProperty("l_TAU", "2")
    # 3. Initialise the calculator for (re-)use:
    calc.initialise()

    # 4. Supply the sample data:
    calc.setObservations(source, dest)
    # 5. Compute the estimate:
    result = calc.computeAverageLocalOfObservations()
    print("TE_Kraskov (KSG)(col_0 -> col_1) = %.4f nats\n" % result)
    
    
    for x in table.itersorted("mse"):
        sensor = x["timeseries"][:,1].astype(np.float64)
        motor  = x["timeseries"][:,4].astype(np.float64)

        pl.plot(sensor)
        pl.plot(motor)
        pl.show()
        # sys.exit()
        # print "s,m", sensor, motor
        # print "s,m (mean)", np.mean(sensor), np.mean(motor)
        # print "s", type(sensor), sensor.shape
        # print "m", type(motor), motor.shape

        # # 4. Supply the sample data:
        # calcMI.initialise()
        # calcMI.setObservations(sensor, motor)
        # # 5. Compute the estimate:
        # result = calcMI.computeAverageLocalOfObservations()
        # print("mse = %f, mi = %.4f nats" % (x["mse"], result))
        
        # 4. Supply the sample data:
        # print calc
        calc.setObservations(sensor, motor)
        # 5. Compute the estimate:
        result = calc.computeAverageLocalOfObservations()
        print("mse: %f, TE_Kraskov (KSG)(col_0 -> col_1) = %.4f nats" % (x["mse"], result))

    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--datadir", dest="datadir", default=".")
    parser.add_argument('-df', "--dofit", action='store_true', help='Do fit a curve to data')
    parser.add_argument("-e", "--epinum", dest="epinum", help="episode number, only with plot_episode", default=0)
    parser.add_argument('-l', "--logfiles", action='append', dest='logfiles',
                        default=[], nargs = "+",
                        help='Add logfiles for analysis')
    parser.add_argument("-lg", "--legend", dest="legend", action="store_true")
    parser.add_argument("-m", "--mode", dest="mode", help="Mode, one of " + ", ".join(modes.keys()), default="mse_consecutive")
    parser.add_argument('-or', "--optrun", default=time.strftime("%Y%m%d-%H%M%S"),
                        help='Name of optimization run')
    parser.add_argument('-ps', "--plotsave", action='store_true', help='Save plot to pdf?')
    parser.add_argument('-s', "--sorted", dest="sorted",
                        action='store_true', help='Sort table by MSE')
    parser.add_argument('-ns', "--no-sorted", dest="sorted",
                        action='store_false', help='Sort table by MSE')

    args = parser.parse_args()
    if len(args.logfiles) < 1:
        print "need to pass at least one logfile"
        sys.exit(1)

    # FIXME: replace this with exec/compile args.mode
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
    elif args.mode == "plot_episode":
        plot_episode(args)
    elif args.mode == "te_alt_motor":
        te_alt_motor(args)
    else:
        print "unknown mode: %s" % args.mode
