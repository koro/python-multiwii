from __future__ import print_function

import tables as tb

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
    
class BFOptMLOld(tb.IsDescription):
    name = tb.StringCol(100)
    params = BFOptMLParams()
    # timeseries = BFOptMLTimeseries()
    # timeseries = tb.CArray()
    perf = BFOptMLPerf()

class BFOptML(tb.IsDescription):
    # name = tb.StringCol(100)
    id = tb.Int64Col()
    alt_p = tb.Int32Col()
    alt_i = tb.Int32Col()
    alt_d = tb.Int32Col()
    vel_p = tb.Int32Col()
    vel_i = tb.Int32Col()
    vel_d = tb.Int32Col()
    alt_target = tb.Float32Col()
    alt_mse    = tb.Float32Col()
    vel_target = tb.Float32Col()
    vel_mse    = tb.Float32Col()
    mse        = tb.Float32Col()
    timeseries = tb.Float32Col(shape=(4000, 5))
    
class BFOptML2(tb.IsDescription):
    # name = tb.StringCol(100)
    id = tb.Int64Col()
    alt_p = tb.Int32Col()
    alt_i = tb.Int32Col()
    alt_d = tb.Int32Col()
    vel_p = tb.Int32Col()
    vel_i = tb.Int32Col()
    vel_d = tb.Int32Col()
    alt_target = tb.Float32Col()
    alt_mse    = tb.Float32Col()
    vel_target = tb.Float32Col()
    vel_mse    = tb.Float32Col()
    mse        = tb.Float32Col()
    timeseries = tb.Float32Col(shape=(4000, 6))

if __name__ == "__main__":
    print("class definition file only")
