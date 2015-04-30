#!/usr/bin/env python

import numpy as np

def objective(params):
    y = params[0]**2 + params[1]**2 + params[2]**2
    print "params", params, "y", y
    return y

def main():
    from hyperopt import hp, fmin, tpe, Trials, rand
    space = [
        hp.quniform("alt_p", -20, 20, 1),
        hp.quniform("alt_i", -20, 20, 1),
        hp.quniform("alt_d", -20, 20, 1),
    ]
    trials = Trials()
    # print "dir(trials)", dir(trials)
    # print trials.trials
    # best = fmin(objective, space, algo=rand.suggest, max_evals=100, trials=trials, rseed=1010101010)
    best = fmin(objective, space, algo=tpe.suggest, max_evals=500, trials=trials, rseed=1010101010)
    # print trials.view()
    # print "spec", trials.specs
    print "best", best

if __name__ == "__main__":
    main()
