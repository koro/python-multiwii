python-multiwii
===============

Python comunication library for multiwii multicopters


Originally by arcoslab/python-multiwii.

Oswald Berthold (2015)

After trying to use pyserial together with the python-multiwii MSP parser i changed my approach and used an external MSP to mavlink bridge (MavHUB [1]) and used mavlink from within python.

The reason is that although the first approach basically works (pure python), i seem to have problems with the performance of pyserial and i can't get more than about 5Hz data rate on the multiwii telemetry.

The whole idea is to use a HOA (Hyperparameter Optimization Algorithm), hyperopt in this case, for optimizing PID parameters with respect to a given cost (MSE setpoint - value). It seems to work, see

    python bf_optimize_mavlink_analyze.py -m pid_pca -l bf_optimize_mavlink.h5

although this is preliminary and more experiments are needed.

Although it is quite slow overall (100 runs of each about 1 minute in total, 30 secs flying evaluation time) there is still room for improvement. The procedure constitutes a kind of Policy Search Reinforcement Learning algorithm.

[1] https://github.com/calihem/mavhub