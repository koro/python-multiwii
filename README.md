python-multiwii
===============

Python comunication library for multiwii multicopters


Originally by arcoslab/python-multiwii

After trying to use pyserial together with the python-multiwii MSP parser i changed my approach and used an external MSP to mavlink bridge (mavhub) and use mavlink from within python.

The reason is that although the first approach basically works (pure python), i seem to have problems with the performance of pyserial and i can't get more than about 5Hz data rate on the multiwii telemetry.

The whole idea is to use a HOA (Hyperparameter Optimization Algorithm), hyperopt in this case for optimizing PID parameters with respect to a given cost (MSE setpoint - value). It works, see

   python bf_optimize_mavlink_analyze.py -m pid_pca -l bf_optimize_mavlink.h5

