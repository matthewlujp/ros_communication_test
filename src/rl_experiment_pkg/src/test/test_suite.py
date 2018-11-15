#!/usr/bin/python
import rospy
from rl_experiment_pkg.srv import Run, Parameter
import sys
import numpy as np

SERVICE_SET_PARAMETER = 'set_parameter'
SERVICE_RUN = 'run_robot'



rospy.init_node('trainer')
rospy.wait_for_service(SERVICE_SET_PARAMETER)
rospy.wait_for_service(SERVICE_RUN)

_parameter_setter = rospy.ServiceProxy(SERVICE_SET_PARAMETER, Parameter)
_runner = rospy.ServiceProxy(SERVICE_RUN, Run)

WEIGHT_SIZE = 2 * 2
BIAS_SIZE = 2


def send_params(raw_params):
    # numpy array to Parameter.weight_cos, weight_sin, bias_cos, bias_sin
    weight_cos = list(raw_params[:WEIGHT_SIZE])
    weight_sin = list(raw_params[WEIGHT_SIZE:2*WEIGHT_SIZE])
    bias_cos = list(raw_params[2*WEIGHT_SIZE:2*WEIGHT_SIZE + BIAS_SIZE])
    bias_sin = list(raw_params[2*WEIGHT_SIZE + BIAS_SIZE:])

    res = _parameter_setter(weight_cos, weight_sin, bias_cos, bias_sin)
    return res.error

    
def run(steps):
    res = _runner(steps)
    return res.error



if __name__ == '__main__':
    # parameters check
    print("send parameters...")    
    raw_params = np.array([
        0.1, 0.2, 0.3, 0.4,
        0.5, 0.6, 0.7, 0.8,
        0.9, 1.0,
        1.1, 1.2,    
    ])

    err = send_params(raw_params)
    if err == '':
        print("parameter sending succeeded!")
    else:
        print("parameters sending got error, {}".format(err))


    # run check
    print("run...")
    err = run(5030)
    if err == '':
        print("steps sending succeeded!")
    else:
        print("step sending got error, {}".format(err))
