#!/usr/bin/python
import rospy
from rl_experiment_pkg.srv import Run, Parameter
import sys
import numpy as np

SERVICE_SET_PARAMETER = '__set_parameter__'
SERVICE_RUN = '__run__'



rospy.init_node('trainer')
rospy.wait_for_service(SERVICE_SET_PARAMETER)
_parameter_setter = rospy.ServiceProxy(SERVICE_SET_PARAMETER, Parameter)


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

    



if __name__ == '__main__':
    raw_params = np.array([
        0.1, 0.2, 0.3, 0.4,
        0.5, 0.6, 0.7, 0.8,
        0.9, 1.0,
        1.1, 1.2,    
    ])

    error = send_params(raw_params)

    if error == '':
        print("parameter seding succeeded!")
    else:
        print("got error, {}".format(error))
