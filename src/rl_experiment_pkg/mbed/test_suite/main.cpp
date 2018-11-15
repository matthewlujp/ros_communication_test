#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "mbed.h"
#include "QEI.h"
#include <rl_experiment_pkg/Run.h>
#include <rl_experiment_pkg/Parameter.h>
#include <Eigen/Dense.h>
using namespace Eigen;
using namespace rl_experiment_pkg;


// define constants
namespace {
    constexpr int NUM_SENSOR = 2;
    constexpr int NUM_ACTUATOR = 2;
}


// define specific types alias 
using Sensors = Matrix<float, 1, NUM_SENSOR>;
using Phases = Matrix<float, 1, NUM_ACTUATOR>;
using Angles = Matrix<float, 1, NUM_ACTUATOR>;
using Time = double;



namespace {
    DigitalOut led1(LED1);
    DigitalOut led2(LED2);
}


// feedback network
namespace FeedbackNet {
    using Weight = Matrix<float, NUM_SENSOR, NUM_ACTUATOR>;
    using Bias = Matrix<float, 1, NUM_ACTUATOR>;

    Weight w_sin, w_cos;
    Bias b_sin, b_cos;

    int get_weight_row(int idx) { return idx / NUM_ACTUATOR; }
    int get_weight_col(int idx) { return idx % NUM_ACTUATOR; }
}


// prototype declaration
void set_parameters(const Parameter::Request&, Parameter::Response&);
void run(const Run::Request&, Run::Response&);

// rosserial related 
namespace rosserial {
    ros::NodeHandle nh;
}





int main() {    
    // init subscriber
    rosserial::nh.initNode();

    auto ss_params = ros::ServiceServer<Parameter::Request, Parameter::Response>("set_parameter", set_parameters);
    auto ss_run = ros::ServiceServer<Run::Request, Run::Response>("run_robot", run);
    rosserial::nh.advertiseService(ss_params);
    rosserial::nh.advertiseService(ss_run);

    while(1) {
        rosserial::nh.spinOnce();
        wait_ms(100);
    }

    return 0;
}



// Callback on receiving data
// iterate over values 
// parameters are w_cos, w_sin, b_con, b_sin
// TODO: consider testing
void set_parameters(const Parameter::Request &req, Parameter::Response &res) {
    // extract weight_cos
    for (int i = 0; i < req.weight_cos_length; i++){
        FeedbackNet::w_cos(FeedbackNet::get_weight_row(i), FeedbackNet::get_weight_col(i)) = req.weight_cos[i];
    }

    // extract weight_sin
    for (int i=0; i< req.weight_sin_length; i++) {
        FeedbackNet::w_sin(FeedbackNet::get_weight_row(i), FeedbackNet::get_weight_col(i)) = req.weight_sin[i];
    }

    // extract bias_cos
    for (int i=0; i< req.bias_cos_length; i++) {
        FeedbackNet::b_cos(0, i) = req.bias_cos[i];
    }

    // extract bias_sin
    for (int i=0; i< req.bias_sin_length; i++) {
        FeedbackNet::b_sin(0, i) = req.bias_sin[i];
    }


    // test values
    FeedbackNet::Weight expected_w_cos, expected_w_sin;
    FeedbackNet::Bias expected_b_cos, expected_b_sin;
    
    expected_w_cos << 0.1, 0.2, 0.3, 0.4;
    expected_w_sin << 0.5, 0.6, 0.7, 0.8;
    expected_b_cos << 0.9, 1.0;
    expected_b_sin << 1.1, 1.2;
    
    bool wrong_flag = false;
    string error_msg = "";
    if (FeedbackNet::w_cos != expected_w_cos) {
        error_msg += "w_cos wrong, ";   
        wrong_flag = true;
    }
    if (FeedbackNet::w_sin != expected_w_sin) {
        error_msg = "w_sin wrong, ";   
        wrong_flag = true;
    }
    if (FeedbackNet::b_cos != expected_b_cos) {
        error_msg = "b_cos wrong, ";   
        wrong_flag = true;
    }
    if (FeedbackNet::b_sin != expected_b_sin) {
        error_msg = "b_sin wrong, ";   
        wrong_flag = true;
    }
    res.error = error_msg.c_str();

    // led1 = 1;
    // if (!wrong_flag) {
    //     for (int i=0; i<20; i++) {
    //         led1 = !led1;
    //         wait_ms(200);
    //     }
    // }
}


// start running caterpillar
void run(const Run::Request &req, Run::Response &res) {
    unsigned int steps = req.steps;
    string error_msg = "";
    if (steps != 5030) {
        error_msg += "get wrong steps";
    } else {
        error_msg += "get right steps";
        // led2 = 1;
        // for (int i=0; i<20; i++) {
        //     led2 = !led2;
        //     wait_ms(200);
        // }
    }
    res.error = error_msg.c_str();
}