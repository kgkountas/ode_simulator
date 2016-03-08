#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>

#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>

//#include "odefun.h"
extern "C" {
#include "odefun.h"
}


using namespace boost::numeric::odeint;

ros::Publisher x_pub;

typedef boost::array<double, n> state_t;

static ros::Time lastTime;
static double u[m] = {0};
static state_t x   = {0};

void odefun_wrapper(const state_t &X, state_t &dX, double t)
{
    odefun(X.data(), dX.data(), u, t);
}

void u_callback(const std_msgs::Float64MultiArray &msg)
{
    if (msg.data.size() != m) {
        ROS_WARN("ode_simulator: u.size != %d\n", m);
        return;
    }
    
    double t0 = lastTime.toSec();
    lastTime = ros::Time::now();
    double tf = lastTime.toSec();
    double dt = 0.01;

    integrate(odefun_wrapper, x, t0, tf, dt);

    for (size_t i = 0; i < msg.data.size(); i++)
        u[i] = msg.data[i];

    std_msgs::Float64MultiArray xout;

    xout.data.clear();
    for (size_t i = 0; i < n; i++)
        xout.data.push_back(x[i]);

    x_pub.publish(xout);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ode_simulator");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string u_topic;
    pnh.param<std::string>("u", u_topic, "u");

    ros::Subscriber u_sub = nh.subscribe(u_topic, 1, u_callback);

    x_pub = nh.advertise<std_msgs::Float64MultiArray>("x", 1);

    lastTime = ros::Time::now();

    ros::spin();

}
