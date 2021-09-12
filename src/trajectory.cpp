#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <vector>
#include <cmath>


// Domain Gen
std::vector<double> createDomain(const double start, const double end, const double step);
std::vector<double> PointToPoint(const std::vector<double> domain, const double q_i, const double q_f, const double dq_i, const double dq_f, const double t_i, const double t_f);

// Cubic Poly
double cubicPolynomial(double a3, double a2, double a1, double a0, double x);

// Trapezodial
double qt(double q_i, double q_f, double ddq_c, double t_f, double t); // Pos
std::vector<double> trapezoidalVelocityProfile(double q_i, double q_f, double ddq_c, double t_f, const std::vector<double> domain);
double sgn_func(double x);

int main(int argc, char **argv){
	ros::init(argc, argv, "traj_gen");
    ros::NodeHandle n;
	ros::Publisher path_pub = n.advertise<std_msgs::Float64>("/two_link/joint1_position_controller/command", 100);

    ros::Rate loop_rate(100);

    double t_i = 0.0f;
    double t_f = 10.0f;
    double timeStep = 0.01;

    double q_i = 0.0f;
    double q_f = M_PI;

    double dq_i = 0;
    double dq_f = 0;
    double ddq_c = 6 * M_PI; // 선정해야하지만, 임의로 선택.

    std::vector<double> t = createDomain(t_i, t_f, timeStep);                // 정의역
    // Cubic
	std::vector<double> q = PointToPoint(t, q_i, q_f, dq_i, dq_f, t_i, t_f); // 치역

	// Trapezoidal
    // std::vector<double> q = trapezoidalVelocityProfile(q_i, q_f, ddq_c, t_f, t);

	std_msgs::Float64 msg;

    float velocity = 0.0;
	while(ros::ok())
	{
		for(int t = 0; t < 1000; t++)
		{
			msg.data = q[t];

		    path_pub.publish(msg);
		    loop_rate.sleep();
		}
        for(int t = 1000; t > 0; t--)
		{
			msg.data = q[t];

		    path_pub.publish(msg);
		    loop_rate.sleep();
		}
	}
	return 0;
}

std::vector<double> createDomain(const double start, const double end, const double step)
{
    std::vector<double> domain;
    for (double i = start; i < end; i += step)
    {
        domain.push_back(i);
    }
    return domain;
}

std::vector<double> PointToPoint(const std::vector<double> domain,
                                 const double q_i,  const double q_f,
                                 const double dq_i, const double dq_f,
                                 const double t_i,  const double t_f){

    std::vector<double> range; // 치역

    double a0 = q_i;
    double a1 = dq_i;
    double a2 = 3 / pow(t_f, 2) * (q_f - q_i) - 2 / t_f * dq_i - 1 / t_f * dq_f;
    double a3 = -2 / pow(t_f, 3) * (q_f - q_i) + 1 / pow(t_f, 2) * (dq_f + dq_i);

    for (int i = 0; i < domain.size(); ++i)
    { // ++i가 더 빠름
        range.push_back(cubicPolynomial(a3, a2, a1, a0, domain[i]));
    }

    return range;
}

double cubicPolynomial(
    double a3, double a2, double a1, double a0, double x)
{
    return a3 * pow(x, 3) + a2 * pow(x, 2) + a1 * x + a0;
}


double sgn_func(double x)
{
    if (x > 0)
    {
        return 1;
    }
    else if (x < 0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

std::vector<double> trapezoidalVelocityProfile(
    double q_i, double q_f,
    double ddq_c,
    double t_f, const std::vector<double> domain)
{
    std::vector<double> range; // 치역

    for (int i = 0; i < domain.size(); ++i)
    {
        range.push_back(qt(q_i, q_f, ddq_c, t_f, domain[i]));
    }
    return range;
}

double qt(
    double q_i, double q_f,
    double ddq_c,
    double t_f, double t)
{
    double t_c = t_f / 2.0 - 1.0 / 2.0 * sqrt((pow(t_f, 2) * ddq_c - 4.0 * (q_f - q_i)) / ddq_c);
    double q;
    if (0 <= t and t < t_c)
    {
        q = q_i + 1.0 / 2.0 * ddq_c * pow(t, 2);
    }
    else if (t_c < t and t <= t_f - t_c)
    {
        q = q_i + ddq_c * t_c * (t - t_c / 2.0);
    }
    else if (t_f - t_c < t and t <= t_f)
    {
        q = q_f - 1.0 / 2.0 * ddq_c * pow((t_f - t), 2);
    }
    return q;
}
