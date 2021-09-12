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
std::vector<double> find_trj_coeff_imposed_vel(double t1, double p1, double v1, double t2, double p2, double v2);

// Trapezodial
double qt(double q_i, double q_f, double ddq_c, double t_f, double t); // Pos
std::vector<double> trapezoidalVelocityProfile(double q_i, double q_f, double ddq_c, double t_f, const std::vector<double> domain);

// Sgn function
double sgn_func(double x);



int main(int argc, char **argv)
{
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
    while (ros::ok())
    {
        for (int t = 0; t < 1000; t++)
        {
            msg.data = q[t];

            path_pub.publish(msg);
            loop_rate.sleep();
        }
        for (int t = 1000; t > 0; t--)
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
                                 const double q_i, const double q_f,
                                 const double dq_i, const double dq_f,
                                 const double t_i, const double t_f)
{

    std::vector<double> range; // 치역
    std::vector<double> coeff;

	coeff = find_trj_coeff_imposed_vel(t_i, q_i, dq_i, t_f, q_f, dq_f);
    
    for (int i = 0; i < domain.size(); ++i)
    { // ++i가 더 빠름
        range.push_back(cubicPolynomial(coeff[3], coeff[2], coeff[1], coeff[0], domain[i]));
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


// c3*t^3 + c2*t^2 + c1*t + c0 = q(t)
// Interpolating polynomials with imposed velocities at path points 
std::vector<double> find_trj_coeff_imposed_vel(double t1, double p1, double v1, double t2, double p2, double v2)
{
	std::vector<double> coeff(4);

	coeff[0] = -(p1 * pow(t2,3) - p2 * pow(t1, 3) - 3.0f * p1 * t1 * pow(t2, 2) + 3.0f * p2 * pow(t1,2) * t2 - t1 * pow(t2,3) * v1 + pow(t1, 3) * t2 * v2 + pow(t1, 2) * pow(t2, 2) * v1 
			-pow(t1, 2) * pow(t2, 2) * v2) / ((t1 - t2) * (pow(t1,2) - 2.0f * t1 * t2 + pow(t2, 2))); 

    coeff[1] = (pow(t1,3) * v2 - pow(t2,3) * v1 - t1 * pow(t2,2) * v1 + 2.0f * pow(t1,2) * t2 * v1 - 2.0f * t1 * pow(t2,2) * v2 + pow(t1, 2) * t2 * v2 - 6.0f * p1 * t1 * t2 + 
		6 * p2 * t1 * t2) / ((t1 - t2) *(pow(t1,2) - 2*t1*t2 + pow(t2,2)));

    coeff[2] = (3.0f*p1*t1 + 3.0f*p1*t2 - 3.0f*p2*t1 - 3.0f*p2*t2 -pow(t1,2)*v1 - 2*pow(t1,2) * v2 + 2*pow(t2,2) * v1 + pow(t2,2) * v2 - t1*t2*v1 + t1*t2*v2)/((t1 - t2)*(pow(t1,2) - 2.0f*t1*t2 + pow(t2,2)));

    coeff[3] = -(2.0f*p1 - 2.0f * p2 - t1*v1 - t1*v2 + t2*v1 + t2*v2)/((t1 - t2)*(pow(t1,2) - 2.0f*t1*t2 + pow(t2,2)));

	return coeff;
}


