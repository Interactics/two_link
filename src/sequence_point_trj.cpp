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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_gen");
    ros::NodeHandle n;
    ros::Publisher path_pub = n.advertise<std_msgs::Float64>("/two_link/joint1_position_controller/command", 100);

    ros::Rate loop_rate(100);


    double t_i = 0.0f;
    double t_f = 1.0f;
    double timeStep = 0.01;

    double q_i = 0.0f;
    double q_f = M_PI;

    double dq_i = 0;
    double dq_f = 0;

    double ddq_c = 6 * M_PI; // 선정해야하지만, 임의로 선택.

    double q_1 = 0;
    double q_2 = 2.0 * M_PI;
    double q_3 = M_PI / 3.0;
    double q_4 = M_PI - 0.1;

    double t_1 = 0;
    double t_2 = 2;
    double t_3 = 3;
    double t_4 = 5;

    double dq_1 = 0;
    double dq_2 = M_PI;
    double dq_3 = -M_PI;
    double dq_4 = 0;



    std::vector<double> t  = createDomain(t_1, t_2, timeStep);  // 정의역
    std::vector<double> t2 = createDomain(t_2, t_3, timeStep); // 정의역
    std::vector<double> t3 = createDomain(t_3, t_4, timeStep); // 정의역

    std::vector<double> q  = PointToPoint(t,  q_1, q_2, dq_1, dq_2, t_1, t_2); // 치역
    std::vector<double> q2 = PointToPoint(t2, q_2, q_3, dq_2, dq_3, t_2, t_3); // 치역
    std::vector<double> q3 = PointToPoint(t3, q_3, q_4, dq_3, dq_4, t_3, t_4); // 치역

    q.insert(q.end(), q2.begin(), q2.end());
    q.insert(q.end(), q3.begin(), q3.end());

    t.insert(t.end(), t2.begin(), t2.end());
    t.insert(t.end(), t3.begin(), t3.end());	

    std_msgs::Float64 msg;

    float velocity = 0.0;
    while (ros::ok())
    {
        for (int i = 0; i < t.size() -1 ; i++)
        {
            msg.data = q[i];

            path_pub.publish(msg);
            loop_rate.sleep();
        }
        for (int i = t.size() -1 ; i > 0; i--)
        {
            msg.data = q[i];

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


