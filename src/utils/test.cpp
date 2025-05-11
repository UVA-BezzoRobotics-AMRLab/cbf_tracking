#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <functional>
#include <numeric>

#include <unsupported/Eigen/Splines>

#include <visualization_msgs/MarkerArray.h>

typedef Eigen::Spline<double, 1, 3> Spline1D;
typedef Eigen::SplineFitting<Spline1D> SplineFitting1D;

class MinJerkTrajectory2D
{
public:
    MinJerkTrajectory2D(const std::vector<double> &start, const std::vector<double> &end, double total_time)
        : start_(start), end_(end), total_time_(total_time)
    {
        coefficients_x_ = computeCoefficients({start_[0], start_[1], start_[2]}, {end_[0], end_[1], end_[2]});
        coefficients_y_ = computeCoefficients({start_[3], start_[4], start_[5]}, {end_[3], end_[4], end_[5]});
    }

    std::vector<double> evalTrajectory(double t) const
    {
        double x = 0.0, y = 0.0;
        for (int p = 0; p < coefficients_x_.size(); ++p)
        {
            x += coefficients_x_[p] * std::pow(t, p);
            y += coefficients_y_[p] * std::pow(t, p);
        }
        return {x, y};
    }

    static double computeArclength(const MinJerkTrajectory2D &segment, double t0, double tf)
    {
        const auto &coeff_x = segment.coefficients_x_;
        const auto &coeff_y = segment.coefficients_y_;
        std::vector<double> d_coeff_x = {coeff_x[1], 2 * coeff_x[2], 3 * coeff_x[3], 4 * coeff_x[4], 5 * coeff_x[5]};
        std::vector<double> d_coeff_y = {coeff_y[1], 2 * coeff_y[2], 3 * coeff_y[3], 4 * coeff_y[4], 5 * coeff_y[5]};

        double s = 0.0;
        double dt = 0.001;
        for (double t = t0; t <= tf; t += dt)
        {
            double dx = 0.0, dy = 0.0;
            double mult = 1.0;
            for (int p = 0; p < d_coeff_x.size(); ++p)
            {
                dx += d_coeff_x[p] * mult;
                dy += d_coeff_y[p] * mult;

                mult *= t;
            }
            s += std::sqrt(dx * dx + dy * dy) * dt;
        }
        return s;
    }

private:
    std::vector<double> computeCoefficients(const std::vector<double> &start_params, const std::vector<double> &end_params) const
    {
        double T = total_time_;
        Eigen::MatrixXd M(6, 6);
        M << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 2, 0, 0, 0,
            1, T, T * T, T * T * T, T * T * T * T, T * T * T * T * T,
            0, 1, 2 * T, 3 * T * T, 4 * T * T * T, 5 * T * T * T * T,
            0, 0, 2, 6 * T, 12 * T * T, 20 * T * T * T;

        Eigen::VectorXd b(6);
        for (int i = 0; i < 3; ++i)
        {
            b[i] = start_params[i];
            b[i + 3] = end_params[i];
        }

        Eigen::VectorXd coeffs = M.fullPivLu().solve(b);
        return std::vector<double>(coeffs.data(), coeffs.data() + coeffs.size());
    }

    std::vector<double> start_, end_;
    double total_time_;
    std::vector<double> coefficients_x_, coefficients_y_;
};

double binarySearch(const MinJerkTrajectory2D &segment, double dl, double start, double end, double tolerance)
{
    double t_left = start;
    double t_right = end;

    double prev_s = 0;
    double s = -1000;

    while (fabs(prev_s - s) > tolerance)
    {
        prev_s = s;

        double t_mid = (t_left + t_right) / 2;
        s = MinJerkTrajectory2D::computeArclength(segment, start, t_mid);

        if (s < dl)
            t_left = t_mid;
        else
            t_right = t_mid;
    }

    return (t_left + t_right) / 2;
}

void publishTrajectory(const std::vector<Eigen::Vector2d> &points, ros::Publisher &publisher, const std::string &ns, const std::string &frame_id, double r, double g, double b)
{
    visualization_msgs::MarkerArray marker_array;
    for (size_t i = 0; i < points.size(); ++i)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = points[i].x();
        marker.pose.position.y = points[i].y();
        marker.pose.position.z = 0.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.a = 1.0;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker_array.markers.push_back(marker);
    }
    publisher.publish(marker_array);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;

    ros::Time::init();

    ros::Publisher time_pub, arc_length_pub;
    time_pub = nh.advertise<visualization_msgs::MarkerArray>("time_parameterized_path", 1);
    arc_length_pub = nh.advertise<visualization_msgs::MarkerArray>("arc_length_parameterized_path", 1);

    std::vector<double> start_params = {10, 0, 0, 10, 0, 0};
    std::vector<double> mid_params = {5, -3, 0, 5, 0, 0};
    std::vector<double> end_params = {0, 0, 0, 0, 0, 0};

    MinJerkTrajectory2D segment1(start_params, mid_params, 2.5);
    MinJerkTrajectory2D segment2(mid_params, end_params, 2.5);

    std::vector<MinJerkTrajectory2D> segments = {segment1, segment2};

    ros::Time start = ros::Time::now();
    double cumsum[segments.size()];
    cumsum[0] = MinJerkTrajectory2D::computeArclength(segments[0], 0, 2.5);

    for (int i = 1; i < segments.size(); ++i)
    {
        cumsum[i] = cumsum[i - 1] + MinJerkTrajectory2D::computeArclength(segments[i], 0, 2.5);
    }

    double total_length = cumsum[segments.size() - 1];

    double M = 20;
    double dl = total_length / M;

    Eigen::RowVectorXd ss, xs, ys;
    ss.resize(M + 1);
    xs.resize(M + 1);
    ys.resize(M + 1);

    for (int i = 0; i <= M; ++i)
    {
        double l = i * dl;

        int min_idx = -1;
        for (int j = 0; j < segments.size(); ++j)
        {
            if (cumsum[j] - l >= 0)
            {
                min_idx = j;
                break;
            }
        }

        double l_before = 0;
        if (min_idx > 0)
        {
            l_before = cumsum[min_idx - 1];
        }

        double ti = binarySearch(segments[min_idx], l - l_before, 0, 2.5, 1e-3);

        std::vector<double> coord = segments[min_idx].evalTrajectory(ti);

        ss(i) = l;
        xs(i) = coord[0];
        ys(i) = coord[1];

        std::cout << coord[0] << ", " << coord[1] << std::endl;
    }

    // reparameterize
    const auto fitX = SplineFitting1D::Interpolate(xs, 3, ss);
    Spline1D splineX(fitX);

    const auto fitY = SplineFitting1D::Interpolate(ys, 3, ss);
    Spline1D splineY(fitY);

    std::vector<Eigen::Vector2d> arclength;
    for (int i = 0; i < 500; ++i)
    {
        double curr_s = i * total_length / 499;

        double x = splineX(curr_s).coeff(0);
        double y = splineY(curr_s).coeff(0);

        double vX = splineX.derivatives(curr_s, 1).coeff(1);
        double vY = splineY.derivatives(curr_s, 1).coeff(1);

        arclength.push_back(Eigen::Vector2d(x, y));
    }

    std::vector<Eigen::Vector2d> timecurve;
    for (int i = 0; i < 500; ++i)
    {
        double t = i * 5.0 / 499;

        std::vector<double> coords;
        if (t < 2.5)
            coords = segments[0].evalTrajectory(t);
        else
            coords = segments[1].evalTrajectory(t);

        double x = coords[0];
        double y = coords[1];

        timecurve.push_back(Eigen::Vector2d(x, y));
    }

    ros::Time end = ros::Time::now();

    std::cout << "time: " << (end - start).toSec() << std::endl;

    ros::Rate rate(10);
    while (ros::ok())
    {
        publishTrajectory(arclength, arc_length_pub, "arc_length_path", "odom", 0.0, 1.0, 0.0); // Green for arc-length parameterized path
        publishTrajectory(timecurve, time_pub, "time_pub", "odom", 1.0, 0.0, 0.0); // Green for arc-length parameterized path

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
