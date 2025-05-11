#include <cmath>
#include <iostream>

#include "filter/filter_nlopt.h"

CBFFilterNLOPT::CBFFilterNLOPT()
{
    dt = .1;
    max_linacc = 4.0;
    max_angvel = 2.0;
    alpha = .1;
    colinear = .1;
    padding = .1;
}

CBFFilterNLOPT::~CBFFilterNLOPT()
{
}

void CBFFilterNLOPT::set_dist_map(const std::shared_ptr<distmap::DistanceMap> &dist_map)
{
    dist_grid_ptr = dist_map;
    std::cout << "[Filter] Distance map set" << std::endl;
}

#ifdef FOUND_PYBIND11
bool CBFFilterNLOPT::set_dist_map(pybind11::dict grid_dict)
{
    nav_msgs::OccupancyGrid grid;

    // Deserialize the header
    pybind11::dict header_dict = grid_dict["header"].cast<pybind11::dict>();
    pybind11::dict stamp_dict = header_dict["stamp"].cast<pybind11::dict>();

    grid.header.seq = header_dict["seq"].cast<uint32_t>();
    grid.header.stamp.sec = stamp_dict["secs"].cast<int32_t>();
    grid.header.stamp.nsec = stamp_dict["nsecs"].cast<int32_t>();
    grid.header.frame_id = header_dict["frame_id"].cast<std::string>();

    // Deserialize the info
    pybind11::dict info_dict = grid_dict["info"].cast<pybind11::dict>();

    grid.info.resolution = info_dict["resolution"].cast<float>();
    grid.info.width = info_dict["width"].cast<uint32_t>();
    grid.info.height = info_dict["height"].cast<uint32_t>();

    pybind11::dict origin_dict = info_dict["origin"].cast<pybind11::dict>();
    pybind11::dict position_dict = origin_dict["position"].cast<pybind11::dict>();

    grid.info.origin.position.x = position_dict["x"].cast<double>();
    grid.info.origin.position.y = position_dict["y"].cast<double>();
    grid.info.origin.position.z = position_dict["z"].cast<double>();

    pybind11::dict orientation_dict = origin_dict["orientation"].cast<pybind11::dict>();

    grid.info.origin.orientation.x = orientation_dict["x"].cast<double>();
    grid.info.origin.orientation.y = orientation_dict["y"].cast<double>();
    grid.info.origin.orientation.z = orientation_dict["z"].cast<double>();
    grid.info.origin.orientation.w = orientation_dict["w"].cast<double>();

    // Deserialize the data
    grid.data = grid_dict["data"].cast<std::vector<int8_t>>();

    boost::shared_ptr<distmap::DistanceMapConverterBase> dist_map_conv;
    dist_map_conv = distmap::make_distance_mapper("distmap/DistanceMapDeadReck");
    if (!dist_map_conv->process(boost::make_shared<const nav_msgs::OccupancyGrid>(grid)))
    {
        std::cout << "Distance map failed to be set from occupancy grid" << std::endl;
        return false;
    }

    dist_grid_ptr = dist_map_conv->getDistanceFieldObstacle();

    std::cout << "[Filter] Distance map set (pybind)" << std::endl;

    return true;
}
#endif

void CBFFilterNLOPT::load_params(const std::map<std::string, double> &params)
{
    dt = params.at("DT");
    max_linacc = params.find("MAX_LINACC") != params.end() ? params.at("MAX_LINACC") : max_linacc;
    max_angvel = params.find("ANGVEL") != params.end() ? params.at("ANGVEL") : max_angvel;
    alpha = params.find("CBF_ALPHA") != params.end() ? params.at("CBF_ALPHA") : alpha;
    colinear = params.find("CBF_COLINEAR") != params.end() ? params.at("CBF_COLINEAR") : colinear;
    padding = params.find("CBF_PADDING") != params.end() ? params.at("CBF_PADDING") : padding;

    std::cout << "[Filter] Parameters loaded" << std::endl;
    std::cout << "dt: " << dt << std::endl;
    std::cout << "max_linacc: " << max_linacc << std::endl;
    std::cout << "max_angvel: " << max_angvel << std::endl;
    std::cout << "alpha: " << alpha << std::endl;
    std::cout << "colinear: " << colinear << std::endl;
    std::cout << "padding: " << padding << std::endl;
}

double CBFFilterNLOPT::eval_objective(const std::vector<double> &x, void *data)
{
    CBFFilterNLOPT *obj = (CBFFilterNLOPT *)data;
    return (obj->desired_w - x[0]) * (obj->desired_w - x[0]) + (obj->desired_a - x[1]) * (obj->desired_a - x[1]);
}

double CBFFilterNLOPT::eval_constraint(const std::vector<double> &desired_input, void *data)
{
    CBFFilterNLOPT *obj = (CBFFilterNLOPT *)data;
    std::shared_ptr<distmap::DistanceMap> dist_grid_ptr = obj->dist_grid_ptr;

    double x = obj->state[0];
    double y = obj->state[1];
    double theta = obj->state[2];
    double v = obj->state[3];

    double eps = 1e-6;

    double desired_a = obj->desired_a;
    double desired_w = obj->desired_w;

    double alpha = obj->alpha;
    double d2 = obj->colinear;

    double xdot = v * cos(theta);
    double ydot = v * sin(theta);

    double dist = dist_grid_ptr->atPositionSafe(x, y, true) + eps;
    double D = dist - obj->padding - eps;

    distmap::DistanceMap::Gradient grad = dist_grid_ptr->gradientAtPosition(x, y, true);
    double dx = grad.dx;
    double dy = grad.dy;

    double grad_norm = sqrt(dx * dx + dy * dy);
    dx *= (-dist / grad_norm);
    dy *= (-dist / grad_norm);

    double p0 = (dx * cos(theta) + dy * sin(theta)) / dist;
    double pp = (-dy * cos(theta) + dx * sin(theta)) / dist;
    double P = p0 + v * d2;

    double Lfh1 = (exp(-P) * xdot / dist) * (-dx + D * (cos(theta) - dx * p0 / dist));
    double Lfh2 = (exp(-P) * ydot / dist) * (-dy + D * (sin(theta) - dy * p0 / dist));
    double Lfh = Lfh1 + Lfh2;

    double Lgh1 = -D * d2 * exp(-P);
    double Lgh2 = D * pp * exp(-P);

    double h = D * exp(-P);

    return -Lfh - Lgh1 * desired_input[1] - Lgh2 * desired_input[0] - alpha * h;
}

void CBFFilterNLOPT::finite_difference(const std::vector<double> &x,
                                       std::vector<double> &grad,
                                       void *data,
                                       double (*func)(const std::vector<double> &x, void *data),
                                       double h)
{
    // copy the input vector
    std::vector<double> x_copy = x;
    for (size_t i = 0; i < x.size(); i++)
    {
        double x1 = x_copy[i];
        x_copy[i] = x1 + h;
        double f1 = func(x_copy, data);
        x_copy[i] = x1 - h;
        double f2 = func(x_copy, data);
        grad[i] = (f1 - f2) / (2 * h);
        x_copy[i] = x1;
    }
}

double CBFFilterNLOPT::objective(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    CBFFilterNLOPT *obj = (CBFFilterNLOPT *)data;
    if (!grad.empty())
    {
        obj->finite_difference(x, grad, data, obj->eval_objective);
    }

    return obj->eval_objective(x, data);
}

double CBFFilterNLOPT::constraint(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    CBFFilterNLOPT *obj = (CBFFilterNLOPT *)data;
    if (!grad.empty())
    {
        obj->finite_difference(x, grad, data, obj->eval_constraint);
    }

    return obj->eval_constraint(x, data);
}

void CBFFilterNLOPT::multi_constraint(unsigned m, double *result, unsigned n, const double *x, double *grad, void *f_data)
{
    CBFFilterNLOPT *obj = (CBFFilterNLOPT *)f_data;

    if (grad)
    {
        std::vector<double> grad_vec(n);
        // copy x to a vector
        std::vector<double> x_vec(x, x + n);
        obj->finite_difference(x_vec, grad_vec, f_data, obj->eval_constraint);
        for (size_t i = 0; i < n; i++)
        {
            grad[i] = grad_vec[i];
        }
    }

    result[0] = obj->eval_constraint(std::vector<double>(x, x + n), f_data);
}

std::vector<double> CBFFilterNLOPT::Solve(const Eigen::Ref<Eigen::VectorXd> &state, const Eigen::Ref<Eigen::VectorXd> &desired_input)
{
    if (dist_grid_ptr.get() == nullptr)
    {
        std::cerr << "Distance map not set" << std::endl;
        return {desired_input[0], desired_input[1]};
    }

    size_t n_vars = 2;
    size_t n_constraints = 1;

    double x = state[0];
    double y = state[1];
    double theta = state[2];
    double v = state[3];

    // if distance is large enough, no use of CBF
    double d = dist_grid_ptr->atPositionSafe(x, y, true);
    if (d > 100)
        return {desired_input[0], desired_input[1]};

    this->state = state;

    desired_w = desired_input[0];
    desired_a = desired_input[1];

    nlopt::opt opt(nlopt::algorithm::LD_SLSQP, n_vars);

    std::vector<double> vars_lower_bound(n_vars);
    std::vector<double> vars_upper_bound(n_vars);

    // angular velocity bounds
    vars_lower_bound[0] = -max_angvel;
    vars_upper_bound[0] = max_angvel;

    // linear acceleration bounds
    // vars_lower_bound[1] = -3.6;
    // robot cannot go backwards, set lower_bound based on current velocity
    // vars_lower_bound[1] = std::max(-3.6, -v*dt);
    vars_lower_bound[1] = -max_linacc;
    vars_upper_bound[1] = max_linacc;

    // adjust desired input slightly if it is on the boundary
    desired_w = std::min(vars_upper_bound[0] - 1e-3, std::max(vars_lower_bound[0] + 1e-3, desired_input[0]));
    desired_a = std::min(vars_upper_bound[1] - 1e-3, std::max(vars_lower_bound[1] + 1e-3, desired_input[1]));

    opt.set_lower_bounds(vars_lower_bound);
    opt.set_upper_bounds(vars_upper_bound);

    opt.set_min_objective(objective, this);

    opt.add_inequality_constraint(constraint, this, 1e-8);
    // opt.add_inequality_mconstraint(multi_constraint, this, {1e-8});

    opt.set_xtol_rel(1e-6);
    opt.set_maxtime(.1);

    std::vector<double> x0(n_vars);
    x0[0] = desired_w;
    x0[1] = desired_a;

    double minf;

    try
    {
        nlopt::result result = opt.optimize(x0, minf);
        std::cout << "found minimum at f(" << x0[0] << "," << x0[1] << ") = " << minf << std::endl;

        double hval = eval_constraint(x0, this);
        std::cout << "constraint value is " << hval << std::endl;

        if (std::isnan(hval))
        {
            double dist = dist_grid_ptr->atPositionSafe(x, y, true);
            distmap::DistanceMap::Gradient grad = dist_grid_ptr->gradientAtPosition(x, y, true);
            
            std::cerr << "dist is " << dist << std::endl;
            std::cerr << "grad is " << grad.dx << "," << grad.dy << std::endl;
        }

    }
    catch (std::exception &e)
    {
        std::cerr << "nlopt failed: " << e.what() << std::endl;
        return {desired_input[0], desired_input[1]};
    }

    std::cout << "[Filter] Desired input: " << desired_input[1] << "," << desired_input[0] << std::endl;
    return x0;
}
