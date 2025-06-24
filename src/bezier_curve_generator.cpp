#include <lqr_controller/bezier_curve_generator.hpp>
#include <cmath>
#include <iostream>

bezier_curve_generator::bezier_curve_generator(uint8_t control_point_mode,
                                                double interpolation_distance
                                                ) : control_point_mode_(control_point_mode),
                                                interpolation_distance_(interpolation_distance)
{
    std::cout << "bezier_curve_generator initializing" << std::endl;
    control_point_distance_.push_back(0);
    control_point_distance_.push_back(0);
    control_point_ratio_.push_back(0);
    control_point_ratio_.push_back(0);
    set_control_point_distance(0.5,0.5);
    set_control_point_ratio(0.5,0.5);
    std::cout << "bezier_curve_generator initialized" << std::endl;
}

bezier_curve_generator::~bezier_curve_generator()
{
    std::cout << "bezier_curve_generator destroyed" << std::endl;
}

void bezier_curve_generator::set_control_point_distance(double distance_to_start,double distance_to_end){
    if(distance_to_start < 0){
        std::cout << "distance_to_start must be greater than 0" << std::endl;
        return;
    }
    if(distance_to_end < 0){
        std::cout << "distance_to_end must be greater than 0" << std::endl;
        return;
    }
    control_point_distance_[0] = distance_to_start;
    control_point_distance_[1] = distance_to_end;
}

void bezier_curve_generator::set_control_point_ratio(double ratio_to_start,double ratio_to_end){
    if(ratio_to_start < 0 || ratio_to_start > 1){
        std::cout << "ratio_to_start must be between 0 and 1" << std::endl;
        return;
    }
    if(ratio_to_end < 0 || ratio_to_end > 1){
        std::cout << "ratio_to_end must be between 0 and 1" << std::endl;
        return;
    }
    control_point_ratio_[0] = ratio_to_start;
    control_point_ratio_[1] = ratio_to_end;
}

void bezier_curve_generator::set_interpolation_distance(double distance){
    if(distance < 0){
        std::cout << "distance must be greater than 0" << std::endl;
        return;
    }
    interpolation_distance_ = distance;
}

vector<vector<double>>  bezier_curve_generator::compute_control_point(const vector<double> &start,
    const vector<double> &end,uint8_t control_point_mode,bool &dir){
    vector<vector<double>>  control_points;

    bool forward = true;
    double yaw_start_to_goal = atan2(end[1]-start[1],end[0]-start[0]);
    double yaw_diff = std::remainder(yaw_start_to_goal-start[2], 2.0 * M_PI);
    if(abs(yaw_diff) > (M_PI/2))
    {
        forward = false;
    }
    dir = forward;
    cout << "yaw diff" << yaw_diff << endl;
    // step 1. compute distance
    double distance = std::hypot(start[0]-end[0],start[1]-end[1]);
    // step 2. compute control point
    double control_point_start[2] = {0,0};
    double control_point_end[2] = {0,0};
    switch(control_point_mode){
        case DISTANCE:{
            if(forward == true){
                control_point_start[0] = start[0] + control_point_distance_[0]*cos(start[2]);
                control_point_start[1] = start[1] + control_point_distance_[0]*sin(start[2]);
                control_point_end[0] = end[0] - control_point_distance_[1]*cos(end[2]);
                control_point_end[1] = end[1] - control_point_distance_[1]*sin(end[2]);
            }else{
                control_point_start[0] = start[0] - control_point_distance_[0]*cos(start[2]);
                control_point_start[1] = start[1] - control_point_distance_[0]*sin(start[2]);
                control_point_end[0] = end[0] + control_point_distance_[1]*cos(end[2]);
                control_point_end[1] = end[1] + control_point_distance_[1]*sin(end[2]);
            }
            break;
        }
        case RATIO:{
            if(forward == true){
                control_point_start[0] = start[0] + control_point_ratio_[0]*distance*cos(start[2]);
                control_point_start[1] = start[1] + control_point_ratio_[0]*distance*sin(start[2]);
                control_point_end[0] = end[0] - control_point_ratio_[1]*distance*cos(end[2]);
                control_point_end[1] = end[1] - control_point_ratio_[1]*distance*sin(end[2]);
            }else{
                control_point_start[0] = start[0] - control_point_ratio_[0]*distance*cos(start[2]);
                control_point_start[1] = start[1] - control_point_ratio_[0]*distance*sin(start[2]);
                control_point_end[0] = end[0] + control_point_ratio_[1]*distance*cos(end[2]);
                control_point_end[1] = end[1] + control_point_ratio_[1]*distance*sin(end[2]);
            }
            
            break;
        }
    }
    control_points.push_back(vector<double>({start[0],start[1]}));
    control_points.push_back(vector<double>({control_point_start[0],control_point_start[1]}));
    control_points.push_back(vector<double>({control_point_end[0],control_point_end[1]}));
    control_points.push_back(vector<double>({end[0],end[1]}));

    return control_points;
}

bool bezier_curve_generator::generate_bezier_curve( vector<vector<double>>  control_points,
                                                    vector<vector<double>> &path){
                                                    
    double t = 0;
    double dt = 0.01;
    while(t<=1){
        double x = pow((1-t),3)*control_points[0][0] + 3*pow((1-t),2)*t*control_points[1][0] + 3*(1-t)*pow(t,2)*control_points[2][0] + pow(t,3)*control_points[3][0];
        double y = pow((1-t),3)*control_points[0][1] + 3*pow((1-t),2)*t*control_points[1][1] + 3*(1-t)*pow(t,2)*control_points[2][1] + pow(t,3)*control_points[3][1];
        vector<double> point;
        point.push_back(x);
        point.push_back(y);
        path.push_back(point);
        // cout << "x: " << x << " y: " << y << endl;

        double dx, dy;
        bezier_derivative(t, control_points[0][0],control_points[0][1], control_points[1][0], control_points[1][1], 
                        control_points[2][0],
                        control_points[2][1], 
                        control_points[3][0], control_points[3][1]
                         ,dx, dy);
        double speed = std::sqrt(dx * dx + dy * dy);
        dt = interpolation_distance_ / speed; // 根据速度调整步长

        t += dt;
    }
    // step 4. compute curvature
    for(size_t i=0;i<path.size()-1;i++){
        double dx = path[i+1][0] - path[i][0];
        double dy = path[i+1][1] - path[i][1];
        double d = sqrt(dx*dx + dy*dy);
        double kappa = (dx*dy)/(d*d*d);
        path[i].push_back(kappa);
    }
    path.push_back(vector<double>({control_points[3][0],control_points[3][1]}));
    // step 5. return path
    return true;
}

void bezier_curve_generator::bezier_derivative(double t, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, double& dx, double& dy) {
    double u = 1 - t;
    dx = -3*u*u*x0 + 3*(1 - 4*t + 3*t*t)*x1 + 3*(2*t - 3*t*t)*x2 + 3*t*t*x3;
    dy = -3*u*u*y0 + 3*(1 - 4*t + 3*t*t)*y1 + 3*(2*t - 3*t*t)*y2 + 3*t*t*y3;
}