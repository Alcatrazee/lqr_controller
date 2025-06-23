#ifndef __BEZIER_CURVE_GENERATOR_HPP__
#define __BEZIER_CURVE_GENERATOR_HPP__

#include <stdint.h>
#include <vector>


using namespace std;
class bezier_curve_generator{
    public:
        bezier_curve_generator(uint8_t control_point_mode=RATIO,double interpolation_distance = 0.3);
        ~bezier_curve_generator();
        void set_control_point_distance(double distance_to_start,double distance_to_end);
        void set_control_point_ratio(double ratio_to_start,double ratio_to_end);
        void set_control_point_mode(uint8_t control_point_mode);
        void set_interpolation_distance(double distance);
        bool generate_bezier_curve( vector<vector<double>>  control_points,vector<vector<double>> &path);
        vector<vector<double>>  compute_control_point(const vector<double> &start,
            const vector<double> &end,
            uint8_t control_point_mode,bool &dir);
        void bezier_derivative(double t, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, double& dx, double& dy);
        enum ControlPointMode{
            DISTANCE,
            RATIO
        };
    private:
        uint8_t control_point_mode_;
        vector<double> control_point_distance_;
        vector<double> control_point_ratio_;
        double interpolation_distance_;
};

#endif