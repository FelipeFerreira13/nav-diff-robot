#ifndef UTILS
#define UTILS

#include <cmath>
#include <deque>
#include <mutex>

struct Pose2D {
    double x;
    double y;
    double theta; // orientation in radians
};

struct Twist2D {
    double vx;     // velocity in x
    double vy;     // velocity in y
    double omega;  // angular velocity
};

namespace Utils
{
    static void rotate( double & x, double & y, double phi){
        float x_ = (cos(phi) * x) - (sin(phi) * y);
        float y_ = (sin(phi) * x) + (cos(phi) * y);

        x = x_;
        y = y_;
    }

    static double Quotient_Remainder( double x, double y ){
        double Quotient = floor( x / y );
        double Remainder = x - (y * Quotient);

        return Remainder;
    }

    static double close_angle( double ang ){
    
        if      ( ang < -180 ) { ang = ang + 360; }
        else if ( ang >  180 ) { ang = ang - 360; }

        return ang;
    }

    static float straight_ang( float angle ){
        double angle_wall = 0;
        const float split = 30;

        double minDiff = 0;
        for (int i = 0; i < (360.0 / split) + 1; i++){
            double diff = std::abs( close_angle( angle - (i * split)) );
            if (i == 0 || ( diff < minDiff)){
                angle_wall = i * split;
                minDiff = diff;
            }
        }
        return angle_wall;
    }



    class RollingMeanAccumulator {
        public:
            explicit RollingMeanAccumulator(size_t window_size = 10) 
                : window_size_(window_size) {}

            void accumulate(double value) {
                values_.push_back(value);
                if (values_.size() > window_size_) {
                    values_.pop_front();
                }
            }

            double getRollingMean() const {
                if (values_.empty()) return 0.0;
                double sum = 0.0;
                for (double value : values_) sum += value;
                return sum / values_.size();
            }

            void clear() {
                values_.clear();
            }

        private:
            std::deque<double> values_;
            size_t window_size_;
    };


}

#endif // UTILS