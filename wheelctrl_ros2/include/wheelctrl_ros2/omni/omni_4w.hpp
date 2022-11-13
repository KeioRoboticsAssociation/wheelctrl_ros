#ifndef _OMNI_4W_
#define _OMNI_4W_

#include "wheel.hpp"

class MeasureOmni4W : public Measuring {
    public:
     MeasureOmni4W(const W_PARAM &_w_param);
     void cal_disp(const float encoder[], size_t mysize, ODOM &dist_r);
     void cal_disp(const float encoder[], size_t mysize, const float &imu,
                   ODOM &dist_r);

    private:
     W_PARAM w_param;
};

class MoveOmni4W : public Moving {
    public:
     MoveOmni4W(const W_PARAM &_w_param);
     void cal_cmd(const ODOM &cmd_r, float wheel_cmd[], size_t mysize);

    private:
     W_PARAM w_param;
};

#endif