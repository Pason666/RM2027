#include "pyro_kin_rudomni.h"
#include <arm_math.h>

namespace pyro
{
    rudomni_kin_t::rudomni_kin_t(float omnid, float rudd, float dty1, float dtx1, float dty2, float dtx2)
     : _omnid(omnid), _rudd(rudd), _dty1(dty1), _dtx1(dtx1), _dty2(dty2), _dtx2(dtx2){}


    float rudomni_kin_t::rad_limit(float rad) const
    {
        // Normalize angle to [-PI, PI]
        while (rad > PI)
            rad -= 2.0f * PI;
        while (rad < -PI)
            rad += 2.0f * PI;
        return rad;
    }

    void rudomni_kin_t::rud_angle_ch(float current_angle, float &target_angle, int &direction) const
    {
        float angle_diff = rad_limit(target_angle - current_angle);
        if(fabsf(angle_diff) > PI / 2)
        {
            target_angle = rad_limit(target_angle + PI);
            direction = -direction;
        }
       
        angle_diff = rad_limit(target_angle - current_angle);
        target_angle = current_angle + angle_diff;
    }

    rudomni_kin_t::rudomni_states_t 
    rudomni_kin_t::solve(float vx, float vy, float wz,
                        const rudomni_states_t &current_states) const
    {
        float mombalance_dvx=0; //力矩平衡(moment_balance)补偿项，单位为m/s

        rudomni_states_t target_states{};

        target_states.modules[0].angle = 0;
        target_states.modules[1].angle = 0;
        target_states.modules[0].speed = (vy)*square2/2-(vx)*square2/2;
        target_states.modules[1].speed = (vy)*square2/2+(vx)*square2/2; //以m/s为单位算出初始值
        target_states.modules[0].direction = -1;
        target_states.modules[1].direction = 1;

        mombalance_dvx =(target_states.modules[0].speed-target_states.modules[1].speed)*(_dty1)*(square2/2)/_dtx2/3.0f ; //计算力矩平衡补偿项，单位为m/s
        mombalance_dvx=0;
        target_states.modules[2].angle = atan2(vy-wz*square2*0.5f, vx+wz*square2*0.5f-mombalance_dvx);
        target_states.modules[3].angle = atan2(vy+wz*square2*0.5f, vx+wz*square2*0.5f-mombalance_dvx);

        target_states.modules[2].speed = sqrtf((vy-wz*square2*0.5f)*(vy-wz*square2*0.5f)+(vx+wz*square2*0.5f-mombalance_dvx)*(vx+wz*square2*0.5f-mombalance_dvx));
        target_states.modules[3].speed = sqrtf((vy+wz*square2*0.5f)*(vy+wz*square2*0.5f)+(vx+wz*square2*0.5f-mombalance_dvx)*(vx+wz*square2*0.5f-mombalance_dvx));

        target_states.modules[2].direction = current_states.modules[2].direction;
        target_states.modules[3].direction = current_states.modules[3].direction;//保持舵轮转向不变，避免切换时的抖动

        rud_angle_ch(current_states.modules[2].angle, target_states.modules[2].angle, target_states.modules[2].direction);
        rud_angle_ch(current_states.modules[3].angle, target_states.modules[3].angle, target_states.modules[3].direction);

        target_states.modules[0].speed = target_states.modules[0].direction*(target_states.modules[0].speed+wz) / _omnid * 20.0f;
        target_states.modules[1].speed = target_states.modules[1].direction*(target_states.modules[1].speed-wz) / _omnid * 20.0f;
        target_states.modules[2].speed = target_states.modules[2].direction*target_states.modules[2].speed / _rudd * 20.0f;
        target_states.modules[3].speed = target_states.modules[3].direction*target_states.modules[3].speed / _rudd * 20.0f; //根据线速度转化为角速度

        if((fabs(vx)<_deadband)&&(fabs(vy)<_deadband)) 
        {
            //target_states.modules[2].angle=-PI/4-0.1f;
            //target_states.modules[3].angle=PI/4+0.1f;
            target_states.modules[2].angle=-0.67f;
            target_states.modules[3].angle=0.67f;
        }

        return target_states;
    }

}