#ifndef __PYRO_KIN_RUDOMNI__
#define __PYRO_KIN_RUDOMNI__

#define square2 1.41421356237f
namespace pyro
{
/**这是半舵半全向的运动学结算，当前以全向轮方向为正方向（即为y轴正方向），同时以右上角的轮子为1号轮子，逆时针依次
  * 编号为2，3，4号。此运动学的坐标系中，舵轮的原点角度为零度（此时舵轮与x轴平行）以逆时针为正方向角度（-pi~pi）
*/
    class rudomni_kin_t
    {
        public:
            rudomni_kin_t(float omnid, float rudd, float dty1, float dtx1, float dty2, float dtx2);//dty1为全向轮到y轴的距离，dty2为舵轮到y轴的距离，这两个值用以平衡力矩计算
            struct rudomni_state_t
            {
                float speed; // Linear speed (m/s)
                float angle; // Steering angle (rad, -PI to +PI)
                int direction; // Wheel direction
            };

            struct rudomni_states_t
            {
                rudomni_state_t modules[4]; // Array storing state for all 4 modules
            };
            [[nodiscard]] rudomni_states_t solve(float vx, float vy, float wz,
                                        const rudomni_states_t &current_states) const;
        private:
            float _omnid;
            float _rudd;;
            float _dty1;
            float _dtx1;
            float _dty2;
            float _dtx2;

            const float _deadband = 1e-3f; 
            
            void rud_angle_ch(float current_angle, float &target_angle, int &direction) const; //舵轮角度调整函数，输入输出均为弧度，输出的角度范围为[-pi/2, pi/2]，同时根据输入的角度调整轮子转向方向
            float rad_limit(float rad) const;
    };
}

#endif