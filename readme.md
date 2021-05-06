This project contains source code and a demo of my article <GNSS/INS/ODO/Wheel Angle Integrated Navigation Algorithm for an All-Wheel Steering Robot>. The demo is about experiment 2 in the article. You can set the value of macro definition in "src/lcdefine.h" to get the result of 4 cases.

Mode #1:USE_ODO=0,USE_STEER=0

Mode #2:USE_ODO=0,USE_STEER=1

Mode #3:USE_ODO=0,USE_STEER=2

Mode #4:USE_ODO=1,USE_STEER=2



File description in the "data" folder

input:

1. GNSS_data.txt : RTK result
2. IMU_data.bin : IMU data
3. odometer_speed : odometer speed data
4. wheel_angle.txt : wheel angle data
5. reference_result.nav : reference, the format is (0,sow(s),latitude(deg),longitude(deg),height(m),vel_north(m/s),vel_east(m/s),vel_down(m/s),roll(deg),pitch(deg),yaw(deg))

output:

1. my_result.nav : result of the poject, the format is as same as reference_result.nav
2. system_error_satte : the system error estimation of this project, the format is (sow(s),**pos_err**(m),**vel_err**(m/s),**att_err**(rad),**gyro_bias**(rad/s),**gyro_scale_factor**,**acce_bias**(m/s^2),**acce_scale_factor**,odometer_scale_factor,wheel_angle_err)

