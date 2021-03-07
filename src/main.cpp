#include <iostream>
#include <fstream>
#include "vel_meas.h"

using namespace std;

int main() {
    Vector3d gyroai,acceai,gyroai_1,gyroai_2,acceai_1,acceai_2;
    double tk=0;
    Vector3d euler;
    Result result={0};
    Lcdata lcdata;
    InsData insdata;
    LcConfig config;
    ImuRaw imu={0};

    ifstream ifimu,ifgnss,ifrefer;
    ifimu.open("../data/IMU_data.bin",ifstream::binary);
    ifgnss.open("../data/GNSS_data.txt");
    ifrefer.open("../data/reference_result.nav");
    ofstream offile;
    offile.open("../data/my_result.nav");
    ofstream oferror;
    oferror.open("../data/system_error_state.txt");

    config.ConfigInit(ifrefer);

#if USE_ODO
    ifstream ifodo;
    ifodo.open("../data/odometer_speed.txt");
    while(insdata.odot<config.starttime){
        ifodo>>insdata.odot>>insdata.odovl>>insdata.odovr;
    }
    ifodo>>insdata.odot>>insdata.odovl>>insdata.odovr;
#endif
#if USE_STEER
    ifstream ifsteer;
    ifsteer.open("../data/wheel_angle.txt");
    while(insdata.steert<config.starttime){
        ifsteer>>insdata.steert>>insdata.steerl>>insdata.steerr;
    }
    ifsteer>>insdata.steert>>insdata.steerl>>insdata.steerr;
#endif
    while (lcdata.t<config.starttime) {
        ifgnss>>lcdata.t>>lcdata.Zk(0)>>lcdata.Zk(1)>>lcdata.Zk(2)>>lcdata.Rk(0)>>lcdata.Rk(1)>>lcdata.Rk(2);
    }
    ifgnss>>lcdata.t>>lcdata.Zk(0)>>lcdata.Zk(1)>>lcdata.Zk(2)>>lcdata.Rk(0)>>lcdata.Rk(1)>>lcdata.Rk(2);
    while (tk<config.starttime){
        ifimu.read((char*)&imu,sizeof(ImuRaw));
        tk=imu.sow;
        gyroai<<imu.dtheta[0],imu.dtheta[1],imu.dtheta[2];
        acceai<<imu.dvel[0],imu.dvel[1],imu.dvel[2];
    }
    Lcinit(lcdata,config);
    insdata_init(insdata,gyroai,acceai,tk,config);

    while (!ifimu.eof()&&!ifgnss.eof()&&(tk<config.endtime)){
        ifimu.read((char*)&imu,sizeof(ImuRaw));
        tk=imu.sow;
        gyroai<<imu.dtheta[0],imu.dtheta[1],imu.dtheta[2];
        acceai<<imu.dvel[0],imu.dvel[1],imu.dvel[2];

        gyroai(0)=(gyroai(0)-insdata.gyrobias(0)*(tk-insdata.tk_now))/(1+insdata.gyroscale(0));
        gyroai(1)=(gyroai(1)-insdata.gyrobias(1)*(tk-insdata.tk_now))/(1+insdata.gyroscale(1));
        gyroai(2)=(gyroai(2)-insdata.gyrobias(2)*(tk-insdata.tk_now))/(1+insdata.gyroscale(2));
        acceai(0)=(acceai(0)-insdata.accebias(0)*(tk-insdata.tk_now))/(1+insdata.accescale(0));
        acceai(1)=(acceai(1)-insdata.accebias(1)*(tk-insdata.tk_now))/(1+insdata.accescale(1));
        acceai(2)=(acceai(2)-insdata.accebias(2)*(tk-insdata.tk_now))/(1+insdata.accescale(2));

        if(tk>(lcdata.t+0.0005)){
#if USE_ODO
            if(tk>insdata.odot){
                if(!ifodo.eof())
                    ifodo>>insdata.odot>>insdata.odovl>>insdata.odovr;
                else
                    break;
            }
#endif
#if USE_STEER
            if(tk>insdata.steert){
                if(!ifsteer.eof())
                    ifsteer>>insdata.steert>>insdata.steerl>>insdata.steerr;
                else
                    break;
            }
#endif
            gyroai_1=gyroai*(lcdata.t-insdata.tk_now)/(tk-insdata.tk_now);
            gyroai_2=gyroai*(tk-lcdata.t)/(tk-insdata.tk_now);
            acceai_1=acceai*(lcdata.t-insdata.tk_now)/(tk-insdata.tk_now);
            acceai_2=acceai*(tk-lcdata.t)/(tk-insdata.tk_now);
            loosecouple(insdata,lcdata,config,gyroai_1,acceai_1,euler,lcdata.t,3,result,offile,oferror);
            loosecouple(insdata,lcdata,config,gyroai_2,acceai_2,euler,tk,0,result,offile,oferror);

            if(!ifgnss.eof()){
                ifgnss>>lcdata.t>>lcdata.Zk(0)>>lcdata.Zk(1)>>lcdata.Zk(2)>>lcdata.Rk(0)>>lcdata.Rk(1)>>lcdata.Rk(2);
#if USE_OUTAGE
                if(lcdata.t>config.outagetime(0)) {
                    while((lcdata.t - config.outagetime(0)) < config.outagetime(1)) {
                        ifgnss >> lcdata.t >> lcdata.Zk(0) >> lcdata.Zk(1) >> lcdata.Zk(2) >> lcdata.Rk(0)
                               >> lcdata.Rk(1) >> lcdata.Rk(2);
                    }
                    config.outagetime(0) += config.outagetime(2);
                }
#endif
            }
            else{
                break;
            }
        }
        else if(tk>(lcdata.t-0.0005)){
            tk=lcdata.t;
#if USE_ODO
            if(tk>insdata.odot){
                if(!ifodo.eof())
                    ifodo>>insdata.odot>>insdata.odovl>>insdata.odovr;
                else
                    break;
            }
#endif
#if USE_STEER
            if(tk>insdata.steert){
                if(!ifsteer.eof())
                    ifsteer>>insdata.steert>>insdata.steerl>>insdata.steerr;
                else
                    break;
            }
#endif
            loosecouple(insdata,lcdata,config,gyroai,acceai,euler,tk,3,result,offile,oferror);

            if(!ifgnss.eof()) {
                ifgnss >> lcdata.t >> lcdata.Zk(0) >> lcdata.Zk(1) >> lcdata.Zk(2) >> lcdata.Rk(0) >> lcdata.Rk(1)
                       >> lcdata.Rk(2);
#if USE_OUTAGE
                if(lcdata.t>config.outagetime(0)) {
                    while((lcdata.t - config.outagetime(0)) < config.outagetime(1)) {
                        ifgnss >> lcdata.t >> lcdata.Zk(0) >> lcdata.Zk(1) >> lcdata.Zk(2) >> lcdata.Rk(0)
                               >> lcdata.Rk(1) >> lcdata.Rk(2);
                    }
                    config.outagetime(0) += config.outagetime(2);
                }
#endif
            }
            else{
                break;
            }
        }
        else{
#if USE_ODO
            if(tk<insdata.odot)
#elif USE_STEER
            if(tk<insdata.steert)
#endif
                loosecouple(insdata,lcdata,config,gyroai,acceai,euler,tk,USE_NHC&&!USE_STEER,result,offile,oferror);
#if USE_ODO
            if(tk>insdata.odot){
                gyroai_1=gyroai*(insdata.odot-insdata.tk_now)/(tk-insdata.tk_now);
                gyroai_2=gyroai*(tk-insdata.odot)/(tk-insdata.tk_now);
                acceai_1=acceai*(insdata.odot-insdata.tk_now)/(tk-insdata.tk_now);
                acceai_2=acceai*(tk-insdata.odot)/(tk-insdata.tk_now);
                loosecouple(insdata,lcdata,config,gyroai_1,acceai_1,euler,insdata.odot,2,result,offile,oferror);
                loosecouple(insdata,lcdata,config,gyroai_2,acceai_2,euler,tk,0,result,offile,oferror);

                if(!ifodo.eof()){
                    ifodo>>insdata.odot>>insdata.odovl>>insdata.odovr;
#if USE_STEER
                    ifsteer>>insdata.steert>>insdata.steerl>>insdata.steerr;
#endif
                }
                else{
                    break;
                }
            }
#elif USE_STEER
            if(tk>insdata.steert){
                gyroai_1=gyroai*(insdata.steert-insdata.tk_now)/(tk-insdata.tk_now);
                gyroai_2=gyroai*(tk-insdata.steert)/(tk-insdata.tk_now);
                acceai_1=acceai*(insdata.steert-insdata.tk_now)/(tk-insdata.tk_now);
                acceai_2=acceai*(tk-insdata.steert)/(tk-insdata.tk_now);
                loosecouple(insdata,lcdata,config,gyroai_1,acceai_1,euler,insdata.steert,1,result,offile,oferror);
                loosecouple(insdata,lcdata,config,gyroai_2,acceai_2,euler,tk,0,result,offile,oferror);

                if(!ifsteer.eof())
                    ifsteer>>insdata.steert>>insdata.steerl>>insdata.steerr;
                else
                    break;
            }
#endif
        }
    }
    offile.close();
    ifgnss.close();
    ifimu.close();

    return 0;
}