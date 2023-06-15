//
// Created by zzx on 2019/12/11.
//

#ifndef INS_LC_H
#define INS_LC_H

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <cmath>
#include <fstream>
using namespace Eigen;

/* WGS84椭球模型参数 */
#define WGS84_RA 6378137.0
#define WGS84_E1 0.00669437999013
#define WGS84_WIE 7.2921151467e-5

#define PI 3.141592653589793
#define R2D 57.29577951308232
#define D2R 0.017453292519943295

class InsData {
public:
    Vector3d Vn_last;
    Vector3d Vn_now;
    Vector3d Rn_last;
    Vector3d Rn_now;
    Quaterniond Qbn_last;
    Quaterniond Qbn_now;
    Quaterniond Qne_last;
    Quaterniond Qne_now;
    Vector3d gyroai_last;
    Vector3d gyroai_now;
    Vector3d acceai_last;
    Vector3d acceai_now;
    Vector3d eulerAngle;
    Vector3d gl;
    Vector3d gyrobias=Vector3d::Zero();
    Vector3d accebias=Vector3d::Zero();
    Vector3d gyroscale=Vector3d::Zero();
    Vector3d accescale=Vector3d::Zero();
    double tk_now;
    double tk_last;
    double RN;
    double RM;
    double odot;
    double odovl;
    double odovr;
    double steert;
    double steerl;
    double steerr;
    double odoscale=0;
    double deltacita=0;
    Matrix3d Cnn;
};

struct LcConfig {
    double starttime = 438080;
    double endtime = 440180;//438080,440180,442050;
    Vector3d outagetime = {starttime+300,30,120}; //300, 250
    double ROLL;             //deg
    double PITCH;            //deg
    double HEADING;          //deg
    double LATITTUDE;        //deg
    double LONGITUDE;        //deg
    double HEIGHT;
    double Vel_n;
    double Vel_e;
    double Vel_d;
    void ConfigInit(std::ifstream &ifrefer){
        double temp = 0, temp0;
        while(temp<starttime){
            ifrefer>>temp0>>temp>>LATITTUDE>>LONGITUDE>>HEIGHT>>Vel_n>>Vel_e>>Vel_d>>ROLL>>PITCH>>HEADING;
        }
    }

    //松组合初始数据
    double arw = 0.1 * D2R / 60.0;        //rad/s/sqrt(s) // 0.003deg/s/sqrt(hr)
    double vrw = 0.1  / 60.0;        //m/s/sqrt(s) // 0.03m/s/sqrt(hr)
    double Vgb = 50 * D2R / 3600.0;        //rad/s // 0.027deg/hr
    double Vab = 50e-5;                       //m/s^2 //15mGal
    double Vgs = 1000e-6;                      // //200ppm
    double Vas = 1000e-6;                      // //200ppm
    double Tgb = 3600;                       // s
    double Tab = 3600;                       // s
    double Tgs = 3600;                       // s
    double Tas = 3600;                       // s
    Vector3d lgnss={-0.345,0,0.186};
    Vector3d Cbveuler={0,0.132,-1.173};     //deg
    Vector3d lodo={0,0,0.932};
    double latstd = 0.1;                    // m
    double lonstd = 0.1;                    // m
    double hstd = 0.1;                      // m
    double vnstd = 0.1;                     // m/s
    double vestd = 0.1;                     // m/s
    double vdstd = 0.1;                     // m/s
    double rollstd = 0.1;                   //deg
    double pitchstd = 0.1;                  //deg
    double yawstd = 0.1;                    //deg
    double lgnssF = lgnss(0);                    //m
    double lgnssR = lgnss(1);                   //m
    double lgnssD = lgnss(2);                   //m
    Matrix3d Cbv=Matrix3d::Identity();
    Vector4d odostd ={0.1,0.1,0.1,0.1};   //m,m,m,deg(姿态辅助观测噪声)
    double Sodostd=0.1;
    double srw=0.00001;
    double citastd=0.1;
    double crw=0.0001;
//    double wheeldist=0.46;
//    double axledist=0.4;
};

void euler2dcm(const Vector3d &eulerAngle, Matrix3d &dcm);
void euler2quat(const Vector3d &eulerAngle,Quaterniond &quaternion);
void dcm2euler(const Matrix3d &dcm,Vector3d &eulerAngle);
//void dcm2quat(const Matrix3d &dcm,Quaterniond &quaternion);
void quat2euler(Quaterniond &quaternion,Vector3d &eulerAngle);
void quat2dcm(Quaterniond &quaternion,Matrix3d &dcm);
void vec2ssm(const Vector3d &vector,Matrix3d &ssm);
void quat2pos(const Quaterniond &quat,double &lat,double &lon);
void pos2quat(double &lat,double &lon,Quaterniond &quat);
void rotvec2quat(const Vector3d &vector,Quaterniond &quaternion);


void radiusmn(const double &lat, double &rm, double &rn);
void getGravity(const double &lat,const double &height,Vector3d &gl);
void insdata_resolve(InsData &insdata);
void insdata_init(InsData &insdata,Vector3d &gyroai,Vector3d &acceai,double &tk,LcConfig &config);
void insdata_update(InsData &insdata,Vector3d &gyroai,Vector3d &acceai, double &tk);

struct Lcdata {
    Matrix3d Cbn;
    double dt,t;
    Vector3d Zk,Rk;
    Vector3d WINn,WIBb;
    Matrix<double,23,23> Phi;
    Matrix<double,20,20> Qm;
    Matrix<double,23,20> G;
    Matrix<double,23,23> Qk;
    Matrix<double,3,23> Hk;
    Matrix<double,23,23> Pk_now;
    Matrix<double,23,23> Pk_last;
    Matrix<double,23,1> error_now;
};

using namespace std;
void Lcinit(Lcdata &Lcdata,LcConfig &config);
void updatePhi(InsData &insdata,Lcdata &Lcdata,LcConfig &config);
void updateQk(Lcdata &Lcdata);
void updateHk(Lcdata &Lcdata,LcConfig &config);
void kalmanpredict(Lcdata &Lcdata,InsData &insdata,LcConfig &config);
void kalmanupdate(Lcdata &Lcdata,InsData &insdata,LcConfig &config);
void feedback(Lcdata &Lcdata,InsData &insdata);

#endif //INS_LC_H
