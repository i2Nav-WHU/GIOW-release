//
// Created by zzx on 2021/3/6.
//
#include "vel_meas.h"

//loosecouple
void loosecouple(InsData &insdata,Lcdata &lcdata,LcConfig &config,Vector3d &gyroai,Vector3d &acceai,Vector3d &euler,
                 double &tk,double type,Result &result,ofstream &offile,ofstream &ofodostd){
    insdata_update(insdata,gyroai,acceai,tk);
    insdata_resolve(insdata);
    kalmanpredict(lcdata,insdata,config);
    if(type==3){
        kalmanupdate(lcdata,insdata,config);
    }
    else if(type==2){
        kalmanodo(lcdata,insdata,config);
    }
    else if(type==1){
#if !USE_STEER
        static int inhc=0;
        inhc=(inhc+1)%20;
        if(inhc==0)
#endif
        kalmannhc(lcdata,insdata,config);
    }
#if ErrorAnalysis
    ofodostd<<std::fixed<<setprecision(8)<<insdata.tk_now<<"  "<<insdata.gyrobias.transpose()<<"  "<<insdata.accebias.transpose()
            <<"  "<<insdata.gyroscale.transpose()<<"  "<<insdata.accescale.transpose()<<"  "<<insdata.odoscale<<"  "<<insdata.deltacita<<std::endl;
#endif
    //导出结果
    quat2euler(insdata.Qbn_now,euler);
    result.t=insdata.tk_now;
    for(int i=0;i<3;i++)
    {
        result.Rn[i]=insdata.Rn_now(i);
        result.Vn[i]=insdata.Vn_now(i);
        result.euler[i]=euler(i)*R2D;
    }
    result.Rn[0]=result.Rn[0]*R2D;
    result.Rn[1]=result.Rn[1]*R2D;
    insdata.eulerAngle<<result.euler[0],result.euler[1],result.euler[2];
    static char out[200];
    sprintf(out, "%d %.8lf %.10lf %.10lf %.3lf %.3lf %.3lf %.3lf %.8lf %.8lf %.8lf \n",0,result.t,
            result.Rn[0],result.Rn[1],result.Rn[2],result.Vn[0],result.Vn[1],result.Vn[2],
            result.euler[0],result.euler[1],result.euler[2]);
    offile<<out;
}

//kalman nhc
void kalmannhc(Lcdata &Lcdata,InsData &insdata,LcConfig &config){
    Vector3d col0=Vector3d::Zero();
    Matrix3d zero=Matrix3d::Zero();
    Matrix3d Cvw=Matrix3d::Identity();
    Matrix<double,3,23> Hk;
    Vector3d lodob;
    Matrix3d Lodob,Cnv,Cnb,HvG3,HvG6,Vimu,WINn;
    Matrix<double,23,23> I=Matrix<double,23,23>::Identity();
    Cnb=Lcdata.Cbn.transpose();
    Cnv=config.Cbv*Cnb;
    lodob=config.lodo;
    vec2ssm(lodob,Lodob);
    vec2ssm(Lcdata.WINn,WINn);
    vec2ssm(insdata.Vn_now,Vimu);
    HvG3=-Cnv*Vimu;
    HvG6=-config.Cbv*Lodob*Lcdata.WIBb.asDiagonal();
#if !USE_STEER
    Hk<<zero,Cnv,HvG3,-config.Cbv*Lodob,zero,HvG6,zero,col0,col0;
#else
    Vector3d cita,Hcita,Vimuv;
    cita<<0,0,0.5*(insdata.steerl+insdata.steerr)-insdata.deltacita;
    euler2dcm(-cita,Cvw);
    Vimuv=Cnv*insdata.Vn_now;
    Hcita<<-Vimuv(0)*sin(cita(2))+Vimuv(1)*cos(cita(2)),
            -Vimuv(0)*cos(cita(2))-Vimuv(1)*sin(cita(2)),0;
#if USE_STEER==2
    Hk<<zero,Cvw*Cnv,Cvw*HvG3,-Cvw*config.Cbv*Lodob,zero,Cvw*HvG6,zero,col0,Hcita;
#else
    Hk<<zero,Cvw*Cnv,Cvw*HvG3,-Cvw*config.Cbv*Lodob,zero,Cvw*HvG6,zero,col0,col0;
#endif
#endif

    Vector2d deltaZk,Rk;
    Matrix2d Rkmat;
    Vector3d vimuv;
    Rk<<config.odostd(1)*config.odostd(1),config.odostd(2)*config.odostd(2);
    vimuv=Cnv*insdata.Vn_now-Cnv*WINn*Lcdata.Cbn*lodob-config.Cbv*Lodob*Lcdata.WIBb;
    vimuv=Cvw*vimuv;
    deltaZk<<vimuv(1),vimuv(2);
    Rkmat=Rk.asDiagonal();

    Matrix<double,23,2> Kk;
    Matrix<double,2,23> Hk2;
    for(int i=0;i<23;i++){
        Hk2(0,i)=Hk(1,i);
        Hk2(1,i)=Hk(2,i);
    }
    Kk=Lcdata.Pk_now*Hk2.transpose()*(Hk2*Lcdata.Pk_now*Hk2.transpose()+Rkmat).inverse();
    Lcdata.error_now=Kk*deltaZk;
    Lcdata.Pk_now=(I-Kk*Hk2)*Lcdata.Pk_now*(I-Kk*Hk2).transpose()+Kk*Rkmat*Kk.transpose();

    feedback(Lcdata,insdata);
}

//kalman odo
void kalmanodo(Lcdata &Lcdata,InsData &insdata,LcConfig &config){
    Vector3d col0=Vector3d::Zero();
    Matrix3d zero=Matrix3d::Zero();
    Matrix3d Cvw=Matrix3d::Identity();
    Vector3d vodo,lodob;
    Matrix3d Lodob,Cnv,Cnb,HvG3,HvG6,Vimu,WINn;
    vodo<<(insdata.odovl+insdata.odovr)/2,0,0;
    vodo=vodo/(1+insdata.odoscale);
    Matrix<double,3,23> Hk;
    Matrix<double,23,23> I=Matrix<double,23,23>::Identity();
    Cnb=Lcdata.Cbn.transpose();
    Cnv=config.Cbv*Cnb;
    lodob=config.lodo;
    vec2ssm(lodob,Lodob);
    vec2ssm(Lcdata.WINn,WINn);
    vec2ssm(insdata.Vn_now,Vimu);
    HvG3=-Cnv*Vimu;
    HvG6=-config.Cbv*Lodob*Lcdata.WIBb.asDiagonal();
#if !USE_STEER
    Hk<<zero,Cnv,HvG3,-config.Cbv*Lodob,zero,HvG6,zero,-vodo,col0;
#else
    Vector3d cita,Hcita,Vimuv;
    cita<<0,0,0.5*(insdata.steerl+insdata.steerr)-insdata.deltacita;
    euler2dcm(-cita,Cvw);
    Vimuv=Cnv*insdata.Vn_now;
    Hcita<<-Vimuv(0)*sin(cita(2))+Vimuv(1)*cos(cita(2)),
           -Vimuv(0)*cos(cita(2))-Vimuv(1)*sin(cita(2)),0;
#if USE_STEER == 1
    Hcita = col0;
#endif
    Hk<<zero,Cvw*Cnv,Cvw*HvG3,-Cvw*config.Cbv*Lodob,zero,Cvw*HvG6,zero,-vodo,Hcita;
#endif

    Vector3d deltaZk,Rk,vimuv;
    Matrix3d Rkmat;
    Rk<<config.odostd(0)*config.odostd(0),config.odostd(1)*config.odostd(1),config.odostd(2)*config.odostd(2);
    vimuv=Cnv*insdata.Vn_now-Cnv*WINn*Lcdata.Cbn*lodob-config.Cbv*Lodob*Lcdata.WIBb;
    deltaZk=Cvw*vimuv-vodo;
    Rkmat=Rk.asDiagonal();

    Matrix<double,23,3> Kk;
    Kk=Lcdata.Pk_now*Hk.transpose()*(Hk*Lcdata.Pk_now*Hk.transpose()+Rkmat).inverse();
    Lcdata.error_now=Kk*deltaZk;
    Lcdata.Pk_now=(I-Kk*Hk)*Lcdata.Pk_now*(I-Kk*Hk).transpose()+Kk*Rkmat*Kk.transpose();

    feedback(Lcdata,insdata);
}
