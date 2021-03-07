//
// Created by zzx on 2019/10/28.
//

#ifndef INS_LCDEFINE_H
#define INS_LCDEFINE_H

#define ErrorAnalysis 1
#define USE_NHC 0
#define USE_ODO 1
#define USE_STEER 1

struct ImuRaw {
    double sow;       /* s */
    double dtheta[3]; /* 陀螺, rad */
    double dvel[3];   /* 加表, m/s */
};
struct Result {
    double t;
    double Rn[3];
    double Vn[3];
    double euler[3];
};


#endif //INS_LCDEFINE_H
