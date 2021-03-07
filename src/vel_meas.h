//
// Created by zzx on 2021/3/6.
//

#ifndef GIOW_VEL_MEAS_H
#define GIOW_VEL_MEAS_H

#include "lc.h"
#include <fstream>
#include <iomanip>
#include "lcdefine.h"
void loosecouple(InsData &insdata,Lcdata &lcdata,LcConfig &config,Vector3d &gyroai,Vector3d &acceai,Vector3d &euler,
                 double &tk,double type,Result &result,ofstream &offile,ofstream &ofodostd);
void kalmannhc(Lcdata &Lcdata,InsData &insData,LcConfig &config);
void kalmanodo(Lcdata &Lcdata,InsData &insdata,LcConfig &config);

#endif //GIOW_VEL_MEAS_H
