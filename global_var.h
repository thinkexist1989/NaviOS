#ifndef GLOBAL_VAR_H
#define GLOBAL_VAR_H

#include "kalman/coordinate.h"

#include "fstream"
#include "iostream"
#include <QMutex>

using namespace std;

/*********Compass****************/
extern bool g_NewAngleMark;       //Recieve New Angle Mark

extern int g_circle;
extern int g_Angle_Ex;             //前一个角度值
extern double g_radian;
extern int g_Angle;               //当前角度值
extern int g_Angle_Bai;           //角度值百位
extern int g_Angle_Shi;           //角度值十位
extern int g_Angle_Ge;            //角度值个位

extern int g_Angle_Change;        //角度变化，向左为负，向右为正

/*********Voice******************/
extern bool g_NewVoiceMark;
extern bool g_NewPlayMark;

extern char g_currentposition;      //导航系统当前位置，每次定位后赋值修改
extern char g_lastposition;        //导航系统上一时刻位置

extern char g_destination;       //导航系统目的地

extern char g_VoiceOrder;           //从语音模块上受到的命令

extern char g_PlayOrder;            //发送给语音模块的命令



/*********UltraSonic*************/
extern bool g_NewLocMark;          //新定位标志

extern int g_RcvNumCnt;           //接收到有效距离个数
extern int g_RcvNum;              //接收模块代号0~3（发射时用）
extern int g_GroupNum;            //接收模块代号0~3（接收时用）
extern int g_Group[4];            //由于参数传递给Calculation时需要矩阵，因此建立
extern float g_dist[4];
extern double g_ValidDist[4];
extern int g_ID[4];

extern RECEIVER_TABLE table;

extern PACKAGE_SINGLE package_single;
extern PACKAGE_MULTI package_multi;


extern COORDINATE c;
extern const double initial_state_uncertainty[9];
extern MATRIX cov;
extern STATE state_abs;
extern STATE state_rel;
extern INPUT input;

extern COORDINATE abs_obs;
extern COORDINATE delta;
extern double vector_obs_abs[3];
extern MATRIX R_obs;
extern MATRIX kalman_gain_rel;
extern MATRIX kalman_gain_abs;

/**********Encoder***********/
extern int g_LeftPulse;                //Left Encoder Pulse
extern int g_RightPulse;               //Right Encoder Pulse
extern double g_LeftDistance;      //Left Encoder Walking Distance
extern double g_RightDistance;    //Right Encoder Walking Distance

extern int g_LeftPulseDisplay;
extern int g_RightPulseDisplay;



extern QMutex g_us_mutex;
extern QMutex g_cp_mutex;

extern ofstream fout;
extern ofstream usdata;
extern ofstream locdata;
extern int g_count;




#endif // GLOBAL_VAR_H
