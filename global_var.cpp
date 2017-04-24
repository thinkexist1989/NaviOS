#include "global_var.h"



/*********Compass****************/
bool g_NewAngleMark;       //Recieve New Angle Mark

int g_circle = 0;          //circle record
int g_Angle_Ex;             //前一个角度值
double g_radian;
int g_Angle;               //当前角度值，单位：度，需转化为弧度制
int g_Angle_Bai;           //角度值百位
int g_Angle_Shi;           //角度值十位
int g_Angle_Ge;            //角度值个位
int g_Angle_Change;        //角度变化，向左为负，向右为正

/*********Voice******************/
bool g_NewVoiceMark;
bool g_NewPlayMark;

char g_currentposition = 0;      //导航系统当前位置，每次定位后赋值修改
char g_lastposition = 0;        //导航系统上一时刻位置

char g_destination = 0;       //导航系统目的地

char g_VoiceOrder = 9;           //从语音模块上受到的命令

char g_PlayOrder = 9;            //发送给语音模块的命令


/*********UltraSonic*************/
bool g_NewLocMark;          //新定位标志

int g_RcvNumCnt;           //接收到有效距离个数
int g_RcvNum = 0;              //接收模块代号0~3（发射时用）
int g_GroupNum;            //接收模块代号0~3（接收时用）
int g_Group[4];            //由于参数传递给Calculation时需要矩阵，因此建立
float g_dist[4];          //单位:mm
double g_ValidDist[4];
int g_ID[4];

RECEIVER_TABLE table;

PACKAGE_SINGLE package_single;
PACKAGE_MULTI package_multi;


STATE state_abs (4000, -1000, 0, 0, abs_cov);
STATE state_rel (0, 0, 0, 0, rel_cov);
INPUT input;

COORDINATE abs_obs;
COORDINATE delta;
double vector_obs_abs[3];
MATRIX R_obs (2, 2);
MATRIX kalman_gain_rel (3, 1);
MATRIX kalman_gain_abs (3, 3);



/**********Encoder***********/
int g_LeftPulse;                //Left Encoder Pulse
int g_RightPulse;               //Right Encoder Pulse
double g_LeftDistance;      //Left Encoder Walking Distance    unit:mm
double g_RightDistance;    //Right Encoder Walking Distance    unit:mm

int g_LeftPulseDisplay;
int g_RightPulseDisplay;


/*******thread lock*********/
QMutex g_us_mutex;
QMutex g_cp_mutex;


/********txt file ouput*****/
ofstream fout;
ofstream usdata;
ofstream locdata;
int g_count = 0;
