#ifndef CONSTANTS_H
#define CONSTANTS_H

/********UltraSonic Constants**********/
#define MAXDIS       2500   //超声波最大接收距离2500mm，超过的认为是干扰信号
#define	MaxRcvByte_C 64	    //接收区缓存大小
#define MaxTxdByte_C 64     //发射区缓存大小
#define	MY_ADDR      0x56	// E8地址
#define TXD_ADDR     0xA0   //发射模块地址
#define SET_WORKMODE 0x03
#define SINGLE_MEASURE 0x20
#define HumanHeight  1500
#define XInit        100
#define YInit        100
#define RELATIVE_X     0
#define RELATIVE_Y     1

/********Compass Constants************/
#define SINGLE_ANGLE 0x01
#define START_CALIBRATION 0x02
#define STOP_CALIBRATION  0x03
#define SET_ZERO 0x04

/*********Voice Constants************/

//VoiceOrder
#define RECIEVED_CALL 0x00
#define CURRENT_POSITION    0x01
#define GO_LAB1055   0x02
#define GO_DATING    0x03

//PlayOrder

#define POSITION_1    0x01
#define POSITION_2    0x02
#define POSITION_3    0x03

#define IN_LAB1056    0x01
#define LAB1056       0x02
#define LAB1057       0x03
#define LAB1055       0x04
#define LAB1050       0x05
#define DATING        0x06
#define ZOULANG       0x07


//超声波阵列中心点坐标
const float RELATIVENODE[3][2] = {
                                  {4905   ,  3150},
                                  {13835  , 7650},
                                  {23810  , 12620},
                                  };


/*********Encoder Constants***************/
#define FORWARD 1
#define BACKWARD 0

#define LEFT_PLUSE       0
#define LEFT_DIRECTION   2
#define RIGHT_PLUSE      1
#define RIGHT_DIRECTION  3

#define PLUSE_LIMIT      1

#define PLUSES_PER_CIRCLE 100
#define WHEEL_DIAMETER   95    //length unit:mm
#define PI               3.14159

#endif // CONSTANTS_H
