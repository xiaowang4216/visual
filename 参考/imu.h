#ifndef _IMU_H
#define _IMU_H

#include "sys.h"
#include "stdbool.h"
 
typedef struct{
        float X;
				float Y;
				float Z;
              }Attitudat_Struct;//MPU姿态数据结构体
 
extern Attitudat_Struct Accel,Gyro,Angle;
extern Attitudat_Struct Gyro_Offset;  //陀螺仪零漂数据
extern short Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z;
 
#define GyroScale    0.000266463 

#define FILTER_NUM 	15
 
#define DEGREES_TO_DECIDEGREES(angle) (angle * 10)
#define DECIDEGREES_TO_DEGREES(angle) (angle / 10)
#define DECIDEGREES_TO_RADIANS(angle) ((angle / 10.0f) * 0.0174532925f)
#define DEGREES_TO_RADIANS(angle) ((angle) * 0.0174532925f)

u8 Gyro_calibration(void);
void Prepare_Data(int *accx,int *accy,int *accz,int *accoutx,int *accouty,int *accoutz);

void MPU_GetAngleDat(void);
 void imuMahonyAHRSupdate(float dt, float gx, float gy, float gz,
													bool useAcc, float ax, float ay, float az,
													bool useMag, float mx, float my, float mz,
													bool useYaw, float yawError);
void imuComputeRotationMatrix(void);
void imuUpdateEulerAngles(float *roll,float *pitch, float *yaw);



#endif
