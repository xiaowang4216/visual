#include "imu.h"
#include "imu.h"
#include  "math.h"
#include "flycontro.h"

#define SPIN_RATE_LIMIT 20

#ifndef sq
#define sq(x) ((x)*(x))
#endif
#define sin_approx(x)   sinf(x)
#define cos_approx(x)   cosf(x)
#define atan2_approx(y,x)   atan2f(y,x)
#define acos_approx(x)      acosf(x)
#define tan_approx(x)       tanf(x)
// Use floating point M_PI instead explicitly.
#define M_PIf       3.14159265358979323846f


 

Attitudat_Struct Accel,Gyro,Angle; //MPU姿态数据
Attitudat_Struct Gyro_Offset;  //陀螺仪零漂数据
short Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z;
void MPU_GetAngleDat(void)
{
  int  Accel_Xtemp,Accel_Ytemp,Accel_Ztemp,Accel_Xtempout,Accel_Ytempout,Accel_Ztempout;
 /// static float Yawerr=0,lastYawerr=0;
  MPU_Get_Accelerometer(&Accel_X,&Accel_Y,&Accel_Z);	//得到加速度传感器数据
  MPU_Get_Gyroscope(&Gyro_X,&Gyro_Y,&Gyro_Z);	//得到陀螺仪数据
  
	Accel_Xtemp  = Accel_X;
	Accel_Ytemp  = Accel_Y;
	Accel_Ztemp  = Accel_Z;
 
  Prepare_Data(&Accel_Xtemp,&Accel_Ytemp,&Accel_Ztemp,&Accel_Xtempout ,&Accel_Ytempout,&Accel_Ztempout);
 
	Accel.X = Accel_Xtempout;     ////8192
	Accel.Y = Accel_Ytempout;
  Accel.Z = Accel_Ztempout; 	
	
	Gyro.X = (Gyro_X - Gyro_Offset.X) ;
	Gyro.Y = (Gyro_Y - Gyro_Offset.Y) ;
	Gyro.Z = (Gyro_Z - Gyro_Offset.Z) ;
 
} 
 

u8 Gyro_calibration(void)
{
//u16 i;
//short  Gyro_X,Gyro_Y,Gyro_Z;
static long int GyroofsetX=0,GyroofsetY=0,GyroofsetZ=0;
static u16 i= 0;
//	for(i=0;i<1000;i++)
//	 {
// MPU_Get_Gyroscope(&Gyro_X,&Gyro_Y,&Gyro_Z);	//得到陀螺仪数据
    GyroofsetX+=Gyro_X;
		GyroofsetY+=Gyro_Y;
		GyroofsetZ+=Gyro_Z;
	  i++;
//		delay_ms(5);
//   }
//	
	if(i==1000) 
	{
   i=0;
   Gyro_Offset.X = GyroofsetX/1000;
   Gyro_Offset.Y = GyroofsetY/1000;
   Gyro_Offset.Z = GyroofsetZ/1000;
	 GyroofsetX=GyroofsetY=GyroofsetZ=0;
		return 1;
	}

	return 0;
}


void Prepare_Data(int *accx,int *accy,int *accz,int *accoutx,int *accouty,int *accoutz)
{  
	static uint8_t 	filter_cnt=0;
	static int16_t	ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];
	int32_t temp1=0,temp2=0,temp3=0;
	uint8_t i;	
	ACC_X_BUF[filter_cnt] = *accx;
	ACC_Y_BUF[filter_cnt] = *accy;
	ACC_Z_BUF[filter_cnt] = *accz;
	for(i=0;i<FILTER_NUM;i++)//滑动平滑滤波
	{
		temp1 += ACC_X_BUF[i];
		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
	}
	*accoutx = temp1 / FILTER_NUM;
	*accouty = temp2 / FILTER_NUM;
	*accoutz = temp3 / FILTER_NUM;
	filter_cnt++;
	if(filter_cnt==FILTER_NUM)	filter_cnt=0;

}












float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;    // quaternion of sensor frame relative to earth frame
static float rMat[3][3];
 
float dcmKiGain = 0.005;
float dcmKpGain = 2.0;

static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

void imuMahonyAHRSupdate(float dt, float gx, float gy, float gz,
													bool useAcc, float ax, float ay, float az,
													bool useMag, float mx, float my, float mz,
													bool useYaw, float yawError)
{
    static float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;    // integral error terms scaled by Ki
    float recipNorm;
    float hx, hy, bx;
    float ex = 0, ey = 0, ez = 0;
    float qa, qb, qc;
    
    // Calculate general spin rate (rad/s) 
    float spin_rate = sqrtf(sq(gx) + sq(gy) + sq(gz));
	  float ez_ef;
    // Use raw heading error (from GPS or whatever else)
    if (useYaw) {
        while (yawError >  M_PIf) yawError -= (2.0f * M_PIf);
        while (yawError < -M_PIf) yawError += (2.0f * M_PIf);

        ez += sin_approx(yawError / 2.0f);
    }

    // Use measured magnetic field vector  
    recipNorm = sq(mx) + sq(my) + sq(mz);
    if (useMag && recipNorm > 0.01f)
			{
        // Normalise magnetometer measurement ?????????
        recipNorm = invSqrt(recipNorm);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;
				// For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
				// This way magnetic field will only affect heading and wont mess roll/pitch angles

				// (hx; hy; 0) - measured mag field vector in EF (assuming Z-component is zero)
				// (bx; 0; 0) - reference mag field vector heading due North in EF (assuming Z-component is zero)
        hx = rMat[0][0] * mx + rMat[0][1] * my + rMat[0][2] * mz;
        hy = rMat[1][0] * mx + rMat[1][1] * my + rMat[1][2] * mz;
        bx = sqrtf(hx * hx + hy * hy);

        // magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
        ez_ef = -(hy * bx);

        // Rotate mag error vector back to BF and accumulate
        ex -= rMat[2][0] * ez_ef;
        ey -= rMat[2][1] * ez_ef;
        ez += rMat[2][2] * ez_ef;
    }

    // Use measured acceleration vector ?????????
    recipNorm = sq(ax) + sq(ay) + sq(az);
    if (useAcc && recipNorm > 0.01f) 
			{
        // Normalise accelerometer measurement ?????????
        recipNorm = invSqrt(recipNorm);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Error is sum of cross product between estimated direction and measured direction of gravity
        ex += (ay * rMat[2][2] - az * rMat[2][1]);
        ey += (az * rMat[2][0] - ax * rMat[2][2]);
        ez += (ax * rMat[2][1] - ay * rMat[2][0]);
    }

    // Compute and apply integral feedback if enabled
    if(dcmKiGain > 0.0f) {   //imuRuntimeConfig->dcm_ki
        // Stop integrating if spinning beyond the certain limit????,??????????
        if (spin_rate < DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)) {
       //     float dcmKiGain = dcm_ki;
            integralFBx += dcmKiGain * ex * dt;    // integral error scaled by Ki
            integralFBy += dcmKiGain * ey * dt;
            integralFBz += dcmKiGain * ez * dt;
        }
    }
    else {
        integralFBx = 0.0f;    // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
        }

    // Calculate kP gain. If we are acquiring initial attitude (not armed and within 20 sec from powerup) scale the kP to converge faster
    //   dcmKpGain =  dcm_kp ;//* imuGetPGainScaleFactor();

    // Apply proportional and integral feedback??????
    gx += dcmKpGain * ex + integralFBx;
    gy += dcmKpGain * ey + integralFBy;
    gz += dcmKpGain * ez + integralFBz;

    // Integrate rate of change of quaternion??????
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);
    // Normalise quaternion
    recipNorm = invSqrt(sq(q0) + sq(q1) + sq(q2) + sq(q3));
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // Pre-compute rotation matrix from quaternion
    imuComputeRotationMatrix();
#if 0	 
//	 	Angle.Y = asin(-2 * q1 * q3  + 2 * q0* q2)* 57.3 ; // pitch  
//	  Angle.X = atan2( 2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
//	  Angle.Z = atan2(2 * q1 * q2  + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* 57.3; // yaw 
#else	 
	 imuUpdateEulerAngles(&Angle.X,&Angle.Y,&Angle.Z);
#endif

}

 void imuComputeRotationMatrix(void)
{
    float q1q1 = sq(q1);
    float q2q2 = sq(q2);
    float q3q3 = sq(q3);

    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}


void imuUpdateEulerAngles(float *roll,float *pitch, float *yaw)
{
	
    /* Compute pitch/roll angles */
	  *roll =  atan2_approx(rMat[2][1], rMat[2][2]) * 57.2958;
	  *pitch=((0.5f * M_PIf) - acos_approx(-rMat[2][0])) *  57.2958;
	  
 
}


