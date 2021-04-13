/************************************************************
 *File		:	euler.c
 *Author	:  @YangTianhao ,490999282@qq.com,@TangJiaxin ,tjx1024@126.com
 *Version	: V1.0
 *Update	: 2017.03.02
 *Description: 	euler caluation functions
 ************************************************************/

#include "euler.h"
#include "math.h"
#include "main.h"
#define euler_dt 0.002f
#define sampleFreq	500.0f		// sample frequency in Hz
#define betaDef		0.1f		// 2 * proportional gain
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain
volatile float beta = betaDef;								// 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
float Yaw_gyro,Roll_gyro,Pitch_gyro;
float Yaw_mag,Roll_accel,Pitch_accel;
float Yaw,Roll,Pitch,Yaw_Offset,Pitch_Offset,Roll_Offset;
float Yaw_quat,Roll_quat,Pitch_quat;
float dt=0.001;//??:dt????kalman???????
float Yawdelta;
float angle, angle_dot;//??????
volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki
float Yaw_watch,Yaw_watch_quat;

int count = 0;
float delay_speed=0;
int euler_count=0;
double yaw_zero_float=0;
double zf_result=0.0f;

float invSqrt(float num);


void init_euler(void){
	
	unsigned char i;
	float ax,ay,az,a;
	ax=ay=az=0.0f;
	if(ax==0&&ay==0&&az==0)
	{
			for(i=0;i<5;i++){
			MPU6500_Read();
			ax+=mpu6500_real_data.Accel_X;
			ay+=mpu6500_real_data.Accel_Y;
			az+=mpu6500_real_data.Accel_Z;
			
			delay_ms(5);
		}
	}
	a=1.0f/sqrt(ax*ax+ay*ay+az*az);
	ax*=a;
	ay*=a;
	az*=a;
	Roll=atan2(-ay,az);
	Pitch=asin(ax);
	Yaw=0;
//	psai yaw theta pitch fai roll
	q0=cos(Roll/2)*cos(Pitch/2)*cos(Yaw/2)+sin(Roll/2)*sin(Pitch/2)*sin(Yaw/2);
	q1=sin(Roll/2)*cos(Pitch/2)*cos(Yaw/2)-cos(Roll/2)*sin(Pitch/2)*sin(Yaw/2);
	q2=cos(Roll/2)*sin(Pitch/2)*cos(Yaw/2)+sin(Roll/2)*cos(Pitch/2)*sin(Yaw/2);
	q3=cos(Roll/2)*cos(Pitch/2)*sin(Yaw/2)-sin(Roll/2)*sin(Pitch/2)*cos(Yaw/2);	
}


/*
void update_euler_mpu(void){
	MPU6500_Read();
	float gx,gy,gz,ax,ay,az;
	gx=mpu6500_real_data.Gyro_X;
	gy=mpu6500_real_data.Gyro_Y;
	gz=mpu6500_real_data.Gyro_Z;
	ax=mpu6500_real_data.Accel_X;
	ay=mpu6500_real_data.Accel_Y;
	az=mpu6500_real_data.Accel_Z;
	MadgwickAHRSupdateIMU(gx,gy,gz,ax,ay,az);

}
*/

/************************************
*Original
************************************/

void update_euler_mpu(void){
	
	float Roll_sin,Roll_cos,Pitch_sin,Pitch_cos;
	float a,gx,gy,gz,dx,dy,dz;
	static const float K=1.0f;  //?????????? ?? ????????
	static const float KP=0.5f; //?????P????? ?????????
	Roll_sin=sin(Roll);
	Roll_cos=cos(Roll);
	Pitch_sin=sin(Pitch);
	Pitch_cos=cos(Pitch);
	
	MPU6500_Read();
	delay_speed=mpu6500_real_data.Gyro_Z * 57.3f;
	a=invSqrt(mpu6500_real_data.Accel_X*mpu6500_real_data.Accel_X
		+mpu6500_real_data.Accel_Y*mpu6500_real_data.Accel_Y
	+mpu6500_real_data.Accel_Z*mpu6500_real_data.Accel_Z);
	mpu6500_real_data.Accel_X*=a;
	mpu6500_real_data.Accel_Y*=a;
	mpu6500_real_data.Accel_Z*=a;

	dx=mpu6500_real_data.Accel_Y*Pitch_cos*Roll_cos-mpu6500_real_data.Accel_Z*Pitch_cos*Roll_sin;
	dy=mpu6500_real_data.Accel_Z*(-Pitch_sin)-mpu6500_real_data.Accel_X*Pitch_cos*Roll_cos;
	dz=mpu6500_real_data.Accel_X*Pitch_cos*Roll_sin+mpu6500_real_data.Accel_Y*Pitch_sin;

	gx=mpu6500_real_data.Gyro_X+KP*dx;
	gy=mpu6500_real_data.Gyro_Y+KP*dy;
	gz=mpu6500_real_data.Gyro_Z+KP*dz;
	
	
	Roll=Roll+(Pitch_cos*gx+Pitch_sin*Roll_sin*gy+Pitch_sin*Roll_cos*gz)/Pitch_cos*euler_dt;
	
	Pitch=Pitch+(Roll_cos*gy-Roll_sin*gz)*euler_dt;
	
	
	Yaw_gyro=Yaw+(Roll_sin*gy+Roll_cos*gz)/Pitch_cos*euler_dt;
//	Yaw_gyro=Yaw+(Roll_sin*gy+Roll_cos*gz)/Pitch_cos*euler_dt;
	
	if(Roll>PI){	
		Roll-=2.0f*PI;
	}else if(Roll<-PI){
		Roll+=2.0f*PI;
	}
	
	if(Pitch>PI/2.0f){	
		Pitch-=PI;
	}else if(Pitch<-PI/2.0f){
		Pitch+=PI;
	}
//	if(remoteState == KEY_REMOTE_STATE)
////	{
////		Yaw=K*Yaw_gyro+(1.0f-K)*Yaw_mag-dynamic_zero_float_offset;
////		//Yaw=-Yaw;
////	}
// if(remoteState == KEY_REMOTE_STATE)
//	{
		
		float Yaw_out=K*Yaw_gyro+(1.0f-K)*Yaw_mag+zf_result;

		Yaw=Yaw_out;
		Yaw_watch=Yaw*57.3f;
//	}
//	else if(remoteState == NORMAL_REMOTE_STATE)
//	{
//		Yaw=K*Yaw_gyro+(1.0f-K)*Yaw_mag-dynamic_zero_float_offset;
//	}
	if(count <= 10)    //??offset?
	{	
		Yaw_Offset = Yaw ;
		Pitch_Offset = 0;
		Roll_Offset = 0;
		
		count++;
	}
	
}



/******************************
*quaternion
******************************/

void update_euler_mpu_quat(void){
	
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float ax,ay,az,gx,gy,gz;
	ax=mpu6500_real_data.Accel_X;
	ay=mpu6500_real_data.Accel_Y;
	az=mpu6500_real_data.Accel_Z;
	gx=mpu6500_real_data.Gyro_X;
	gy=mpu6500_real_data.Gyro_Y;
	gz=mpu6500_real_data.Gyro_Z;
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1+ q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0*= recipNorm;
	q1*= recipNorm;
	q2*= recipNorm;
	q3*= recipNorm;
	
 Yaw_quat = atan2f(2.0f*(q0*q3+q1*q2), 2.0f*(q0*q0+q1*q1)-1.0f);
  Pitch_quat = asinf(-2.0f*(q1*q3-q0*q2));
  Roll_quat = atan2f(2.0f*(q0*q1+q2*q3),2.0f*(q0*q0+q3*q3)-1.0f);	

//	Yaw_quat = -atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1); 
//	Pitch_quat = -asin(-2*q1*q3 + 2*q0*q2);   
//	Roll_quat =  atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1);
	Yaw_watch_quat=Yaw_quat*57.3f;
//  Yaw=Yaw_quat;

}




float invSqrt(float num) {
	float halfnum = 0.5f * num;
	float y = num;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfnum * y * y));
	return y;
}
