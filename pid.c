

#include "pid.h"

#define PID_IMAX             (40)
#define PID_IMIN              (-40)

extern float deltat;

void pid_init(struct PID * pid, float pid_val[][3], float inpid_val[][3])
{	
	pid->err[0]		= 0.0;	
        pid->err[1]		= 0.0;	
	pid->err[2]		= 0.0;	

        pid->Angularerr[0]     = 0.0;
        pid->Angularerr[1]     = 0.0;
        pid->Angularerr[2]     = 0.0;

	pid->integral[0]	        = 0.0;
        pid->integral[1]	        = 0.0;
	pid->integral[2]	        = 0.0;

	pid->err_last[0]	        = 0.0;
        pid->err_last[1]	        = 0.0;
	pid->err_last[2]	        = 0.0;
        
        pid->inner_last[0]	= 0.0;
        pid->inner_last[1]	= 0.0;
	pid->inner_last[2]	= 0.0;

	pid->output[0]		= 0.0;
        pid->output[1]		= 0.0;
	pid->output[2]		= 0.0;
	
	pid->Kp[0]		= pid_val[0][0];//outer Roll PID
	pid->Ki[0]		        = pid_val[0][1];
	pid->Kd[0]		= pid_val[0][2];
        
        pid->Kp[1]		= pid_val[1][0];//outer Pitch PID
	pid->Ki[1]		        = pid_val[1][1];
	pid->Kd[1]		= pid_val[1][2];
        
        pid->Kp[2]		= pid_val[2][0];//outer Yaw PID
	pid->Ki[2]		        = pid_val[2][1];
	pid->Kd[2]		= pid_val[2][2];
        
        pid->iKp[0]		= inpid_val[0][0];//inner Roll PID
	pid->iKi[0]		= inpid_val[0][1];
	pid->iKd[0]		= inpid_val[0][2];
        
        pid->iKp[1]		= inpid_val[1][0];//inner Pitch PID
	pid->iKi[1]		= inpid_val[1][1];
	pid->iKd[1]		= inpid_val[1][2];
        
        pid->iKp[2]		= inpid_val[2][0];//inner Yaw PID
	pid->iKi[2]		= inpid_val[2][1];
	pid->iKd[2]		= inpid_val[2][2];
}
void pid_gain_update(struct PID * pid, float pid_val[][3], float inpid_val[][3])
{
        pid->Kp[0]		= pid_val[0][0];//outer Roll PID
	pid->Ki[0]		        = pid_val[0][1];
	pid->Kd[0]		= pid_val[0][2];
        
        pid->Kp[1]		= pid_val[1][0];//outer Pitch PID
	pid->Ki[1]		        = pid_val[1][1];
	pid->Kd[1]		= pid_val[1][2];
        
        pid->Kp[2]		= pid_val[2][0];//outer Yaw PID
	pid->Ki[2]		        = pid_val[2][1];
	pid->Kd[2]		= pid_val[2][2];
}

void __pid_update(__PID * pid, float * setting_angle, float * Euler_angle, float * angular_velocity)
{
  pid_update(pid, setting_angle[0], Euler_angle[0], angular_velocity[0], 0);
  pid_update(pid, setting_angle[1], Euler_angle[1], angular_velocity[1], 1);
  pid_update(pid, setting_angle[2], Euler_angle[2], angular_velocity[2], 2);
}
#if 0 //double loop PID 
void pid_update(__PID * pid, float set, float actual, float angular_velocity, int axis)
{  
  //set 목표각도
  //actual 현재각도
  //angular_velocity 현재 각속도
  float Kp_term, Ki_term, Kd_term;
  float D_err = 0.0f;
  angular_velocity = angular_velocity * deltat;
  switch (axis)
  {
  case 0: //roll
    {
        //Outter Loop PID (standard PID)
        pid->err[0] = set - actual; //오차 = 목표치 - 현재값	
      // if (pid->err[0] >  PID_IMAX)
      //   pid->err[0] = PID_IMAX;
      // if (pid->err[0] < PID_IMIN)
      //   pid->err[0] = PID_IMIN;
        Kp_term = pid->Kp[0] * pid->err[0]; //p항 = Kp * 오차
        
        pid->integral[0] += pid->err[0] * deltat;//오차적분 = 오차적분 + 오차 * dt
        Ki_term = pid->Ki[0] * pid->integral[0];//i항 = Ki * 오차적분
        
        D_err = (pid->err[0] - pid->err_last[0]) / deltat;//오차미분 = (현재오차-이전오차)/dt
        Kd_term = pid->Kd[0] * D_err;//d항 = Kd * 오차미분
	
	pid->output[0] = Kp_term + Ki_term + Kd_term;//제어량 = Kp항 + Ki항 + Kd항
	pid->err_last[0] = pid->err[0];//현재오차를 이전오차로.
        
        //Inner Loop PID (double loop PID)
        Kp_term = (pid->output[0] + angular_velocity) * pid->iKp[0];
        //Kd_term = (angular_velocity - pid->inner_last[0]) / deltat * pid->iKd[0];
        Kd_term = (angular_velocity - pid->inner_last[0]) * pid->iKd[0];
        pid->output[0] = Kp_term + Kd_term;       
        pid->inner_last[0] = angular_velocity;
        
      break;
    }
  case 1: //pitch
    {
        //Outter Loop PID (standard PID)
        pid->err[1] = set - actual; //오차 = 목표치 - 현재값	
       //if (pid->err[1] >  PID_IMAX)
       //  pid->err[1] = PID_IMAX;
       //if (pid->err[1] < PID_IMIN)
       //  pid->err[1] = PID_IMIN;
        Kp_term = pid->Kp[1] * pid->err[1]; //p항 = Kp * 오차
        
        pid->integral[1] += pid->err[1] * deltat;//오차적분 = 오차적분 + 오차 * dt
        Ki_term = pid->Ki[1] * pid->integral[1];//i항 = Ki * 오차적분
        
        D_err = (pid->err[1] - pid->err_last[1]) / deltat;//오차미분 = (현재오차-이전오차)/dt
        Kd_term = pid->Kd[1] * D_err;//d항 = Kd * 오차미분
	
	pid->output[1] = Kp_term + Ki_term + Kd_term;//제어량 = Kp항 + Ki항 + Kd항
	pid->err_last[1] = pid->err[1];//현재오차를 이전오차로.
        
        //Inner Loop PID (double loop PID)
        Kp_term = (pid->output[1] + angular_velocity) * pid->iKp[1];
        //Kd_term = (angular_velocity - pid->inner_last[1]) / deltat * pid->iKd[1];
        Kd_term = (angular_velocity - pid->inner_last[1]) * pid->iKd[1];
        pid->output[1] = Kp_term + Kd_term;       
        pid->inner_last[1] = angular_velocity;
        
      break;
    }
  case 2:  //yaw
    {
       //Outter Loop PID (standard PID)
        pid->err[2] = set - actual; //오차 = 목표치 - 현재값	
       // if (pid->err[2] >  PID_IMAX)
       //   pid->err[2] = PID_IMAX;
       // if (pid->err[2] < PID_IMIN)
       //   pid->err[2] = PID_IMIN;
        Kp_term = pid->Kp[2] * pid->err[2]; //p항 = Kp * 오차
        
        pid->integral[2] += pid->err[2] * deltat;//오차적분 = 오차적분 + 오차 * dt
        Ki_term = pid->Ki[2] * pid->integral[2];//i항 = Ki * 오차적분
        
        D_err = (pid->err[2] - pid->err_last[2]) / deltat;//오차미분 = (현재오차-이전오차)/dt
        Kd_term = pid->Kd[2] * D_err;//d항 = Kd * 오차미분
	
	pid->output[2] = Kp_term + Ki_term + Kd_term;//제어량 = Kp항 + Ki항 + Kd항
	pid->err_last[2] = pid->err[2];//현재오차를 이전오차로.
        
        //Inner Loop PID (double loop PID)
        Kp_term = (pid->output[2] + angular_velocity) * pid->iKp[2];
        //Kd_term = (angular_velocity - pid->inner_last[2]) / deltat * pid->iKd[2];
        Kd_term = (angular_velocity - pid->inner_last[2]) * pid->iKd[2];
        pid->output[2] = Kp_term + Kd_term;       
        pid->inner_last[2] = angular_velocity;
        
      break;
    }
  }
	
}
#else

void pid_update(__PID * pid, float set, float actual,float angular_velocity, int axis)
{  
  float Kp_term, Ki_term, Kd_term;
  float D_err = 0.0f;
  switch (axis)
  {
  case 0: //roll
    {
        pid->err[0] = set - actual; //오차 = 목표치 - 현재값	
        if (pid->err[0] >  PID_IMAX)
          pid->err[0] = PID_IMAX;
        if (pid->err[0] < PID_IMIN)
          pid->err[0] = PID_IMIN;
        Kp_term = pid->Kp[0] * pid->err[0]; //p항 = Kp * 오차
        
        pid->integral[0] += pid->err[0] * deltat;//오차적분 = 오차적분 + 오차 * dt
        Ki_term = pid->Ki[0] * pid->integral[0];//i항 = Ki * 오차적분
        
        D_err = (pid->err[0] - pid->err_last[0]) / deltat;//오차미분 = (현재오차-이전오차)/dt
        Kd_term = pid->Kd[0] * D_err;//d항 = Kd * 오차미분
	
	pid->output[0] = Kp_term + Ki_term + Kd_term;//제어량 = Kp항 + Ki항 + Kd항
	pid->err_last[0] = pid->err[0];//현재오차를 이전오차로.
      break;
    }
  case 1: //pitch
    {
        pid->err[1] = set - actual; //오차 = 목표치 - 현재값	
        if (pid->err[1] >  PID_IMAX)
          pid->err[1] = PID_IMAX;
        if (pid->err[1] < PID_IMIN)
          pid->err[1] = PID_IMIN;
        Kp_term = pid->Kp[1] * pid->err[1]; //p항 = Kp * 오차
        
        pid->integral[1] += pid->err[1] * deltat;//오차적분 = 오차적분 + 오차 * dt
        Ki_term = pid->Ki[1] * pid->integral[1];//i항 = Ki * 오차적분
        
        D_err = (pid->err[1] - pid->err_last[1]) / deltat;//오차미분 = (현재오차-이전오차)/dt
        Kd_term = pid->Kd[1] * D_err;//d항 = Kd * 오차미분
	
	pid->output[1] = Kp_term + Ki_term + Kd_term;//제어량 = Kp항 + Ki항 + Kd항
	pid->err_last[1] = pid->err[1];//현재오차를 이전오차로.
      break;
    }
  case 2:  //yaw
    {
        pid->err[2] = set - actual; //오차 = 목표치 - 현재값	
        
        if (pid->err[2] < -180)                 //correct angle jump ( ex: 180 -> -180)
          pid->err[2] = pid->err[2] + 360;
        else if (pid->err[2] > 180)
          pid->err[2] = pid->err[2] - 360;
        
        if (pid->err[2] >  PID_IMAX)
          pid->err[2] = PID_IMAX;
        if (pid->err[2] < PID_IMIN)
          pid->err[2] = PID_IMIN;
        Kp_term = pid->Kp[2] * pid->err[2]; //p항 = Kp * 오차
        
        pid->integral[2] += pid->err[2] * deltat;//오차적분 = 오차적분 + 오차 * dt
        Ki_term = pid->Ki[2] * pid->integral[2];//i항 = Ki * 오차적분
        
        D_err = (pid->err[2] - pid->err_last[2]) / deltat;//오차미분 = (현재오차-이전오차)/dt
        Kd_term = pid->Kd[2] * D_err;//d항 = Kd * 오차미분
	
	pid->output[2] = Kp_term + Ki_term + Kd_term;//제어량 = Kp항 + Ki항 + Kd항
	pid->err_last[2] = pid->err[2];//현재오차를 이전오차로.
      break;
    }
  }
	
}
#endif