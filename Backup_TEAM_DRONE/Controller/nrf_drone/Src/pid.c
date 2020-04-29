#include "pid.h"

#define PID_IMAX             (90.0f)
#define PID_IMIN              (-90.0f)

//extern float deltat;
//
//void pid_init(__PID * pid, float pid_val[][3], float inpid_val[][3])
//{	
//	pid->err[0]		= 0.0f;	
//        pid->err[1]		= 0.0f;	
//	pid->err[2]		= 0.0f;	
//
//        pid->rateError[0]     = 0.0f;
//        pid->rateError[1]     = 0.0f;
//        pid->rateError[2]     = 0.0f;
//
//	pid->integral[0]	        = 0.0f;
//        pid->integral[1]	        = 0.0f;
//	pid->integral[2]	        = 0.0f;
//
//	pid->err_last[0]	        = 0.0f;
//        pid->err_last[1]	        = 0.0f;
//	pid->err_last[2]	        = 0.0f;
//        
//        pid->preRateError[0]    = 0.0f;
//        pid->preRateError[1]    = 0.0f;
//        pid->preRateError[2]    = 0.0f;
//        
//	pid->output[0]		= 0.0f;
//        pid->output[1]		= 0.0f;
//	pid->output[2]		= 0.0f;
//	
//	pid->Kp[0]		= pid_val[0][0];//outer Roll PID
//	pid->Ki[0]		        = pid_val[0][1];
//	pid->Kd[0]		= pid_val[0][2];
//        
//        pid->Kp[1]		= pid_val[1][0];//outer Pitch PID
//	pid->Ki[1]		        = pid_val[1][1];
//	pid->Kd[1]		= pid_val[1][2];
//        
//        pid->Kp[2]		= pid_val[2][0];//outer Yaw PID
//	pid->Ki[2]		        = pid_val[2][1];
//	pid->Kd[2]		= pid_val[2][2];
//        
//        pid->iKp[0]		= inpid_val[0][0];//inner Roll PID
//	pid->iKi[0]		= inpid_val[0][1];
//	pid->iKd[0]		= inpid_val[0][2];
//        
//        pid->iKp[1]		= inpid_val[1][0];//inner Pitch PID
//	pid->iKi[1]		= inpid_val[1][1];
//	pid->iKd[1]		= inpid_val[1][2];
//        
//        pid->iKp[2]		= inpid_val[2][0];//inner Yaw PID
//	pid->iKi[2]		= inpid_val[2][1];
//	pid->iKd[2]		= inpid_val[2][2];
//}
//void pid_gain_update(__PID * pid, float pid_val[][3], float inpid_val[][3])
//{
//        pid->Kp[0]		= pid_val[0][0];//outer Roll PID
//	pid->Ki[0]		        = pid_val[0][1];
//	pid->Kd[0]		= pid_val[0][2];
//        
//        pid->Kp[1]		= pid_val[1][0];//outer Pitch PID
//	pid->Ki[1]		        = pid_val[1][1];
//	pid->Kd[1]		= pid_val[1][2];
//        
//        pid->Kp[2]		= pid_val[2][0];//outer Yaw PID
//	pid->Ki[2]		        = pid_val[2][1];
//	pid->Kd[2]		= pid_val[2][2];
//}
//
//void __pid_update(__PID * pid, float * setting_angle, float * Euler_angle, float * angular_velocity, float deltat)
//{
//  pid_update(pid, setting_angle[0], Euler_angle[0], angular_velocity[0], 0, deltat);
//  pid_update(pid, setting_angle[1], Euler_angle[1], angular_velocity[1], 1, deltat);
//  pid_update(pid, setting_angle[2], Euler_angle[2], angular_velocity[2], 2, deltat);
//}
//#if 1 //double loop PID 
//void pid_update(__PID * pid, float set, float actual, float angular_velocity, int axis, float deltat)
//{  
//  //set 목표각도
//  //actual 현재각도
//  //angular_velocity 현재 각속도
//  float Kp_term, Ki_term, Kd_term;
//  float D_err = 0.0f;
//  float stabilPgain = 0.0f;
//  float stabilIgain = 0.0f;   
//    
//  switch (axis)
//  {
//  case 0: //roll
//    {      
//        pid->err[0] = set - actual;                                             //오차 = 목표치 - 현재값	
//
//        if (pid->err[0] >  PID_IMAX)
//        pid->err[0] = PID_IMAX;
//        if (pid->err[0] < PID_IMIN)
//        pid->err[0] = PID_IMIN;
//        
//        stabilPgain = pid->Kp[0] * pid->err[0];                                          //p항 = Kp * 오차
//        stabilIgain += pid->Ki[0] *pid->err[0] * deltat;
//          
//        pid->rateError[0] = stabilPgain - angular_velocity;
//
//        Kp_term = pid->iKp[0] * pid->rateError[0];                                     //p항 = KpGain * rate오차.        
//
//        pid->integral[0] += (pid->rateError[0] * deltat);                              //오차적분 = 오차적분 + (오차rate * dt).        
//        Ki_term = pid->iKi[0] * pid->integral[0];                                         //i항 = Ki * 오차적분.        
//
//        D_err = (pid->rateError[0] - pid->preRateError[0]) / deltat;             //오차미분 = (현재rate오차-이전rate오차)/dt.        
//        Kd_term = pid->iKd[0] * D_err;                                                       //d항 = Kd * rate오차미분.	
//        
//        pid->output[0] = Kp_term + Ki_term + Kd_term + stabilIgain;                               //제어량 = Kp항 + Ki항 + Kd항
//
//        pid->preRateError[0] = pid->rateError[0];                                      //현재rate오차를 이전rate오차로.
//        
//      break;
//    }
//  case 1: //pitch
//    {
//        pid->err[1] = set - actual; //오차 = 목표치 - 현재값	
//
//        if (pid->err[1] >  PID_IMAX)
//        pid->err[1] = PID_IMAX;
//        if (pid->err[1] < PID_IMIN)
//        pid->err[1] = PID_IMIN;
//        
//        stabilPgain = pid->Kp[1] * pid->err[1];                                          //p항 = Kp * 오차
//        stabilIgain += pid->Ki[1] *pid->err[1] * deltat;
//
//        
//        pid->rateError[1] =  stabilPgain - angular_velocity;
//
//        Kp_term = pid->iKp[1] * pid->rateError[1];                                     //p항 = KpGain * rate오차.        
//
//        pid->integral[1] += (pid->rateError[1] * deltat);                              //오차적분 = 오차적분 + 오차rate * dt.        
//        Ki_term = pid->iKi[1] * pid->integral[1];                                         //i항 = Ki * 오차적분.        
//
//        D_err = (pid->rateError[1] - pid->preRateError[1]) / deltat;             //오차미분 = (현재rate오차-이전rate오차)/dt.        
//        Kd_term = pid->iKd[1] * D_err;                                                       //d항 = Kd * rate오차미분.	
//        
//        pid->output[1] = Kp_term + Ki_term + Kd_term + stabilIgain;                               //제어량 = Kp항 + Ki항 + Kd항
//
//        pid->preRateError[1] = pid->rateError[1];                                      //현재rate오차를 이전rate오차로.
//
//        
//      break;
//    }
//  case 2:  //yaw
//    {
//        pid->err[2] = set - actual; //오차 = 목표치 - 현재값	
//
//        if (pid->err[2] >  PID_IMAX)
//        pid->err[2] = PID_IMAX;
//        if (pid->err[2] < PID_IMIN)
//        pid->err[2] = PID_IMIN;
//        
//        stabilPgain = pid->Kp[2] * pid->err[2];                                           //p항 = Kp * 오차
//        stabilIgain += pid->Ki[2] *pid->err[2] * deltat;
//
//        pid->rateError[2] = stabilPgain - angular_velocity;
//
//        Kp_term = pid->iKp[2] * pid->rateError[2];                                     //p항 = KpGain * rate오차.        
//
//        pid->integral[2] += (pid->rateError[2] * deltat);                              //오차적분 = 오차적분 + 오차rate * dt.        
//        Ki_term = pid->iKi[2] * pid->integral[2];                                         //i항 = Ki * 오차적분.        
//
//        D_err = (pid->rateError[2] - pid->preRateError[2]) / deltat;             //오차미분 = (현재rate오차-이전rate오차)/dt.        
//        Kd_term = pid->iKd[2] * D_err;                                                       //d항 = Kd * rate오차미분.	
//        
//        pid->output[2] = Kp_term + Ki_term + Kd_term + stabilIgain;                               //제어량 = Kp항 + Ki항 + Kd항
//
//        pid->preRateError[2] = pid->rateError[2];                                      //현재rate오차를 이전rate오차로.
//
//      break;
//    }
//  }
//	
//}
//#else
//
//void pid_update(__PID * pid, float set, float actual,float angular_velocity, int axis, float deltat)
//{  
//  float Kp_term, Ki_term, Kd_term;
//  float D_err = 0.0f;
//  switch (axis)
//  {
//  case 0: //roll
//    {
//        pid->err[0] = set - actual; //오차 = 목표치 - 현재값	
//        if (pid->err[0] >  PID_IMAX)
//          pid->err[0] = PID_IMAX;
//        if (pid->err[0] < PID_IMIN)
//          pid->err[0] = PID_IMIN;
//        Kp_term = pid->Kp[0] * pid->err[0]; //p항 = Kp * 오차
//        
//        pid->integral[0] += pid->err[0] * deltat;//오차적분 = 오차적분 + 오차 * dt
//        Ki_term = pid->Ki[0] * pid->integral[0];//i항 = Ki * 오차적분
//        
//        D_err = (pid->err[0] - pid->err_last[0]) / deltat;//오차미분 = (현재오차-이전오차)/dt
//        Kd_term = pid->Kd[0] * D_err;//d항 = Kd * 오차미분
//	
//	pid->output[0] = Kp_term + Ki_term + Kd_term;//제어량 = Kp항 + Ki항 + Kd항
//	pid->err_last[0] = pid->err[0];//현재오차를 이전오차로.
//      break;
//    }
//  case 1: //pitch
//    {
//        pid->err[1] = set - actual; //오차 = 목표치 - 현재값	
//        if (pid->err[1] >  PID_IMAX)
//          pid->err[1] = PID_IMAX;
//        if (pid->err[1] < PID_IMIN)
//          pid->err[1] = PID_IMIN;
//        Kp_term = pid->Kp[1] * pid->err[1]; //p항 = Kp * 오차
//        
//        pid->integral[1] += pid->err[1] * deltat;//오차적분 = 오차적분 + 오차 * dt
//        Ki_term = pid->Ki[1] * pid->integral[1];//i항 = Ki * 오차적분
//        
//        D_err = (pid->err[1] - pid->err_last[1]) / deltat;//오차미분 = (현재오차-이전오차)/dt
//        Kd_term = pid->Kd[1] * D_err;//d항 = Kd * 오차미분
//	
//	pid->output[1] = Kp_term + Ki_term + Kd_term;//제어량 = Kp항 + Ki항 + Kd항
//	pid->err_last[1] = pid->err[1];//현재오차를 이전오차로.
//      break;
//    }
//  case 2:  //yaw
//    {
//        pid->err[2] = set - actual; //오차 = 목표치 - 현재값	
//        
//        if (pid->err[2] < -180)                 //correct angle jump ( ex: 180 -> -180)
//          pid->err[2] = pid->err[2] + 360;
//        else if (pid->err[2] > 180)
//          pid->err[2] = pid->err[2] - 360;
//        
//        if (pid->err[2] >  PID_IMAX)
//          pid->err[2] = PID_IMAX;
//        if (pid->err[2] < PID_IMIN)
//          pid->err[2] = PID_IMIN;
//        Kp_term = pid->Kp[2] * pid->err[2]; //p항 = Kp * 오차
//        
//        pid->integral[2] += pid->err[2] * deltat;//오차적분 = 오차적분 + 오차 * dt
//        Ki_term = pid->Ki[2] * pid->integral[2];//i항 = Ki * 오차적분
//        
//        D_err = (pid->err[2] - pid->err_last[2]) / deltat;//오차미분 = (현재오차-이전오차)/dt
//        Kd_term = pid->Kd[2] * D_err;//d항 = Kd * 오차미분
//	
//	pid->output[2] = Kp_term + Ki_term + Kd_term;//제어량 = Kp항 + Ki항 + Kd항
//	pid->err_last[2] = pid->err[2];//현재오차를 이전오차로.
//      break;
//    }
//  }	
//}
//#endif
//
//void Parsing_inPID_val(uint8_t* arr, float inpid_val[][3])
//{
//    char* iR_P, * iR_I, * iR_D, * iP_P, * iP_I, * iP_D, * iY_P, * iY_I, * iY_D;
//    iR_P = strtok((char*)arr, ",");
//    iR_I = strtok(NULL, ",");
//    iR_D = strtok(NULL,",");
//    iP_P = strtok(NULL,",");
//    iP_I = strtok(NULL,",");
//    iP_D = strtok(NULL,",");
//    iY_P = strtok(NULL,",");
//    iY_I = strtok(NULL,",");
//    iY_D = strtok(NULL,",");
//     
//    inpid_val[0][0] = atof(iR_P);
//    inpid_val[0][1] = atof(iR_I);
//    inpid_val[0][2] = atof(iR_D);
//    inpid_val[1][0] = atof(iP_P);
//    inpid_val[1][1] = atof(iP_I);
//    inpid_val[1][2] = atof(iP_D);
//    inpid_val[2][0] = atof(iY_P);
//    inpid_val[2][1] = atof(iY_I);
//    inpid_val[2][2] = atof(iY_D);
//    
//    
//}
//
//void Parsing_PID_val(uint8_t* arr, float pid_val[][3])
//{
//    char* R_P, * R_I, * R_D, * P_P, * P_I, * P_D, * Y_P, * Y_I, * Y_D;
//    R_P = strtok((char*)arr, ",");
//    R_I = strtok(NULL, ",");
//    R_D = strtok(NULL,",");
//    P_P = strtok(NULL,",");
//    P_I = strtok(NULL,",");
//    P_D = strtok(NULL,",");
//    Y_P = strtok(NULL,",");
//    Y_I = strtok(NULL,",");
//    Y_D = strtok(NULL,",");
//     
//    pid_val[0][0] = atof(R_P);
//    pid_val[0][1] = atof(R_I);
//    pid_val[0][2] = atof(R_D);
//    pid_val[1][0] = atof(P_P);
//    pid_val[1][1] = atof(P_I);
//    pid_val[1][2] = atof(P_D);
//    pid_val[2][0] = atof(Y_P);
//    pid_val[2][1] = atof(Y_I);
//    pid_val[2][2] = atof(Y_D);   
//}
//
//void Parsing_Throttle_val(uint8_t* arr, int *Controller_1)
//{
//    char* T;
//    T = strtok((char*)arr, ",");         
//    *Controller_1 = atoi(T);   
//}
//
//void Parsing_SettingPoint_val(uint8_t* arr, float* setting_angle)
//{
//    char* Setting_R, *Setting_P, *Setting_Y;
//    Setting_R = strtok((char*)arr, ",");
//    Setting_P = strtok(NULL, ",");
//    Setting_Y = strtok(NULL,",");    
//     
//    setting_angle[0] = atof(Setting_R);
//    setting_angle[1] = atof(Setting_P);
//    setting_angle[2] = atof(Setting_Y);   
//}
