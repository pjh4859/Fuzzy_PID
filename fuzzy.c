#include "fuzzy.h"

#define NB_err          -35.0f
#define NS_err          -15.0f
#define ZE_err          0.0f
#define PS_err          15.0f
#define PB_err          35.0f
#define DNB_derr        -20.0f
#define DNS_derr        -10.0f
#define DZE_derr        0.0f
#define DPS_derr        10.0f
#define DPB_derr        20.0f

_Fuzzy_stru Fuzzy_Matrix[5][5];

float NB, NS, ZE, PS, PB;
float DNB, DNS, DZE, DPS, DPB;
//float Kp, Ki, Kd, out, err_sum, error, d_err, prev_err;

_Fuzzy_stru Kp_small, Kp_medismall, Kp_medium, Kp_medilarge, Kp_large;
_Fuzzy_stru Ki_small, Ki_medismall, Ki_medium, Ki_medilarge, Ki_large;
_Fuzzy_stru Kd_small, Kd_medismall, Kd_medium, Kd_medilarge, Kd_large;

const float Kp_s = 1.2f, Kp_m = 2.5f, Kp_b = 3.8f, Kp_vb = 6.0f;
const float Ki_s = 0.003f, Ki_m = 0.005f, Ki_b = 0.007f, Ki_vb = 0.009f;
const float Kd_s = 0.003f, Kd_m = 0.005f, Kd_b = 0.007f, Kd_vb = 0.009f;

float Rule_Base[5][5][3] = {{{101.0f, 14.0f, 4.0f}, {101.0f, 14.0f, 4.0f}, {103.0f, 12.0f, 1.0f}, {104.0f, 14.0f, 1.0f}, {102.0f, 14.0f, 3.0f}},
                            {{101.0f, 14.0f, 4.0f}, {102.0f, 13.0f, 4.0f}, {104.0f, 11.0f, 2.0f}, {103.0f, 13.0f, 2.0f}, {102.0f, 14.0f, 4.0f}},
                            {{101.0f, 13.0f, 4.0f}, {102.0f, 12.0f, 3.0f}, {104.0f, 11.0f, 2.0f}, {102.0f, 12.0f, 3.0f}, {101.0f, 13.0f, 4.0f}},
                            {{102.0f, 14.0f, 4.0f}, {103.0f, 13.0f, 2.0f}, {104.0f, 11.0f, 2.0f}, {102.0f, 13.0f, 4.0f}, {101.0f, 14.0f, 4.0f}},
                            {{102.0f, 14.0f, 3.0f}, {104.0f, 14.0f, 1.0f}, {103.0f, 12.0f, 1.0f}, {101.0f, 14.0f, 4.0f}, {101.0f, 14.0f, 4.0f}}};


void fuzzy_init() {
    int i, j, k;
    float arbi;
    NB = 0.0f; NS = 0.0f; ZE = 0.0f; PS = 0.0f; PB = 0.0f;
    DNB = 0.0f; DNS = 0.0f; DZE = 0.0f; DPS = 0.0f; DPB = 0.0f; 

    for (i= 0; i<5; i++)
    {
      for (j=0;j<5;j++)
      {
        for(k=0;k<3;k++)
        {
          arbi = Rule_Base[i][j][k];
          if (arbi>100.0f)
          {
            if (arbi == 101.0f) Rule_Base[i][j][k] = Kp_vb;
            else if (arbi == 102.0f) Rule_Base[i][j][k] = Kp_b;
            else if (arbi == 103.0f) Rule_Base[i][j][k] = Kp_m;
            else if (arbi == 104.0f) Rule_Base[i][j][k] = Kp_s;
          }
          else if (arbi > 10.0f && arbi < 100.0f)
          {
            if (arbi ==11.0f) Rule_Base[i][j][k] = Ki_vb;
            else if (arbi == 12.0f) Rule_Base[i][j][k] = Ki_b;
            else if (arbi == 13.0f) Rule_Base[i][j][k] = Ki_m;
            else if (arbi == 14.0f) Rule_Base[i][j][k] = Ki_s;
          }
          else 
          {
            if (arbi ==1.0f) Rule_Base[i][j][k] = Kd_vb;
            else if (arbi == 2.0f) Rule_Base[i][j][k] = Kd_b;
            else if (arbi == 3.0f) Rule_Base[i][j][k] = Kd_m;
            else if (arbi == 4.0f) Rule_Base[i][j][k] = Kd_s;
          }
        }
      }
    }
    for (i = 0; i < 5; i++) {
        for (j = 0; j < 5; j++) 
        {
          Fuzzy_Matrix[i][j].Fuzzy_Val = 0.0f;
          Fuzzy_Matrix[i][j].index = i + j + 1;
          Fuzzy_Matrix[i][j].Selected_PID[0]=Rule_Base[i][j][0];
          Fuzzy_Matrix[i][j].Selected_PID[1]=Rule_Base[i][j][1];
          Fuzzy_Matrix[i][j].Selected_PID[2]=Rule_Base[i][j][2];
        }
    }
}

//fuzzification
void Fuzzification(float setting_angle, float Euler_angle, float *prev_err)
{
  float error = setting_angle - Euler_angle;
  float d_err = error - *prev_err;
  //float d_err = 5.0f;

    if (error <= NB_err)
    {
        NB = 1.0f;
        NS = 0.0f;
        ZE = 0.0f;
        PS = 0.0f;
        PB = 0.0f;
    }
    else if (error > NB_err && error <= NS_err)
    {
        NB = (1.0f / ((NS_err) - (NB_err))) * ((-error) + (NS_err));
        NS = (1.0f / ((NS_err) - (NB_err))) * (error - (NB_err));
        ZE = 0.0f;
        PS = 0.0f;
        PB = 0.0f;
    }
    else if (error > NS_err && error <= ZE_err)
    {
        NB = 0.0f;
        NS = (1.0f / (ZE_err - (NS_err))) * ((-error) + (ZE_err));
        ZE = (1.0f / (ZE_err - (NS_err))) * (error - (NS_err));
        PS = 0.0f;
        PB = 0.0f;
    }
    else if (error > ZE_err && error <= PS_err)
    {
        NB = 0.0f;
        NS = 0.0f;
        ZE = (1.0f / (PS_err - ZE_err)) * ((-error) + (PS_err));
        PS = (1.0f / (PS_err - ZE_err)) * (error - (ZE_err));
        PB = 0.0f;
    }
    else if (error > PS_err && error <= PB_err)
    {
        NB = 0.0f;
        NS = 0.0f;
        ZE = 0.0f;
        PS = (1.0f / (PB_err - (PS_err))) * ((-error) + (PB_err));
        PB = (1.0f / ((PB_err) - (PS_err))) * (error - (PS_err));
    }
    else
    {
        NB = 0.0f;
        NS = 0.0f;
        ZE = 0.0f;
        PS = 0.0f;
        PB = 1.0f;
    }
//---------d_err-----------------------------------------------
    if (d_err <= DNB_derr)
    {
        DNB = 1.0f;
        DNS = 0.0f;
        DZE = 0.0f;
        DPS = 0.0f;
        DPB = 0.0f;
    }
    else if (d_err > DNB_derr && d_err <= DNS_derr)
    {
        DNB = (1.0f / ((DNS_derr) - (DNB_derr))) * ((-d_err) + (DNS_derr));
        DNS = (1.0f / ((DNS_derr) - (DNB_derr))) * (d_err - (DNB_derr));
        DZE = 0.0f;
        DPS = 0.0f;
        DPB = 0.0f;
    }
    else if (d_err > DNS_derr && d_err <= DZE_derr)
    {
        DNB = 0.0f;
        DNS = (1.0f / (DZE_derr - (DNS_derr))) * ((-d_err) + (DZE_derr));
        DZE = (1.0f / (DZE_derr - (DNS_derr))) * (d_err - (DNS_derr));
        DPS = 0.0f;
        DPB = 0.0f;
    }
    else if (d_err > DZE_derr && d_err <= DPS_derr)
    {
        DNB = 0.0f;
        DNS = 0.0f;
        DZE = (1.0f / (DPS_derr - DZE_derr)) * ((-d_err) + (DPS_derr));
        DPS = (1.0f / (DPS_derr - DZE_derr)) * (d_err - (DZE_derr));
        DPB = 0.0f;
    }
    else if (d_err > DPS_derr && d_err <= DPB_derr)
    {
        DNB = 0.0f;
        DNS = 0.0f;
        DZE = 0.0f;
        DPS = (1.0f / (DPB_derr - (DPS_derr))) * ((-d_err) + (DPB_derr));
        DPB = (1.0f / ((DPB_derr) - (DPS_derr))) * (d_err - (DPS_derr));
    }
    else
    {
        DNB = 0.0f;
        DNS = 0.0f;
        DZE = 0.0f;
        DPS = 0.0f;
        DPB = 1.0f;
    }    
    *prev_err = error;
}

//creation of the fuzzy matrix
void Create_Fuzzy_Matrix()
{
    Fuzzy_Matrix[0][0].Fuzzy_Val = min(NB, DNB);
    Fuzzy_Matrix[0][1].Fuzzy_Val = min(NB, DNS);
    Fuzzy_Matrix[0][2].Fuzzy_Val = min(NB, DZE);
    Fuzzy_Matrix[0][3].Fuzzy_Val = min(NB, DPS);
    Fuzzy_Matrix[0][4].Fuzzy_Val = min(NB, DPB);

    Fuzzy_Matrix[1][0].Fuzzy_Val = min(NS, DNB);
    Fuzzy_Matrix[1][1].Fuzzy_Val = min(NS, DNS);
    Fuzzy_Matrix[1][2].Fuzzy_Val = min(NS, DZE);
    Fuzzy_Matrix[1][3].Fuzzy_Val = min(NS, DPS);
    Fuzzy_Matrix[1][4].Fuzzy_Val = min(NS, DPB);

    Fuzzy_Matrix[2][0].Fuzzy_Val = min(ZE, DNB);
    Fuzzy_Matrix[2][1].Fuzzy_Val = min(ZE, DNS);
    Fuzzy_Matrix[2][2].Fuzzy_Val = min(ZE, DZE);
    Fuzzy_Matrix[2][3].Fuzzy_Val = min(ZE, DPS);
    Fuzzy_Matrix[2][4].Fuzzy_Val = min(ZE, DPB);

    Fuzzy_Matrix[3][0].Fuzzy_Val = min(PS, DNB);
    Fuzzy_Matrix[3][1].Fuzzy_Val = min(PS, DNS);
    Fuzzy_Matrix[3][2].Fuzzy_Val = min(PS, DZE);
    Fuzzy_Matrix[3][3].Fuzzy_Val = min(PS, DPS);
    Fuzzy_Matrix[3][4].Fuzzy_Val = min(PS, DPB);

    Fuzzy_Matrix[4][0].Fuzzy_Val = min(PB, DNB);
    Fuzzy_Matrix[4][1].Fuzzy_Val = min(PB, DNS);
    Fuzzy_Matrix[4][2].Fuzzy_Val = min(PB, DZE);
    Fuzzy_Matrix[4][3].Fuzzy_Val = min(PB, DPS);
    Fuzzy_Matrix[4][4].Fuzzy_Val = min(PB, DPB);
}

//This part h=gives us the physical values for the coefficients
void Defuzzification(float *Kp, float *Ki, float *Kd) {
  _Fuzzy_stru Kp_small_flagarr[4] = {Fuzzy_Matrix[0][0], Fuzzy_Matrix[0][1], Fuzzy_Matrix[1][0], Fuzzy_Matrix[1][1]};
  _Fuzzy_stru Kp_medismall_flagarr[6] = {Fuzzy_Matrix[0][2], Fuzzy_Matrix[0][3], Fuzzy_Matrix[1][2], Fuzzy_Matrix[2][0], Fuzzy_Matrix[2][1], Fuzzy_Matrix[3][0]};
  _Fuzzy_stru Kp_medium_flagarr[5] = {Fuzzy_Matrix[2][2], Fuzzy_Matrix[1][3], Fuzzy_Matrix[3][1], Fuzzy_Matrix[0][4], Fuzzy_Matrix[4][0]};
  _Fuzzy_stru Kp_medilarge_flagarr[6] = {Fuzzy_Matrix[4][2], Fuzzy_Matrix[4][1], Fuzzy_Matrix[3][2], Fuzzy_Matrix[2][4], Fuzzy_Matrix[2][3], Fuzzy_Matrix[1][4]};
  _Fuzzy_stru Kp_large_flagarr[4] = {Fuzzy_Matrix[4][4], Fuzzy_Matrix[4][3], Fuzzy_Matrix[3][4], Fuzzy_Matrix[3][3]};
     
  Find_Maxarr(Kp_small_flagarr, 4, 1); //Find_Maxarr(array name, number of array element, K_value sequence).
  Find_Maxarr(Kp_medismall_flagarr, 6, 2);
  Find_Maxarr(Kp_medium_flagarr, 5, 3);
  Find_Maxarr(Kp_medilarge_flagarr, 6, 4);
  Find_Maxarr(Kp_large_flagarr, 4, 5);
   

  if (!(Kp_small.Fuzzy_Val == 0 && Kp_medismall.Fuzzy_Val == 0 && Kp_medium.Fuzzy_Val == 0 && Kp_medilarge.Fuzzy_Val == 0 && Kp_large.Fuzzy_Val == 0))
  {
    *Kp = (Kp_small.Selected_PID[0] * Kp_small.Fuzzy_Val + Kp_medismall.Selected_PID[0] * Kp_medismall.Fuzzy_Val + Kp_medium.Selected_PID[0] * Kp_medium.Fuzzy_Val \
        + Kp_medilarge.Selected_PID[0] * Kp_medilarge.Fuzzy_Val + Kp_large.Selected_PID[0] * Kp_large.Fuzzy_Val) \
       / (Kp_small.Fuzzy_Val + Kp_medismall.Fuzzy_Val + Kp_medium.Fuzzy_Val + Kp_medilarge.Fuzzy_Val + Kp_large.Fuzzy_Val);

    *Ki = (Kp_small.Selected_PID[1] * Kp_small.Fuzzy_Val + Kp_medismall.Selected_PID[1] * Kp_medismall.Fuzzy_Val + Kp_medium.Selected_PID[1] * Kp_medium.Fuzzy_Val \
    + Kp_medilarge.Selected_PID[1] * Kp_medilarge.Fuzzy_Val + Kp_large.Selected_PID[1] * Kp_large.Fuzzy_Val) \
    / (Kp_small.Fuzzy_Val + Kp_medismall.Fuzzy_Val + Kp_medium.Fuzzy_Val + Kp_medilarge.Fuzzy_Val + Kp_large.Fuzzy_Val);

    *Kd = (Kp_small.Selected_PID[2] * Kp_small.Fuzzy_Val + Kp_medismall.Selected_PID[2] * Kp_medismall.Fuzzy_Val + Kp_medium.Selected_PID[2] * Kp_medium.Fuzzy_Val \
        + Kp_medilarge.Selected_PID[2] * Kp_medilarge.Fuzzy_Val + Kp_large.Selected_PID[2] * Kp_large.Fuzzy_Val) \
       / (Kp_small.Fuzzy_Val + Kp_medismall.Fuzzy_Val + Kp_medium.Fuzzy_Val + Kp_medilarge.Fuzzy_Val + Kp_large.Fuzzy_Val);
  }  
}

void Find_Maxarr(_Fuzzy_stru flagarr[], int flag, int flag2)
{
  int max_flag;
  _Fuzzy_stru arbi_K;
  max_flag = 0;
  arbi_K = flagarr[0];
  for (int i=0;i<flag-1;i++)
  {
    if (flagarr[max_flag].Fuzzy_Val < flagarr[i+1].Fuzzy_Val)
    {
      max_flag = i+1;      
    }    
  }  
  arbi_K = flagarr[max_flag];
  switch(flag2)
  {
    case 1:
    {
      Kp_small = arbi_K;
      break;
    }
    case 2:
    {
      Kp_medismall = arbi_K;
      break;
    }
    case 3:
    {
      Kp_medium = arbi_K;
      break;
    }
    case 4:
    {
      Kp_medilarge = arbi_K;
      break;
    }
    case 5:
    {
      Kp_large = arbi_K;
      break;
    }
    case 6:
    {
      Ki_small = arbi_K;
      break;
    }
    case 7:
    {
      Ki_medismall = arbi_K;
      break;
    }
    case 8:
    {
      Ki_medium = arbi_K;
      break;
    }
    case 9:
    {
      Ki_medilarge = arbi_K;
      break;
    }
    case 10:
    {
      Ki_large = arbi_K;
      break;
    }
    case 11:
    {
      Kd_small = arbi_K;
      break;
    }
    case 12:
    {
      Kd_medismall = arbi_K;
      break;
    }
    case 13:
    {
      Kd_medium = arbi_K;
      break;
    }
    case 14:
    {
      Kd_medilarge = arbi_K;
      break;
    }
    case 15:
    {
      Kd_large = arbi_K;
      break;
    }
  }
}