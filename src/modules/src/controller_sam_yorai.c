//
// Created by chris on 7/29/21.
//

#include <math.h>
#include <stdlib.h>
#include "param.h"
#include "log.h"
#include "math3d.h"
#include "controller_sam_yorai.h"
#include "debug.h"
#include "attitude_controller.h"


#define ROWS 4
#define COLUMNS 4


<<<<<<< HEAD
static double_t horizon = 0.15;   //was 0.7
=======
static double_t horizon = 0.2;   //was 0.7
>>>>>>> 1cbf950d8ce41908d4595a2b29cc90c7cea459df
static float alpha[4][4] = {
                        {20, 0,  0, 0},
                        {0, 50, 0, 0},
                        {0, 0, 50, 0},
                        {0, 0, 0, 50} };
static double_t init_input[4] = {0, 0, 0, 0};
static double g = 9.81;
static double m = 35.89 / 1000;

static float massThrust = 132000;


static double_t time = 0;

setpoint_t desired_wb;

typedef struct {
    double m[4][4];
} m_4d;

m_4d past_mat;


void controllerSamYoraiReset(void){
    //static bool REACHED_SETPOINT = false;
}

void controllerSamYoraiInit(void){
    controllerSamYoraiReset();
    attitudeControllerInit((float)1.0/ATTITUDE_RATE);
}

bool controllerSamYoraiTest(void)
{
<<<<<<< HEAD
    return true;
}


static struct mat33 matinv_3d(struct mat33 mat) {
    float a = mat.m[0][0];
    float b = mat.m[0][1];
    float c = mat.m[0][2];
    float d = mat.m[1][0];
    float e = mat.m[1][1];
    float f = mat.m[1][2];
    float gg = mat.m[2][0];
    float h = mat.m[2][1];
    float i = mat.m[2][2];

    float A = (e*i-f*h);
    float B = -1*(d*i- f * gg);
    float C = (d*h- e * gg);
    float D = -1*(b*i-c*h);
    float E = (a*i- c * gg);
    float F = -1*(a*h- b * gg);
    float G = (b*f-c*e);
    float H = -1*(a*f-c*d);
    float I = (a*e-b*d);

    float A_det = a*A + b*B + c*C;

    /*float m_adj[3][3] = {{A, D, G},
                         {B, E, H},
                         {C, F, I}};
        */

    struct mat33 m_inv;
    m_inv.m[0][0] = (1/A_det)*A;
    m_inv.m[0][1] =  (1/A_det)*D;
    m_inv.m[0][2] = (1/A_det)*G;

    m_inv.m[1][0] = (1/A_det)*B;
    m_inv.m[1][1] =  (1/A_det)*E;
    m_inv.m[1][2] = (1/A_det)*H;

    m_inv.m[2][0] = (1/A_det)*C;
    m_inv.m[2][1] =  (1/A_det)*F;
    m_inv.m[2][2] = (1/A_det)*I;

    return m_inv;
}


m_4d matinv_4d(float matrix_in[ROWS][COLUMNS]){
=======
    bool pass = true;

    pass &= attitudeControllerTest();

    return pass;
}


//static struct mat33 matinv_3d(struct mat33 mat) {
//    float a = mat.m[0][0];
//    float b = mat.m[0][1];
//    float c = mat.m[0][2];
//    float d = mat.m[1][0];
//    float e = mat.m[1][1];
//    float f = mat.m[1][2];
//    float gg = mat.m[2][0];
//    float h = mat.m[2][1];
//    float i = mat.m[2][2];
//
//    float A = (e*i-f*h);
//    float B = -1*(d*i- f * gg);
//    float C = (d*h- e * gg);
//    float D = -1*(b*i-c*h);
//    float E = (a*i- c * gg);
//    float F = -1*(a*h- b * gg);
//    float G = (b*f-c*e);
//    float H = -1*(a*f-c*d);
//    float I = (a*e-b*d);
//
//    float A_det = a*A + b*B + c*C;
//
//    /*float m_adj[3][3] = {{A, D, G},
//                         {B, E, H},
//                         {C, F, I}};
//        */
//
//    struct mat33 m_inv;
//    m_inv.m[0][0] = (1/A_det)*A;
//    m_inv.m[0][1] =  (1/A_det)*D;
//    m_inv.m[0][2] = (1/A_det)*G;
//
//    m_inv.m[1][0] = (1/A_det)*B;
//    m_inv.m[1][1] =  (1/A_det)*E;
//    m_inv.m[1][2] = (1/A_det)*H;
//
//    m_inv.m[2][0] = (1/A_det)*C;
//    m_inv.m[2][1] =  (1/A_det)*F;
//    m_inv.m[2][2] = (1/A_det)*I;
//
//    return m_inv;
//}


m_4d matinv_4d(double_t matrix_in[ROWS][COLUMNS]){
>>>>>>> 1cbf950d8ce41908d4595a2b29cc90c7cea459df

    //create mat_inv array of pointers
    m_4d mat_inv;

<<<<<<< HEAD
    float a = matrix_in[0][0];
    float b = matrix_in[0][1];
    float c = matrix_in[0][2];
    float d = matrix_in[0][3];

    float e = matrix_in[1][0];
    float f = matrix_in[1][1];
    float gg = matrix_in[1][2];
    float h = matrix_in[1][3];

    float i = matrix_in[2][0];
    float j = matrix_in[2][1];
    float k = matrix_in[2][2];
    float l = matrix_in[2][3];

    float mm = matrix_in[3][0];
    float n = matrix_in[3][1];
    float o = matrix_in[3][2];
    float p = matrix_in[3][3];
=======
    double_t a = matrix_in[0][0];
    double_t b = matrix_in[0][1];
    double_t c = matrix_in[0][2];
    double_t d = matrix_in[0][3];

    double_t e = matrix_in[1][0];
    double_t f = matrix_in[1][1];
    double_t gg = matrix_in[1][2];
    double_t h = matrix_in[1][3];

    double_t i = matrix_in[2][0];
    double_t j = matrix_in[2][1];
    double_t k = matrix_in[2][2];
    double_t l = matrix_in[2][3];

    double_t mm = matrix_in[3][0];
    double_t n = matrix_in[3][1];
    double_t o = matrix_in[3][2];
    double_t p = matrix_in[3][3];
>>>>>>> 1cbf950d8ce41908d4595a2b29cc90c7cea459df


    //cofactors

    //2x2 cofactors A
<<<<<<< HEAD
    float A1 = (float)pow(-1,1+1)*(k*p-l*o);
    float A2 = (float)pow(-1, 1+2)*(j*p - l*n);
    float A3 = (float)pow(-1, 1+3)*(o*j - n*k);

    float A =(float)pow(-1, 1+1) * (f*(A1) + gg*(A2) + h * (A3));

    //2x2 cofactors B
    float B1 = (float)pow(-1, 1+1) * (k*p - l*o);
    float B2 = (float)pow(-1, 1+2) * (i*p - mm*l);
    float B3 = (float)pow(-1, 1+3) * (i*o - mm*k);

    float B = (float)pow(-1, 1+2) * (e*(B1) + gg*(B2) + h*(B3));

    //2x2 cofactors C
    float C1 = (float)pow(-1, 1+1) * (o*j - l*n);
    float C2 = (float)pow(-1, 1+2) * (i*o - mm*l);
    float C3 = (float)pow(-1, 1+3) * (i*n - j*mm);

    float C = (float)pow(-1, 1+3) * (e*(C1) + f*(C2) + h*(C3));

    //2x2 cofactors D
    float D1 = (float)pow(-1, 1+1) * (j*o - n*k);
    float D2 = (float)pow(-1, 1+2) * (i*o - mm*k);
    float D3 = (float)pow(-1, 1+3) * (i*n - j*mm);

    float D = (float)pow(-1, 1+4) * (e*(D1) + f*(D2) + gg*(D3));


    //2x2 cofactors E
    float E1 = (float)pow(-1, 1+1) * (k*p - l*o);
    float E2 = (float)pow(-1, 1+2) * (j*p - n*l);
    float E3 = (float)pow(-1, 1+3) * (j*o - n*k);

    float E = (float)pow(-1, 2+1) * (b*(E1) + c*(E2) + d*(E3));

    //2x2 cofactors F
    float F1 = (float)pow(-1, 1+1) * (k*p - l*o);
    float F2 = (float)pow(-1, 1+2) * (i*p - mm*l);
    float F3 = (float)pow(-1, 1+3) * (i*o - mm*k);

    float F =  (float)pow(-1, 2+2) * (a*(F1) + c*(F2) + d*(F3));

    //2x2 cofactors G
    float G1 = (float)pow(-1, 1+1) * (j*p - l*n);
    float G2 = (float)pow(-1, 1+2) * (i*p - mm*l);
    float G3 = (float)pow(-1, 1+3) * (i*n - mm*j);

    float G =  (float)pow(-1, 2+3) * (a*(G1) + b*(G2) + d*(G3));

    //2x2 cofactors H
    float H1 = (float)pow(-1, 1+1) * (o*j - n*k);
    float H2 = (float)pow(-1, 1+2) * (i*o - mm*k);
    float H3 = (float)pow(-1, 1+3) * (i*n - mm*j);

    float H =  (float)pow(-1, 2+4) * (a*(H1) + b*(H2) + c*(H3));

    //2x2 cofactors I
    float I1 = (float)pow(-1, 1+1) * (gg*p - o*h);
    float I2 = (float)pow(-1, 1+2) * (f*p - n*h);
    float I3 = (float)pow(-1, 1+3) * (f*o - n*gg);

    float I =  (float)pow(-1, 3+1) * (b*(I1) + c*(I2) + d*(I3));

    //2x2 cofactors J
    float J1 = (float)pow(-1, 1+1) * (gg*p - o*h);
    float J2 = (float)pow(-1, 1+2) * (e*p - mm*h);
    float J3 = (float)pow(-1, 1+3) * (e*o - mm*gg);

    float J =  (float)pow(-1, 3+2) * (a*(J1) + c*(J2) + d*(J3));

    //2x2 cofactors K
    float K1 = (float)pow(-1, 1+1) * (f*p - n*h);
    float K2 = (float)pow(-1, 1+2) * (e*p - mm*h);
    float K3 = (float)pow(-1, 1+3) * (e*n - mm*f);

    float K =  (float)pow(-1, 3+3) * (a*(K1) + b*(K2) + d*(K3));

    //2x2 cofactors L
    float L1 = (float)pow(-1, 1+1) * (f*o - n*gg);
    float L2 = (float)pow(-1, 1+2) * (e*o - mm*gg);
    float L3 = (float)pow(-1, 1+3) * (e*n - mm*f);

    float L =  (float)pow(-1, 3+4) * (a*(L1) + b*(L2) + c*(L3));


    //2x2 cofactors M
    float M1 = (float)pow(-1, 1+1) * (gg*l - k*h);
    float M2 = (float)pow(-1, 1+2) * (f*l - j*h);
    float M3 = (float)pow(-1, 1+3) * (f*k - j*gg);

    float M = (float)pow(-1, 4+1) * (b*(M1) + c*(M2) + d*(M3));

    //2x2 cofactors N
    float N1 = (float)pow(-1, 1+1) * (gg*l - k*h);
    float N2 = (float)pow(-1, 1+2) * (e*l - i*h);
    float N3 = (float)pow(-1, 1+3) * (e*k -i*gg);

    float N =  (float)pow(-1, 4+2) * (a*(N1) + c*(N2) + d*(N3));

    //2x2 cofactors O
    float O1 = (float)pow(-1, 1+1) * (f*l - j*h);
    float O2 = (float)pow(-1, 1+2) * (e*l - i*h);
    float O3 = (float)pow(-1, 1+3) * (e*j - i*f);

    float O =  (float)pow(-1, 4+3) * (a*(O1) + b*(O2) + d*(O3));

    //2x2 cofactors P
    float P1 = (float)pow(-1, 1+1) * (f*k - gg*j);
    float P2 = (float)pow(-1, 1+2) * (e*k - i*gg);
    float P3 = (float)pow(-1, 1+3) * (e*j - i*f);

    float P = (float)pow(-1, 4+4) * (a*(P1) + b*(P2) + c*(P3));


    //determinant
    float A_det = a*A + b*B + c*C + d*D;
=======
    double_t A1 = pow(-1,1+1)*(k*p-l*o);
    double_t A2 = pow(-1, 1+2)*(j*p - l*n);
    double_t A3 = pow(-1, 1+3)*(o*j - n*k);

    double_t A = pow(-1, 1+1) * (f*(A1) + gg*(A2) + h * (A3));

    //2x2 cofactors B
    double_t B1 = pow(-1, 1+1) * (k*p - l*o);
    double_t B2 = pow(-1, 1+2) * (i*p - mm*l);
    double_t B3 = pow(-1, 1+3) * (i*o - mm*k);

    double_t B = pow(-1, 1+2) * (e*(B1) + gg*(B2) + h*(B3));

    //2x2 cofactors C
    double_t C1 = pow(-1, 1+1) * (o*j - l*n);
    double_t C2 = pow(-1, 1+2) * (i*o - mm*l);
    double_t C3 = pow(-1, 1+3) * (i*n - j*mm);

    double_t C = pow(-1, 1+3) * (e*(C1) + f*(C2) + h*(C3));

    //2x2 cofactors D
    double_t D1 = pow(-1, 1+1) * (j*o - n*k);
    double_t D2 = pow(-1, 1+2) * (i*o - mm*k);
    double_t D3 = pow(-1, 1+3) * (i*n - j*mm);

    double_t D = pow(-1, 1+4) * (e*(D1) + f*(D2) + gg*(D3));


    //2x2 cofactors E
    double_t E1 = pow(-1, 1+1) * (k*p - l*o);
    double_t E2 = pow(-1, 1+2) * (j*p - n*l);
    double_t E3 = pow(-1, 1+3) * (j*o - n*k);

    double_t E = pow(-1, 2+1) * (b*(E1) + c*(E2) + d*(E3));

    //2x2 cofactors F
    double_t F1 = pow(-1, 1+1) * (k*p - l*o);
    double_t F2 = pow(-1, 1+2) * (i*p - mm*l);
    double_t F3 = pow(-1, 1+3) * (i*o - mm*k);

    double_t F =  pow(-1, 2+2) * (a*(F1) + c*(F2) + d*(F3));

    //2x2 cofactors G
    double_t G1 = pow(-1, 1+1) * (j*p - l*n);
    double_t G2 = pow(-1, 1+2) * (i*p - mm*l);
    double_t G3 = pow(-1, 1+3) * (i*n - mm*j);

    double_t G =  pow(-1, 2+3) * (a*(G1) + b*(G2) + d*(G3));

    //2x2 cofactors H
    double_t H1 = pow(-1, 1+1) * (o*j - n*k);
    double_t H2 = pow(-1, 1+2) * (i*o - mm*k);
    double_t H3 = pow(-1, 1+3) * (i*n - mm*j);

    double_t H =  pow(-1, 2+4) * (a*(H1) + b*(H2) + c*(H3));

    //2x2 cofactors I
    double_t I1 = pow(-1, 1+1) * (gg*p - o*h);
    double_t I2 = pow(-1, 1+2) * (f*p - n*h);
    double_t I3 = pow(-1, 1+3) * (f*o - n*gg);

    double_t I =  pow(-1, 3+1) * (b*(I1) + c*(I2) + d*(I3));

    //2x2 cofactors J
    double_t J1 = pow(-1, 1+1) * (gg*p - o*h);
    double_t J2 = pow(-1, 1+2) * (e*p - mm*h);
    double_t J3 = pow(-1, 1+3) * (e*o - mm*gg);

    double_t J =  pow(-1, 3+2) * (a*(J1) + c*(J2) + d*(J3));

    //2x2 cofactors K
    double_t K1 = pow(-1, 1+1) * (f*p - n*h);
    double_t K2 = pow(-1, 1+2) * (e*p - mm*h);
    double_t K3 = pow(-1, 1+3) * (e*n - mm*f);

    double_t K =  pow(-1, 3+3) * (a*(K1) + b*(K2) + d*(K3));

    //2x2 cofactors L
    double_t L1 = pow(-1, 1+1) * (f*o - n*gg);
    double_t L2 = pow(-1, 1+2) * (e*o - mm*gg);
    double_t L3 = pow(-1, 1+3) * (e*n - mm*f);

    double_t L =  pow(-1, 3+4) * (a*(L1) + b*(L2) + c*(L3));


    //2x2 cofactors M
    double_t M1 = pow(-1, 1+1) * (gg*l - k*h);
    double_t M2 = pow(-1, 1+2) * (f*l - j*h);
    double_t M3 = pow(-1, 1+3) * (f*k - j*gg);

    double_t M = pow(-1, 4+1) * (b*(M1) + c*(M2) + d*(M3));

    //2x2 cofactors N
    double_t N1 = pow(-1, 1+1) * (gg*l - k*h);
    double_t N2 = pow(-1, 1+2) * (e*l - i*h);
    double_t N3 = pow(-1, 1+3) * (e*k -i*gg);

    double_t N =  pow(-1, 4+2) * (a*(N1) + c*(N2) + d*(N3));

    //2x2 cofactors O
    double_t O1 = pow(-1, 1+1) * (f*l - j*h);
    double_t O2 = pow(-1, 1+2) * (e*l - i*h);
    double_t O3 = pow(-1, 1+3) * (e*j - i*f);

    double_t O =  pow(-1, 4+3) * (a*(O1) + b*(O2) + d*(O3));

    //2x2 cofactors P
    double_t P1 = pow(-1, 1+1) * (f*k - gg*j);
    double_t P2 = pow(-1, 1+2) * (e*k - i*gg);
    double_t P3 = pow(-1, 1+3) * (e*j - i*f);

    double_t P = pow(-1, 4+4) * (a*(P1) + b*(P2) + c*(P3));


    //determinant
    double_t A_det = a*A + b*B + c*C + d*D;
>>>>>>> 1cbf950d8ce41908d4595a2b29cc90c7cea459df
    //DEBUG_PRINT("determinant: %f\n", (double) A_det);

    if (A_det == 0)  {
        DEBUG_PRINT("UNDEFINED INVERSE, RETURNING last matrix \n");
        m_4d last_mat;

        for (int jj = 0; jj < ROWS; jj++) {
            for (int kk = 0; kk < COLUMNS; kk++) {

                //if (jj==kk){
                //    zeros.m[jj][kk] = 1;
                //}
                //else{
                    last_mat.m[jj][kk] = past_mat.m[jj][kk];
                //}
            }
        }
        return last_mat;
    }
    //adjugate matrix
<<<<<<< HEAD
    float Adj[4][4] = {{A, E, I, M},
=======
    double_t Adj[4][4] = {{A, E, I, M},
>>>>>>> 1cbf950d8ce41908d4595a2b29cc90c7cea459df
                       {B, F, J, N},
                       {C, G, K, O},
                       {D, H, L, P}};



    for (int jj = 0; jj < ROWS; jj++) {
        for (int kk = 0; kk < COLUMNS; kk++) {
            mat_inv.m[jj][kk] = (1.0 / (double) A_det) * (double) Adj[jj][kk];
            past_mat.m[jj][kk] = mat_inv.m[jj][kk];
        }
    }

    return mat_inv;
}

double_t * f(double_t * state, double_t * u){
    //construct temporary state in dynamics
    static double_t state_temp[9];
    for (int i=0; i < 9; i++){
        state_temp[i] = *(state + i);
    }
    //construct temp input for dynamics
    static double_t input_temp[4];
    for (int i=0;i < 4; i++){
        input_temp[i] = *(u + i);
    }


    static double_t state_d[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    double_t eul_angles[3] = {state_temp[3], state_temp[4], state_temp[5]}; //phi, theta, psi
    //struct mat33 I_moment = mscl((float)(pow(10,-5)), mdiag((float)(2.3951), (float)(2.3951), (float)(3.2346)));

    //struct mat33 I_moment_inv = matinv_3d(I_moment);
    //for (int i = 0; i < 3; i++){
    //    for (int j=0; j< 3; j++){
    //        printf("Moment (%d, %d): %f \n", i, j, I_moment.m[i][j]);
    //    }
    //}
    //
    //for (int i = 0; i < 3; i++){
    //    for (int j=0; j< 3; j++){
    //        printf("Moment inv (%d, %d): %f \n", i, j, I_moment_inv.m[i][j]);
    //    }
    //}
<<<<<<< HEAD
    struct vec vel;
    struct vec omega_b;


    //linear velocity of body
    vel.x = (float)state_temp[6];
    vel.y = (float)state_temp[7];
    vel.z = (float)state_temp[8];

    //angular velocity of body
    omega_b.x = (float)state_temp[9];
    omega_b.y = (float)state_temp[10];
    omega_b.z = (float)state_temp[11];
=======
    //struct vec vel;
    //struct vec omega_b;


    //linear velocity of body
    //vel.x = (float)state_temp[6];
    //vel.y = (float)state_temp[7];
    //vel.z = (float)state_temp[8];

    //angular velocity of body
    //omega_b.x = (float)state_temp[9];
    //omega_b.y = (float)state_temp[10];
    //omega_b.z = (float)state_temp[11];
>>>>>>> 1cbf950d8ce41908d4595a2b29cc90c7cea459df

    //struct mat33 Twb;
    struct mat33 Rwb;

    //crate Rwb and Twb matrices
    Rwb.m[0][0] = (float)(cos(eul_angles[1]) * cos(eul_angles[2]));
    Rwb.m[0][1] =(float)(sin(eul_angles[0]) * sin(eul_angles[1]) * cos(eul_angles[2]) - sin(eul_angles[2])*cos(eul_angles[0]));
    Rwb.m[0][2] =(float)(sin(eul_angles[1]) * cos(eul_angles[0]) * cos(eul_angles[2]) + sin(eul_angles[0])*sin(eul_angles[2]));

    Rwb.m[1][0] = (float)(sin(eul_angles[2]) * cos(eul_angles[1]));
    Rwb.m[1][1] = (float)(sin(eul_angles[0]) * sin(eul_angles[1]) * sin(eul_angles[2]) + cos(eul_angles[0]) * cos(eul_angles[2]));
    Rwb.m[1][2] = (float)(sin(eul_angles[1]) * sin(eul_angles[2]) * cos(eul_angles[0]) - sin(eul_angles[0]) * cos(eul_angles[2]));

    Rwb.m[2][0] = (float)(-1.0*sin(eul_angles[1]));
    Rwb.m[2][1] = (float)(sin(eul_angles[0]) * cos(eul_angles[1]));
    Rwb.m[2][2] = (float)(cos(eul_angles[0]) * cos(eul_angles[1]));


    //for (int j = 0; j < 3; j++){
    //    for (int k=0; k <3; k++){
    //        printf("matrix (Rwb) (%i, %i) : %f \n",j, k, Rwb.m[j][k] );
    //    }
    //}

    //double phi = eul_angles[0];
    //double theta = eul_angles[1];

    //Twb.m[0][0] = 1.0;
    //Twb.m[0][1] = (float)(sin((double)phi) * tan((double)theta));
    //Twb.m[0][2] = (float)(cos((double)phi) * tan((double)theta));
    //
    //Twb.m[1][0] = 0.0;
    //Twb.m[1][1] = (float)cos((double)phi);
    //Twb.m[1][2] = -1*(float)sin((double)phi);
    //
    //Twb.m[2][0] = 0;
    //Twb.m[2][1] = (float)(sin((double)phi) / cos((double)theta));
    //Twb.m[2][2] = (float)(cos((double)phi) / cos((double)theta));

    //set vel
    state_d[0] = state_temp[6];
    state_d[1] = state_temp[7];
    state_d[2] = state_temp[8];

    //set angle velocity
    //struct vec angles_update;
    //angles_update = mvmul(Twb, omega_b);
    state_d[3] = input_temp[1]; //(double_t)angles_update.x; //phi
    state_d[4] = input_temp[2]; //(double_t)angles_update.y; //theta
    state_d[5] = input_temp[3]; //(double_t)angles_update.z; //psi

    //set linear acceleration
    struct vec z_w;
    z_w.x = 0;
    z_w.y = 0;
    z_w.z = 1;

    struct vec z_b;
    z_b.x = Rwb.m[0][2];
    z_b.y = Rwb.m[1][2];
    z_b.z = Rwb.m[2][2];

    struct vec acc = vdiv(vadd(vscl((float)(-1.0*m*g), z_w), vscl((float)input_temp[0], z_b)), (float)m);

    state_d[6] = (double_t)acc.x;
    state_d[7] = (double_t)acc.y;
    state_d[8] = (double_t)acc.z;

    //input moments
    //struct vec moments;
    //moments.x = (float)input_temp[1];
    //moments.y = (float)input_temp[2];
    //moments.z = (float)input_temp[3];

    //printf("moment1 : %f \n ", moments.x);
    //printf("moment2 : %f \n ", moments.y);
    //printf("moment3 : %f \n ", moments.z);


    // set body rate acceleration
    //struct vec angle_acc = mvmul(I_moment_inv, vadd(vcross(vscl(-1, omega_b), mvmul(I_moment, omega_b)), moments));
    //
    //state_d[9] =  (double_t)angle_acc.x;
    //state_d[10] = (double_t)angle_acc.y;
    //state_d[11] = (double_t)angle_acc.z;

    return state_d;
}

double_t * sam_simulation(double_t * state, double_t * input, double_t t_step){
    float t = 0;
    //float y_output[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    //construct temporary state in simulator
    static double_t state_temp[9];
    for (int i=0; i < 9; i++){
        state_temp[i] = *(state + i);
    }

    //construct temp input for simulator
    static double_t input_temp[4];
    for (int i=0;i < 4;i++){
        input_temp[i] = *(input + i);
    }

    double_t state_d_vector[9];
    double_t * state_d;
    //iterate
<<<<<<< HEAD
    while (t < horizon){
=======
    while ((double) t < horizon){
>>>>>>> 1cbf950d8ce41908d4595a2b29cc90c7cea459df
        state_d = f(state_temp, input_temp);
        for (int i=0; i < 9; i++){
            state_d_vector[i] = *(state_d + i);
        }

        for (int i = 0; i < 9; i ++){
            state_temp[i] = state_temp[i] + state_d_vector[i]*t_step;
        }
        t = t+(float)t_step;
    }
    return state_temp;
}

double_t * yorai_h(double_t * s){

    double_t state[9];
    for (int i =0; i < 9; i++){
       state[i] = *(s + i);
    }
    static double_t h_state[4] = {0, 0, 0, 0};
    h_state[0] = state[0];
    h_state[1] = state[1];
    h_state[2] = state[2];
    h_state[3] = state[5];

    return h_state;
}


float* ref_traj(double t){

    static float traj[4] = {0, 0, 0, 0};
    traj[0] = (float)(.6*cos(t));
    traj[1] = (float)(.6*sin(t));
    traj[2] = (float)(1.0 + .6*sin(t));
    traj[3] = (float)(t);

    return traj;
}


void controllerSamYorai(control_t* control, setpoint_t* setpoint,
                        const sensorData_t* sensors, const state_t* state_cf,
                        const uint32_t tick){

    //controller runs at 500 Hz
<<<<<<< HEAD
=======
    // desired_wb.thrust = 0.5;
>>>>>>> 1cbf950d8ce41908d4595a2b29cc90c7cea459df
    if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)){
        //returns actual control inputs (thrust, m_x, m_y, m_z)

        //get thrust from previously calculated value


        //use pid to calculate desired moments based on error between desired angular velocity and actual angular velocity
        attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                                         degrees(desired_wb.attitudeRate.roll), degrees(desired_wb.attitudeRate.pitch),
                                         degrees(desired_wb.attitudeRate.yaw));

<<<<<<< HEAD


        control->thrust = massThrust * desired_wb.thrust;

=======
>>>>>>> 1cbf950d8ce41908d4595a2b29cc90c7cea459df
        attitudeControllerGetActuatorOutput(&control->roll, &control->pitch, &control->yaw);

        control->yaw = -control->yaw;


<<<<<<< HEAD
          if (control->thrust == 0)
          {
            control->thrust = 0;
            control->roll = 0;
            control->pitch = 0;
            control->yaw = 0;

            attitudeControllerResetAllPID();

          }
=======
        DEBUG_PRINT("THRUST: %f \n", (double) control->thrust);
        DEBUG_PRINT("ROLL: %d \n ", control->roll);
        DEBUG_PRINT("PITCH: %d \n", control->pitch);
        DEBUG_PRINT("YAW: %d \n", -control->yaw);


          //if (control->thrust == 0)
          //{
          //  control->thrust = 0;
          //  control->roll = 0;
          //  control->pitch = 0;
          //  control->yaw = 0;
          //
          //  attitudeControllerResetAllPID();
          //
          //}
>>>>>>> 1cbf950d8ce41908d4595a2b29cc90c7cea459df
    }


    //code runs at 100 Hz
    if (RATE_DO_EXECUTE(POSITION_RATE, tick)){
        //this runs yorai's controller for calculating forward simulation of model


        //intialize variable
        double eps = 0.00001;
        double_t dt = (1.0/ATTITUDE_RATE);
<<<<<<< HEAD
        float Jac[ROWS][COLUMNS];
=======
        double_t Jac[ROWS][COLUMNS];
>>>>>>> 1cbf950d8ce41908d4595a2b29cc90c7cea459df

        //gather current state
        double_t state[9] = {(double_t) state_cf->position.x, (double_t) state_cf->position.y, (double_t) state_cf->position.z,
                              (double_t) state_cf->attitude.roll, (double_t) state_cf->attitude.pitch, (double_t) state_cf->attitude.yaw,
                              (double_t) state_cf->velocity.x, (double_t) state_cf->velocity.y, (double_t) state_cf->velocity.z};
                              //(double_t) radians(sensors->gyro.x), (double_t) -radians(sensors->gyro.y), (double_t) radians(sensors->gyro.z)};

        //gather current input  (thrust, roll rate, pitch rate, yaw rate)
        init_input[0] = (double_t) desired_wb.thrust;
        init_input[1] = (double_t) desired_wb.attitudeRate.roll;
        init_input[2] = (double_t) desired_wb.attitudeRate.pitch;
        init_input[3] = (double_t) desired_wb.attitudeRate.yaw;

        //DEBUG_PRINT("Gathered CURRENT INPUT (YORAI-SAM) \n");

        //DEBUG_PRINT("INIT THRUST: %f\n", (double) init_input[0]);
        //DEBUG_PRINT("INIT M1: %f\n", (double) init_input[1]);
        //DEBUG_PRINT("INIT M2: %f \n", (double) init_input[2]);
        //DEBUG_PRINT("INIT M3: %f \n", (double) init_input[3]);
        //
        //
        //DEBUG_PRINT("POSITION X: %f \n", (double)  state_cf->position.x);
        //DEBUG_PRINT("POSITION Y: %f \n", (double)  state_cf->position.y);
        //DEBUG_PRINT("POSITION Z: %f \n", (double)  state_cf->position.z);
        //
        //DEBUG_PRINT("VELOCITY X: %f \n", (double)  state_cf->velocity.x);
        //DEBUG_PRINT("VELOCITY y: %f \n", (double)  state_cf->velocity.y);
        //DEBUG_PRINT("VELOCITY z: %f \n", (double)  state_cf->velocity.z);
        //
        //DEBUG_PRINT("GYRO X: %f \n", (double)  sensors->gyro.x);
        //DEBUG_PRINT("GYRO y: %f \n", (double)  sensors->gyro.y);
        //DEBUG_PRINT("GYRO z: %f \n", (double)  sensors->gyro.z);


        //calculate Jacobian

        //calculate center_g
        double_t center_g[4] = {0, 0, 0, 0};
        double_t * s_pointer;

        //DEBUG_PRINT("START CALCULATING JACOBIAN \n");
    
        s_pointer = sam_simulation(state, init_input, dt);

        double_t * yorai_row_pointer;
        static double_t sam_mod_state[9];

        for (int i=0; i < 9; i++){
            sam_mod_state[i] = *(s_pointer + i);
        }


        yorai_row_pointer = yorai_h(sam_mod_state);
        for (int i=0; i < 4; i++){
            center_g[i] = *(yorai_row_pointer + i);
        }

        //DEBUG_PRINT("center G 1: %f \n", (double) center_g[0]);
        //DEBUG_PRINT("center G 2: %f \n", (double) center_g[1]);
        //DEBUG_PRINT("center G 3: %f \n", (double) center_g[2]);
        //DEBUG_PRINT("center G 4: %f \n", (double) center_g[3]);

        //input calculate
        static double_t input_jac[4];
        double_t element_add[4] = {eps, 0, 0, 0};
        for (int i =0; i < 4;i++){
            input_jac[i] = init_input[i] +  element_add[i];
        }


        //DEBUG_PRINT("input 1: %f\n", (double) input_jac[0]);
        //DEBUG_PRINT("input 2: %f\n", (double) input_jac[1]);
        //DEBUG_PRINT("input 3: %f \n", (double) input_jac[2]);
        //DEBUG_PRINT("input 4: %f \n", (double) input_jac[3]);

        //calculate first row of Jacobian
        double_t yorai_row[4];
        s_pointer = sam_simulation(state, input_jac, dt);
        for (int i=0; i < 9; i++){
            sam_mod_state[i] = *(s_pointer + i);
        }


        yorai_row_pointer = yorai_h(sam_mod_state);
        for (int i=0; i < 4; i++){
            yorai_row[i] = *(yorai_row_pointer + i);
            //DEBUG_PRINT("row 1 yorai val: %f \n", (double) yorai_row[i]);
        }



        for (int i=0; i < 4; i++){
<<<<<<< HEAD
            Jac[i][0] = (float)(((double)(yorai_row[i] - center_g[i]))*(1.0/(double)(eps)));
=======
            Jac[i][0] = (yorai_row[i] - center_g[i]) / ((double)(eps));
            DEBUG_PRINT("JACK row 1: %f \n", Jac[i][0]);
>>>>>>> 1cbf950d8ce41908d4595a2b29cc90c7cea459df
        }

        //calculate second row of Jacobian
        static double_t input_jac_2[4];
        double_t element_add_2[4] = {0, eps, 0, 0};
        for (int i =0; i< 4;i++){
            input_jac_2[i] = init_input[i] +  element_add_2[i];
        }

        double_t yorai_row_2[4];
        s_pointer = sam_simulation(state, input_jac_2, dt);
        for (int i=0; i < 9; i++){
            sam_mod_state[i] = *(s_pointer + i);
        }


        yorai_row_pointer = yorai_h(sam_mod_state);
        for (int i=0; i < 4; i++){
            yorai_row_2[i] = *(yorai_row_pointer + i);
            //DEBUG_PRINT("row 2 yorai val: %f \n", (double) yorai_row_2[i]);
        }

        for (int i=0; i < 4; i++){
<<<<<<< HEAD
            Jac[i][1] = (float)(((double)(yorai_row_2[i] - center_g[i]))*(1.0/(double)(eps)));
=======
            Jac[i][1] =(yorai_row_2[i] - center_g[i]) / ((double)(eps));
            DEBUG_PRINT("JACK row 2: %f \n",Jac[i][1]);
>>>>>>> 1cbf950d8ce41908d4595a2b29cc90c7cea459df
        }

        //calculate third row of Jacobian
        static double_t input_jac_3[4];
        double_t element_add_3[4] = {0, 0, eps, 0};
        for (int i =0; i< 4;i++){
            input_jac_3[i] = init_input[i] +  element_add_3[i];
        }


        s_pointer = sam_simulation(state, input_jac_3, dt);
        for (int i=0; i < 9; i++){
            sam_mod_state[i] = *(s_pointer + i);
        }

        yorai_row_pointer = yorai_h(sam_mod_state);
        double_t yorai_row_3[4];
        for (int i=0; i < 4; i++){
            yorai_row_3[i] = *(yorai_row_pointer + i);
            //DEBUG_PRINT("row 3 yorai val: %f \n", (double) yorai_row_3[i]);
        }

        for (int i=0; i < 4; i++){
<<<<<<< HEAD
            Jac[i][2] = (float)(((double)(yorai_row_3[i] - center_g[i]))*(1.0/(double)(eps)));
=======
            Jac[i][2] = (yorai_row_3[i] - center_g[i]) / ((double)(eps));
            DEBUG_PRINT("JACK row 3: %f \n", (double) Jac[i][2]);
>>>>>>> 1cbf950d8ce41908d4595a2b29cc90c7cea459df
        }

        //calculate fourth row of Jacobian
        static double_t input_jac_4[4];
        double_t element_add_4[4] = {0, 0, 0, eps};
        for (int i =0; i< 4;i++){
            input_jac_4[i] = init_input[i] +  element_add_4[i];
        }

        s_pointer = sam_simulation(state, input_jac_4, dt);
        for (int i=0; i < 9; i++){
            sam_mod_state[i] = *(s_pointer + i);
        }

        yorai_row_pointer = yorai_h(sam_mod_state);
        double_t yorai_row_4[4];
        for (int i=0; i < 4; i++){
            yorai_row_4[i] = *(yorai_row_pointer + i);
            //DEBUG_PRINT("row 4 yorai val: %f \n", (double) yorai_row_4[i]);
        }

        for (int i=0; i < 4; i++){
<<<<<<< HEAD
            Jac[i][3] = (float)(((double)(yorai_row_4[i] - center_g[i]))*(1.0/(double)(eps)));
=======
            Jac[i][3] =(yorai_row_4[i] - center_g[i]) / ((double)(eps));
            DEBUG_PRINT("JACK row 4: %f \n", (double) Jac[i][3]);
>>>>>>> 1cbf950d8ce41908d4595a2b29cc90c7cea459df
        }


        //get reference point from setpoint
        //DEBUG_PRINT("GET REFERENCE FROM SETPOINT \n");
        float ref_point[4];
        float * ref_ptr;
        ref_ptr = ref_traj((double) (time + horizon));

        for (int i =0; i < 4; i++){
            ref_point[i] = *(ref_ptr + i);
        }

        //predict state based on horizon and input
        double_t prediction[4];

        //input array
        //DEBUG_PRINT("PREDICT STATE BASED ON HORIZON AND INPUT \n");
        static double_t state_pred[9];
        s_pointer = sam_simulation(state, init_input, (double_t) dt);
        for (int i = 0; i < 9; i++){
            state_pred[i] = *(s_pointer +i);
        }

        yorai_row_pointer = yorai_h(state_pred);
        for (int i =0; i < 4; i++){
            prediction[i] = *(yorai_row_pointer + i);
        }

<<<<<<< HEAD
        DEBUG_PRINT("predicted point (x): %f \n", (double)prediction[0]);
        DEBUG_PRINT("predicted point (y): %f \n", (double)prediction[1]);
        DEBUG_PRINT("predicted point (z): %f \n", (double)prediction[2]);
        DEBUG_PRINT("predicted point (t): %f \n", (double)prediction[3]);
        //
        DEBUG_PRINT("ref point x: %f: \n", (double) ref_point[0]);
        DEBUG_PRINT("ref point y: %f: \n", (double) ref_point[1]);
        DEBUG_PRINT("ref point z: %f: \n", (double) ref_point[2]);
        DEBUG_PRINT("ref point t: %f: \n", (double) ref_point[3]);
=======
        //DEBUG_PRINT("predicted point (x): %f \n", (double)prediction[0]);
        //DEBUG_PRINT("predicted point (y): %f \n", (double)prediction[1]);
        //DEBUG_PRINT("predicted point (z): %f \n", (double)prediction[2]);
        //DEBUG_PRINT("predicted point (t): %f \n", (double)prediction[3]);
        //
        //DEBUG_PRINT("ref point x: %f: \n", (double) ref_point[0]);
        //DEBUG_PRINT("ref point y: %f: \n", (double) ref_point[1]);
        //DEBUG_PRINT("ref point z: %f: \n", (double) ref_point[2]);
        //DEBUG_PRINT("ref point t: %f: \n", (double) ref_point[3]);
>>>>>>> 1cbf950d8ce41908d4595a2b29cc90c7cea459df

        //DEBUG_PRINT("alpha: %f \n ", (double ) alpha[1][2]);
        //DEBUG_PRINT("FIRST ROW OF JAC: %f \n", (double)Jac[0][0]);


        //calculate input derivative
        double_t diff_ref_pred[4];
        for (int i = 0; i < 4;i++){
            diff_ref_pred[i] = (double_t) ref_point[i] - prediction[i];
        }

        //calulcate inverse of 4x4 matrix
        m_4d Jac_inv;

        //DEBUG_PRINT("INVERT MATRIX \n");
        //DEBUG_PRINT("INVERT MATRIX \n");
        Jac_inv = matinv_4d(Jac);

        //DEBUG_PRINT("FIRST ROW OF JAC INV: %f \n", (double)Jac_inv.m[0][0]);
        //DEBUG_PRINT("SEC ROW OF JAC INV: %f \n", (double)Jac_inv.m[1][1]);
        //DEBUG_PRINT("THIRD ROW OF JAC INV: %f \n", (double)Jac_inv.m[2][2]);
        //DEBUG_PRINT("FOURTH ROW OF JAC INV: %f \n", (double)Jac_inv.m[3][3]);

        double u_d[4] = {0, 0, 0, 0};

        //matrix multiplication
        for (int i= 0; i < 4; i++){
            for(int j=0; j< 4;j++){
                u_d[i] += (double) alpha[i][j] * Jac_inv.m[j][i];

            }
            u_d[i] *= (double) diff_ref_pred[i];
        }


        //set inputs
        double u_new[4] = {0, 0, 0, 0};
        for (int i = 0; i < 4; i++) {
            u_new[i] = (double) init_input[i] + u_d[i] * (double) dt;
        }

        //increase time
        time = time + dt;
<<<<<<< HEAD
        DEBUG_PRINT("Time: %f \n", (double)time);
=======
        //DEBUG_PRINT("Time: %f \n", (double)time);

        DEBUG_PRINT("UPDATED THRUST: %f\n", (double) u_new[0]);
        DEBUG_PRINT("UPDATED ROLL RATE: %f\n", (double) u_new[1]);
        DEBUG_PRINT("UPDATED PITCH RATE: %f \n", (double) u_new[2]);
        DEBUG_PRINT("UPDATED YAW RATE: %f \n", (double) u_new[3]);

>>>>>>> 1cbf950d8ce41908d4595a2b29cc90c7cea459df

        //return input
        desired_wb.thrust = (float)u_new[0];
        desired_wb.attitudeRate.roll = (float)(u_new[1]);
        desired_wb.attitudeRate.pitch = (float)(u_new[2]);
        desired_wb.attitudeRate.yaw = (float)(u_new[3]);


<<<<<<< HEAD

    }


    //DEBUG_PRINT("UPDATED THRUST: %f\n", (double) u_new[0]);
    //DEBUG_PRINT("UPDATED ROLL: %f\n", (double) u_new[1]);
    //DEBUG_PRINT("UPDATED PITCH: %f \n", (double) u_new[2]);
    //DEBUG_PRINT("UPDATED YAW: %f \n", (double) u_new[3]);

=======
    }


    //set thrust
    control->thrust = massThrust * desired_wb.thrust;         
>>>>>>> 1cbf950d8ce41908d4595a2b29cc90c7cea459df

}