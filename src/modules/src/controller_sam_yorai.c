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

#define ROWS 4
#define COLUMNS 4


static float horizon = (float)(0.2);   //was 0.7
static float alpha[4][4] = {
                        {20, 0,  0, 0},
                        {0, 50, 0, 0},
                        {0, 0, 50, 0},
                        {0, 0, 0, 50} };
static float init_input[4] = {0, 0, 0, 0};
static double g = 9.81;
static double m = 35.89 / 1000;

typedef struct {
    float m[4][4];
} m_4d;


void controllerSamYoraiReset(void){
}

void controllerSamYoraiInit(void){
    controllerSamYoraiReset();
}

bool controllerSamYoraiTest(void)
{
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

    //create mat_inv array of pointers
    m_4d mat_inv;

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


    //cofactors

    //2x2 cofactors A
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
    //printf("determinant: %f\n", A_det);

    if (A_det == 0)  {
        DEBUG_PRINT("UNDEFINED INVERSE, RETURNING IDENTITY \n");
        m_4d ident;

        for (int jj = 0; jj < ROWS; jj++) {
            for (int kk = 0; kk < COLUMNS; kk++) {

                if (jj==kk){
                    ident.m[jj][kk] = 1;
                }
                else{
                    ident.m[jj][kk] = 0;
                }
            }
        }
        return ident;
    }
    //adjugate matrix
    float Adj[4][4] = {{A, E, I, M},
                       {B, F, J, N},
                       {C, G, K, O},
                       {D, H, L, P}};



    for (int jj = 0; jj < ROWS; jj++) {
        for (int kk = 0; kk < COLUMNS; kk++) {
            mat_inv.m[jj][kk] = (1 / A_det) * Adj[jj][kk];
        }
    }

    return mat_inv;
}

float* f(float * state, float * u){
    //construct temporary state in dynamics
    static float state_temp[12];
    for (int i=0; i < 12; i++){
        state_temp[i] = *(state + i);
    }

    //construct temp input for dynamics
    static float input_temp[4];
    for (int i=0;i < 4;i++){
        input_temp[i] = *(u + i);
    }


    static float state_d[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    float eul_angles[3] = {state_temp[3], state_temp[4], state_temp[5]}; //phi, theta, psi
    struct mat33 I_moment = mscl((float)(10e-5), mdiag((float)(2.3951), (float)(2.3951), (float)(3.2346)));
    struct mat33 I_moment_inv = matinv_3d(I_moment);
    struct vec vel;
    struct vec omega_b;

    vel.x = state_temp[6];
    vel.y = state_temp[7];
    vel.z = state_temp[8];

    omega_b.x = state_temp[9];
    omega_b.y = state_temp[10];
    omega_b.z = state_temp[11];

    struct mat33 Twb;
    struct mat33 Rwb;

    //crate Rwb and Twb matrices
    Rwb.m[0][0] = cos((double)eul_angles[1]) * cos((double)eul_angles[2]);
    Rwb.m[0][1] = sin((double)eul_angles[0]) * sin((double)eul_angles[1]) * cos((double)eul_angles[2]) - sin((double)eul_angles[2])*cos((double)eul_angles[0]);
    Rwb.m[0][2] = sin((double)eul_angles[1]) * cos((double)eul_angles[0]) * cos((double)eul_angles[2]) + sin((double)eul_angles[0])*sin((double)eul_angles[2]);

    Rwb.m[1][0] = sin((double)eul_angles[2]) * cos((double)eul_angles[1]);
    Rwb.m[1][1] = sin((double)eul_angles[0]) * sin((double)eul_angles[1]) * sin((double)eul_angles[2]) + cos((double)eul_angles[0]) * cos((double)eul_angles[2]);
    Rwb.m[1][2] = sin((double)eul_angles[1]) * sin((double)eul_angles[2]) * cos((double)eul_angles[0]) - sin((double)eul_angles[0]) * cos((double)eul_angles[2]);

    Rwb.m[2][0] = (float)(-sin((double)(eul_angles[1])));
    Rwb.m[2][1] = sin((double)eul_angles[0]) * cos((double)eul_angles[1]);
    Rwb.m[2][2] = cos((double)eul_angles[0]) * cos((double)eul_angles[1]);


    float phi = eul_angles[0];
    float theta = eul_angles[1];

    Twb.m[0][0] = 1;
    Twb.m[0][1] = sin((double)phi) * tan((double)theta);
    Twb.m[0][2] = cos((double)phi) * tan((double)theta);

    Twb.m[1][0] = 0;
    Twb.m[1][1] = cos((double)phi);
    Twb.m[1][2] = -sin((double)phi);

    Twb.m[2][0] = 0;
    Twb.m[2][1] = sin((double)phi) / cos((double)theta);
    Twb.m[2][2] = cos((double)phi) / cos((double)theta);

    //set vel
    state_d[0] = vel.x;
    state_d[1] = vel.y;
    state_d[2] = vel.z;

    //set angles
    struct vec angles_update;
    angles_update = mvmul(Twb, omega_b);
    state_d[3] = angles_update.x; //phi
    state_d[4] = angles_update.y; //theta
    state_d[5] = angles_update.z; //psi

    //set linear acceleration
    struct vec z_w;
    z_w.x = 0;
    z_w.y = 0;
    z_w.z = 0;

    struct vec z_b;
    z_b.x = Rwb.m[0][2];
    z_b.y = Rwb.m[1][2];
    z_b.z = Rwb.m[2][2];

    struct vec acc = vdiv(vadd(vscl((float)(-m*g), z_w), vscl(input_temp[0], z_b)), (float)m);

    state_d[6] = acc.x;
    state_d[7] = acc.y;
    state_d[8] = acc.z;

    //input moments
    struct vec moments;
    moments.x = input_temp[1];
    moments.y = input_temp[2];
    moments.z = input_temp[3];


    // Body Rate acceleration
    struct vec angle_acc = vadd(mvmul(I_moment_inv, vcross(vscl(-1, omega_b), mvmul(I_moment, omega_b))), moments);

    state_d[9] = angle_acc.x;
    state_d[10] = angle_acc.y;
    state_d[11] = angle_acc.z;
    return state_d;
}

float* sam_simulation(float * state, float * input, float t_step){
    float t = 0;
    //float y_output[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    //construct temporary state in simulator
    static float state_temp[12];
    for (int i=0; i < 12; i++){
        state_temp[i] = *(state + i);
    }

    //construct temp input for simulator
    static float input_temp[4];
    for (int i=0;i < 4;i++){
        input_temp[i] = *(input + i);
    }

    float state_d_vector[12];
    float * state_d;
    //iterate
    while (t < horizon){
        state_d = f(state_temp, input_temp);
        for (int i=0; i < 12; i++){
            state_d_vector[i] = *(state_d + i);
        }

        for (int i = 0; i < 12; i ++){
            state_temp[i] = state_temp[i] + state_d_vector[i]*t_step;
        }
        t = t+t_step;
    }
    return state_temp;
}

float * yorai_h(float * s){

    float state[12];
    for (int i =0; i < 12; i++){
       state[i] = *(s + i);
    }
    static float h_state[4] = {0, 0, 0, 0};
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

    //intialize variable
    float eps = 1e-5;
    float dt = (1.0f/ATTITUDE_RATE);
    float Jac[ROWS][COLUMNS];

    //controller runs at 500 Hz
    if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)){
        return;
    }

    //gather current state
    float state[12] = {state_cf->position.x, state_cf->position.y, state_cf->position.z,
                       state_cf->attitude.roll, state_cf->attitude.pitch, state_cf->attitude.yaw,
                       state_cf->velocity.x, state_cf->velocity.y, state_cf->velocity.z,
                       sensors->gyro.x, sensors->gyro.y, sensors->gyro.z};
                                                                                                 //gather current input
    init_input[0] = control->thrust;
    init_input[1] = control->roll;
    init_input[2] = control->pitch;
    init_input[3] = control->yaw;

    //DEBUG_PRINT("Gathered CURRENT INPUT (YORAI-SAM) \n");

    //DEBUG_PRINT("THRUST: %f\n", (double) init_input[0]);
    //DEBUG_PRINT("ROLL: %f\n", (double) init_input[1]);
    //DEBUG_PRINT("PITCH: %f \n", (double) init_input[2]);
    //DEBUG_PRINT("YAW: %f \n", (double) init_input[3]);

    //calculate Jacobian

    //calculate center_g
    float center_g[4] = {0, 0, 0, 0};
    float * s_pointer;

    //DEBUG_PRINT("START CALCULATING JACOBIAN \n");
    
    s_pointer = sam_simulation(state, init_input, dt);

    float * yorai_row_pointer;
    static float sam_mod_state[12];

    for (int i=0; i < 12; i++){
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
    static float input_jac[4];
    float element_add[4] = {eps, 0, 0, 0};
    for (int i =0; i < 4;i++){
        input_jac[i] = init_input[i] +  element_add[i];
    }


    //DEBUG_PRINT("input 1: %f\n", (double) input_jac[0]);
    //DEBUG_PRINT("input 2: %f\n", (double) input_jac[1]);
    //DEBUG_PRINT("input 3: %f \n", (double) input_jac[2]);
    //DEBUG_PRINT("input 4: %f \n", (double) input_jac[3]);

    //calculate first row of Jacobian
    float yorai_row[4];
    s_pointer = sam_simulation(state, input_jac, dt);
    for (int i=0; i < 12; i++){
        sam_mod_state[i] = *(s_pointer + i);
    }


    yorai_row_pointer = yorai_h(sam_mod_state);
    for (int i=0; i < 4; i++){
        yorai_row[i] = *(yorai_row_pointer + i);
        //DEBUG_PRINT("row 1 yorai val: %f \n", (double) yorai_row[i]);
    }



    for (int i=0; i < 4; i++){
        Jac[i][0] = (float)(((double)(yorai_row[i] - center_g[i]))*(1.0/(double)(eps)));
    }

    //calculate second row of Jacobian
    static float input_jac_2[4];
    float element_add_2[4] = {0, eps, 0, 0};
    for (int i =0; i< 4;i++){
        input_jac_2[i] = init_input[i] +  element_add_2[i];
    }

    float yorai_row_2[4];
    s_pointer = sam_simulation(state, input_jac_2, dt);
    for (int i=0; i < 12; i++){
        sam_mod_state[i] = *(s_pointer + i);
    }


    yorai_row_pointer = yorai_h(sam_mod_state);
    for (int i=0; i < 4; i++){
        yorai_row_2[i] = *(yorai_row_pointer + i);
        //DEBUG_PRINT("row 2 yorai val: %f \n", (double) yorai_row_2[i]);
    }

    for (int i=0; i < 4; i++){
        Jac[i][1] = (float)(((double)(yorai_row_2[i] - center_g[i]))*(1.0/(double)(eps)));
    }

    //calculate third row of Jacobian
    static float input_jac_3[4];
    float element_add_3[4] = {0, 0, eps, 0};
    for (int i =0; i< 4;i++){
        input_jac_3[i] = init_input[i] +  element_add_3[i];
    }


    s_pointer = sam_simulation(state, input_jac_3, dt);
    for (int i=0; i < 12; i++){
        sam_mod_state[i] = *(s_pointer + i);
    }

    yorai_row_pointer = yorai_h(sam_mod_state);
    float yorai_row_3[4];
    for (int i=0; i < 4; i++){
        yorai_row_3[i] = *(yorai_row_pointer + i);
        //DEBUG_PRINT("row 3 yorai val: %f \n", (double) yorai_row_3[i]);
    }

    for (int i=0; i < 4; i++){
        Jac[i][2] = (float)(((double)(yorai_row_3[i] - center_g[i]))*(1.0/(double)(eps)));
    }

    //calculate fourth row of Jacobian
    static float input_jac_4[4];
    float element_add_4[4] = {0, 0, 0, eps};
    for (int i =0; i< 4;i++){
        input_jac_4[i] = init_input[i] +  element_add_4[i];
    }

    s_pointer = sam_simulation(state, input_jac_4, dt);
    for (int i=0; i < 12; i++){
        sam_mod_state[i] = *(s_pointer + i);
    }

    yorai_row_pointer = yorai_h(sam_mod_state);
    float yorai_row_4[4];
    for (int i=0; i < 4; i++){
        yorai_row_4[i] = *(yorai_row_pointer + i);
        //DEBUG_PRINT("row 4 yorai val: %f \n", (double) yorai_row_4[i]);
    }

    for (int i=0; i < 4; i++){
        Jac[i][3] = (float)(((double)(yorai_row_4[i] - center_g[i]))*(1.0/(double)(eps)));
    }


    //get reference point from setpoint
    //DEBUG_PRINT("GET REFERENCE FROM SETPOINT \n");
    float ref_point[4];
    float * ref_ptr;
    ref_ptr = ref_traj((double) (tick + horizon));

    for (int i =0; i < 4; i++){
        ref_point[i] = *(ref_ptr + i);
    }

    //predict state based on horizon and input
    float prediction[4];

    //input array
    //DEBUG_PRINT("PREDICT STATE BASED ON HORIZON AND INPUT \n");
    static float state_pred[12];
    s_pointer = sam_simulation(state, init_input,dt);
    for (int i = 0; i < 12; i++){
        state_pred[i] = *(s_pointer +i);
    }

    yorai_row_pointer = yorai_h(state_pred);
    for (int i =0; i < 4; i++){
        prediction[i] = *(yorai_row_pointer + i);
    }

    //DEBUG_PRINT("predicted point (x): %f \n", (double)prediction[0]);
    //DEBUG_PRINT("predicted point (y): %f \n", (double)prediction[1]);
    //DEBUG_PRINT("predicted point (z): %f \n", (double)prediction[2]);
    //DEBUG_PRINT("predicted point (t): %f \n", (double)prediction[3]);
    //
    //DEBUG_PRINT("ref point: %f: \n", (double) ref_point[1]);
    //DEBUG_PRINT("alpha: %f \n ", (double ) alpha[1][2]);
    //DEBUG_PRINT("FIRST ROW OF JAC: %f \n", (double)Jac[0][0]);


    //calculate input derivative
    float diff_ref_pred[4];
    for (int i = 0; i < 4;i++){
        diff_ref_pred[i] = ref_point[i] - prediction[i];
    }

    //calulcate inverse of 4x4 matrix
    m_4d Jac_inv;

    DEBUG_PRINT("INVERT MATRIX \n");
    Jac_inv = matinv_4d(Jac);

    //DEBUG_PRINT("FIRST ROW OF JAC INV: %f \n", (double)Jac_inv.m[0][0]);
    //DEBUG_PRINT("SEC ROW OF JAC INV: %f \n", (double)Jac_inv[1][1]);
    //DEBUG_PRINT("THIRD ROW OF JAC INV: %f \n", (double)Jac_inv[2][2]);
    //DEBUG_PRINT("FOURTH ROW OF JAC INV: %f \n", (double)Jac_inv[3][3]);
    //
    float u_d[4] = {0, 0, 0, 0};

    //matrix multiplication
    for (int i= 0; i < 4; i++){
        for(int j=0; j< 4;j++){
            u_d[i] += alpha[i][j] * Jac_inv.m[j][i];

        }
        u_d[i] *= diff_ref_pred[i];
    }


    //set inputs
    float u_new[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; i++) {
        u_new[i] = init_input[i] + u_d[i]*dt;
    }

    //return input

    control->thrust = u_new[0];
    control->roll = (int16_t)(u_new[1]);
    control->pitch =(int16_t)(u_new[2]);
    control->yaw = (int16_t)(u_new[3]);

    //free matrix inv
    //for (int i =0; i <3; i++){
    //    free(Jac_inv[i]);
    //}


    DEBUG_PRINT("UPDATED THRUST: %f\n", (double) u_new[0]);
    DEBUG_PRINT("UPDATED ROLL: %f\n", (double) u_new[1]);
    DEBUG_PRINT("UPDATED PITCH: %f \n", (double) u_new[2]);
    DEBUG_PRINT("UPDATED YAW: %f \n", (double) u_new[3]);


}