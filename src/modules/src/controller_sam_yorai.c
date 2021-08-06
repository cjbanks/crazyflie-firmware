//
// Created by chris on 7/29/21.
//

#include <math.h>
#include "param.h"
#include "log.h"
#include "math3d.h"
#include "controller_sam_yorai.h"

static float horizon = (float)(0.7);
static float alpha[4][4] = {
                        {20, 0,  0, 0},
                        {0, 50, 0, 0},
                        {0, 0, 50, 0},
                        {0, 0, 0, 50} };
static float init_input[4] = {0, 0, 0, 0};
static double g = 9.81;
static double m = 35.89 / 1000;


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

void matinv_4d(float matrix_in[4][4], float* mat_inv[4][4]){
    //static float mat_inv[4][4];

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
    float A = f*(k*p-l*o) + gg*(j * p - l * n) + h * (o * j - n * k);
    float B = -1*(e*(k*p - l*o) + gg*(i * p - mm*l) + h*(i*o - mm*k));
    float C = e*(o*j - l*n) + f*(i*o - mm*l) + h*(i*n - j*mm);
    float D = -1*(e*(j*o - n*k) + f*(i*o - mm*k) + gg*(i*n - j*mm));

    float E = -1*(b*(k*p - l*o) + c*(j*p - n*l) + d*(j*o - n*k));
    float F = a*(k*p - l*o) + c*(i*p - mm*l) + d*(i*o - mm*k);
    float G = -1*(a*(j*p - l*n) + b*(i*p - mm*l) + d*(i*n - mm*j));
    float H = a*(o*j - n*k) + b*(i*o - mm*k) + c*(i*n - mm*j);

    float I = b*(gg*p - o*h) + c*(f*p - n*h) + d*(f*o - n*gg);
    float J = -1*(a*(gg*p - o*h) + c*(e*p - mm*h) + d*(e*o - mm*gg));
    float K = a*(f*p - n*h) + b*(e*p - mm*h) + d*(e*n - mm*f);
    float L = -1*(a*(f*o - n*gg) + b*(e*o - mm*gg) + c*(e*n - mm*f));

    float M = -1*(b*(gg*l - k*h) + c*(f*l - j*k) + d*(f*k - j*gg));
    float N = a*(gg*l - k*h) + c*(e*l - i*h) + d*(e*k -i*gg);
    float O = -1*(a*(f*l - j*h) + b*(e*l - i*h) + d*(e*j - i*f));
    float P = a*(f*k - j*gg) + b*(e*k - i*gg) + c*(e*j - i*f);


    //determinant
    float A_det = a*A + b*B + c*C + d*D;

    //adjugate matrix
    float Adj[4][4] = { {A, E, I, M},
                        {B, F, J, N},
                        {C, G, K, O},
                        {D, H, L, P}};



    for (int jj = 0; jj < 4; jj++) {
        for (int kk = 0; kk < 4; kk++) {
            *mat_inv[jj][kk] = (1 / A_det) * Adj[jj][kk];
        }
    }

     //return *mat_inv;
}

float* f(float state[12], float u[4]){
    static float state_d[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    float eul_angles[3] = {state[3], state[4], state[5]}; //phi, theta, psi
    struct mat33 I_moment = mscl((float)(10e-5), mdiag((float)(2.3951), (float)(2.3951), (float)(3.2346)));
    struct mat33 I_moment_inv = matinv_3d(I_moment);
    struct vec vel;
    struct vec omega_b;

    vel.x = state[6];
    vel.y = state[7];
    vel.z = state[8];

    omega_b.x = state[9];
    omega_b.y = state[10];
    omega_b.z = state[11];


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

    struct vec acc = vdiv(vadd(vscl((float)(-m*g), z_w), vscl(u[0], z_b)), (float)m);

    state_d[6] = acc.x;
    state_d[7] = acc.y;
    state_d[8] = acc.z;


    // Body Rate acceleration
    struct vec angle_acc = mvmul(I_moment_inv, vcross(vscl(-1, omega_b), mvmul(I_moment, omega_b)));

    state_d[9] = angle_acc.x;
    state_d[10] = angle_acc.y;
    state_d[11] = angle_acc.z;
    return state_d;
}

float* sam_simulation(float state[12], float input[4], float t_step){
    float t = 0;
    //float y_output[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    float * state_d[13];
    //iterate
    while (t < horizon){
        state_d[12] = f(&state[12], &input[4]);
        for (int i = 0; i < 12; i ++){
            state[i] = state[i] + *state_d[i]*t_step;
        }
        t = t+t_step;
    }
    return state;
}

void yorai_h(float state[12], float * h_state[4]){

    //static float h_state[4] = {0, 0, 0, 0};
    *h_state[0] = state[0];
    *h_state[1] = state[1];
    *h_state[2] = state[2];
    *h_state[3] = state[5];

    //return *h_state;
}


void ref_traj(double t, float* traj[4]){

    //static float traj[4] = {0, 0, 0, 0};
    *traj[0] = (float)(.6*cos(t));
    *traj[1] = (float)(.6*sin(t));
    *traj[2] = (float)(1.0 + .6*sin(t));
    *traj[3] = (float)(t);

    //return traj;
}


void controllerSamYorai(control_t* control, setpoint_t* setpoint,
                        const sensorData_t* sensors, const state_t* state_cf,
                        const uint32_t tick){

    //intialize variable
    float eps = 1e-5;
    float dt = (1.0f/ATTITUDE_RATE);
    float Jac[4][4];


    //gather current state
    float state[12] = {state_cf->position.x, state_cf->position.y, state_cf->position.z,
                       state_cf->attitude.roll, state_cf->attitude.pitch, state_cf->attitude.yaw,
                       state_cf->velocity.x, state_cf->velocity.y, state_cf->velocity.z,
                       sensors->gyro.x, sensors->gyro.y, sensors->gyro.z};

    //controller runs at 500 Hz
    if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)){
        return;
    }


    //gather current input
    init_input[0] = control->thrust;
    init_input[1] = control->roll;
    init_input[2] = control->pitch;
    init_input[3] = control->yaw;

    //calculate Jacobian

    //calculate first row
    float center_g[4] = {0, 0, 0, 0};
    
    yorai_h(sam_simulation(&state[12], init_input, dt), (float **) &center_g[4]);

    //input calculate
    float input_jac[4];
    float element_add[4] = {eps, 0, 0, 0};
    for (int i =0; i < 4;i++){
        input_jac[i] = init_input[i] +  element_add[i];
    }

    static float yorai_row[4] = {0, 0, 0, 0};
    yorai_h(sam_simulation(&state[12],input_jac, dt), (float **) &yorai_row);
    for (int i=0; i < 4; i++){
        Jac[i][0] = (float)(((double)(yorai_row[i] - center_g[i]))*(1.0/(double)(eps)));
    }

    //calculate second row
    float input_jac_2[4];
    float element_add_2[4] = {0, eps, 0, 0};
    for (int i =0; i< 4;i++){
        input_jac_2[i] = init_input[i] +  element_add_2[i];
    }

    static float yorai_row_2[4] = {0, 0, 0, 0};
    yorai_h(sam_simulation(&state[12],input_jac_2, dt), (float **)yorai_row_2);
    for (int i=0; i < 4; i++){
        Jac[i][1] = (float)(((double)(yorai_row_2[i] - center_g[i]))*(1.0/(double)(eps)));
    }

    //calculate third row
    float input_jac_3[4];
    float element_add_3[4] = {0, 0, eps, 0};
    for (int i =0; i< 4;i++){
        input_jac_3[i] = init_input[i] +  element_add_3[i];
    }

    static float  yorai_row_3[4] = {0, 0, 0, 0};
    yorai_h(sam_simulation(&state[12],input_jac_3, dt), (float **)yorai_row_3);
    for (int i=0; i < 4; i++){
        Jac[i][2] = (float)(((double)(yorai_row_3[i] - center_g[i]))*(1.0/(double)(eps)));
    }

    //calculate fourth row
    float input_jac_4[4];
    float element_add_4[4] = {0, 0, 0, eps};
    for (int i =0; i< 4;i++){
        input_jac_4[i] = init_input[i] +  element_add_4[i];
    }


    static float yorai_row_4[4] = {0, 0, 0, 0};
    yorai_h(sam_simulation(&state[12],input_jac_4, dt), (float **)yorai_row_4);
    for (int i=0; i < 4; i++){
        Jac[i][3] = (float)(((double)(yorai_row_4[i] - center_g[i]))*(1.0/(double)(eps)));
    }

    //get reference point from setpoint
    static float ref_point[4];
    ref_traj(tick, (float **)&ref_point);

    //predict state based on horizon and input
    static float  prediction[4] = {0, 0, 0, 0};

    //input array
    //float input_arr[4] = {0, 0, 0, 0};
    //input_arr[0] = control->thrust;
    //input_arr[1] = control->roll;
    //input_arr[2] = control->pitch;
    //input_arr[3] = control->yaw;

    yorai_h(sam_simulation(&state[12], init_input, dt), (float **)prediction);


    //calculate input derivative
    float diff_ref_pred[4];
    for (int i = 0; i< 4;i++){
        diff_ref_pred[i] = ref_point[i] - prediction[i];
    }

    //calulcate inverse of 4x4 matrix
    static float Jac_inv[4][4];
    matinv_4d(Jac,(float *(*)[4])&Jac_inv[4][4]);

    float u_d[4] = {0, 0, 0, 0};

    //matrix multiplication
    for (int i= 0; i < 4; i++){
        for(int j=0; j< 4;j++){
            u_d[i] += alpha[i][j] * Jac_inv[j][i];

        }
        u_d[i] *= diff_ref_pred[i];
    }


    //struct {
    //    float thrust;
    //    int16_t moment_x;
    //    int16_t moment_y;
    //    int16_t moment_z;
    //} u_new;

    //set inputs
    //u_new.thrust = u_d[0];
    //u_new.moment_x = (int16_t)(u_d[1]);
    //u_new.moment_y = (int16_t)(u_d[2]);
    //u_new.moment_z = (int16_t)(u_d[3]);
    float u_new[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; i++) {
        u_new[i] = init_input[i] + u_d[i]*dt;
    }

    //return input
    control->thrust = u_new[0];
    control->roll = (int16_t)(u_new[1]);
    control->pitch =(int16_t)(u_new[2]);
    control->yaw = (int16_t)(u_new[3]);
}