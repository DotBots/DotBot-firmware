#include <stdbool.h>
#include <stdint.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include "string.h"

#include "ekf.h"

#include "nrf.h"


//=========================== defines ==========================================
#define M_2PI 2 * M_PI

//=========================== prototypes =======================================


//=========================== variables ========================================


//=========================== public ===========================================

void EKF_init(KalmanFilter_t *ekf_p,float *x0, float *P0)
{

    // Copy inital state x0
    EKF_set_state(ekf_p, x0);


    // Copy the initial Covariance
    ekf_p->P[0] = P0[0];
    ekf_p->P[6] = P0[1];
    ekf_p->P[12] = P0[2];
    ekf_p->P[18] = P0[3];
    ekf_p->P[24] = P0[4];

    // Copy the Diagonal of the process Noise Matrix (The other values are zeros)
    // from the main defines of ekf.h, to a local variable
    ekf_p->Q[0] = EKF_Q_X * EKF_Q_X;
    ekf_p->Q[1] = EKF_Q_Y * EKF_Q_Y;
    ekf_p->Q[2] = EKF_Q_THETA * EKF_Q_THETA;
    ekf_p->Q[3] = EKF_Q_V * EKF_Q_V;
    ekf_p->Q[4] = EKF_Q_W * EKF_Q_W;

    // Configure the TIMER
}

void EKF_predict(KalmanFilter_t *ekf_p,float dt)
{
    /*
    Here we implement the following non-linear model
    X = X + V*dt*cos(theta)
    Y = Y + V*dt*cos(theta)
    Theta = theta + dt*W
    V = V
    W = W
    */

    // ##################################################
    // Run the state transition with the non-linear model
    // ##################################################
    
    _EKF_state_transition(ekf_p, dt);


    // ##################################################
    // Calculate the current Jacobian
    // ##################################################
    float f02, f03, f12, f13, f24;  // We declare the variable for only the non-zero and non-one elements
    float st, ct;                   // We declare temp variable to store trigonometric functions. We don't want to calculate them twice.

    // compute once the trig funtions
    st = sin(ekf_p->theta);
    ct = cos(ekf_p->theta);

    // Compute the relevant Jacobian functions
    f02 = -1 * ekf_p->V * dt * st;
    f03 =               dt * ct;
    f12 =      ekf_p->V * dt * ct;
    f13 =               dt * st;
    f24 =               dt;


    // ##################################################
    // Transition the Covariance Matrix
    // ##################################################

    // Create a local pointer of P and Q, for clarity (I don't want to have to write ekf_p->P seven million times)
    float *P = ekf_p->P;
    float *Q = ekf_p->Q;

    // We update the Covariance Matrix
    P[0] = Q[0] + f02 * (f02 * P[12] + f03 * P[17] + P[2]) + f02 * P[10] + f03 * (f02 * P[13] + f03 * P[18] + P[3]) + f03 * P[15] + P[0];
    P[1] = f02 * P[11] + f03 * P[16] + f12 * (f02 * P[12] + f03 * P[17] + P[2]) + f13 * (f02 * P[13] + f03 * P[18] + P[3]) + P[1];
    P[2] = f02 * P[12] + f03 * P[17] + f24 * (f02 * P[14] + f03 * P[19] + P[4]) + P[2];
    P[3] = f02 * P[13] + f03 * P[18] + P[3];
    P[4] = f02 * P[14] + f03 * P[19] + P[4];
    P[5] = f02 * (f12 * P[12] + f13 * P[17] + P[7]) + f03 * (f12 * P[13] + f13 * P[18] + P[8]) + f12 * P[10] + f13 * P[15] + P[5];
    P[6] = Q[1] + f12 * (f12 * P[12] + f13 * P[17] + P[7]) + f12 * P[11] + f13 * (f12 * P[13] + f13 * P[18] + P[8]) + f13 * P[16] + P[6];
    P[7] = f12 * P[12] + f13 * P[17] + f24 * (f12 * P[14] + f13 * P[19] + P[9]) + P[7];
    P[8] = f12 * P[13] + f13 * P[18] + P[8];
    P[9] = f12 * P[14] + f13 * P[19] + P[9];
    P[10] = f02 * (f24 * P[22] + P[12]) + f03 * (f24 * P[23] + P[13]) + f24 * P[20] + P[10];
    P[11] = f12 * (f24 * P[22] + P[12]) + f13 * (f24 * P[23] + P[13]) + f24 * P[21] + P[11];
    P[12] = Q[2] + f24 * (f24 * P[24] + P[14]) + f24 * P[22] + P[12];
    P[13] = f24 * P[23] + P[13];
    P[14] = f24 * P[24] + P[14];
    P[15] = f02 * P[17] + f03 * P[18] + P[15];
    P[16] = f12 * P[17] + f13 * P[18] + P[16];
    P[17] = f24 * P[19] + P[17];
    P[18] = Q[3] + P[18];
    P[19] = P[19];
    P[20] = f02 * P[22] + f03 * P[23] + P[20];
    P[21] = f12 * P[22] + f13 * P[23] + P[21];
    P[22] = f24 * P[24] + P[22];
    P[23] = P[23];
    P[24] = Q[4] + P[24];


}

void EKF_update_lh2_xy(KalmanFilter_t *ekf_p, float *z)
{
    // Define vector variables
    float y[2];   //residual (2,1),  y = z - h(x)
    float K[10];  // Kalman gain (5,2), K = P H.t (H P H.t + R)

    // ##################################################
    //             Calculate the Kalman Gain
    // ##################################################
    //  
    // Calculation performed is the following
    // K = P H.t (H P H.t + R)
    //
    _EKF_compute_K_lh2_xy(ekf_p, K);

    // ##################################################
    //              Calculate the Residual
    // ##################################################
    //
    // Calculation performed is the following
    // y = z - h(x)
    //
    // Observation model is:
    //
    // H =  [1, 0, 0, 0, 0]
    //      [0, 1, 0, 0, 0]
    //

    y[0] = z[0] - ekf_p->x;
    y[1] = z[1] - ekf_p->y;

    // ##################################################
    //                  Update State
    // ##################################################
    //
    // Calculations is:
    // x = x + K @ y
    ekf_p->x     += K[0] * y[0] + K[1] * y[1];
    ekf_p->y     += K[2] * y[0] + K[3] * y[1];
    ekf_p->theta += K[4] * y[0] + K[5] * y[1];
    ekf_p->V     += K[6] * y[0] + K[7] * y[1];
    ekf_p->W     += K[8] * y[0] + K[9] * y[1];

    // Constraint theta angle to [-pi, +pi]
    ekf_p->theta = _EKF_angle_overflow(ekf_p->theta);

    // ##################################################
    //              Update Posterior P
    // ##################################################
    _EKF_update_post_P_lh2_xy(ekf_p, K);

}

void EKF_update_gyro_W(KalmanFilter_t *ekf_p, float z)
{
    // Define vector variables
    float y;  // residual (1,1),  y = z - h(x)
    float K[5]; // Kalman gain (5,1), K = P H.t (H P H.t + R)

    // ##################################################
    //             Calculate the Kalman Gain
    // ##################################################
    //
    // Calculation performed is the following
    // K = P H.t (H P H.t + R)
    //
    _EKF_compute_K_gyro_W(ekf_p, K);

    // ##################################################
    //              Calculate the Residual
    // ##################################################
    //
    // Calculation performed is the following
    // y = z - h(x)
    //
    // Observation model is:
    //
    // H =  [0, 0, 0, 0, 1]
    //
    y = z - ekf_p->W;


    // ##################################################
    //                  Update State
    // ##################################################
    //
    // Calculations is:
    // x = x + K @ y
    ekf_p->x += K[0] * y;
    ekf_p->y += K[1] * y;
    ekf_p->theta += K[2] * y;
    ekf_p->V += K[3] * y;
    ekf_p->W += K[4] * y;

    // Constraint theta angle to [-pi, +pi]
    ekf_p->theta = _EKF_angle_overflow(ekf_p->theta);

    // ##################################################
    //              Update Posterior P
    // ##################################################
    _EKF_update_post_P_gyro_W(ekf_p, K);

}

void EKF_set_state(KalmanFilter_t *ekf_p,float *x0)
{

    // Copy inital state x0
    ekf_p->x     = x0[0];
    ekf_p->y     = x0[1];
    ekf_p->theta = x0[2];
    ekf_p->V     = x0[3];
    ekf_p->W     = x0[4];
}

void EKF_set_P(KalmanFilter_t *ekf_p,float *P0)
{

    // Copy the initial Covariance
    memcpy(ekf_p->P, P0, 25*4); // 25*4 becasue we are copying 25 floats.
} 

//=========================== private ==========================================

void _EKF_state_transition(KalmanFilter_t *ekf_p,float dt)
{
    // Get local copies of the state variables
    float x = ekf_p->x;
    float y = ekf_p->y;
    float theta = ekf_p->theta;
    float V = ekf_p->V;
    float W = ekf_p->W;

    // Run the state update
    ekf_p->x = x + dt * V * cos(theta);
    ekf_p->y = y + dt * V * sin(theta);
    ekf_p->theta = theta + dt * W;

    // Regularize the value of Theta between -pi/2 and pi/2
    ekf_p->theta = _EKF_angle_overflow(ekf_p->theta);
}

void _EKF_compute_K_lh2_xy(KalmanFilter_t *ekf_p, float *K)
{
    // Create a local pointer of P, for clarity (I don't want to have to write ekf_p->P seven million times)
    float *P = ekf_p->P;

    // Bring a few important values from the R matrix
    float r00 = EKF_R_LH2__0_0;
    float r11 = EKF_R_LH2__1_1;

    // Calculate K
    float denom = (r00 * r11 + r00 * P[6] + r11 * P[0] + P[0] * P[6] - P[1] * P[5]);  // Commonly appearing term

    K[0] = ((r11 + P[6]) * P[0] - P[1] * P[5]) / denom;
    K[1] = r00 * P[1] / denom;
    K[2] = r11 * P[5] / denom;
    K[3] = ((r00 + P[0]) * P[6] - P[1] * P[5]) / denom;
    K[4] = ((r11 + P[6]) * P[10] - P[5] * P[11]) / denom;
    K[5] = ((r00 + P[0]) * P[11] - P[1] * P[10]) / denom;
    K[6] = ((r11 + P[6]) * P[15] - P[5] * P[16]) / denom;
    K[7] = ((r00 + P[0]) * P[16] - P[1] * P[15]) / denom;
    K[8] = ((r11 + P[6]) * P[20] - P[5] * P[21]) / denom;
    K[9] = ((r00 + P[0]) * P[21] - P[1] * P[20]) / denom;
}

void _EKF_update_post_P_lh2_xy(KalmanFilter_t *ekf_p, float *K)
{

    // Create a local copy of P. This will help update the actual value of ekf_p->P later
    float P[25];
    memcpy(P, ekf_p->P, 25 * 4); // 25*4 becasue we are copying 25 floats.

    // Get values from the R, Noise Matrices.
    //float r00 = EKF_R_LH2__0_0;
    //float r11 = EKF_R_LH2__1_1;

    // Update the posterior  Covariance
    // Implement the following formula
    //
    // P_post = (I - K @ H) @ P
    //

    // Get repeated operations in a variable
    float temp0 = (1 - K[0]);
    float temp1 = (1 - K[3]);

    ekf_p->P[0] = temp0 * P[0] - K[1] * P[5];
    ekf_p->P[1] = temp0 * P[1] - K[1] * P[6];
    ekf_p->P[2] = temp0 * P[2] - K[1] * P[7];
    ekf_p->P[3] = temp0 * P[3] - K[1] * P[8];
    ekf_p->P[4] = temp0 * P[4] - K[1] * P[9];
    ekf_p->P[5] = temp1 * P[5] - K[2] * P[0];
    ekf_p->P[6] = temp1 * P[6] - K[2] * P[1];
    ekf_p->P[7] = temp1 * P[7] - K[2] * P[2];
    ekf_p->P[8] = temp1 * P[8] - K[2] * P[3];
    ekf_p->P[9] = temp1 * P[9] - K[2] * P[4];
    ekf_p->P[10] = -K[4] * P[0] - K[5] * P[5] + P[10];
    ekf_p->P[11] = -K[4] * P[1] - K[5] * P[6] + P[11];
    ekf_p->P[12] = -K[4] * P[2] - K[5] * P[7] + P[12];
    ekf_p->P[13] = -K[4] * P[3] - K[5] * P[8] + P[13];
    ekf_p->P[14] = -K[4] * P[4] - K[5] * P[9] + P[14];
    ekf_p->P[15] = -K[6] * P[0] - K[7] * P[5] + P[15];
    ekf_p->P[16] = -K[6] * P[1] - K[7] * P[6] + P[16];
    ekf_p->P[17] = -K[6] * P[2] - K[7] * P[7] + P[17];
    ekf_p->P[18] = -K[6] * P[3] - K[7] * P[8] + P[18];
    ekf_p->P[19] = -K[6] * P[4] - K[7] * P[9] + P[19];
    ekf_p->P[20] = -K[8] * P[0] - K[9] * P[5] + P[20];
    ekf_p->P[21] = -K[8] * P[1] - K[9] * P[6] + P[21];
    ekf_p->P[22] = -K[8] * P[2] - K[9] * P[7] + P[22];
    ekf_p->P[23] = -K[8] * P[3] - K[9] * P[8] + P[23];
    ekf_p->P[24] = -K[8] * P[4] - K[9] * P[9] + P[24];

    // Re-simmetricize the Posterior Covariance
    // Equation   (P + P.T)/2
    _EKF_P_symmetrization(ekf_p);
}

void _EKF_compute_K_gyro_W(KalmanFilter_t *ekf_p, float *K)
{
    // Create a local pointer of P, for clarity (I don't want to have to write ekf_p->P seven million times)
    float *P = ekf_p->P;

    // Bring a few important values from the R matrix
    float R = EKF_R_GYRO__0_0;

    // Calculate K
    K[0] = P[4] / (P[24] + R);
    K[1] = P[9] / (P[24] + R);
    K[2] = P[14] / (P[24] + R);
    K[3] = P[19] / (P[24] + R);
    K[4] = P[24] / (P[24] + R);
}

void _EKF_update_post_P_gyro_W(KalmanFilter_t *ekf_p, float *K)
{

    // Create a local copy of P. This will help update the actual value of ekf_p->P later
    float P[25];
    float r = EKF_R_GYRO__0_0;
    memcpy(P, ekf_p->P, 25 * 4); // 25*4 becasue we are copying 25 floats.

    // Update the posterior  Covariance
    // Implement one of the 2 following  formulas
    //
    // P_post = (I - K * H) * P * (I - K * H).T + K * R * K.T
    // 
    // or
    // 
    // P_post = (I - K @ H) @ P
    //

    // Long calculation
    // P_post = (I - K * H) * P * (I - K * H).T + K * R * K.T
    ekf_p->P[0] = -(-K[0] * P[24] + P[4]) * K[0] + (K[0] * K[0]) * r - K[0] * P[20] + P[0];
    ekf_p->P[1] = -(-K[0]*P[24] + P[4])*K[1] + K[0]*K[1]*r - K[0]*P[21] + P[1];   
    ekf_p->P[2] = -(-K[0]*P[24] + P[4])*K[2] + K[0]*K[2]*r - K[0]*P[22] + P[2];   
    ekf_p->P[3] = -(-K[0]*P[24] + P[4])*K[3] + K[0]*K[3]*r - K[0]*P[23] + P[3];   
    ekf_p->P[4] = (1 - K[4])*(-K[0]*P[24] + P[4]) + K[0]*K[4]*r;
    ekf_p->P[5] = ekf_p->P[1];
    ekf_p->P[6] = -(-K[1] * P[24] + P[9]) * K[1] + (K[1] * K[1]) * r - K[1] * P[21] + P[6];
    ekf_p->P[7] = -(-K[1]*P[24] + P[9])*K[2] + K[1]*K[2]*r - K[1]*P[22] + P[7];   
    ekf_p->P[8] = -(-K[1]*P[24] + P[9])*K[3] + K[1]*K[3]*r - K[1]*P[23] + P[8];   
    ekf_p->P[9] = (1 - K[4])*(-K[1]*P[24] + P[9]) + K[1]*K[4]*r;
    ekf_p->P[10] = ekf_p->P[2];
    ekf_p->P[11] = ekf_p->P[7];
    ekf_p->P[12] = -(-K[2] * P[24] + P[14]) * K[2] + (K[2] * K[2]) * r - K[2] * P[22] + P[12];
    ekf_p->P[13] = -(-K[2]*P[24] + P[14])*K[3] + K[2]*K[3]*r - K[2]*P[23] + P[13];
    ekf_p->P[14] = (1 - K[4])*(-K[2]*P[24] + P[14]) + K[2]*K[4]*r;
    ekf_p->P[15] = ekf_p->P[3];
    ekf_p->P[16] = ekf_p->P[8];
    ekf_p->P[17] = ekf_p->P[13];
    ekf_p->P[18] = -(-K[3] * P[24] + P[19]) * K[3] + (K[3] * K[3]) * r - K[3] * P[23] + P[18];
    ekf_p->P[19] = (1 - K[4])*(-K[3]*P[24] + P[19]) + K[3]*K[4]*r;
    ekf_p->P[20] = ekf_p->P[4];
    ekf_p->P[21] = ekf_p->P[9];
    ekf_p->P[22] = ekf_p->P[14];
    ekf_p->P[23] = ekf_p->P[19];
    ekf_p->P[24] = (1 - K[4]) * (1 - K[4]) * P[24] + (K[4] * K[4]) * r;

    // Short Equation
    // P_post = (I - K @ H) @ P
    //
    // ekf_p->P[0] = -K[0] * P[20] + P[0];
    // ekf_p->P[1] = -K[0] * P[21] + P[1];
    // ekf_p->P[2] = -K[0] * P[22] + P[2];
    // ekf_p->P[3] = -K[0] * P[23] + P[3];
    // ekf_p->P[4] = -K[0] * P[24] + P[4];
    // ekf_p->P[5] = -K[1] * P[20] + P[5];
    // ekf_p->P[6] = -K[1] * P[21] + P[6];
    // ekf_p->P[7] = -K[1] * P[22] + P[7];
    // ekf_p->P[8] = -K[1] * P[23] + P[8];
    // ekf_p->P[9] = -K[1] * P[24] + P[9];
    // ekf_p->P[10] = -K[2] * P[20] + P[10];
    // ekf_p->P[11] = -K[2] * P[21] + P[11];
    // ekf_p->P[12] = -K[2] * P[22] + P[12];
    // ekf_p->P[13] = -K[2] * P[23] + P[13];
    // ekf_p->P[14] = -K[2] * P[24] + P[14];
    // ekf_p->P[15] = -K[3] * P[20] + P[15];
    // ekf_p->P[16] = -K[3] * P[21] + P[16];
    // ekf_p->P[17] = -K[3] * P[22] + P[17];
    // ekf_p->P[18] = -K[3] * P[23] + P[18];
    // ekf_p->P[19] = -K[3] * P[24] + P[19];
    // ekf_p->P[20] = (1 - K[4]) * P[20];
    // ekf_p->P[21] = (1 - K[4]) * P[21];
    // ekf_p->P[22] = (1 - K[4]) * P[22];
    // ekf_p->P[23] = (1 - K[4]) * P[23];
    // ekf_p->P[24] = (1 - K[4]) * P[24];

    // Re-simmetricize the Posterior Covariance
    // Equation   (P + P.T)/2
    // _EKF_P_symmetrization(ekf_p);
}

void _EKF_P_symmetrization(KalmanFilter_t *ekf_p)
{
    // Re-simmetricize the Posterior Covariance
    // Equation   (P + P.T)/2

    ekf_p->P[1] = 0.5 * (ekf_p->P[1] + ekf_p->P[5]);
    ekf_p->P[2] = 0.5 * (ekf_p->P[2] + ekf_p->P[10]);
    ekf_p->P[3] = 0.5 * (ekf_p->P[3] + ekf_p->P[15]);
    ekf_p->P[4] = 0.5 * (ekf_p->P[4] + ekf_p->P[20]);
    ekf_p->P[5] = ekf_p->P[1];
    ekf_p->P[7] = 0.5 * (ekf_p->P[7] + ekf_p->P[11]);
    ekf_p->P[8] = 0.5 * (ekf_p->P[8] + ekf_p->P[16]);
    ekf_p->P[9] = 0.5 * (ekf_p->P[9] + ekf_p->P[21]);
    ekf_p->P[10] = ekf_p->P[2];
    ekf_p->P[11] = ekf_p->P[7];
    ekf_p->P[13] = 0.5 * (ekf_p->P[13] + ekf_p->P[17]);
    ekf_p->P[14] = 0.5 * (ekf_p->P[14] + ekf_p->P[22]);
    ekf_p->P[15] = ekf_p->P[3];
    ekf_p->P[16] = ekf_p->P[8];
    ekf_p->P[17] = ekf_p->P[13];
    ekf_p->P[19] = 0.5 * (ekf_p->P[19] + ekf_p->P[23]);
    ekf_p->P[20] = ekf_p->P[4];
    ekf_p->P[21] = ekf_p->P[9];
    ekf_p->P[22] = ekf_p->P[14];
    ekf_p->P[23] = ekf_p->P[19];
}

float _EKF_angle_overflow(float rad)
{
    // Calculate the equivalent angle between -2π and +2π
    float angle_mod = fmod(rad, M_2PI);

    // If the angle is less than -π, add 2π to make it positive
    if (angle_mod < -M_PI)
    {
        angle_mod += M_2PI;
    }
    // If the angle is greater than +π, subtract 2π to make it negative
    else if (angle_mod > M_PI)
    {
        angle_mod -= M_2PI;
    }

    return angle_mod;
}