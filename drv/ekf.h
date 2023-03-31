#ifndef __EKF_H
#define __EKF_H

#include <math.h>


//=========================== defines ==========================================

#define RADIANS_TO_DEGREES(radians) ((radians)*180 / M_PI)
#define DEGREES_TO_RADIANS(degrees) ((degrees)*M_PI / 180)

// Define Initial Covariance P
#define EKF_P0_X 1.0f
#define EKF_P0_Y 1.0f
#define EKF_P0_THETA 1.0f
#define EKF_P0_V 1.0f
#define EKF_P0_W 1.0f

// Define the Q diagonal
#define EKF_Q_X 1.0f
#define EKF_Q_Y 1.0f
#define EKF_Q_THETA 0.1f
#define EKF_Q_V 1.0f
#define EKF_Q_W 1.0f

// Define Sensor Noise Matrices
// LH2
#define EKF_R_LH2__0_0 1.0f
#define EKF_R_LH2__1_1 1.0f
// Gyro
#define EKF_R_GYRO__0_0 0.0000014884f

typedef struct {
    // Define the state variables
    float x;
    float y;
    float theta;    // rad
    float V;
    float W;

    // Define the important matrices
    float P[25];
    float Q[5];  // For Q we keep only the diagonal
    // The R and H matrices are stored in their respective update function.
} KalmanFilter_t;

//=========================== Public  ==========================================

/**
 * @brief   Initialize the Extended Kalman Filter
 *
 * @param[in] ekf_p     Pointer to the ekf struct
 * @param[in] x0        Initial state of the filter. [x, y, theta, V, W]
 * @param[in] P0        Diagonal values of the initial Covariance Matrix. [Px, Py, Ptheta, Pv, Pw]
 */
void EKF_init(KalmanFilter_t *ekf_p, float *x0 ,float *P0);

/**
 * @brief   Run the prediction step of the Kalman filter
 *
 * @param[in] ekf_p     Pointer to the ekf struct
 * @param[in] dt        Delta-T, the timestep. in sec
 */
void EKF_predict(KalmanFilter_t *ekf_p, float dt);

/**
 * @brief  Update the kalman filter with an LH2 measurement [x,y]
 *
 * @param[in] ekf_p     Pointer to the ekf struct
 * @param[in] z         LH2 measurement, size [x,y], in cm 
 */
void EKF_update_lh2_xy(KalmanFilter_t *ekf_p, float *z);

/**
 * @brief  Update the kalman filter with an Gyro measurement [Theta]
 *
 * @param[in] ekf_p     Pointer to the ekf struct
 * @param[in] z         LH2 measurement, size [Theta], in rad/s
 */
void EKF_update_gyro_W(KalmanFilter_t *ekf_p, float z);

/**
 * @brief  Set the state vector [x,y,theta,V,W] of the Kalman filter
 *
 * @param[out] ekf_p    Pointer to the ekf struct to be updated
 * @param[in]  x0       New state to copy into the ekf structure
 */
void EKF_set_state(KalmanFilter_t *ekf_p, float *x0);

/**
 * @brief  Set the Covariance matrix P[25] of the Kalman filter
 *
 * @param[out] ekf_p    Pointer to the ekf struct to be updated
 * @param[in]  P0       New Covariance matrix to copy into the ekf structure
 */
void EKF_set_P(KalmanFilter_t *ekf_p, float *P0);

//=========================== Private ==========================================

/**
 * @brief  Propagate the non-linear model 1 timestep into the future [Theta]
 *
 * @param[in]  ekf_p     Pointer to the ekf struct
 * @param[in]  dt        Delta-T, the timestep. in sec
 */
void _EKF_state_transition(KalmanFilter_t *ekf_p, float dt);

/**
 * @brief  Calculate the Kalman Gain for an LH2 measurement [x,y]
 *
 * @param[in]  ekf_p     Pointer to the ekf struct
 * @param[out] K         Kalman Gain matrix (5,2)
 */
void _EKF_compute_K_lh2_xy(KalmanFilter_t *ekf_p, float *K);

/**
 * @brief  Calculate the posterior covariance after an LH2 measurement [x,y]
 *         
 *
 * @param[in]  ekf_p     Pointer to the ekf struct
 * @param[in] K         Kalman Gain matrix (5,2)
 */
void _EKF_update_post_P_lh2_xy(KalmanFilter_t *ekf_p, float *K);

/**
 * @brief  Calculate the Kalman Gain for an Gyro measurement [W]
 *
 * @param[in]  ekf_p     Pointer to the ekf struct
 * @param[out] K         Kalman Gain matrix (5,1)
 */
void _EKF_compute_K_gyro_W(KalmanFilter_t *ekf_p, float *K);

/**
 * @brief  Calculate the posterior covariance after an Gryo measurement [W]
 *
 *
 * @param[in]  ekf_p     Pointer to the ekf struct
 * @param[in]  K         Kalman Gain matrix (5,1)
 */
void _EKF_update_post_P_gyro_W(KalmanFilter_t *ekf_p, float *K);

/**
 * @brief  Make the Covariance matrix P symmetric
 *
 * @param[in]  ekf_p     Pointer to the ekf struct
 */
void _EKF_P_symmetrization(KalmanFilter_t *ekf_p);

/**
 * @brief  Make sure the angle Theta is always between -180deg and +180deg.
 *         Returns the constrained angle
 *
 * @param[in]  rad        angle to constraint
 */
float _EKF_angle_overflow(float rad);


#endif