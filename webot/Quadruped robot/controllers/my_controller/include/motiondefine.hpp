#include <stdio.h>
#include <stdlib.h>
// #include <sys/mman.h>
#include <sys/time.h>
#include <vector>
#include <iostream>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <Eigen/SVD>
#include <fstream>
#include <webots/Robot.hpp>
#include <webots/Device.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/GPS.hpp>


using namespace std;
using namespace Eigen;
using namespace webots;

#ifndef MotionControl_H_H
#define MotionControl_H_H
namespace webots{
class PositionSensor;
class Motor;
class TouchSensor;
class InertialUnit;
class DistanceSensor;
class GPS;
}
class MotionControl
{
    public:
        char *robot;
        Vector<float, 12> jointPresentPos;  // present joint 0-11
        
        // Vector<float, 12> jointCmdPos;  // command joint 0-11

        float timeForGaitPeriod;  // The time of the whole period
        float timePeriod;  // The time of one period
        float timePresent;  
        float timeOneSwingPeriod;  // The swing time for diag legs
        Matrix<float, 4, 2> timeForStancePhase;  // startTime, endTime: LF, RF, LH, RH
        Vector<float, 3> targetCoMVelocity;  // X, Y , alpha c in world cordinate
        Vector<float, 3> presentCoMVelocity;  // X, Y , alpha c in world cordinate
        Matrix<float, 4, 3> targetCoMPosition;  // X, Y , alpha in world cordinate
        float yawVelocity;   // yaw velocity from imu
        Vector<bool, 4> stanceFlag;  // True, False: LF, RF, LH, RH
        Vector<float, 4> timePresentForSwing;
        float L1, L2, L3;  // The length of L
        float width, length;
        Matrix<float, 4, 2> shoulderPos;  // X-Y: LF, RF, LH, RH
        Matrix<float, 4, 3> stancePhaseStartPos;
        Matrix<float, 4, 3> stancePhaseEndPos;
        Matrix<float, 4, 3> legPresentPos;  // present X-Y-Z: LF, RF, LH, RH in Shoulder cordinate
        Matrix<float, 4, 3> legCmdPos;  // command X-Y-Z: LF, RF, LH, RH in Shoulder cordinate
        Matrix<float, 4, 3> leg2CoMPrePos;  // present X-Y-Z: LF, RF, LH, RH in CoM cordinate
        Matrix<float, 4, 3> leg2CoMCmdPos;  // command X-Y-Z: LF, RF, LH, RH in CoM cordinate
        Vector<float, 12> joint_pre_pos;  // present joint angle 0-11
        Vector<float, 12> joint_cmd_pos;  // command joint angle 0-11
        Vector<float, 12> jointPresentVel;  // present motor 0-11
        Matrix<float, 1, 3> swingPhaseVelocity;
        Vector<float, 6> legPresentVel;
        Vector<float, 3> imuVel;
        Vector<float, 6> legCmdVel;
        Matrix<float, 6, 6> A;     //VMC
        Vector<float, 6> B;        //VMC
        Matrix<float, 4, 9>jacobian; 
        float z_pre_vel;         // z present velocity from imu
        float jointCmdPos[12];  // command motor 0-11
        float jointCmdPosLast[12];
        float jointCmdVel[12];
        float motorTorque[1];
        float motorInitPos[12];   // init motor angle of quadruped state
        float pid_motortorque[12];
        float jacobian_motortorque[12]; //motor torque in VMC
        float motorCmdTorque[12];
        Vector<float, 3> imu_num;        //VMC
        bool initFlag;
        
        
        char positionName[12][22] = {"LFL0_position sensor", "LFL1_position sensor", "LFL2_position sensor", "RFL0_position sensor", "RFL1_position sensor", "RFL2_position sensor", "LBL0_position sensor", "LBL1_position sensor", "LBL2_position sensor", "RBL0_position sensor", "RBL1_position sensor", "RBL2_position sensor"};
        char motorName[12][22] = {"LFL0_rotational motor", "LFL1_rotational motor", "LFL2_rotational motor", "RFL0_rotational motor", "RFL1_rotational motor", "RFL2_rotational motor", "LBL0_rotational motor", "LBL1_rotational motor", "LBL2_rotational motor", "RBL0_rotational motor", "RBL1_rotational motor", "RBL2_rotational motor"};
        char touchsensorName[4][16] = {"LF_touch sensor", "RF_touch sensor", "LB_touch sensor", "RB_touch sensor"};
        char stateval[4] = {'a', 'b', 'c', 'd'};
        char distanceSensorName[2][20] = {"distance sensor x", "distance sensor y"};
        PositionSensor *Ps[12];
        Motor *Tor[12];
        TouchSensor *Ts[12];
        DistanceSensor *Ds[2];
        GPS *gps;
        Vector<float, 3> force_LF;
        Vector<float, 3> force_RF;
        Vector<float, 3> force_LH;
        Vector<float, 3> force_RH;
        InertialUnit *imu;
        float Start_time = 0.0;
        float fly_time;
        int swingFlag; // 0 is 2 and 3 swing ;1 is 1 and 4 swing
        Vector<float, 4>state;//present leg touch sensor
        int state_val;
        Vector<float, 12> jacobian_torque;
        Vector<float, 13> X;//state of body
        Vector<float, 65> XX;//state of body
        Vector<float, 13> X_ref;//target state of body
        float present_x = 0.0;
        float present_y = 0.0;
        float present_z = 0.0;
        float present_vx = 0.0;
        float present_vy = 0.0;
        float present_vz = 0.0;
        Vector<float, 3> last_leg_position = {0.0256826, 1.97193e-06, -0.641148};
        Vector<float, 3> present_leg_position;
        Vector<float, 2> distance;
        Vector<float, 3>global_xyz;
        Vector<float, 3>global_last_xyz;
        Vector<float, 3>mark_xyz;
        
    
        // the parameters of creeping gait
        // float motIniPoCreep[12];    // init motor angle of creeping gait
        // float H_onestep;  // The height of one step
        // float Yaw_rad;      // The rad of yaw angle
        // float k1,k2,k3;
        // float L_diag;  // half of body diagonal size
        // float beta_diag, alpha_diag; // structural angle of the body
        // float v_body_x, v_body_y;    // the velocity of CoM
        // float v_leg[4][2];  // the velocity of 4 legs
        // float endPosition[4][2];  // the final feet position after one gait cycle
        // Matrix<float, 4, 3> initPosS2L;  // init position from Shoulder to Leg
        // Matrix<float, 4, 2> initPosC2L;  // init position from CoM to Leg
        // Matrix<float, 4, 2> initPosC2S;  // init position from CoM to Shoulder
        // float p_w2c[4][3];  // The relative position from world to CoM
        // float p_c2s[4][3];  // The relative position from CoM to Shoulder
        // float p_w2f[4][3];  // The relative position from world to leg
        // float yawCreep;  // The command yaw of creeping gait
        // float L1_creep, L2_creep, L3_creep;  // The length of creeping legs
        // Matrix<float, 4, 3> legCmdVia;  // The intermediate variables for legCmdPos calculation of creeping gait
        
        void setInitPos(Matrix<float, 4, 3> initPosition);
        void setCoMVel(Vector<float, 3> tCV);
        void nextStep();
        void inverseKinematics();   // standing state
        void setInitial();
        void updateState();
        void creepingGait(float X_tar, float Y_tar, float Yaw_tar);   // the creeping gait from gecko_inspired
        void creepingIK();          // The inverse kinematics of creeping gait
        void standing2creeping();   // transfer from standing to creeping gait
        void creeping2standing();   // transfer from creeping to standing gait
        void forwardKinematics();
        void jacobians();
        void vmc();
        void pid();
        void nextstep_timemachine();
                                                                                                                                                                                                                                                                                                                                                  
        void MotionContr(float tP, float tFGP, Matrix<float, 4, 2> tFSP,  Robot *robot);
        void Sensor_update();

        void Setjoint();
        void State_start();
        void state_judgement();
        void stance_MPC();
        void swing_VMC();
        void present_pxyz();

};
#endif