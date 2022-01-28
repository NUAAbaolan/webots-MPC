#include <motiondefine.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <Eigen/SVD>
#include <Eigen/MatrixFunctions>
#include <math.h>
#include <qpOASES.hpp>
#define PI 3.1415926
#define TIME_STEP 10

using namespace std;
using namespace Eigen;
void MotionControl::MotionContr(float tP, float tFGP, Matrix<float, 4, 2> tFSP, Robot *robot)
{
    initFlag = false;
    timePeriod = tP;
    timeForGaitPeriod = tFGP;
    timeForStancePhase = tFSP;
    timePresent = 0.0;
    timePresentForSwing << 0.0, 0.0, 0.0, 0.0;
    targetCoMVelocity << 0.0, 0.0, 0.0;
    L1 = 0.4;
    L2 = 0.4;
    L3 = 0.0;
    width = 0.6;
    length = 0.48;
    shoulderPos << width/2, length/2, width/2, -length/2, -width/2, length/2, -width/2, -length/2;  // X-Y: LF, RF, LH, RH
    for (int i=0; i<12; i++)
    {
      Ps[i] = robot->getPositionSensor(positionName[i]);
      Ps[i]->enable(TIME_STEP);
      Tor[i] = robot->getMotor(motorName[i]);
    }
    for (int i=0; i<4; i++)
    {
      Ts[i] = robot->getTouchSensor(touchsensorName[i]);
      Ts[i]->enable(TIME_STEP);
    }
    for (int i = 0; i < 2; i++)
    {
        Ds[i] = robot->getDistanceSensor(distanceSensorName[i]);
        Ds[i]->enable(TIME_STEP);
    }
   imu = robot->getInertialUnit("inertial unit");
   imu->enable(TIME_STEP);
   gps = robot->getGPS("global_gps");
   gps->enable(TIME_STEP);
}
void MotionControl::Sensor_update()
{
    Vector<float, 12> jointLastPos;
    for (int i = 0; i < 12; i++)
    {
        jointLastPos(i) = jointPresentPos(i);
    }
    Vector<float, 3> Lastimu_num;
    for (int i = 0; i < 3; i++)
    {
        Lastimu_num(i) = imu_num(i);
    }
    global_last_xyz = global_xyz;
    for (int i=0; i<12; i++)
    {
        jointPresentPos(i) = Ps[i]->getValue();
    }
    // float temp_touch1 = Ts[0]->getValue();
    // float temp_touch2 = Ts[1]->getValue();
    // float temp_touch3 = Ts[2]->getValue();
    // float temp_touch4 = Ts[3]->getValue();
    // cout <<"touch force"<< Ts[0]->getValues()[0]<<" "<< Ts[0]->getValues()[1]<<" "<<Ts[0]->getValues()[2]<< endl;
    force_LF << Ts[0]->getValues()[0], Ts[0]->getValues()[2], Ts[0]->getValues()[1];
    force_RF << Ts[1]->getValues()[0], Ts[1]->getValues()[2], Ts[1]->getValues()[1];
    force_LH << Ts[2]->getValues()[0], Ts[2]->getValues()[2], Ts[2]->getValues()[1];
    force_RH << Ts[3]->getValues()[0], Ts[3]->getValues()[2], Ts[3]->getValues()[1];
    // cout <<"touch force1：   "<< force_LF.transpose()<< endl;
    // cout <<"touch force2：   "<< force_RF.transpose()<< endl;
    // cout <<"touch force3：   "<< force_LH.transpose()<< endl;
    // cout <<"touch force4：   "<< force_RH.transpose()<< endl;
    // state << temp_touch1, temp_touch2, temp_touch3, temp_touch4;
    for (int i = 0; i < 3; i++)
    {
        imu_num(i) = imu->getRollPitchYaw()[i];
    }
    for (int i = 0; i < 12; i++)
    {
        jointPresentVel(i) = (jointPresentPos(i) - jointLastPos(i))/ timePeriod;
    }

    for (int i = 0; i < 3; i++)
    {
        imuVel(i) = (imu_num(i) - Lastimu_num(i)) / timePeriod;
    }
    for (int i = 0; i < 2; i++)
    {
        distance(i) = Ds[i]->getValue();
    }
    global_xyz << gps->getValues()[2], gps->getValues()[0], gps->getValues()[1];
}
// void MotionControl::state_judgement()
// {
    // Vector<float, 4> temp_4leg;
    // temp_4leg << 1, 1, 1, 1;
    // Vector<float, 4> temp_3leg1;
    // temp_3leg1 << 0, 1, 1, 1;
    // Vector<float, 4> temp_3leg2;
    // temp_3leg2 << 1, 0, 1, 1;
    // Vector<float, 4> temp_3leg3;
    // temp_3leg3 << 1, 1, 0, 1;
    // Vector<float, 4> temp_3leg4;
    // temp_3leg4 << 1, 1, 1, 0;
    // Vector<float, 4> temp_2leg1;
    // temp_2leg1 << 1, 0, 0, 1;
    // Vector<float, 4> temp_2leg2;
    // temp_2leg2 << 0, 1, 1, 0;
    // if (state == temp_4leg)
      // state_val = 1;
    // else if(state == temp_3leg1)
      // state_val = 2;
    // else if(state == temp_3leg2)
      // state_val = 3;
    // else if(state == temp_3leg3)
      // state_val = 4;
    // else if(state == temp_3leg4)
      // state_val = 5;
    // else if(state == temp_2leg1)
      // state_val = 6;
    // else if(state == temp_2leg2)
      // state_val = 7;
// }
void MotionControl::Setjoint()
{
    for (int i= 0 ; i<12; i++)
    {
        if (jacobian_torque(i) < -2000 || jacobian_torque(i) > 2000)
        jacobian_torque(i) = 0.0;
    }
    Vector<float, 12> temp_jointCmdPos, temp_jointCmdVel;
    for(uint8_t joints=0; joints<12; joints++)
    {
        temp_jointCmdPos(joints) = jointCmdPos[joints];
        temp_jointCmdVel(joints) = jointCmdVel[joints];
    }
    Vector<float, 12> temp_motorCmdTorque;
    temp_motorCmdTorque = 0.0* (temp_jointCmdPos - jointPresentPos) + 0.0* (temp_jointCmdVel - jointPresentVel);
    jacobian_torque += temp_motorCmdTorque;
    for (int i = 0; i < 12; i++)
    {
        Tor[i]->setTorque(jacobian_torque(i));
    }
}

void MotionControl::setInitPos(Matrix<float, 4, 3> initPosition)
{
    stancePhaseStartPos = initPosition;
    stancePhaseEndPos = initPosition;
    legPresentPos = initPosition;
    legCmdPos = initPosition;
    targetCoMPosition << 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0;
}

void MotionControl::setCoMVel(Vector<float, 3> tCV)
{
    targetCoMVelocity = tCV;
}

void MotionControl::State_start()
{
    Sensor_update();
    for (int i = 0; i < 12 ;i++)
    {
        jointCmdPos[i] = jointPresentPos(i);
    }
    for (int i= 0; i<12; i++)
    {
        Tor[i]->setPosition(jointCmdPos[i]);
    }
    Start_time += timePeriod;
}
void MotionControl::present_pxyz()
{
    if (swingFlag == 0)
        present_leg_position = (legPresentPos.row(0) + legPresentPos.row(3)) * 0.5;
    else
        present_leg_position = (legPresentPos.row(1) + legPresentPos.row(2)) * 0.5;
    Vector<float, 3> temp_posiiton;
    temp_posiiton = present_leg_position - last_leg_position;
    last_leg_position = present_leg_position;
    // present_x = -temp_posiiton(0) + present_x;
    // present_y = -temp_posiiton(1) + present_y;
    // present_x = distance(0) -1.0297;
    // present_y = 1.39323 - distance(1);
    present_x = global_xyz(0) - mark_xyz(0);
    present_y = global_xyz(1) - mark_xyz(1);
    present_z = global_xyz(2) - mark_xyz(2);
    present_vx = (global_xyz(0) - global_last_xyz(0)) / timePeriod;
    present_vy = (global_xyz(1) - global_last_xyz(1)) / timePeriod;
    present_vz = (global_xyz(2) - global_last_xyz(2)) / timePeriod;
}
void MotionControl::stance_MPC()
{
    Matrix<float, 13, 13> A_mpc;
    Matrix<float, 13, 6> B_mpc;
    Matrix<float, 6, 6> jacobian_Matrix;
    Matrix<float, 3, 3> I;
    Matrix<float, 3, 3> R;
    R << cos(imu_num[2])/cos(imu_num[0]), sin(imu_num[2])/cos(imu_num[0]), 0,
         -sin(imu_num[2]), cos(imu_num[2]), 0,
         cos(imu_num[2]) * tan(imu_num[0]), sin(imu_num[2]) * tan(imu_num[0]), 1;
    float m = 50;//50.5
    //I 为转动惯量
    I << (0.2 * 0.2 + 0.4 * 0.4) * m/12, 0, 0,
         0, (0.2 * 0.2 + 0.8 * 0.8) * m/12, 0,
         0, 0, (0.8 * 0.8 + 0.4 * 0.4) * m/12;
    Matrix<float, 3, 3> R_z;
    Matrix<float, 3, 3> R_y;
    Matrix<float, 3, 3> R_x;
    R_z << cos(imu_num[2]), sin(imu_num[2]), 0,
           -sin(imu_num[2]), cos(imu_num[2]), 0,
           0, 0, 1;
    R_y << cos(imu_num[0]), 0, -sin(imu_num[0]),
           0, 1, 0,
           sin(imu_num[0]), 0, cos(imu_num[0]);
    R_x << 1, 0, 0,
           0, cos(imu_num[1]), sin(imu_num[0]),
           0, -sin(imu_num[1]), cos(imu_num[1]);
    Matrix<float, 3, 3> R_sum;
    R_sum = R_z * R_y * R_x;
    I = R_sum * I * R_sum.transpose();
    //dot x = A * x + B * u
    A_mpc = MatrixXf::Zero(13, 13);
    A_mpc.block(0,6,3,3) = R;
    A_mpc.block(3,9,3,3) = MatrixXf::Identity(3, 3);
    A_mpc(11,12) = 1;
    A_mpc(11,9) = 4;
    cout << "state:   "<< swingFlag << endl;
    Vector<float, 3>temp_presentCmdVel;
    Vector<float, 3>temp_presentCmdVel1;
    // float present_x;
    // float present_y;
    // float present_z;
    if (swingFlag == 0)
    {
        float xf = leg2CoMPrePos(0, 0);// - 0.1;//+ 0.01;
        float yf = leg2CoMPrePos(0, 1);// - 0.000155;
        float zf = leg2CoMPrePos(0, 2);
        float xh = leg2CoMPrePos(3, 0);// - 0.1;
        float yh = leg2CoMPrePos(3, 1);// - 0.000155;//- 0.000155;
        float zh = leg2CoMPrePos(3, 2);
        B_mpc = MatrixXf::Zero(13, 6);
        Matrix<float, 3, 3> temp_F;
        Matrix<float, 3, 3> temp_H;
        temp_F << 0, -zf, yf,
                  zf, 0, -xf,
                  -yf, xf, 0;
        temp_H << 0, -zh, yh,
                  zh, 0, -xh,
                  -yh, xh, 0;
        B_mpc.block(6,0,3,3) = I.inverse() * temp_F ;
        B_mpc.block(6,3,3,3) = I.inverse() * temp_H ;
        B_mpc.block(9,0,3,3) = (1/m) * MatrixXf::Identity(3, 3);
        B_mpc.block(9,3,3,3) = (1/m) * MatrixXf::Identity(3, 3);
        Matrix<float, 3, 3> temp_jacobian;
        temp_jacobian << jacobian(3 ,0), jacobian(3 ,1), jacobian(3 ,2),
                         jacobian(3 ,3), jacobian(3 ,4), jacobian(3 ,5),
                         jacobian(3 ,6), jacobian(3 ,7), jacobian(3 ,8);
                         Matrix<float, 3, 3> temp_jacobian1;
        temp_jacobian1 << jacobian(0 ,0), jacobian(0 ,1), jacobian(0 ,2),
                         jacobian(0 ,3), jacobian(0 ,4), jacobian(0 ,5),
                         jacobian(0 ,6), jacobian(0 ,7), jacobian(0 ,8);
        temp_presentCmdVel1 = temp_jacobian1 * jointPresentVel.segment(0, 3);
        temp_presentCmdVel = temp_jacobian * jointPresentVel.segment(9, 3);
        temp_presentCmdVel = (temp_presentCmdVel + temp_presentCmdVel1) * 0.5;
        // present_x = -legPresentPos(3, 0) + 0.0256827;
        // present_y = -legPresentPos(3, 1) + 1.96289e-06;
        // present_z = -legPresentPos(3, 2) - 0.641149;
    }
    else
    {
        float xf = leg2CoMPrePos(1, 0);// -0.1;//+ 0.01;//0.00132
        float yf = leg2CoMPrePos(1, 1);// - 0.000155;
        float zf = leg2CoMPrePos(1, 2);
        float xh = leg2CoMPrePos(2, 0);// -0.1;//+ 0.01;
        float yh = leg2CoMPrePos(2, 1);// - 0.000155;
        float zh = leg2CoMPrePos(2, 2);
        B_mpc = MatrixXf::Zero(13, 6);
        Matrix<float, 3, 3> temp_F;
        Matrix<float, 3, 3> temp_H;
        temp_F << 0, -zf, yf,
                  zf, 0, -xf,
                  -yf, xf, 0;
        temp_H << 0, -zh, yh,
                  zh, 0, -xh,
                  -yh, xh, 0;
        B_mpc.block(6,0,3,3) = I.inverse() * temp_F;
        B_mpc.block(6,3,3,3) = I.inverse() * temp_H;
        B_mpc.block(9,0,3,3) = (1/m) * MatrixXf::Identity(3, 3);
        B_mpc.block(9,3,3,3) = (1/m) * MatrixXf::Identity(3, 3);
        Matrix<float, 3, 3> temp_jacobian;
        temp_jacobian << jacobian(2 ,0), jacobian(2 ,1), jacobian(2 ,2),
                         jacobian(2 ,3), jacobian(2 ,4), jacobian(2 ,5),
                         jacobian(2 ,6), jacobian(2 ,7), jacobian(2 ,8);
        Matrix<float, 3, 3> temp_jacobian1;
        temp_jacobian1 << jacobian(1 ,0), jacobian(1 ,1), jacobian(1 ,2),
                         jacobian(1 ,3), jacobian(1 ,4), jacobian(1 ,5),
                         jacobian(1 ,6), jacobian(1 ,7), jacobian(1 ,8);
        temp_presentCmdVel1 = temp_jacobian1 * jointPresentVel.segment(3, 3);
        temp_presentCmdVel = temp_jacobian * jointPresentVel.segment(6, 3);
        temp_presentCmdVel = (temp_presentCmdVel + temp_presentCmdVel1) * 0.5;
        // present_x = -legPresentPos(2, 0) + 0.0256827;
        // present_y = -legPresentPos(2, 1) + 1.96289e-06;
        // present_z = -legPresentPos(2, 2) - 0.641149;
    }
    static float target_taoz = 0.0;
    static float target_x = 0.0;
    static float target_y = 0.0;
    target_taoz = target_taoz + targetCoMVelocity[2] * timePeriod;
    target_x = target_x + targetCoMVelocity[0] * timePeriod;
    target_y = target_y + targetCoMVelocity[1] * timePeriod;
    Vector<float, 3> target_tao;
    target_tao << 0, 0, target_taoz;
    X_ref.head(3) = target_tao;
    Vector<float, 3> target_dottao;
    target_dottao << 0, 0, targetCoMVelocity[2];
    X_ref.segment(6, 3)= R.inverse() * target_dottao;
    X_ref.segment(3, 3) << target_x, target_y, 0;
    X_ref.tail(4) << targetCoMVelocity[0], targetCoMVelocity[1], 0, -9.81;
    // static float present_x = 0.0;
    // static float present_y = 0.0;
    //static float present_z = 0.0;
    presentCoMVelocity[0] = -temp_presentCmdVel[0];
    presentCoMVelocity[1] = -temp_presentCmdVel[1];
    presentCoMVelocity[2] = -temp_presentCmdVel[2];
    static float present_z_vel = 0.0;
    static float present_x_vel = 0.0;
    static float present_y_vel = 0.0;
    if (presentCoMVelocity[2] < 0.1 && presentCoMVelocity[2] > -0.1)
        present_z_vel = presentCoMVelocity[2];
    if (presentCoMVelocity[0] < targetCoMVelocity[0] + 0.5 && presentCoMVelocity[0] > - targetCoMVelocity[0] - 0.5)
        present_x_vel = presentCoMVelocity[0];
    if (presentCoMVelocity[1] < 0.5 && presentCoMVelocity[1] > -0.5)
        present_y_vel = presentCoMVelocity[1];
    // present_x = presentCoMVelocity[0] * timePeriod + present_x;
    // present_y = presentCoMVelocity[1] * timePeriod + present_y;
    //present_z = presentCoMVelocity[2] * timePeriod + present_z;
    X.head(3) << imu_num[1], imu_num[0], imu_num[2];
    X.segment(3, 3) << present_x, present_y, present_z;
    Vector<float, 3> temp_imuVel;
    temp_imuVel << imuVel[1], imuVel[0], imuVel[2];
    X.segment(6, 3) = R.inverse() * temp_imuVel;
    X.tail(4) << present_vx, present_vy, present_vz, -9.81;
    Matrix<float, 19, 19> temp_AB_mpc;
    temp_AB_mpc.block(0, 0, 13, 13) = A_mpc;
    temp_AB_mpc.block(0, 13, 13, 6) = B_mpc;
    temp_AB_mpc.block(13, 0, 6, 19) = MatrixXf::Zero(6, 19);
    Matrix<float, 19, 19> temp_AB;
    temp_AB_mpc = timePeriod * temp_AB_mpc;
    temp_AB = temp_AB_mpc.exp();
    Matrix<float, 13, 13> temp_A;
    Matrix<float, 13, 6> temp_B;
    temp_A = temp_AB.block(0, 0, 13, 13);
    temp_B = temp_AB.block(0, 13, 13, 6);

    //5步预测
    int n = 5;
    Vector<float, 65> X_Xref;
    Matrix<float, 65, 13> A_qp;
    Matrix<float, 65, 30> B_qp;
    A_qp = MatrixXf::Zero(65,13);
    B_qp = MatrixXf::Zero(65,30);
    for (int i = 0; i < n; i++)
    {
        X_Xref.segment(13 * i, 13) = X_ref;
        A_qp.block(13 * i, 0, 13, 13) = temp_A.pow(i + 1);
        for (int j = 0; j < i + 1; j++)
        B_qp.block(13 * i, 6 * j, 13, 6) = temp_A.pow(i-j) * temp_B;
    }
    // floatWeight function L K
    Matrix<float, 65, 65> L;
    Matrix<float, 30, 30> K;
    L = MatrixXf::Identity(65, 65);
    for(int i = 0; i < 5; i++)
    {
        L(12 + i * 13, 12 + i* 13) = 0;
        L(0 + 13 *i, 0 + 13 * i) = 10;
        L(1 + 13 *i, 1 + 13 * i) = 70;
        L(2 + 13 *i, 2 + 13 * i) = 1;

        L(3 + 13 *i, 3 + 13 * i) = 120;//
        L(4 + 13 *i, 4 + 13 * i) = 350;//
        L(5 + 13 *i, 5 + 13 * i) = 90;//

        L(6 + 13 *i, 6 + 13 * i) = 1;
        L(7 + 13 *i, 7 + 13 * i) = 1;
        L(8 + 13 *i, 8 + 13 * i) = 1;

        L(9 + 13 * i, 9 + 13 * i) = 1;//
        L(10 + 13 * i, 10 + 13 * i) = 1;//
        L(11 + 13 * i, 11 + 13 * i) = 1;//
    }
    K = 0.000001 * MatrixXf::Identity(30, 30);
    Matrix<float, 30, 30> temp_H;
    Vector<float, 30> temp_g;
    // Matrix<float, 8 * n, 6 * n> fA;
    temp_H = 2 * (B_qp.transpose() * L * B_qp + K);
    temp_g = 2 * B_qp.transpose() * L * (A_qp * X - X_Xref);
    Matrix<float, 8, 6> temp_uA;
    Matrix<float, 40, 30> temp_tA;
    Vector<float, 30> temp_Forcesum;
    Vector<float, 6> temp_Force;
    USING_NAMESPACE_QPOASES
    real_t H[30*30];
    real_t g[30];
    for (int i = 0; i < 30; i++)
    {
        for (int j = 0; j < 30; j++)
        {
             H[i * 30 + j] = temp_H(i, j);
        }
        g[i] = temp_g(i);
    }
	real_t fA[40*30];
	real_t lb[30];
	real_t ub[30];
	real_t lbA[40];
	real_t ubA[40];
    float u = 4.0;
    temp_uA <<u, 0, 1, 0, 0, 0,
              -u, 0, 1, 0, 0, 0,
              0, u, 1, 0, 0, 0,
              0, -u, 1, 0, 0, 0,
              0, 0, 0, u, 0, 1,
              0, 0, 0, -u, 0, 1,
              0, 0, 0, 0, u, 1,
              0, 0, 0, 0, -u, 1;
    for (int i = 0; i< n; i++)
    {
        temp_tA.block(8 * i, 6 * i, 8, 6) = temp_uA;
    }
    temp_tA = MatrixXf::Zero(40,30);
    for (int i = 0; i < 40; i++)
    {
        for (int j = 0; j < 30; j++)
        {
            fA[i * 40 + j] = temp_tA(i, j);
        }
    }
    for (int i = 0; i < 30; i++)
    {
        lb[i] = -660;
        ub[i] = 660;
    }
    for (int i = 0; i < 10; i++)
    {
        lb[3 * i + 2] = 0;
    }
    for (int i = 0; i < 40; i++)
    {
        lbA[i] = 0;
        ubA[i] = 1000000;
    }
	QProblem example( 30,40 );
	int_t nWSR = 60;
	example.init( H,g,fA,lb,ub,lbA,ubA,nWSR);
	real_t xOpt[30];
	example.getPrimalSolution( xOpt );
    for (int i = 0; i < 30; i++)
    {
        temp_Forcesum(i) = xOpt[i];
    }
    temp_Force << xOpt[0], xOpt[1], xOpt[2], xOpt[3], xOpt[4], xOpt[5];
    XX = A_qp * X + B_qp * temp_Forcesum ;
    // X = XX.head(13);
    // cout << "force:   "<< temp_Force.transpose()<<endl;
    cout << "imu: "<< imu_num.transpose()<<endl;
    // cout << "X:   "<< X.transpose()<<endl;
    if (swingFlag == 0)
    {
        jacobian_Matrix.block(0,0,3,3) << jacobian(0 ,0), jacobian(0 ,1), jacobian(0 ,2),
                                          jacobian(0 ,3), jacobian(0 ,4), jacobian(0 ,5),
                                          jacobian(0 ,6), jacobian(0 ,7), jacobian(0 ,8);
        jacobian_Matrix.block(3,3,3,3) << jacobian(3 ,0), jacobian(3 ,1), jacobian(3 ,2),
                                          jacobian(3 ,3), jacobian(3 ,4), jacobian(3 ,5),
                                          jacobian(3 ,6), jacobian(3 ,7), jacobian(3 ,8);
        jacobian_Matrix.block(0,3,3,3) = MatrixXf::Zero(3, 3);
        jacobian_Matrix.block(3,0,3,3) = MatrixXf::Zero(3, 3);
        Vector<float, 6> temp_torque;
        temp_torque = -jacobian_Matrix.transpose() * temp_Force;
        jacobian_torque.head(3) = temp_torque.head(3);
        jacobian_torque.tail(3) = temp_torque.tail(3);
    }
    else
    {
        jacobian_Matrix.block(0,0,3,3) << jacobian(1 ,0), jacobian(1 ,1), jacobian(1 ,2),
                                          jacobian(1 ,3), jacobian(1 ,4), jacobian(1 ,5),
                                          jacobian(1 ,6), jacobian(1 ,7), jacobian(1 ,8);
        jacobian_Matrix.block(3,3,3,3) << jacobian(2 ,0), jacobian(2 ,1), jacobian(2 ,2),
                                          jacobian(2 ,3), jacobian(2 ,4), jacobian(2 ,5),
                                          jacobian(2 ,6), jacobian(2 ,7), jacobian(2 ,8);
        jacobian_Matrix.block(0,3,3,3) = MatrixXf::Zero(3, 3);
        jacobian_Matrix.block(3,0,3,3) = MatrixXf::Zero(3, 3);
        Vector<float, 6> temp_torque;
        temp_torque = -jacobian_Matrix.transpose() * temp_Force;
        jacobian_torque.segment(3, 6) = temp_torque;
    }
}
void MotionControl::swing_VMC()
{
    float H = 0.2;
    float T = 0.3;//0.38
    float S = T * targetCoMVelocity[0];
    float t = fly_time ;
    float x = S * (t/T-sin(2*PI*t/T)/(2*PI));
    float vx = S * (1/T-cos(2*PI*t/T)/T);
    float sgn;
    if ( t >= 0 && t < T/2)
    {
       sgn = 1;
    }
    else
    {
       sgn = -1;
    }
    float z = H * (sgn * (2*(t/T-sin(4*PI*t/T)/(4*PI))-1)+1);
    float vz = H * (sgn *(2*(1/T-cos(4*PI*t/T)/T)-1)+1);
    float y = targetCoMVelocity[1] * t;
    float vy = targetCoMVelocity[1];
    float swingfx_kp = 24;
    float swingfy_kp = 20;
    float swingfz_kp = 26;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   ;
    float swingfx_kd = 12;
    float swingfy_kd = 9.2;
    float swingfz_kd = 18;
    
    float swinghx_kp = 24;
    float swinghy_kp = 20;
    float swinghz_kp = 26;
    
    float swinghx_kd = 12;
    float swinghy_kd = 9.2;
    float swinghz_kd = 18;
    Matrix<float, 6, 6>jacobian_swingMatrix;
    if (swingFlag == 0)
    {
        legCmdPos(1,0) = -0.025 + x ;
        legCmdPos(1,1) = -5.95758e-06  + y + 2 * sin(imu_num(1)) * legPresentPos(3, 2) ;//+ 2 * sin(imu_num(1)) * legPresentPos(3, 2)
        // legCmdPos(1,2) = -0.64 + z;
        legCmdPos(1,2) = legPresentPos(3, 2) + z;
        legCmdPos(2,0) = -0.025 + x ;//- legCmdPos(3,0);
        legCmdPos(2,1) = -5.95758e-06  + y + 2 * sin(imu_num(1)) * legPresentPos(0, 2);
        legCmdPos(2,2) = legPresentPos(3, 2) + z;
        legCmdVel(0) = vx;
        legCmdVel(1) = vy;
        legCmdVel(2) = vz;
        legCmdVel(3) = vx;
        legCmdVel(4) = vy;
        legCmdVel(5) = vz;
        jacobian_swingMatrix.block(0,0,3,3) << jacobian(1 ,0), jacobian(1 ,1), jacobian(1 ,2),
                                               jacobian(1 ,3), jacobian(1 ,4), jacobian(1 ,5),
                                               jacobian(1 ,6), jacobian(1 ,7), jacobian(1 ,8);
        jacobian_swingMatrix.block(3,3,3,3) << jacobian(2 ,0), jacobian(2 ,1), jacobian(2 ,2),
                                               jacobian(2 ,3), jacobian(2 ,4), jacobian(2 ,5),
                                               jacobian(2 ,6), jacobian(2 ,7), jacobian(2 ,8);
        jacobian_swingMatrix.block(0,3,3,3) = MatrixXf::Zero(3, 3);
        jacobian_swingMatrix.block(3,0,3,3) = MatrixXf::Zero(3, 3);
        Vector<float, 6> temp_jointPresentVel;
        temp_jointPresentVel = jointPresentVel.segment(3, 6);
        legPresentVel = jacobian_swingMatrix * temp_jointPresentVel;
        Vector<float, 6> temp_forceswing;
        temp_forceswing(0) = swingfx_kp * (legCmdPos(1,0)- legPresentPos(1,0)) - swingfx_kd * (legPresentVel(0) - legCmdVel(0));
        temp_forceswing(1) = swingfy_kp * (legCmdPos(1,1)- legPresentPos(1,1)) - swingfy_kd * (legPresentVel(1) - legCmdVel(1));
        temp_forceswing(2) = swingfz_kp * (legCmdPos(1,2)- legPresentPos(1,2)) - swingfz_kd * (legPresentVel(2) - legCmdVel(2));
        temp_forceswing(3) = swinghx_kp * (legCmdPos(2,0)- legPresentPos(2,0)) - swinghx_kd * (legPresentVel(3) - legCmdVel(3));
        temp_forceswing(4) = swinghy_kp * (legCmdPos(2,1)- legPresentPos(2,1)) - swinghy_kd * (legPresentVel(4) - legCmdVel(4));
        temp_forceswing(5) = swinghz_kp * (legCmdPos(2,2)- legPresentPos(2,2)) - swinghz_kd * (legPresentVel(5) - legCmdVel(5));
        jacobian_torque.segment(3, 6) = jacobian_swingMatrix.transpose() * temp_forceswing;
    }
    else
    {
        legCmdPos(0,0) = -0.025 + x ;//- legPresentPos(2,0);
        legCmdPos(0,1) = -5.95758e-06  + y + 2 * sin(imu_num(1)) * legPresentPos(2, 2);
        legCmdPos(0,2) = legPresentPos(2, 2) + z;
        legCmdPos(3,0) = -0.025 + x ;//- legPresentPos(2,0);
        legCmdPos(3,1) = -5.95758e-06  + y + 2 * sin(imu_num(1)) * legPresentPos(1, 2);
        legCmdPos(3,2) = legPresentPos(2, 2) + z;
        legCmdVel(0) = vx;
        legCmdVel(1) = vy;
        legCmdVel(2) = vz;
        legCmdVel(3) = vx;
        legCmdVel(4) = vy;
        legCmdVel(5) = vz;
        jacobian_swingMatrix.block(0,0,3,3) << jacobian(0 ,0), jacobian(0 ,1), jacobian(0 ,2),
                                               jacobian(0 ,3), jacobian(0 ,4), jacobian(0 ,5),
                                               jacobian(0 ,6), jacobian(0 ,7), jacobian(0 ,8);
        jacobian_swingMatrix.block(3,3,3,3) << jacobian(3 ,0), jacobian(3 ,1), jacobian(3 ,2),
                                               jacobian(3 ,3), jacobian(3 ,4), jacobian(3 ,5),
                                               jacobian(3 ,6), jacobian(3 ,7), jacobian(3 ,8);
        jacobian_swingMatrix.block(0,3,3,3) = MatrixXf::Zero(3, 3);
        jacobian_swingMatrix.block(3,0,3,3) = MatrixXf::Zero(3, 3);
        Vector<float, 6> temp_jointPresentVel;
        temp_jointPresentVel.head(3) = jointPresentVel.head(3);
        temp_jointPresentVel.tail(3) = jointPresentVel.tail(3);
        legPresentVel = jacobian_swingMatrix * temp_jointPresentVel;
        Vector<float, 6> temp_forceswing;
        temp_forceswing(0) = swingfx_kp * (legCmdPos(0,0)- legPresentPos(0,0)) - swingfx_kd * (legPresentVel(0) - legCmdVel(0));
        temp_forceswing(1) = swingfy_kp * (legCmdPos(0,1)- legPresentPos(0,1)) - swingfy_kd * (legPresentVel(1) - legCmdVel(1));
        temp_forceswing(2) = swingfz_kp * (legCmdPos(0,2)- legPresentPos(0,2)) - swingfz_kd * (legPresentVel(2) - legCmdVel(2));
        temp_forceswing(3) = swinghx_kp * (legCmdPos(3,0)- legPresentPos(3,0)) - swinghx_kd * (legPresentVel(3) - legCmdVel(3));
        temp_forceswing(4) = swinghy_kp * (legCmdPos(3,1)- legPresentPos(3,1)) - swinghy_kd * (legPresentVel(4) - legCmdVel(4));
        temp_forceswing(5) = swinghz_kp * (legCmdPos(3,2)- legPresentPos(3,2)) - swinghz_kd * (legPresentVel(5) - legCmdVel(5));
        Vector<float, 6> temp_torqueswing;
        temp_torqueswing = jacobian_swingMatrix.transpose() * temp_forceswing;
        jacobian_torque.head(3) = temp_torqueswing.head(3);
        jacobian_torque.tail(3) = temp_torqueswing.tail(3);
    }
    
    fly_time += timePeriod;
    if (fly_time > T + 0.005)
    {
        fly_time = 0.0;
        if (swingFlag == 0)
        swingFlag = 1;
        else 
        swingFlag = 0;
    }
    
}

void MotionControl::inverseKinematics()
{
    float jo_ang[4][3] = {0};
    static int times = 0;
    motorInitPos[0] = 0.0;
    motorInitPos[1] = 0.0;
    motorInitPos[2] = 0.0;
    motorInitPos[3] = 0.0;
    motorInitPos[4] = 0.0;
    motorInitPos[5] = 0.0;
    motorInitPos[6] = 0.0;
    motorInitPos[7] = 0.0;
    motorInitPos[8] = 0.0;
    motorInitPos[9] = 0.0;
    motorInitPos[10] = 0.0;
    motorInitPos[11] = 0.0;

    if(times!=0)
    {
        for(int joints=0; joints<12; joints++)
        {
            jointCmdPosLast[joints] = jointCmdPos[joints];
        }
    }

    for(int leg_num = 0; leg_num < 4; leg_num++)
    {
        float x = legCmdPos(leg_num,0);
        float y = legCmdPos(leg_num,1);
        float z = legCmdPos(leg_num,2);
        float theta0 = atan (- y / z);
        float theta2 = - acos ((pow(( sin(theta0) * y - cos(theta0) * z), 2)  + pow(x, 2) - pow(L1, 2) - pow(L2, 2)) / ( 2 * L1 * L2)) ; 
        float cos_theta1 = ((sin (theta2) * y - cos (theta0) * z) * (L1 + L2 * cos (theta2)) - x * L2 * sin(theta2)) / (pow((sin (theta2) * y - cos (theta0) * z), 2) + pow(x, 2));
        float sin_theta1 = ((L1 + L2 * cos (theta2)) * x + (sin (theta2) * y - cos (theta0) * z) * L2 * sin(theta2)) / (-pow((sin (theta2) * y - cos (theta0) * z), 2) - pow(x, 2));
        float theta1 = atan (sin_theta1/ cos_theta1);
        jo_ang[leg_num][0] = theta0;
        jo_ang[leg_num][1] = theta1;
        jo_ang[leg_num][2] = theta2;

    }
    
    jointCmdPos[0] = motorInitPos[0] + jo_ang[0][0];
    jointCmdPos[1] = motorInitPos[1] + jo_ang[0][1];
    jointCmdPos[2] = motorInitPos[2] + jo_ang[0][2];
    jointCmdPos[3] = motorInitPos[3] + jo_ang[1][0];
    jointCmdPos[4] = motorInitPos[4] + jo_ang[1][1];
    jointCmdPos[5] = motorInitPos[5] + jo_ang[1][2];
    jointCmdPos[6] = motorInitPos[6] + jo_ang[2][0];
    jointCmdPos[7] = motorInitPos[7] + jo_ang[2][1];
    jointCmdPos[8] = motorInitPos[8] + jo_ang[2][2];
    jointCmdPos[9] = motorInitPos[9] + jo_ang[3][0];
    jointCmdPos[10] = motorInitPos[10] + jo_ang[3][1];
    jointCmdPos[11] = motorInitPos[11] + jo_ang[3][2];

    if(times!=0)
    {
        for(int joints=0; joints<12; joints++)
        {
            jointCmdVel[joints] = (jointCmdPos[joints] - jointCmdPosLast[joints]) / timePeriod;
        }
    }
    else
    {
        for(int joints=0; joints<12; joints++)
        {
            jointCmdVel[joints] = 0;
        }
    }
    times++;
}

void MotionControl::forwardKinematics()
{
    float joint_pres_pos[4][3];
    joint_pres_pos[0][0] = jointPresentPos[0];
    joint_pres_pos[0][1] = jointPresentPos[1];
    joint_pres_pos[0][2] = jointPresentPos[2];
    joint_pres_pos[1][0] = jointPresentPos[3];
    joint_pres_pos[1][1] = jointPresentPos[4];
    joint_pres_pos[1][2] = jointPresentPos[5];
    joint_pres_pos[2][0] = jointPresentPos[6];
    joint_pres_pos[2][1] = jointPresentPos[7];
    joint_pres_pos[2][2] = jointPresentPos[8];
    joint_pres_pos[3][0] = jointPresentPos[9];
    joint_pres_pos[3][1] = jointPresentPos[10];
    joint_pres_pos[3][2] = jointPresentPos[11];

    for(int leg_nums = 0; leg_nums < 4; leg_nums++)
    {
        legPresentPos(leg_nums,0) = -L1 * sin(joint_pres_pos[leg_nums][1]) - L2 * sin(joint_pres_pos[leg_nums][1] + joint_pres_pos[leg_nums][2]);
        legPresentPos(leg_nums,1) = L1 * sin(joint_pres_pos[leg_nums][0]) * cos(joint_pres_pos[leg_nums][1]) + L2 * sin(joint_pres_pos[leg_nums][0]) * cos(joint_pres_pos[leg_nums][1] + joint_pres_pos[leg_nums][2]);
        legPresentPos(leg_nums,2) = -L1 * cos(joint_pres_pos[leg_nums][0]) * cos(joint_pres_pos[leg_nums][1]) - L2 * cos(joint_pres_pos[leg_nums][0]) * cos(joint_pres_pos[leg_nums][1] + joint_pres_pos[leg_nums][2]);
         
        leg2CoMPrePos(leg_nums,0) = shoulderPos(leg_nums,0) + legPresentPos(leg_nums,0);
        leg2CoMPrePos(leg_nums,1) = shoulderPos(leg_nums,1) + legPresentPos(leg_nums,1);
        leg2CoMPrePos(leg_nums,2) = legPresentPos(leg_nums,2);
    }
}
void MotionControl::jacobians()
{
    jacobian(0 ,0) = 0;
    jacobian(0 ,1) =  -L1 * cos(jointPresentPos(1)) -L2 * cos(jointPresentPos(1) + jointPresentPos(2));
    jacobian(0 ,2) =  -L2 * cos(jointPresentPos(1) + jointPresentPos(2));
    jacobian(0 ,3) = L1 * cos(jointPresentPos(0)) * cos(jointPresentPos(1)) + L2 * cos(jointPresentPos(0)) * cos(jointPresentPos(1) + jointPresentPos(2));
    jacobian(0 ,4) = -L1 * sin(jointPresentPos(0)) * sin(jointPresentPos(1)) - L2 * sin(jointPresentPos(0)) * sin(jointPresentPos(1) + jointPresentPos(2));
    jacobian(0 ,5) = -L2 * sin(jointPresentPos(0)) * sin(jointPresentPos(1) + jointPresentPos(2));
    jacobian(0 ,6) = L1 * sin(jointPresentPos(0)) * cos(jointPresentPos(1)) + L2 * sin(jointPresentPos(0)) * cos(jointPresentPos(1) + jointPresentPos(2));
    jacobian(0 ,7) = L1 * cos(jointPresentPos(0)) * sin(jointPresentPos(1)) + L2 * cos(jointPresentPos(0)) * sin(jointPresentPos(1) + jointPresentPos(2));
    jacobian(0 ,8) = L2 * cos(jointPresentPos(0)) * sin(jointPresentPos(1) + jointPresentPos(2));

    jacobian(1 ,0) = 0;
    jacobian(1 ,1) =  -L1 * cos(jointPresentPos(4)) -L2 * cos(jointPresentPos(4) + jointPresentPos(5));
    jacobian(1 ,2) =  -L2 * cos(jointPresentPos(4) + jointPresentPos(5));
    jacobian(1 ,3) = L1 * cos(jointPresentPos(3)) * cos(jointPresentPos(4)) + L2 * cos(jointPresentPos(3)) * cos(jointPresentPos(4) + jointPresentPos(5));
    jacobian(1 ,4) = -L1 * sin(jointPresentPos(3)) * sin(jointPresentPos(4)) - L2 * sin(jointPresentPos(3)) * sin(jointPresentPos(4) + jointPresentPos(5));
    jacobian(1 ,5) = -L2 * sin(jointPresentPos(3)) * sin(jointPresentPos(4) + jointPresentPos(5));
    jacobian(1 ,6) = L1 * sin(jointPresentPos(3)) * cos(jointPresentPos(4)) + L2 * sin(jointPresentPos(3)) * cos(jointPresentPos(4) + jointPresentPos(5));
    jacobian(1 ,7) = L1 * cos(jointPresentPos(3)) * sin(jointPresentPos(4)) + L2 * cos(jointPresentPos(3)) * sin(jointPresentPos(4) + jointPresentPos(5));
    jacobian(1 ,8) = L2 * cos(jointPresentPos(3)) * sin(jointPresentPos(4) + jointPresentPos(5));

    jacobian(2, 0) = 0;
    jacobian(2 ,1) =  -L1 * cos(jointPresentPos(7)) -L2 * cos(jointPresentPos(7) + jointPresentPos(8));
    jacobian(2 ,2) =  -L2 * cos(jointPresentPos(7) + jointPresentPos(8));
    jacobian(2 ,3) = L1 * cos(jointPresentPos(6)) * cos(jointPresentPos(7)) + L2 * cos(jointPresentPos(6)) * cos(jointPresentPos(7) + jointPresentPos(8));
    jacobian(2 ,4) = -L1 * sin(jointPresentPos(6)) * sin(jointPresentPos(7)) - L2 * sin(jointPresentPos(6)) * sin(jointPresentPos(7) + jointPresentPos(8));
    jacobian(2 ,5) = -L2 * sin(jointPresentPos(6)) * sin(jointPresentPos(7) + jointPresentPos(8));
    jacobian(2 ,6) = L1 * sin(jointPresentPos(6)) * cos(jointPresentPos(7)) + L2 * sin(jointPresentPos(6)) * cos(jointPresentPos(7) + jointPresentPos(8));
    jacobian(2 ,7) = L1 * cos(jointPresentPos(6)) * sin(jointPresentPos(7)) + L2 * cos(jointPresentPos(6)) * sin(jointPresentPos(7) + jointPresentPos(8));
    jacobian(2 ,8) = L2 * cos(jointPresentPos(6)) * sin(jointPresentPos(7) + jointPresentPos(8));

    jacobian(3 ,0) = 0;
    jacobian(3 ,1) =  -L1 * cos(jointPresentPos(10)) -L2 * cos(jointPresentPos(10) + jointPresentPos(11));
    jacobian(3 ,2) =  -L2 * cos(jointPresentPos(10) + jointPresentPos(11));
    jacobian(3 ,3) = L1 * cos(jointPresentPos(9)) * cos(jointPresentPos(10)) + L2 * cos(jointPresentPos(9)) * cos(jointPresentPos(10) + jointPresentPos(11));
    jacobian(3 ,4) = -L1 * sin(jointPresentPos(9)) * sin(jointPresentPos(10)) - L2 * sin(jointPresentPos(9)) * sin(jointPresentPos(10) + jointPresentPos(11));
    jacobian(3 ,5) = -L2 * sin(jointPresentPos(9)) * sin(jointPresentPos(10) + jointPresentPos(11));
    jacobian(3 ,6) = L1 * sin(jointPresentPos(9)) * cos(jointPresentPos(10)) + L2 * sin(jointPresentPos(9)) * cos(jointPresentPos(10) + jointPresentPos(11));
    jacobian(3 ,7) = L1 * cos(jointPresentPos(9)) * sin(jointPresentPos(10)) + L2 * cos(jointPresentPos(9)) * sin(jointPresentPos(10) + jointPresentPos(11));
    jacobian(3 ,8) = L2 * cos(jointPresentPos(9)) * sin(jointPresentPos(10) + jointPresentPos(11));
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 9; j++)
        {
            if (jacobian(i, j) < -10 || jacobian (i, j) > 10)
            {
                jacobian(i ,j) = 0.0;
                cout << "jacobian"<<endl;
            }
        }
     
    }
 }