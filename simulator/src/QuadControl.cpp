#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#include <iostream>

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  float d = 0.7071*L;

  float c = collThrustCmd;
  float tau_x_bar = momentCmd.x / d;
  float tau_y_bar = momentCmd.y / d;
  float tau_z_bar = momentCmd.z / kappa;

  cmd.desiredThrustsN[0] = 0.25 * ( c + tau_x_bar + tau_y_bar - tau_z_bar ); // front left
  cmd.desiredThrustsN[1] = 0.25 * ( c - tau_x_bar + tau_y_bar + tau_z_bar ); // front right
  cmd.desiredThrustsN[2] = 0.25 * ( c + tau_x_bar - tau_y_bar + tau_z_bar ); // rear left
  cmd.desiredThrustsN[3] = 0.25 * ( c - tau_x_bar - tau_y_bar - tau_z_bar ); // rear right

  // Command limitations

  for (int i = 0; i < 4; i++) {      
      cmd.desiredThrustsN[i] = CONSTRAIN(cmd.desiredThrustsN[i], minMotorThrust, maxMotorThrust);
  }

  // Uncomment the lines below when tuning the drone mass parameter in Scenario 1.
  //
  // These lines will command the same thrust in each propeller, each
  // of these thrusts being equal to 1/4 of the total collective thrust.
  //
  // When the mass parameter matches the drone mass in the simulation,
  // the drone should hover.
  // mass = 0.f;
  // cmd.desiredThrustsN[0] = mass * 9.81f / 4.f; // front left
  // cmd.desiredThrustsN[1] = mass * 9.81f / 4.f; // front right
  // cmd.desiredThrustsN[2] = mass * 9.81f / 4.f; // rear left
  // cmd.desiredThrustsN[3] = mass * 9.81f / 4.f; // rear right

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // Using intermediate variables for readability

  float Kp_p = kpPQR.x;
  float Kp_q = kpPQR.y;
  float Kp_r = kpPQR.z;

  float p_c = pqrCmd.x;
  float q_c = pqrCmd.y;
  float r_c = pqrCmd.z;

  float p_a = pqr.x;
  float q_a = pqr.y;
  float r_a = pqr.z;

  momentCmd.x = Ixx * Kp_p * (p_c - p_a) + (Izz - Iyy) * r_a * q_a;
  momentCmd.y = Iyy * Kp_q * (q_c - q_a) + (Ixx - Izz) * p_a * r_a;
  momentCmd.z = Izz * Kp_r * (r_c - r_a) + (Iyy - Ixx) * q_a * p_a;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // Using intermediate variables for readability

  float R11_a = R(0,0);
  float R12_a = R(0,1);
  float R13_a = R(0,2);
  float R21_a = R(1,0);
  float R22_a = R(1,1);
  float R23_a = R(1,2);
  float R31_a = R(2,0);
  float R32_a = R(2,1);
  float R33_a = R(2,2);

  float R13_c = - mass * accelCmd.x / collThrustCmd;
  float R23_c = - mass * accelCmd.y / collThrustCmd;

  // Note: maxTiltAngle should have been called maxR since it is used
  // to control rotation matrix elements and not angle values.
  R13_c = CONSTRAIN(R13_c, -maxTiltAngle, maxTiltAngle);
  R23_c = CONSTRAIN(R23_c, -maxTiltAngle, maxTiltAngle);

  float R13_c_dot = kpBank * ( R13_c - R13_a );
  float R23_c_dot = kpBank * ( R23_c - R23_a );

  pqrCmd.x = ( R21_a * R13_c_dot - R11_a * R23_c_dot ) / R33_a;
  pqrCmd.y = ( R22_a * R13_c_dot - R12_a * R23_c_dot ) / R33_a;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // Input command limitations

  velZCmd = CONSTRAIN(velZCmd, -maxDescentRate, maxAscentRate);

  // Errors

  float altitudeError = posZCmd - posZ;
  integratedAltitudeError += altitudeError * dt;
  float derivatedError = velZCmd - velZ;

  // Oputout command

  float z_dot_dot = kpPosZ*altitudeError + KiPosZ*integratedAltitudeError + kpVelZ*derivatedError + accelZCmd;
  thrust = mass * (9.81 - z_dot_dot) / R(2,2);

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // Input command limitations

  velCmd.x = CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY);
  velCmd.y = CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY);
  accelCmdFF.x = CONSTRAIN(accelCmdFF.x, -maxAccelXY, maxAccelXY);
  accelCmdFF.y = CONSTRAIN(accelCmdFF.y, -maxAccelXY, maxAccelXY);

  // Output command

  accelCmd.x = kpPosXY * ( posCmd.x - pos.x ) + kpVelXY * ( velCmd.x - vel.x ) + accelCmdFF.x;
  accelCmd.y = kpPosXY * ( posCmd.y - pos.y ) + kpVelXY * ( velCmd.y - vel.y ) + accelCmdFF.y;

  // Output command limitations

  accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
  accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0.0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // Limit yawCmd from 0 to 2*pi.
  // Limit yawError from -pi to pi.
  // Ref: https://knowledge.udacity.com/questions/270792
  //      https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code

  yawCmd = fmodf(yawCmd, 2*M_PI);

  float yawError = yawCmd - yaw;

  if (yawError > M_PI) {
    yawError = yawError - 2.0 * M_PI;
  } else if (yawError < -M_PI) {
    yawError = yawError + 2.0 * M_PI;
  }

  // Output command

  yawRateCmd = kpYaw * (yawError - yaw);

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
