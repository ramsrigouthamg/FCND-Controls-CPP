#  Implemented Controller

## Implemented body rate control in C++ ##

The controller is a proportional controller on body rates to commanded moments. The difference between commanded body rates and current body rates is then multiplied by the corresponding kpPQR gains.
Moments of inertia of the drone were taken into account when calculating the commanded moments

```
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
  momentCmd = pqrCmd - pqr;
  //momentCmd.x = Ixx * kpPQR.x* momentCmd.x;
  momentCmd.x = Ixx * kpPQR.x* momentCmd.x;
  momentCmd.y = Iyy * kpPQR.y* momentCmd.y;
  momentCmd.z = Izz * kpPQR.z* momentCmd.z;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}
```

## Implemented roll pitch control in C++ ##
First the thrust value is converted into acceleration. Drone's mass is accounted for when calculating targeted angles. The acceleration is used to compute the target angles for roll and pitch. Constraints are applied on the angles.
Using the rotation matrix, the target rates p_c and q_c are then calculated.

```
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


	float target_x = 0.0f;
	float target_y = 0.0f;

	if (collThrustCmd > 0) {

		// coll thrustcmd to accel.
		float collective_accel = collThrustCmd / mass;
		target_x = -CONSTRAIN(accelCmd.x / collective_accel, -maxTiltAngle, maxTiltAngle);
		target_y = -CONSTRAIN(accelCmd.y / collective_accel, -maxTiltAngle, maxTiltAngle);

	}

	// acceleration error
	float b_x_c_dot = kpBank * (target_x - R(0, 2));
	float b_y_c_dot = kpBank * (target_y - R(1, 2));

	// desired roll-pitch 
	float p_c = (1 / R(2, 2))*(R(1, 0)*b_x_c_dot - R(0, 0) * b_y_c_dot);
	float q_c = (1 / R(2, 2))*(R(1, 1)*b_x_c_dot - R(0, 1) * b_y_c_dot);


	pqrCmd.x = p_c;
	pqrCmd.y = q_c;
	pqrCmd.z = 0;
	/////////////////////////////// END STUDENT CODE ////////////////////////////

	return pqrCmd;
}
```

## Implemented altitude controller in C++ ##

The position and the velocity are used to compute the drone commanded acceleration using a PID controller. 
The acceleration is converted into a thrust taking drone's mass into account. An integral control is also added to handle the weight non-idealities.

```
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
  float pos_error = posZCmd - posZ;
  float p_term_z = kpPosZ * pos_error;
  float z_dot_commanded = p_term_z + velZCmd;
  z_dot_commanded = CONSTRAIN(z_dot_commanded, -maxDescentRate, maxAscentRate);
  integratedAltitudeError +=  pos_error *dt;
  float z_dot_dot_commanded = KiPosZ* integratedAltitudeError + kpVelZ *(z_dot_commanded- velZ)+ accelZCmd;

  thrust = -mass * (z_dot_dot_commanded - CONST_GRAVITY) / R(2, 2);

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

```


## Implemented lateral position control in C++ ##

A PD controller is implemented to convert the NE position and velocity 
to a commanded acceleration. Appropriate constraints are used.
```
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


  velCmd += kpPosXY * (posCmd - pos);

  if (velCmd.mag() > maxSpeedXY)
  {
	  velCmd *= maxSpeedXY / velCmd.mag();
  }

  accelCmd += kpVelXY * (velCmd - vel);

  if (accelCmd.mag() > maxAccelXY)
  {
	  accelCmd *= maxAccelXY / accelCmd.mag();
  }

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}
```


## Implemented yaw control in C++ ##

A P controller for yaw is implemented as follows :

```
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

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  float yawError = yawCmd - yaw;
  yawError = fmodf(yawError, 2.0f * F_PI);

  if (yawError > F_PI)
	  yawError = yawError - 2.0f * F_PI;
  else if (yawError < -F_PI)
	  yawError = yawError + 2.0f * F_PI;

  yawRateCmd = kpYaw * yawError;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}
```


## Implemented calculating the motor commands given commanded thrust and moments in C++ ##

The totalthrust and moments are used to calculate the desired thrust on
each of the 4 motors. The rotor dynamics are used to find the appropriate equations.
```
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

	float l = L / sqrt(2);
	float A = collThrustCmd;
	float B = momentCmd.x / l;
	float C = momentCmd.y / l;
	float D = momentCmd.z / kappa;

	float F0 = (A + B + C - D) / 4.0f;// front left
	float F1 = (A - B + C + D) / 4.0f; // front right
	float F2 = (A + B - C + D) / 4.0f; // rear left
	float F3 = (A - B - C - D) / 4.0f; // rear right 

	cmd.desiredThrustsN[0] = F0;
	cmd.desiredThrustsN[1] = F1;
	cmd.desiredThrustsN[2] = F2;
	cmd.desiredThrustsN[3] = F3;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

```
## Flight Evaluation ##
C++ controller is successfully able to fly the provided test trajectory and visually passes inspection of the scenarios leading up to the test trajectory

