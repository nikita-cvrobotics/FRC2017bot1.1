/*
 * RobotMecanum.h
 *
 *  Created on: Feb 14, 2017
 *      Author: Adam
 */

#ifndef SRC_ROBOTMECANUM_H_
#define SRC_ROBOTMECANUM_H_

#include <Math.h>

#define PI 3.14159265

class RobotMecanum {

public:
	//--- Motor Items ---
	Spark *_motor_FL;
	Spark *_motor_FR;
	TalonSRX *_motor_BL;
	TalonSRX *_motor_BR;
	int inv_FL = 1;
	int inv_FR = 1;
	int inv_BL = 1;
	int inv_BR = 1;
	double sensitivity = 0.25;
	double turn_threshold = 0.2;
	const int BASIC_DRIVE = 0;
	const int ADVANCED_DRIVE = 1;
	int current_drive = BASIC_DRIVE;
	//--- Gyroscope Items ---
	ADXRS450_Gyro *_robot_gyro;
	double init_angle = 0;
	double current_angle = 0;
	double lock_angle = 0;

	void initialize();

	void setGlobalGyro();

	void setGyroLock();

	//Input for Set() should be in degrees.


	void setDriveSystem(int new_drive);



};


#endif /* SRC_ROBOTMECANUM_H_ */
