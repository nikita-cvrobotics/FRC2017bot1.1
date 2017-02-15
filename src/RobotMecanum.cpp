#include <WPIlib.h>
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
	const int BASIC_DRIVE = 0;
	const int ADVANCED_DRIVE = 1;
	int current_drive = BASIC_DRIVE;
	//--- Gyroscope Items ---
	ADXRS450_Gyro *_robot_gyro;
	double init_angle = 0;
	double current_angle = 0;
	double lock_angle = 0;

	void initialize() {
		_motor_FL = new Spark(0);
		_motor_FR = new Spark(1);
		_motor_BL = new TalonSRX(3);
		_motor_BR = new TalonSRX(2);
		_robot_gyro = new ADXRS450_Gyro();
		setGlobalGyro();
	}
	void setGlobalGyro() {
		init_angle = _robot_gyro->GetAngle();
	}
	void setGyroLock() {
		lock_angle = _robot_gyro->GetAngle();
	}
	//Input for Set() should be in degrees.
	void Set(double angle, double magnitude, double r) {
		if (current_drive == ADVANCED_DRIVE) {
			driveAdvanced(angle, magnitude, r);
		} else {
			driveBasic(cos(angle * PI / 180) * magnitude, sin(angle * PI / 180) * magnitude, r);
		}
	}
	void SetCartesian(double x, double y, double r) {
		if (current_drive == ADVANCED_DRIVE) {
			int send_angle = atan2(x, y);
			int send_magnitude = sqrt(x*x + y*y);
			driveAdvanced(send_angle, send_magnitude, r);
		} else {
			driveBasic(x, y, r);
		}
	}
	void setDriveSystem(int new_drive) {
		current_drive = new_drive;
	}
	void driveAdvanced(double angle, double magnitude, double r) {
		current_angle = _robot_gyro->GetAngle();

	}
	void driveBasic(double x, double y, double r) {
		_motor_FL->Set((y + x + r) * sensitivity * inv_FL);
		_motor_FR->Set((y - x - r) * sensitivity * inv_FL);
		_motor_BL->Set((y - x + r) * sensitivity * inv_FL);
		_motor_BR->Set((y + x - r) * sensitivity * inv_FL);
	}
};
