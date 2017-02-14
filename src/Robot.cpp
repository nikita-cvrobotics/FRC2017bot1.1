#include <iostream>
#include <memory>
#include <string>
#include <WPIlib.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <math.h>

class Robot: public frc::IterativeRobot {

public:
	Spark *motorfrontleft;
	Spark *motorfrontright;
	Spark *motorbackright;
	Spark *motorbackleft;
	XboxController *exampleStick;
	Compressor *compressor;
	DoubleSolenoid *claw;
	DoubleSolenoid *lift;

	ADXRS450_Gyro *Gyroo;
	double initAngle = 0;
	double lockAngle = 0;
	double currAngle = 0;


	void RobotInit() {
		exampleStick = new XboxController(0);
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
		motorfrontleft = new Spark(0);
		motorfrontright = new Spark(1);
		motorbackright = new Spark(2);
		motorbackleft = new Spark(3);
		Gyroo = new ADXRS450_Gyro();
		compressor = new Compressor(0);
		claw = new DoubleSolenoid(0, 1);
		lift = new DoubleSolenoid(2, 3);


	}

	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	int i = 0;

	void AutonomousInit() override {
		autoSelected = chooser.GetSelected();
		// std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}

		i = 0;


	}

	void AutonomousPeriodic() {
		if (autoSelected == autoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}

	}
	double defOffset = -90;
	void TeleopInit() {
		initAngle = Gyroo->GetAngle();
		lockAngle = initAngle;
		compressor->SetClosedLoopControl(false);
	}

	double sensitivity = 0.25;
	bool lastClawBtn = false;
	bool lastLiftBtn = false;
	bool lastGyroReset = false;
	bool clawOn = false;
	bool liftOn = false;
	void TeleopPeriodic() {

		bool clawBtn = exampleStick->GetRawButton(1);
		bool liftBtn = exampleStick->GetRawButton(2);
		bool gyroReset = exampleStick->GetRawButton(7);

		if (gyroReset && !lastGyroReset) {
			initAngle = Gyroo->GetAngle();
			lockAngle = initAngle;
		}
		lastGyroReset = gyroReset;
		if (clawBtn && !lastClawBtn) {
			if (clawOn) {
				claw->Set(DoubleSolenoid::kForward);
			} else {
				claw->Set(DoubleSolenoid::kReverse);
			}
			clawOn = !clawOn;
		}

		if (liftBtn && !lastLiftBtn) {
			if (liftOn) {
				lift->Set(DoubleSolenoid::kForward);
			} else {
				lift->Set(DoubleSolenoid::kReverse);
			}
			liftOn = !liftOn;
		}

		lastClawBtn = clawBtn;
		lastLiftBtn = liftBtn;
		/*
		//DRIVE METHOD 1
		double Joystickvalue = exampleStick->GetRawAxis(1) * sensitivity;
		double TurnJoystickvalue = exampleStick->GetRawAxis(4) * sensitivity;
		double SideJoystickvalue = exampleStick->GetRawAxis(0) * sensitivity;
		motorfrontleft->Set(- Joystickvalue + TurnJoystickvalue + SideJoystickvalue);
		motorfrontright->Set(Joystickvalue + TurnJoystickvalue + SideJoystickvalue);
		motorbackright->Set(Joystickvalue + TurnJoystickvalue + - SideJoystickvalue);
		motorbackleft->Set(- Joystickvalue + TurnJoystickvalue + - SideJoystickvalue);
		//SmartDashboard::PutString("DB/String 0", "test");
		*/

		//DRIVE METHOD 2

		double jsX = exampleStick->GetRawAxis(0);
		double jsY = exampleStick->GetRawAxis(1);
		double jsR = exampleStick->GetRawAxis(4);
		double jsAngle = genAngle(jsX, jsY) + defOffset;
		double jsMagnitude = genMagnitude(jsX, jsY);
		currAngle = Gyroo->GetAngle();
		double robotAngle = jsAngle - (currAngle - initAngle);
		double robotMagnitude = jsMagnitude;
		driveRobot(robotAngle, robotMagnitude, jsR);

		//UPDATE DASHBOARD VALUES
		SmartDashboard::PutString("DB/String 0", "Angle: " + std::to_string(Gyroo->GetAngle()));
		SmartDashboard::PutString("DB/String 1", std::to_string(genAngle(exampleStick->GetRawAxis(0), exampleStick->GetRawAxis(1))));
		SmartDashboard::PutString("DB/String 2", std::to_string(genMagnitude(exampleStick->GetRawAxis(0), exampleStick->GetRawAxis(1))));

	}
	void driveRobot(double angle, double magnitude, double rotation) {
		double drV = sin(angle * 3.14159 / 180.0) * magnitude * sensitivity;
		double drH = cos(angle * 3.14159 / 180.0) * magnitude * sensitivity;
		double currRate = Gyroo->GetRate();
		double turnSpd = -0.35 * ((currRate + (-rotation * 100)) / 60);
		if (rotation < 0.2 && rotation > -0.2) {
			turnSpd = -0.5 * ((currAngle - lockAngle) / 60);
		} else {
			lockAngle = currAngle;
		}
		SmartDashboard::PutString("DB/String 3", "Rate: " + std::to_string(currRate));
		SmartDashboard::PutString("DB/String 4", "TurnSpd: " + std::to_string(turnSpd));
		SmartDashboard::PutString("DB/String 5", "currAngle: " + std::to_string(0.35 * (currAngle)));
		SmartDashboard::PutString("DB/String 6", "initAngle: " + std::to_string(0.35 * (initAngle)));
		motorfrontleft->Set(- drV + turnSpd + drH);
		motorfrontright->Set(drV + turnSpd + drH);
		motorbackright->Set(drV + turnSpd + - drH);
		motorbackleft->Set(- drV + turnSpd + - drH);
	}
	double genMagnitude(double x, double y) {
		double result = sqrt(x * x + y * y);
		return result;
	}

	double genAngle(double x, double y) {
		double result = 0;
		result = atan(y/x) * 180 / 3.1415927;
		if (x < 0) {
			result = result + 180;
		}
		result = result + 90;
		return result;
	}

	void TestPeriodic() {
		lw->Run();
	}

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;
};

START_ROBOT_CLASS(Robot)
