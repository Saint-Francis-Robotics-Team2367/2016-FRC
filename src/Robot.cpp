#include "WPILib.h"
#include "ADIS16448_IMU.h"
#include "PracticalSocket.h"
#include "AHRS.h"

/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */
class Robot: public SampleRobot
{
	enum RobotState {
		DISABLED,
		AUTONOMOUS,
		TELEOP,
	};
	/*
	enum AutoSelection {
		DONOTHING,
		LOWBAR,
		MOAT,
		ROCKWALL,
		ROUGHTERRAIN,
		RAMPARTS,
	};
	 */
	enum AutoState {
		STOP,
		INITFORWARD,
		FOUNDRAMPF,
		ONDEFENSEF,
		PASSEDDEFENSEF,
		CORRECTINGANGLE,
		INITBACKWARD,
		FOUNDRAMPB,
		ONDEFENSEB,
		PASSEDDEFENSEB,

		SECONDFORWARD,
		DRIVENBACK,
	};

	//RobotDrive myRobot; // robot drive system
	Joystick stick; // only joystick

	CANTalon *leftMotorF;
	CANTalon *rightMotorF;
	CANTalon *leftMotorB;
	CANTalon *rightMotorB;
	RobotDrive *drive;

	CANTalon *leftShooter;
	CANTalon *rightShooter;
	CANTalon *indexMotor;
	CANTalon *gateMotor;
	//CANTalon *chevalAngler;

	//Servo *xServo;
	Servo *yServo;
	//float xPosition;
	float yPosition;

	bool reverseForward = false;
	bool abortAuto = false;

	float deadZone = 0.1;
	float joyScale = 1.0 - deadZone;
	float minTurnPower = 0.2;
	float turnScale = 1.0 - minTurnPower;
	float turnSlope = turnScale / joyScale;
	float verticalShift = minTurnPower - turnSlope * deadZone;

	Joystick *joystickMain;
	Joystick *joystickSecond;

	ADIS16448_IMU *imu;
	AHRS *ahrs;

	DoubleSolenoid *blocker;
	DoubleSolenoid *shooterA;
	DoubleSolenoid *shooterB;

	DigitalInput *chevalUpLimit;

	//Ultrasonic *frontL;
	//Ultrasonic *frontR;
	AnalogInput *ultra;
	//AnalogInput *ultra2;

	bool autoInit;
	bool teleopInit;
	bool pidInit;

	float initialY, initialZ, timeInitial, timeInitial2;
	float degreesToCorrect;
	int timesAuto;
	float timeReceive;

	RobotState robotState;
	//AutoSelection autoSelection;
	AutoState autoState;

	//UDPSocket *sock;
	//uint8_t buff[641];

	SendableChooser *chooser;
	const std::string autoNothing = "Do Nothing";
	const std::string autoLowBar = "Low Bar";
	const std::string autoMoat = "Moat";
	const std::string autoRockWall = "Rock Wall";
	const std::string autoRoughTerrain = "Rough Terrain";
	const std::string autoRamparts = "Ramparts";
	std::string autoSelected;

public:
	Robot() :
		//myRobot(0, 1),	// these must be initialized in the same order
		stick(0)		// as they are declared above.
{
		rightMotorF = new CANTalon(14);
		rightMotorB = new CANTalon(12);
		leftMotorF = new CANTalon(11);
		leftMotorB = new CANTalon(13);

		leftShooter = new CANTalon(17);
		rightShooter = new CANTalon(16);
		indexMotor = new CANTalon(15);

		gateMotor = new CANTalon(18);

		//chevalAngler = new CANTalon(0);//TODO CHANGE ME

		joystickMain = new Joystick(0);
		joystickSecond = new Joystick(1);

		//xServo = new Servo(1);
		yServo = new Servo(0);
		//xPosition = 0.5;
		yPosition = 0.3;

		chevalUpLimit = new DigitalInput(1);

		leftMotorF->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);
		rightMotorF->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);
		leftMotorB->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);
		rightMotorB->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);

		leftShooter->SetControlMode(CANSpeedController::kPercentVbus);
		rightShooter->SetControlMode(CANSpeedController::kPercentVbus);
		indexMotor->SetControlMode(CANSpeedController::kPercentVbus);

		drive = new RobotDrive(leftMotorF, leftMotorB, rightMotorF, rightMotorB);

		imu = new ADIS16448_IMU();
		ahrs = new AHRS(SPI::Port::kMXP);

		shooterA = new DoubleSolenoid(2, 3);
		shooterB = new DoubleSolenoid(0, 1);
		blocker = new DoubleSolenoid(6, 7);

		ultra = new AnalogInput(3);
		//ultra2 = new AnalogInput(3);

		//frontL = new Ultrasonic(0, 1);
		//frontR = new Ultrasonic(2, 3);

		//frontL->SetEnabled(true);
		//		frontR->SetEnabled(true);
		//		frontL->SetAutomaticMode(true);
		//		frontR->SetAutomaticMode(true);

		autoInit = false;
		teleopInit = false;
		pidInit = false;

		//sock = new UDPSocket(5809);

		//Note SmartDashboard is not initialized here, wait until RobotInit to make SmartDashboard calls
		//myRobot.SetExpiration(0.1);
}

	void RobotInit()
	{
		SmartDashboard::PutNumber("Auto Speed", 0.75);
		imu->Calibrate();
		//autoSelection = LOWBAR;
		autoState = INITFORWARD;

		SmartDashboard::PutNumber("InitialY", initialY);
		SmartDashboard::PutNumber("InitialZ", initialZ);

		chooser = new SendableChooser();
		chooser->AddDefault(autoNothing, (void*)&autoNothing);
		chooser->AddObject(autoMoat, (void*)&autoMoat);
		chooser->AddObject(autoLowBar, (void*)&autoLowBar);
		chooser->AddObject(autoRockWall, (void*)&autoRockWall);
		chooser->AddDefault(autoRoughTerrain, (void*)&autoRoughTerrain);
		chooser->AddObject(autoRamparts, (void*)&autoRamparts);
		SmartDashboard::PutData("Auto Modes", chooser);
	}

	void PIDInit(float p, float i, float d) {
		DriverStation::ReportError("PID Init");

		leftMotorF->SetEncPosition(0);
		rightMotorF->SetEncPosition(0);
		leftMotorB->SetEncPosition(0);
		rightMotorB->SetEncPosition(0);

		leftMotorF->SetPID(p, i, d);
		rightMotorF->SetPID(p, i, d);
		leftMotorB->SetPID(p, i, d);
		rightMotorB->SetPID(p, i, d);

		leftMotorF->SetSensorDirection(true);
		rightMotorF->SetSensorDirection(true);
		leftMotorB->SetSensorDirection(true);
		rightMotorB->SetSensorDirection(true);

		leftMotorF->SetControlMode(CANSpeedController::kPosition);
		rightMotorF->SetControlMode(CANSpeedController::kPosition);
		leftMotorB->SetControlMode(CANSpeedController::kPosition);
		rightMotorB->SetControlMode(CANSpeedController::kPosition);

		leftMotorF->Set(0);
		rightMotorF->Set(0);
		leftMotorB->Set(0);
		rightMotorB->Set(0);
	}

	void driveInit() {
		leftMotorF->SetControlMode(CANSpeedController::kPercentVbus);
		rightMotorF->SetControlMode(CANSpeedController::kPercentVbus);
		leftMotorB->SetControlMode(CANSpeedController::kPercentVbus);
		rightMotorB->SetControlMode(CANSpeedController::kPercentVbus);

		resetMotors();
	}

	void autonomousInit() {
		timeInitial = Timer::GetFPGATimestamp();
		timeInitial2 = timeInitial + 1.5;

		leftMotorF->SetEncPosition(0);
		rightMotorF->SetEncPosition(0);
		leftMotorB->SetEncPosition(0);
		rightMotorB->SetEncPosition(0);

		initialY = imu->GetAngleY();
		initialZ = ahrs->GetYaw();

		timesAuto++;

		DriverStation::ReportError("Auto init");
	}

	void teleoperatedInit() {
		//memset(buff, 0, sizeof(buff));

		//rightMotorF->SetInverted(false);

		DriverStation::ReportError("Teleop Init");

		leftMotorF->SetControlMode(CANSpeedController::kPercentVbus);
		rightMotorF->SetControlMode(CANSpeedController::kPercentVbus);
		leftMotorB->SetControlMode(CANSpeedController::kPercentVbus);
		rightMotorB->SetControlMode(CANSpeedController::kPercentVbus);
		leftShooter->SetControlMode(CANSpeedController::kPercentVbus);
		rightShooter->SetControlMode(CANSpeedController::kPercentVbus);
		indexMotor->SetControlMode(CANSpeedController::kPercentVbus);
	}

	bool turnToAngle (float degreesFromInit) {
		DriverStation::ReportError("Turning");
		// good turning PID's are 0.7, 0, 0
		if (pidInit == false) {
			PIDInit(0.7, 0, 0);
			pidInit = true;
		}

		if (initialZ + degreesFromInit > ahrs->GetYaw()) {
			resetMotors();
			leftMotorF->Set(leftMotorF->GetSetpoint() + 35);
			rightMotorF->Set(rightMotorF->GetSetpoint() + 35);
			return false;
		} else {
			pidInit = false;
			return true;
		}
	}

	bool driveDistance (float ticks, float p, float i, float d) {
		if (pidInit == false) {
			pidInit = true;
			PIDInit(p, i, d);
		}

		int x = -ticks;
		leftMotorF->Set(x);
		rightMotorF->Set(-x);
		leftMotorB->Set(x);
		rightMotorB->Set(-x);
		if (abs(abs(leftMotorF->GetEncPosition()) - ticks) < 500 &&
				abs(abs(rightMotorF->GetEncPosition()) - ticks) < 500) {
			return true;
		} else {
			return false;
		}
	}

	double convertToInches (double voltage) {
		return (voltage / 0.000977 / 10.0 / 2.54);
	}

	void Autonomous()
	{
		while (IsAutonomous()) {
			if (IsEnabled()) {
				resetMotors();
				SmartDashboard::PutNumber("Left Speed", leftMotorF->GetSpeed());
				SmartDashboard::PutNumber("IMU Y", imu->GetAngleY());
				SmartDashboard::PutNumber("InitialZ", initialZ);

				autoSelected = *((std::string*)chooser->GetSelected());

				if (autoInit == false) {
					autoInit = true;
					autonomousInit();
					autoState = INITFORWARD;
					if (autoSelected == autoNothing)
						DriverStation::ReportError("Auto Nothing");
					else if (autoSelected == autoLowBar) {
						shooterA->Set(DoubleSolenoid::kForward);
						shooterB->Set(DoubleSolenoid::kForward);
						blocker->Set(DoubleSolenoid::kForward);
						//shooterB->Set(DoubleSolenoid::kForward);
						//wasDown=true;
						DriverStation::ReportError("Auto Low Bar");
					} else if (autoSelected == autoRoughTerrain)
						DriverStation::ReportError("Auto Rough Terrain");
					else if (autoSelected == autoRamparts)
						DriverStation::ReportError("Auto Ramparts");
				}

				if (autoSelected != autoNothing && autoSelected != autoLowBar) {
					if (timeInitial + 1.5 > Timer::GetFPGATimestamp()) {
						drive->ArcadeDrive(1.0, 0);
					} else {
						drive->ArcadeDrive(0.0, 0);
					}
				}/* else if (autoSelected == autoLowBar) {
					if (autoState == INITFORWARD) {
						if (timeInitial + 1.5 > Timer::GetFPGATimestamp()) {
							drive->ArcadeDrive(0.0, 0);
						} else {
							if (timeInitial2 + )
						}
					} else {
						drive->ArcadeDrive(0.0, 0);
					}
				} */

				/*
				if (autoState != STOP) {
					if (autoSelected == autoNothing) {
						autoState = STOP;
						DriverStation::ReportWarning("Auto done");
					} else {
						if (autoState == INITFORWARD) {
							if (timeInitial + 2.5 > Timer::GetFPGATimestamp()) {
								drive->ArcadeDrive(0.5, 0);
								if (ahrs->GetRoll() < -6) {
									DriverStation::ReportError("Change this later");
									autoState = FOUNDRAMPF;
									timeInitial = Timer::GetFPGATimestamp();
								}
							} else {
								autoState = STOP;
								DriverStation::ReportWarning("Aborting autonomous; didn't detect ramp");
							}
						} else if (autoState == FOUNDRAMPF) {
							// driving over defense
							if (timeInitial + 0.75 > Timer::GetFPGATimestamp()) {
								drive->ArcadeDrive(1.0, 0);
							} else {
								if (abs(ahrs->GetRoll()) < 3) {
									// found ground
									autoState = CORRECTINGANGLE;
									timeInitial = Timer::GetFPGATimestamp();
									degreesToCorrect = ahrs->GetYaw() - initialZ;
								} else {
									// needs extra forward to find ground
									autoState = ONDEFENSEF;
									timeInitial = Timer::GetFPGATimestamp();
								}
							}
						} else if (autoState == ONDEFENSEF) {
							if (timeInitial + 1.25 > Timer::GetFPGATimestamp()) {
								drive->ArcadeDrive(0.75, 0);
								if (abs(ahrs->GetRoll()) < 3) {
									// found ground
									autoState = CORRECTINGANGLE;
									timeInitial = Timer::GetFPGATimestamp();
									// THIS MAY BE WRONG; CHECK IT LATER
									degreesToCorrect = ahrs->GetYaw() - initialZ;
								}
							} else {
								autoState = STOP;
								DriverStation::ReportWarning("Aborting autonomous; didn't detect passing defense");
							}
						} else if (autoState == CORRECTINGANGLE) {
							if (turnToAngle(degreesToCorrect) == false) {
								// do nothing
							} else {
								autoState = INITBACKWARD;
							}
						} else if (autoState == INITBACKWARD) {
							if (timeInitial + 1.5 > Timer::GetFPGATimestamp()) {
								drive->ArcadeDrive(-0.5, 0);
								if (ahrs->GetRoll() > 6) {
									autoState = FOUNDRAMPB;
									timeInitial = Timer::GetFPGATimestamp();
								}
							} else {
								autoState = STOP;
								DriverStation::ReportWarning("Aborting autonomous; didn't detect ramp backwards");
							}
						} else if (autoState == FOUNDRAMPB) {
							// driving over defense
							if (timeInitial + 0.75 > Timer::GetFPGATimestamp()) {
								drive->ArcadeDrive(-1.0, 0);
							} else {
								if (abs(ahrs->GetRoll()) < 3) {
									// found ground
									autoState = STOP;
									DriverStation::ReportWarning("Auto done");
								} else {
									// needs extra backward to find ground
									autoState = ONDEFENSEB;
									timeInitial = Timer::GetFPGATimestamp();
								}
							}
						} else if (autoState == ONDEFENSEB) {
							if (timeInitial + 1.25 > Timer::GetFPGATimestamp()) {
								drive->ArcadeDrive(-0.75, 0);
								if (abs(ahrs->GetRoll()) < 3) {
									// found ground
									autoState = STOP;
									DriverStation::ReportWarning("Auto done");
								}
							} else {
								autoState = STOP;
								DriverStation::ReportWarning("Aborting autonomous; didn't detect passing defense backwards");
							}
						}

						// good PID's for forward are 0.1, 0, 14.35
					}
				} else {
					drive->ArcadeDrive(0.0, 0.0);
				}
*/
				//Wait(0.005);
			} else {
				autoInit = false;
			}
		}
	}

	void OperatorControl()
	{
		resetMotors();
		bool wasDown,wasUp;
		double lastResetTime = Timer::GetFPGATimestamp();
		//myRobot.SetSafetyEnabled(true);
		while (IsOperatorControl())
		{
			if (IsEnabled()) {
				if (teleopInit == false) {
					teleopInit = true;
					teleoperatedInit();
				}
				/*
				SmartDashboard::PutBoolean("Limit",chevalUpLimit->Get());
				if(!chevalUpLimit->Get() && joystickMain->GetRawButton(5))
				{
					chevalAngler->Set(0.2);
				}
				else if(joystickMain->GetRawButton(6))
				{
					chevalAngler->Set(-0.2);
				}
				else
				{
					chevalAngler->Set(0);
				}
				*/

				if (Timer::GetFPGATimestamp() - lastResetTime > 0.1)
				{
					resetMotors();
					lastResetTime = Timer::GetFPGATimestamp();
				}
				SmartDashboard::PutNumber("Left Speed", leftMotorF->GetSpeed());
				SmartDashboard::PutNumber("Joy Forward", -joystickMain->GetRawAxis(1));
				SmartDashboard::PutNumber("Joy Side", -joystickMain->GetRawAxis(4));
				SmartDashboard::PutNumber("XRLV Max", ultra->GetVoltage() / 0.00488 / 2.54);
				SmartDashboard::PutNumber("IMU Y", imu->GetAngleY());
				//SmartDashboard::PutNumber("HRLV Max", ultra2->GetVoltage() / 0.00488 / 2.54);
				//SmartDashboard::PutNumber("HRLV Max", ultra2->GetVoltage() / 0.000977 / 10.0 / 2.54);
				//SmartDashboard::PutNumber("Left Ultra", frontL->GetRangeInches());
				//SmartDashboard::PutNumber("Right Ultra", frontR->GetRangeInches());

				// x button on joystick to reverse forward
				/*
				if (joystickMain->GetRawButton(3)) {
					reverseForward = !reverseForward;
				}
				*/

//				float rawTurn = -joystickMain->GetRawAxis(4);
//				// gets out of turning deadzone
//				if (abs(rawTurn) > deadZone) {
//					if (rawTurn > 0)
//						drive->ArcadeDrive(joystickMain->GetRawAxis(1) * (reverseForward ? 1.0 : -1.0),
//								(rawTurn * turnSlope + verticalShift));
//					else
//						drive->ArcadeDrive(joystickMain->GetRawAxis(1) * (reverseForward ? 1.0 : -1.0),
//								(rawTurn * turnSlope - verticalShift));
//				} else {
//					// within turning deadzone
//					drive->ArcadeDrive(-joystickMain->GetRawAxis(1), 0);
//				}

				// 2 for left trigger, 3 for right trigger
				if (joystickMain->GetRawAxis(2) >= 0.1 || joystickSecond->GetRawAxis(2) >= 0.1) {
					// intake
					leftShooter->Set(-0.45);
					rightShooter->Set(0.45);
					indexMotor->Set(-0.45);
				} else if (joystickMain->GetRawAxis(3) >= 0.1 || joystickSecond->GetRawAxis(3) >= 0.1) {
					// shoot
					leftShooter->Set(1.0);
					rightShooter->Set(-1.0);
					indexMotor->Set(0);
				} else if (joystickMain->GetRawButton(6) || joystickSecond->GetRawButton(6)) {
					// low speed for low goal
					leftShooter->Set(0.35);
					rightShooter->Set(-0.35);
				} else {
					leftShooter->Set(0);
					rightShooter->Set(0);
					indexMotor->Set(0);
				}

				if (joystickMain->GetRawButton(5)) {
					indexMotor->Set(0.45);
				}

				if (joystickMain->GetRawButton(4)) {
					//up
					shooterA->Set(DoubleSolenoid::kReverse);
					shooterB->Set(DoubleSolenoid::kReverse);
					blocker->Set(DoubleSolenoid::kReverse);
					//shooterB->Set(DoubleSolenoid::kReverse);
					wasUp=true;
				} else if (joystickMain->GetRawButton(1)) {
					//down
					shooterA->Set(DoubleSolenoid::kForward);
					shooterB->Set(DoubleSolenoid::kForward);
					blocker->Set(DoubleSolenoid::kForward);
					//shooterB->Set(DoubleSolenoid::kForward);
					wasDown=true;
				}
				/*
				if (joystickMain->GetRawButton(2)) {
					blocker->Set(DoubleSolenoid::kForward);
				} else if (joystickMain->GetRawButton(3)) {
					blocker->Set(DoubleSolenoid::kReverse);
				}
				if (blocker->Get() == DoubleSolenoid::kForward) {
					gateMotor->Set(-1.0);
				} else {
					gateMotor->Set(0);
				}
				*/

				//SmartDashboard::PutNumber("X Val", joystickSecond->GetRawAxis(4));
				SmartDashboard::PutNumber("Y Val", joystickSecond->GetRawAxis(1));
				//SmartDashboard::PutNumber("X Position", xPosition);
				//if ((joystickSecond->GetRawButton(5) || joystickSecond->GetRawButton(6)) && abs(joystickSecond->GetRawAxis(4)) > 0.2)
					//xPosition += joystickSecond->GetRawAxis(4) / 80.0;
				// CAMERA BUT SERVO IS BROKEN
				DriverStation::ReportError("Camera moving; commented out now");
				/*
				if ((joystickSecond->GetRawButton(5) || joystickSecond->GetRawButton(6)) && abs(joystickSecond->GetRawAxis(1)) > 0.2)
					yPosition += joystickSecond->GetRawAxis(1) / 80.0;

				SmartDashboard::PutNumber("Y Position", yPosition);
				if (joystickSecond->GetRawButton(1)) {
					// for downish
					//xPosition = 0.5;
					yPosition = 0.8;
				} else if (joystickSecond->GetRawButton(2)) {
					// for straight forward
					//xPosition = 0.5;
					yPosition = 0.76;
				} else if (joystickSecond->GetRawButton(3)) {
					// for up
					//xPosition = 0.3;
					yPosition = 0.7;
				} else if (joystickSecond->GetRawButton(4)) {
					//xPosition = 0;
					yPosition = 0.7;
				}*/

				/*
				//if (xPosition > 1)
				//	xPosition = 1;
				if (yPosition > 0.8)
					yPosition = 0.8;
				//if (xPosition < 0)
				//	xPosition = 0;
				if (yPosition < 0.2)
					yPosition = 0.2;
				//xServo->Set(xPosition);
				yServo->Set(yPosition);
				*/

				/*
				if (joystickSecond->GetRawButton(7) || joystickSecond->GetRawButton(8)) {
					float autoSpeed = SmartDashboard::GetNumber("Auto Speed", 0);
					if (!abortAuto) {
						if (autoInit == false) {
							autoInit = true;
							autonomousInit();
							autoState = INITFORWARD;
						}
						if (autoState == INITFORWARD) {
							if (timeInitial + 1.5 > Timer::GetFPGATimestamp()) {
								drive->ArcadeDrive(autoSpeed, 0);
								if (imu->GetAngleY() - initialY > 2.5) {
									autoState = FOUNDRAMPF;
									timeInitial = Timer::GetFPGATimestamp();
								}
							} else {
								autoState = STOP;
								DriverStation::ReportWarning("Aborting autonomous; didn't detect ramp");
							}
						} else if (autoState == FOUNDRAMPF) {
							// driving over defense
							if (timeInitial + 0.75 > Timer::GetFPGATimestamp()) {
								drive->ArcadeDrive(1.0, 0);
							} else {
								if (abs(imu->GetAngleY() - initialY) < 2.5) {
									// found ground
									autoState = CORRECTINGANGLE;
									timeInitial = Timer::GetFPGATimestamp();
									degreesToCorrect = ahrs->GetYaw() - initialZ;
								} else {
									DriverStation::ReportWarning("Giving extra push");
									// needs extra forward to find ground
									autoState = ONDEFENSEF;
									timeInitial = Timer::GetFPGATimestamp();
								}
							}
						} else if (autoState == ONDEFENSEF) {
							if (timeInitial + 1.25 > Timer::GetFPGATimestamp()) {
								drive->ArcadeDrive(0.75, 0);
								if (abs(imu->GetAngleY() - initialY) < 2.5) {
									// found ground
									autoState = STOP;
									timeInitial = Timer::GetFPGATimestamp();
									// THIS MAY BE WRONG; CHECK IT LATER
									degreesToCorrect = ahrs->GetYaw() - initialZ;
									DriverStation::ReportWarning("Auto done");
								}
							} else {
								autoState = STOP;
								DriverStation::ReportWarning("Aborting autonomous; didn't detect passing defense");
							}
						}
					}
				} else {
					autoInit = false;
				}
				*/
			} else {
				teleopInit = false;
			}
		}
	}

	void resetMotors() {
		if (drive->GetGlobalError().GetCode() != 0) {
			drive->GetGlobalError().GetOriginatingObject()->ClearError();

			leftMotorF->ClearError();
			rightMotorF->ClearError();
			leftMotorB->ClearError();
			rightMotorB->ClearError();
			leftShooter->ClearError();
			rightShooter->ClearError();
			indexMotor->ClearError();

			drive->GetGlobalError().Clear();

			leftMotorF->EnableControl();
			rightMotorF->EnableControl();
			leftMotorB->EnableControl();
			rightMotorB->EnableControl();
			leftShooter->EnableControl();
			rightShooter->EnableControl();
			indexMotor->EnableControl();
		}
		drive->GetGlobalError().Clear();
		leftMotorF->EnableControl();
		rightMotorF->EnableControl();
		leftMotorB->EnableControl();
		rightMotorB->EnableControl();
		leftShooter->EnableControl();
		rightShooter->EnableControl();
		indexMotor->EnableControl();
	}

	void Test()
	{
	}
};

START_ROBOT_CLASS(Robot)
