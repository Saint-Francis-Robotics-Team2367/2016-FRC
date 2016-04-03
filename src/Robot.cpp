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
		FOUNDRAMP,
		ONDEFENSE,
		PASSEDDEFENSE,
		CORRECTINGANGLE,
		BACKWARD,
		FINISHED,

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

	Servo *xServo;
	Servo *yServo;
	float xPosition;
	float yPosition;

	bool reverseForward = false;

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

	DoubleSolenoid *shooterA;
	DoubleSolenoid *shooterB;

	//Ultrasonic *frontL;
	//Ultrasonic *frontR;
	AnalogInput *ultra;
	//AnalogInput *ultra2;

	bool autoInit;
	bool teleopInit;
	bool pidInit;

	float initialY, initialZ, timeInitial, timeInitial2;
	int timesAuto;
	float timeReceive;

	RobotState robotState;
	//AutoSelection autoSelection;
	AutoState autoState;

	UDPSocket *sock;
	uint8_t buff[641];

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
		rightMotorF = new CANTalon(12);
		rightMotorB = new CANTalon(11);
		leftMotorF = new CANTalon(14);
		leftMotorB = new CANTalon(13);

		leftShooter = new CANTalon(17);
		rightShooter = new CANTalon(16);
		indexMotor = new CANTalon(15);

		joystickMain = new Joystick(0);
		joystickSecond = new Joystick(1);

		xServo = new Servo(1);
		yServo = new Servo(0);
		xPosition = 0.5;
		yPosition = 0.5;

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

		shooterA = new DoubleSolenoid(0, 1);
		shooterB = new DoubleSolenoid(2, 3);

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

		sock = new UDPSocket(5809);
		//Note SmartDashboard is not initialized here, wait until RobotInit to make SmartDashboard calls
		//myRobot.SetExpiration(0.1);
}

	void RobotInit()
	{
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
		timeInitial2 = 0;

		leftMotorF->SetEncPosition(0);
		rightMotorF->SetEncPosition(0);
		leftMotorB->SetEncPosition(0);
		rightMotorB->SetEncPosition(0);

		initialY = imu->GetAngleY();
		initialZ = imu->GetAngleZ() - 12;

		timesAuto++;

		DriverStation::ReportError("Resetting motor values");
	}

	void teleoperatedInit() {
		memset(buff, 0, sizeof(buff));

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

		if (initialZ + degreesFromInit > imu->GetAngleZ()) {
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
				//SmartDashboard::PutNumber("AngleX", imu->GetAngleX());
				//SmartDashboard::PutNumber("AngleY", imu->GetAngleY());
				//SmartDashboard::PutNumber("AngleZ", imu->GetAngleZ());
				//SmartDashboard::PutNumber("Left P", leftMotorF->GetP());
				//SmartDashboard::PutNumber("Left I",leftMotorF->GetI());
				//SmartDashboard::PutNumber("Left D", leftMotorF->GetD());
				//SmartDashboard::PutNumber("Left Encoder", leftMotorF->GetEncPosition());
				//SmartDashboard::PutNumber("Right Encoder", rightMotorF->GetEncPosition());
				SmartDashboard::PutNumber("Left Speed", leftMotorF->GetSpeed());
				//SmartDashboard::PutNumber("Left Ultra", frontL->GetRangeInches());
				//SmartDashboard::PutNumber("Right Ultra", frontR->GetRangeInches());
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
						//wasDown=true;
						DriverStation::ReportError("Auto Low Bar");
					} else if (autoSelected == autoRoughTerrain)
						DriverStation::ReportError("Auto Rough Terrain");
					else if (autoSelected == autoRamparts)
						DriverStation::ReportError("Auto Ramparts");
				}
				if (autoState != STOP) {
					if (autoSelected == autoNothing) {
						autoState = STOP;
					} else if (autoSelected == autoLowBar) {
						if (autoState == INITFORWARD) {
							if (timeInitial + 1.5 > Timer::GetFPGATimestamp()) {
								drive->ArcadeDrive(0.5, 0);
								if (ahrs->GetRoll() < -6) {
									autoState = FOUNDRAMP;
									timeInitial = Timer::GetFPGATimestamp();
								}
							} else {
								autoState = STOP
										DriverStation::ReportWarning("Aborting autonomous; didn't detect ramp");
							}
						} else if (autoState == FOUNDRAMP) {
							// driving over defense
							if (timeInitial + 0.75 > Timer::GetFPGATimestamp()) {
								drive->ArcadeDrive(1.0, 0);
							} else {
								if (abs(ahrs->GetRoll()) < 3) {
									// found ground
									autoState = CORRECTINGANGLE;
									timeInitial = Timer::GetFPGATimestamp();
								} else {
									// needs extra forward to find ground
									autoState = ONDEFENSE;
									timeInitial = Timer::GetFPGATimestamp();
								}
							}
						} else if (autoState == ONDEFENSE) {
							drive->ArcadeDrive(0.75, 0);
							autoState = STOP;
							DriverStation::ReportWarning("Aborting autonomous; didn't detect passing defense");
						} else if (autoState == CORRECTINGANGLE) {

						}
						/*
					if (autoState == INITFORWARD) {
						if (timeInitial + 2 > Timer::GetFPGATimestamp()) {
							timeInitial2 = Timer::GetFPGATimestamp();
						} else if (timeInitial2 + 5 > Timer::GetFPGATimestamp()) {
							drive->ArcadeDrive(0.55, 0);
						} else {
							DriverStation::ReportError("Auto Done");
							autoState = STOP;
						}
					} else {
						timeInitial = 0;
						timeInitial2 = 0;
						drive->ArcadeDrive(0.0, 0.0);
					}

					/*
					if (timeInitial + 8 > Timer::GetFPGATimestamp()) {
						double distance = convertToInches(ultra->GetVoltage());
						if (distance > 15 && distance < 30 && timesAuto < 3) {
							timesAuto++;
						} else if (distance > 15 && distance < 30 && timesAuto >= 3) {
							autoState = STOP;
							DriverStation::ReportError("Found wall");
						} else {
							drive->ArcadeDrive(0.45, 0);
						}
					} else {
						autoState = STOP;
						DriverStation::ReportError("Didn't find wall; aborting autonomous");
					}
						 */

						/*
					// good PID's are 0.1, 0, 14.35
					if (autoState == INITFORWARD && imu->GetAngleY() - initialY < 2.5) {

						if (timeInitial + 4 > Timer::GetFPGATimestamp()) {
							drive->ArcadeDrive(0.45, 0);
						} else {
							DriverStation::ReportError("Couldn't find defense's ramp; aborting autonomous");
							autoState = STOP;
						}

					} else if (autoState == INITFORWARD && imu->GetAngleY() - initialY > 2.5) {
						autoState = FOUNDRAMP;
						DriverStation::ReportError("Found Ramp");
						timeInitial = Timer::GetFPGATimestamp();
					} else if (autoState == FOUNDRAMP) {
						/*
						if (driveDistance(5000, 0.1, 0, 14.35) == true) {
							// finished driving distance
							autoState = DRIVENBACK;
							DriverStation::ReportError("Finished Back");

							leftMotorF->SetControlMode(CANSpeedController::kPercentVbus);
							rightMotorF->SetControlMode(CANSpeedController::kPercentVbus);

							resetMotors();
							timeInitial = Timer::GetFPGATimestamp();
						} else {
							// do nothing - continue driving
						}

						if (timeInitial + 2 > Timer::GetFPGATimestamp()) {
							drive->ArcadeDrive(-0.45, 0);
						} else {
							DriverStation::ReportError("Finished driving back");
							timeInitial = Timer::GetFPGATimestamp();
						}
					} else if (autoState == DRIVENBACK) {
						if (timeInitial + 4 > Timer::GetFPGATimestamp())
							drive->ArcadeDrive(0.7, 0.0);
						else {
							autoState = STOP;
						}
						/*
						else if (timeInitial + 1.25 > Timer::GetFPGATimestamp()) {
							//autoState = ONDEFENSE;
							timeInitial = Timer::GetFPGATimestamp();
							drive->ArcadeDrive(0.7, 0.0);
						} else {
							DriverStation::ReportError("Didn't find angle from being on top of defense; aborting autonomous");
							autoState = STOP;
						}

					}  else if (autoState == ONDEFENSE) {
						if (timeInitial + 1.25 > Timer::GetFPGATimestamp()) {
							if (abs(imu->GetAngleY() - initialY) < 2.5) {
								autoState = PASSEDDEFENSE;
							} else {
								drive->ArcadeDrive(0.7, 0);
							}
						} else {
							autoState = STOP;
							DriverStation::ReportError("Didn't find loss of angle from passing defense; aborting autonomous");
						}
					} else if (autoState == PASSEDDEFENSE) {

						if (timeInitial + 2.5 > Timer::GetFPGATimestamp()) {
							double distance = convertToInches(ultra2->GetVoltage());
							if (distance > 15 && distance < 30 && timesAuto < 3) {
								timesAuto++;
							} else if (distance > 15 && distance < 30 && timesAuto >= 3) {
								autoState = STOP;
								DriverStation::ReportError("Found wall");
							} else {
								drive->ArcadeDrive(0.45, 0);
							}
						} else {
							autoState = STOP;
							DriverStation::ReportError("Didn't find wall; aborting autonomous");
						}

					}  else if (autoState == STOP) {
						initialY = 0;
						timeInitial = 0;
						drive->ArcadeDrive(0.0, 0.0);
					}*/
					} else {
						if (timeInitial + 1.5 > Timer::GetFPGATimestamp()) {
							drive->ArcadeDrive(1.0, 0);
						} else {
							DriverStation::ReportError("Auto Done");
							autoState = STOP;
							timeInitial = 0;
							timeInitial2 = 0;
							drive->ArcadeDrive(0.0, 0.0);
						}
					}
				}
				/*else if (autoSelected == autoRoughTerrain) {

					if (timeInitial + 8 > Timer::GetFPGATimestamp()) {
						double distance = convertToInches(ultra->GetVoltage());
						if (distance > 15 && distance < 30 && timesAuto < 3) {
							timesAuto++;
						} else if (distance > 15 && distance < 30 && timesAuto >= 3) {
							autoState = STOP;
							DriverStation::ReportError("Found wall");
						} else {
							drive->ArcadeDrive(0.45, 0);
						}
					} else {
						autoState = STOP;
						DriverStation::ReportError("Didn't find wall; aborting autonomous");
					}


					// good PID's are 0.1, 0, 14.35
					if (autoState == INITFORWARD && imu->GetAngleY() - initialY < 2.5) {

						if (timeInitial + 4 > Timer::GetFPGATimestamp()) {
							drive->ArcadeDrive(0.45, 0);
						} else {
							DriverStation::ReportError("Couldn't find defense's ramp; aborting autonomous");
							autoState = STOP;
						}

					} else if (autoState == INITFORWARD && imu->GetAngleY() - initialY > 2.5) {
						autoState = FOUNDRAMP;
						DriverStation::ReportError("Found Ramp");
						timeInitial = Timer::GetFPGATimestamp();
					} else if (autoState == FOUNDRAMP) {

						if (driveDistance(5000, 0.1, 0, 14.35) == true) {
							// finished driving distance
							autoState = DRIVENBACK;
							DriverStation::ReportError("Finished Back");

							leftMotorF->SetControlMode(CANSpeedController::kPercentVbus);
							rightMotorF->SetControlMode(CANSpeedController::kPercentVbus);

							resetMotors();
							timeInitial = Timer::GetFPGATimestamp();
						} else {
							// do nothing - continue driving
						}

						if (timeInitial + 2 > Timer::GetFPGATimestamp()) {
							drive->ArcadeDrive(-0.45, 0);
						} else {
							DriverStation::ReportError("Finished driving back");
							timeInitial = Timer::GetFPGATimestamp();
						}
					} else if (autoState == DRIVENBACK) {
						if (timeInitial + 4 > Timer::GetFPGATimestamp())
							drive->ArcadeDrive(0.85, 0.0);
						else {
							autoState = STOP;
						}

						else if (timeInitial + 1.25 > Timer::GetFPGATimestamp()) {
							//autoState = ONDEFENSE;
							timeInitial = Timer::GetFPGATimestamp();
							drive->ArcadeDrive(0.7, 0.0);
						} else {
							DriverStation::ReportError("Didn't find angle from being on top of defense; aborting autonomous");
							autoState = STOP;
						}

					}  else if (autoState == ONDEFENSE) {
						if (timeInitial + 1.25 > Timer::GetFPGATimestamp()) {
							if (abs(imu->GetAngleY() - initialY) < 2.5) {
								autoState = PASSEDDEFENSE;
							} else {
								drive->ArcadeDrive(0.7, 0);
							}
						} else {
							autoState = STOP;
							DriverStation::ReportError("Didn't find loss of angle from passing defense; aborting autonomous");
						}
					} else if (autoState == PASSEDDEFENSE) {

						if (timeInitial + 2.5 > Timer::GetFPGATimestamp()) {
							double distance = convertToInches(ultra2->GetVoltage());
							if (distance > 15 && distance < 30 && timesAuto < 3) {
								timesAuto++;
							} else if (distance > 15 && distance < 30 && timesAuto >= 3) {
								autoState = STOP;
								DriverStation::ReportError("Found wall");
							} else {
								drive->ArcadeDrive(0.45, 0);
							}
						} else {
							autoState = STOP;
							DriverStation::ReportError("Didn't find wall; aborting autonomous");
						}

					}  else if (autoState == STOP) {
						initialY = 0;
						timeInitial = 0;
						drive->ArcadeDrive(0.0, 0.0);
					}
				} else if (autoSelected == autoRamparts) {

					if (timeInitial + 8 > Timer::GetFPGATimestamp()) {
						double distance = convertToInches(ultra->GetVoltage());
						if (distance > 15 && distance < 30 && timesAuto < 3) {
							timesAuto++;
						} else if (distance > 15 && distance < 30 && timesAuto >= 3) {
							autoState = STOP;
							DriverStation::ReportError("Found wall");
						} else {
							drive->ArcadeDrive(0.45, 0);
						}
					} else {
						autoState = STOP;
						DriverStation::ReportError("Didn't find wall; aborting autonomous");
					}


					// good PID's are 0.1, 0, 14.35
					if (autoState == INITFORWARD && imu->GetAngleY() - initialY < 2.5) {

						if (timeInitial + 4 > Timer::GetFPGATimestamp()) {
							drive->ArcadeDrive(0.45, 0);
						} else {
							DriverStation::ReportError("Couldn't find defense's ramp; aborting autonomous");
							autoState = STOP;
						}

					} else if (autoState == INITFORWARD && imu->GetAngleY() - initialY > 2.5) {
						autoState = FOUNDRAMP;
						DriverStation::ReportError("Found Ramp");
						timeInitial = Timer::GetFPGATimestamp();
					} else if (autoState == FOUNDRAMP) {

						if (driveDistance(5000, 0.1, 0, 14.35) == true) {
							// finished driving distance
							autoState = DRIVENBACK;
							DriverStation::ReportError("Finished Back");

							leftMotorF->SetControlMode(CANSpeedController::kPercentVbus);
							rightMotorF->SetControlMode(CANSpeedController::kPercentVbus);

							resetMotors();
							timeInitial = Timer::GetFPGATimestamp();
						} else {
							// do nothing - continue driving
						}

						if (timeInitial + 2 > Timer::GetFPGATimestamp()) {
							drive->ArcadeDrive(-0.45, 0);
						} else {
							DriverStation::ReportError("Finished driving back");
							timeInitial = Timer::GetFPGATimestamp();
						}
					} else if (autoState == DRIVENBACK) {

						if (timeInitial + 3.0 > Timer::GetFPGATimestamp())
							drive->ArcadeDrive(1.0, 0.0);
						else {
							autoState = STOP;
						}
						/*
						else if (timeInitial + 1.25 > Timer::GetFPGATimestamp()) {
							//autoState = ONDEFENSE;
							timeInitial = Timer::GetFPGATimestamp();
							drive->ArcadeDrive(0.7, 0.0);
						} else {
							DriverStation::ReportError("Didn't find angle from being on top of defense; aborting autonomous");
							autoState = STOP;
						}

					}  else if (autoState == ONDEFENSE) {
						if (timeInitial + 1.25 > Timer::GetFPGATimestamp()) {
							if (abs(imu->GetAngleY() - initialY) < 2.5) {
								autoState = PASSEDDEFENSE;
							} else {
								drive->ArcadeDrive(0.7, 0);
							}
						} else {
							autoState = STOP;
							DriverStation::ReportError("Didn't find loss of angle from passing defense; aborting autonomous");
						}
					} else if (autoState == PASSEDDEFENSE) {

						if (timeInitial + 2.5 > Timer::GetFPGATimestamp()) {
							double distance = convertToInches(ultra2->GetVoltage());
							if (distance > 15 && distance < 30 && timesAuto < 3) {
								timesAuto++;
							} else if (distance > 15 && distance < 30 && timesAuto >= 3) {
								autoState = STOP;
								DriverStation::ReportError("Found wall");
							} else {
								drive->ArcadeDrive(0.45, 0);
							}
						} else {
							autoState = STOP;
							DriverStation::ReportError("Didn't find wall; aborting autonomous");
						}

					}  else if (autoState == STOP) {
						initialY = 0;
						timeInitial = 0;
						drive->ArcadeDrive(0.0, 0.0);
					}
				} else if (autoSelected == autoMoat) {
					if (imu->GetAngleY() - initialY < 3 && autoState == INITFORWARD) {

						if (timeInitial + 4 < Timer::GetFPGATimestamp()) {
							drive->ArcadeDrive(0.45, 0);
						} else {
							DriverStation::ReportError("Couldn't find defense's ramp; abandoned autonomous");
							autoState = STOP;
						}

					} else if (imu->GetAngleY() - initialY > 2.5 && autoState == INITFORWARD) {
						autoState = FOUNDRAMP;
						DriverStation::ReportError("Found Ramp");
					} else if (autoState == FOUNDRAMP) {
						if (driveDistance(5000, 0.1, 0, 14.35) == true) {
							// finished driving distance
							autoState = DRIVENBACK;
							DriverStation::ReportError("Finished Back");

							leftMotorF->SetControlMode(CANSpeedController::kPercentVbus);
							rightMotorF->SetControlMode(CANSpeedController::kPercentVbus);

							resetMotors();
							timeInitial = Timer::GetFPGATimestamp();
						} else {
							// do nothing - continue driving
						}
					} else if (autoState == DRIVENBACK) {
						DriverStation::ReportError("Driving Forward");

						if (timeInitial + 1.25 > Timer::GetFPGATimestamp())
							drive->ArcadeDrive(1.0, 0.0);
						else
							autoState = STOP;
					} else if (autoState == STOP) {
						initialY = 0;
						timeInitial = 0;
						drive->ArcadeDrive(0.0,0.0);
					}
				} else if (autoSelected == autoRockWall) {
					if (imu->GetAngleY() - initialY < 3 && autoState == INITFORWARD) {
						DriverStation::ReportError("Init Forward");
						SmartDashboard::PutNumber("Diff", imu->GetAngleY() - initialY);
						if (timeInitial + 4 < Timer::GetFPGATimestamp())
							drive->ArcadeDrive(0.45, 0);
						else
							autoState = STOP;
					} else if (imu->GetAngleY() - initialY > 2.5 && autoState == INITFORWARD) {
						DriverStation::ReportError("Found Ramp");

						autoState = FOUNDRAMP;
						drive->ArcadeDrive(0.0, 0.0);

						PIDInit(0.2, 0, 0.1);
					} else if (autoState == FOUNDRAMP) {
						DriverStation::ReportError("Driving Back");
						int x = -5000;
						leftMotorF->Set(x);
						rightMotorF->Set(-x);
						if (abs(abs(leftMotorF->GetEncPosition()) - 5000) < 500 &&
								abs(abs(rightMotorF->GetEncPosition()) - 5000) < 500) {
							autoState = DRIVENBACK;
							DriverStation::ReportError("Finished Back");

							leftMotorF->SetControlMode(CANSpeedController::kPercentVbus);
							rightMotorF->SetControlMode(CANSpeedController::kPercentVbus);

							resetMotors();
							timeInitial = Timer::GetFPGATimestamp();
						}
					} else if (autoState == DRIVENBACK) {
						DriverStation::ReportError("Driving Forward");

						if (timeInitial + 1.25 > Timer::GetFPGATimestamp())
							drive->ArcadeDrive(1.0, 0.0);
						else
							autoState = STOP;
						//autoState = STOP;
					} else if (autoState == STOP) {
						initialY = 0;
						timeInitial = 0;
						drive->ArcadeDrive(0.0, 0.0);
					}
				} else if (autoSelected == autoRoughTerrain) {
					if (imu->GetAngleY() - initialY < 3 && autoState == INITFORWARD) {
						DriverStation::ReportError("Init Forward");
						SmartDashboard::PutNumber("Diff", imu->GetAngleY() - initialY);
						if (timeInitial + 4 < Timer::GetFPGATimestamp())
							drive->ArcadeDrive(0.45, 0);
						else
							autoState = STOP;
					} else if (imu->GetAngleY() - initialY > 2.5 && autoState == INITFORWARD) {
						DriverStation::ReportError("Found Ramp");

						autoState = FOUNDRAMP;
						drive->ArcadeDrive(0.0, 0.0);
						PIDInit(0.2, 0, 0.1);
					} else if (autoState == FOUNDRAMP) {
						DriverStation::ReportError("Driving Back");
						int x = 8000;
						leftMotorF->Set(-x);
						rightMotorF->Set(x);
						if (abs(abs(leftMotorF->GetEncPosition()) - x) < 500 &&
								abs(abs(rightMotorF->GetEncPosition()) - x) < 500) {
							autoState = DRIVENBACK;
							DriverStation::ReportError("Finished Back");

							leftMotorF->SetControlMode(CANSpeedController::kPercentVbus);
							rightMotorF->SetControlMode(CANSpeedController::kPercentVbus);

							resetMotors();
							timeInitial = Timer::GetFPGATimestamp();
						}
					} else if (autoState == DRIVENBACK) {
						DriverStation::ReportError("Driving Forward");

						if (timeInitial + 2.0 > Timer::GetFPGATimestamp())
							drive->ArcadeDrive(0.8, 0.0);
						else
							autoState = STOP;
						//autoState = STOP;
					} else if (autoState == STOP) {
						initialY = 0;
						timeInitial = 0;
						drive->ArcadeDrive(0.0,0.0);
					}
				} else if (autoSelected == autoRamparts) {
					if (imu->GetAngleY() - initialY < 3 && autoState == INITFORWARD) {
						DriverStation::ReportError("Init Forward");
						SmartDashboard::PutNumber("Diff", imu->GetAngleY() - initialY);
						if (timeInitial + 4 < Timer::GetFPGATimestamp())
							drive->ArcadeDrive(0.45, 0);
						else
							autoState = STOP;
					} else if (imu->GetAngleY() - initialY > 2.5 && autoState == INITFORWARD) {
						DriverStation::ReportError("Found Ramp");

						autoState = FOUNDRAMP;
						drive->ArcadeDrive(0.0, 0.0);

						PIDInit(0.2, 0, 0.1);
					} else if (autoState == FOUNDRAMP) {
						DriverStation::ReportError("Driving Back");
						int x = -5000;
						leftMotorF->Set(x);
						rightMotorF->Set(-x);
						if (abs(abs(leftMotorF->GetEncPosition()) - 5000) < 500 &&
								abs(abs(rightMotorF->GetEncPosition()) - 5000) < 500) {
							autoState = DRIVENBACK;
							DriverStation::ReportError("Finished Back");

							leftMotorF->SetControlMode(CANSpeedController::kPercentVbus);
							rightMotorF->SetControlMode(CANSpeedController::kPercentVbus);

							resetMotors();
							timeInitial = Timer::GetFPGATimestamp();
						}
					} else if (autoState == DRIVENBACK) {
						DriverStation::ReportError("Driving Forward");

						if (timeInitial + 1.25 > Timer::GetFPGATimestamp())
							drive->ArcadeDrive(1.0, 0.0);
						else
							autoState = STOP;
						//autoState = STOP;
					} else if (autoState == STOP) {
						initialY = 0;
						timeInitial = 0;
						drive->ArcadeDrive(0.0,0.0);
					}
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

				if (Timer::GetFPGATimestamp() - lastResetTime > 0.1)
				{
					resetMotors();
					lastResetTime = Timer::GetFPGATimestamp();
				}
				/*
				if((leftMotorF->GetOutputCurrent() > 40 && rightMotorF->GetOutputCurrent() > 40 ) || !joystickSecond->GetRawButton(10))
				{
					if(leftMotorB->GetAnalogInVel()!=0)
					{
						leftMotorB->ConfigNeutralMode(CANTalon::kNeutralMode_Coast);
						rightMotorB->ConfigNeutralMode(CANTalon::kNeutralMode_Coast);
					}
					leftMotorB->Disable();
					rightMotorB->Disable();
				}
				*/
				SmartDashboard::PutNumber("AngleX", imu->GetAngleX());
				SmartDashboard::PutNumber("AngleY", imu->GetAngleY());
				SmartDashboard::PutNumber("AngleZ", imu->GetAngleZ());
				//SmartDashboard::PutNumber("Left P", leftMotorF->GetP());
				//SmartDashboard::PutNumber("Left I",leftMotorF->GetI());
				//SmartDashboard::PutNumber("Left D", leftMotorF->GetD());
				//SmartDashboard::PutNumber("Left Encoder", leftMotorF->GetEncPosition());
				//SmartDashboard::PutNumber("Right Encoder", rightMotorF->GetEncPosition());
				SmartDashboard::PutNumber("Left Speed", leftMotorF->GetSpeed());
				SmartDashboard::PutNumber("Joy Forward", -joystickMain->GetRawAxis(1));
				SmartDashboard::PutNumber("XRLV Max", ultra->GetVoltage() / 0.00488 / 2.54);
				//SmartDashboard::PutNumber("HRLV Max", ultra2->GetVoltage() / 0.00488 / 2.54);
				//SmartDashboard::PutNumber("HRLV Max", ultra2->GetVoltage() / 0.000977 / 10.0 / 2.54);
				//SmartDashboard::PutNumber("Left Ultra", frontL->GetRangeInches());
				//SmartDashboard::PutNumber("Right Ultra", frontR->GetRangeInches());

				// x button on joystick to reverse forward
				if (joystickMain->GetRawButton(3)) {
					reverseForward = !reverseForward;
				}

				float rawTurn = -joystickMain->GetRawAxis(4);
				// gets out of turning deadzone
				if (abs(rawTurn) > deadZone) {
					if (rawTurn > 0)
						drive->ArcadeDrive(joystickMain->GetRawAxis(1) * (reverseForward ? 1.0 : -1.0),
										   (rawTurn * turnSlope + verticalShift));
					else
						drive->ArcadeDrive(joystickMain->GetRawAxis(1) * (reverseForward ? 1.0 : -1.0),
										   (rawTurn * turnSlope - verticalShift));
				} else {
					// within turning deadzone
					drive->ArcadeDrive(-joystickMain->GetRawAxis(1), 0);
				}

				/*
				float rawForward = -joystickMain->GetRawAxis(1);
				if (rawForward > 0) {
					drive->ArcadeDrive(rawForward * 0.8 + 0.2, -joystickMain->GetRawAxis(4));
				} else {
					drive->ArcadeDrive(rawForward * 0.8 - 0.2, -joystickMain->GetRawAxis(4));
				}
				 */

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
				} else if (joystickMain->GetRawButton(6)) {
					leftShooter->Set(0.35);
					rightShooter->Set(-0.35);
				} else {
					leftShooter->Set(0);
					rightShooter->Set(0);
					indexMotor->Set(0);
				}

				if (joystickMain->GetRawButton(2)) {
					indexMotor->Set(0.45);
				}

				if (joystickMain->GetRawButton(1)) {
					//up
					shooterA->Set(DoubleSolenoid::kReverse);
					shooterB->Set(DoubleSolenoid::kReverse);
					wasUp=true;
				} else if (joystickMain->GetRawButton(4)) {
					//down
					shooterA->Set(DoubleSolenoid::kForward);
					shooterB->Set(DoubleSolenoid::kForward);
					wasDown=true;
				}

				SmartDashboard::PutNumber("X Val", joystickSecond->GetRawAxis(4));
				SmartDashboard::PutNumber("Y Val", joystickSecond->GetRawAxis(1));
				SmartDashboard::PutNumber("X Position", xPosition);
				SmartDashboard::PutNumber("Y Position", yPosition);
				if ((joystickSecond->GetRawButton(5) || joystickSecond->GetRawButton(6)) && abs(joystickSecond->GetRawAxis(4)) > 0.2)
					xPosition += joystickSecond->GetRawAxis(4) / 80.0;
				if ((joystickSecond->GetRawButton(5) || joystickSecond->GetRawButton(6)) && abs(joystickSecond->GetRawAxis(1)) > 0.2)
					yPosition += joystickSecond->GetRawAxis(1) / 80.0;

				if (joystickSecond->GetRawButton(1)) {
					// for straight forward
					xPosition = 0.5;
					yPosition = 0.6;
				} else if (joystickSecond->GetRawButton(2)) {
					// for shooting
					xPosition = 0.5;
					yPosition = 0.35;
				} else if (joystickSecond->GetRawButton(3)) {
					// for intake
					xPosition = 0.3;
					yPosition = 0.7;
				} else if (joystickSecond->GetRawButton(4)) {
					// for checking if there's a ball
					xPosition = 0;
					yPosition = 0.7;
				}

				if (xPosition > 1)
					xPosition = 1;
				if (yPosition > 1)
					yPosition = 1;
				if (xPosition < 0)
					xPosition = 0;
				if (yPosition < 0)
					yPosition = 0;
				xServo->Set(xPosition);
				yServo->Set(yPosition);

				/*
				if (sock->hasPendingPacket()) {
					DriverStation::ReportError("Got Packet");
					SmartDashboard::PutNumber("TimeSince", Timer::GetFPGATimestamp() - timeReceive);
					timeReceive = Timer::GetFPGATimestamp();
					sock->recv(buff, 641);
					SmartDashboard::PutNumber("First", buff[250]);
					SmartDashboard::PutNumber("Middle", buff[320]);
					SmartDashboard::PutNumber("Last", buff[370]);
					//DriverStation::ReportError((char*)buff);
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
