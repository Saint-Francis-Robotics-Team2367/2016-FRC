#include "WPILib.h"
#include "ADIS16448_IMU.h"
#include "PracticalSocket.h"

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
		DRIVENBACK,
		SECONDFORWARD,
		OVERDEFENSE,
		FINISHED,
	};

	//RobotDrive myRobot; // robot drive system
	Joystick stick; // only joystick

	CANTalon *leftMotor;
	CANTalon *rightMotor;
	RobotDrive *drive;

	CANTalon *leftShooter;
	CANTalon *rightShooter;
	CANTalon *indexMotor;

	Joystick *joystickMain;

	ADIS16448_IMU *imu;

	DoubleSolenoid *shooterAngler;

	Ultrasonic *frontL;
	Ultrasonic *frontR;

	bool autoInit;
	bool teleopInit;

	float initialY, initialZ, timeInitial;
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
		leftMotor = new CANTalon(7);
		rightMotor = new CANTalon(4);
		leftShooter = new CANTalon(2);
		rightShooter = new CANTalon(3);
		indexMotor = new CANTalon(1);

		joystickMain = new Joystick(0);

		leftMotor->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);
		rightMotor->ConfigNeutralMode(CANSpeedController::kNeutralMode_Brake);

		leftShooter->SetControlMode(CANSpeedController::kPercentVbus);
		rightShooter->SetControlMode(CANSpeedController::kPercentVbus);
		indexMotor->SetControlMode(CANSpeedController::kPercentVbus);

		drive = new RobotDrive(leftMotor, rightMotor);

		imu = new ADIS16448_IMU();

		shooterAngler = new DoubleSolenoid(0, 1);

		frontL = new Ultrasonic(0, 1);
		frontR = new Ultrasonic(2, 3);

		frontL->SetEnabled(true);
		frontR->SetEnabled(true);
		frontL->SetAutomaticMode(true);
		frontR->SetAutomaticMode(true);

		autoInit = false;
		teleopInit = false;

		sock = new UDPSocket(5809);
		//Note SmartDashboard is not initialized here, wait until RobotInit to make SmartDashboard calls
		//myRobot.SetExpiration(0.1);
}

	void RobotInit()
	{
		//imu->Calibrate();
		//autoSelection = LOWBAR;
		autoState = INITFORWARD;

		SmartDashboard::PutNumber("InitialY", initialY);
		SmartDashboard::PutNumber("InitialZ", initialZ);

		CameraServer::GetInstance()->SetQuality(50);
		//the camera name (ex "cam0") can be found through the roborio web interface
		CameraServer::GetInstance()->StartAutomaticCapture("cam0");

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

		leftMotor->SetEncPosition(0);
		rightMotor->SetEncPosition(0);

		leftMotor->SetPID(p, i, d);
		rightMotor->SetPID(p, i, d);

		leftMotor->SetSensorDirection(true);
		rightMotor->SetSensorDirection(true);

		//rightMotor->SetInverted(true);

		leftMotor->SetControlMode(CANSpeedController::kPosition);
		rightMotor->SetControlMode(CANSpeedController::kPosition);

		leftMotor->Set(0);
		rightMotor->Set(0);
	}

	void autonomousInit() {
		leftMotor->SetEncPosition(0);
		rightMotor->SetEncPosition(0);

		initialY = imu->GetAngleY();
		initialZ = imu->GetAngleZ() - 12;

		timesAuto++;

		DriverStation::ReportError("Resetting motor values");
	}

	void teleoperatedInit() {
		memset(buff, 0, sizeof(buff));

		rightMotor->SetInverted(false);

		leftMotor->SetControlMode(CANSpeedController::kPercentVbus);
		rightMotor->SetControlMode(CANSpeedController::kPercentVbus);
	}

	void Autonomous()
	{
		while (IsAutonomous()) {
			if (IsEnabled()) {
				resetMotors();
				SmartDashboard::PutNumber("AngleX", imu->GetAngleX());
				SmartDashboard::PutNumber("AngleY", imu->GetAngleY());
				SmartDashboard::PutNumber("AngleZ", imu->GetAngleZ());
				SmartDashboard::PutNumber("Left P", leftMotor->GetP());
				SmartDashboard::PutNumber("Left I",leftMotor->GetI());
				SmartDashboard::PutNumber("Left D", leftMotor->GetD());
				SmartDashboard::PutNumber("Left Encoder", leftMotor->GetEncPosition());
				SmartDashboard::PutNumber("Right Encoder", rightMotor->GetEncPosition());
				SmartDashboard::PutNumber("Left Speed", leftMotor->GetSpeed());
				SmartDashboard::PutNumber("Left Ultra", frontL->GetRangeInches());
				SmartDashboard::PutNumber("Right Ultra", frontR->GetRangeInches());
				SmartDashboard::PutNumber("InitialZ", initialZ);

				autoSelected = *((std::string*)chooser->GetSelected());

				if (autoInit == false) {
					autoInit = true;
					autonomousInit();
					autoState = INITFORWARD;

					// REMOVE THIS
					PIDInit(0.7, 0, 0);
					timeInitial = Timer::GetFPGATimestamp();
				}
				if (autoSelected == autoNothing) {
					autoState = FINISHED;
				} else if (autoSelected == autoLowBar) {
					// good PID's are 0.1, 0, 14.35
					if (autoState == INITFORWARD) {
						/*
						int x = 10000;
						leftMotor->Set(x);
						rightMotor->Set(-x);
						if (abs(abs(leftMotor->GetEncPosition()) - x) < 100 &&
								abs(abs(rightMotor->GetEncPosition()) - x) < 100) {
							autoState = FOUNDRAMP;
							resetMotors();
							//leftMotor->Set(leftMotor->GetEncPosition());
							//rightMotor->Set(rightMotor->GetEncPosition());
						}
						*/

						DriverStation::ReportError("Trying to turn");
						// good turning PID's are 0.7, 0, 0
						if (initialZ + 540 > imu->GetAngleZ()) {
							resetMotors();
							leftMotor->Set(leftMotor->GetSetpoint() + 35);
							rightMotor->Set(rightMotor->GetSetpoint() + 35);
						} else {
							autoState = FOUNDRAMP;
						}
					}
				} else if (autoSelected == autoMoat) {
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
						leftMotor->Set(x);
						rightMotor->Set(-x);
						if (abs(abs(leftMotor->GetEncPosition()) - 5000) < 500 &&
								abs(abs(rightMotor->GetEncPosition()) - 5000) < 500) {
							autoState = DRIVENBACK;
							DriverStation::ReportError("Finished Back");

							leftMotor->SetControlMode(CANSpeedController::kPercentVbus);
							rightMotor->SetControlMode(CANSpeedController::kPercentVbus);

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
						leftMotor->Set(x);
						rightMotor->Set(-x);
						if (abs(abs(leftMotor->GetEncPosition()) - 5000) < 500 &&
								abs(abs(rightMotor->GetEncPosition()) - 5000) < 500) {
							autoState = DRIVENBACK;
							DriverStation::ReportError("Finished Back");

							leftMotor->SetControlMode(CANSpeedController::kPercentVbus);
							rightMotor->SetControlMode(CANSpeedController::kPercentVbus);

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
						leftMotor->Set(-x);
						rightMotor->Set(x);
						if (abs(abs(leftMotor->GetEncPosition()) - x) < 500 &&
								abs(abs(rightMotor->GetEncPosition()) - x) < 500) {
							autoState = DRIVENBACK;
							DriverStation::ReportError("Finished Back");

							leftMotor->SetControlMode(CANSpeedController::kPercentVbus);
							rightMotor->SetControlMode(CANSpeedController::kPercentVbus);

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
						leftMotor->Set(x);
						rightMotor->Set(-x);
						if (abs(abs(leftMotor->GetEncPosition()) - 5000) < 500 &&
								abs(abs(rightMotor->GetEncPosition()) - 5000) < 500) {
							autoState = DRIVENBACK;
							DriverStation::ReportError("Finished Back");

							leftMotor->SetControlMode(CANSpeedController::kPercentVbus);
							rightMotor->SetControlMode(CANSpeedController::kPercentVbus);

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
				//Wait(0.005);
			} else {
				autoInit = false;
			}
		}
	}

	void OperatorControl()
	{
		resetMotors();
		//myRobot.SetSafetyEnabled(true);
		while (IsOperatorControl())
		{
			if (IsEnabled()) {
				if (teleopInit == false) {
					teleopInit = true;
					teleoperatedInit();
				}
				SmartDashboard::PutNumber("AngleX", imu->GetAngleX());
				SmartDashboard::PutNumber("AngleY", imu->GetAngleY());
				SmartDashboard::PutNumber("AngleZ", imu->GetAngleZ());
				SmartDashboard::PutNumber("Left P", leftMotor->GetP());
				SmartDashboard::PutNumber("Left I",leftMotor->GetI());
				SmartDashboard::PutNumber("Left D", leftMotor->GetD());
				SmartDashboard::PutNumber("Left Encoder", leftMotor->GetEncPosition());
				SmartDashboard::PutNumber("Right Encoder", rightMotor->GetEncPosition());
				SmartDashboard::PutNumber("Left Speed", leftMotor->GetSpeed());
				SmartDashboard::PutNumber("Joy Forward", -joystickMain->GetRawAxis(1));
				//SmartDashboard::PutNumber("Left Ultra", frontL->GetRangeInches());
				//SmartDashboard::PutNumber("Right Ultra", frontR->GetRangeInches());

				float rawForward = -joystickMain->GetRawAxis(1);

				if (rawForward > 0) {
					drive->ArcadeDrive(rawForward * 0.8 + 0.2, -joystickMain->GetRawAxis(4));
				} else {
					drive->ArcadeDrive(rawForward * 0.8 - 0.2, -joystickMain->GetRawAxis(4));
				}

				// 2 for left trigger, 3 for right trigger
				if (joystickMain->GetRawAxis(2) >= 0.1) {
					// intake
					leftShooter->Set(0.45);
					rightShooter->Set(-0.45);
					indexMotor->Set(0.45);
				} else if (joystickMain->GetRawAxis(3) >= 0.1) {
					// shoot
					leftShooter->Set(-1.0);
					rightShooter->Set(1.0);
					indexMotor->Set(0);
				} else {
					leftShooter->Set(0);
					rightShooter->Set(0);
					indexMotor->Set(0);
				}

				if (joystickMain->GetRawButton(4)) {
					shooterAngler->Set(DoubleSolenoid::kForward);
				} else if (joystickMain->GetRawButton(1)) {
					shooterAngler->Set(DoubleSolenoid::kReverse);
				}

				// for shooting: wait until shooter motors are at full power, then push ball
				if (joystickMain->GetRawButton(2)) {
					indexMotor->Set(-0.5);
				}

				if (sock->hasPendingPacket())
				{
					DriverStation::ReportError("Got Packet");
					SmartDashboard::PutNumber("TimeSince", Timer::GetFPGATimestamp() - timeReceive);
					timeReceive = Timer::GetFPGATimestamp();
					sock->recv(buff, 641);
					SmartDashboard::PutNumber("First", buff[250]);
					SmartDashboard::PutNumber("Middle", buff[320]);
					SmartDashboard::PutNumber("Last", buff[370]);
					//DriverStation::ReportError((char*)buff);
				}
			} else {
				teleopInit = false;
			}
		}
	}

	void DisabledPeriodic() {
		resetMotors();
		drive->SetLeftRightMotorOutputs(0, 0);
		leftShooter->Set(0);
		rightShooter->Set(0);
		indexMotor->Set(0);
		robotState = RobotState::DISABLED;

		SmartDashboard::PutBoolean("Driving forward", false);
	}

	void resetMotors() {
		if (drive->GetGlobalError().GetCode() != 0) {
			drive->GetGlobalError().GetOriginatingObject()->ClearError();

			leftMotor->ClearError();
			rightMotor->ClearError();
			leftShooter->ClearError();
			rightShooter->ClearError();
			indexMotor->ClearError();

			drive->GetGlobalError().Clear();

			leftMotor->EnableControl();
			rightMotor->EnableControl();
			leftShooter->EnableControl();
			rightShooter->EnableControl();
			indexMotor->EnableControl();
		}
		drive->GetGlobalError().Clear();
		leftMotor->EnableControl();
		rightMotor->EnableControl();
		leftShooter->EnableControl();
		rightShooter->EnableControl();
		indexMotor->EnableControl();
	}

	void Test()
	{
	}
};

START_ROBOT_CLASS(Robot)
