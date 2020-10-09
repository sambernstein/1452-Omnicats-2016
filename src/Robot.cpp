#include "WPILib.h"
#include <iostream>

#define Revolution 498

class Robot: public IterativeRobot {

private:
	//Camera
	IMAQdxSession session;
	Image *frame;
	IMAQdxError imaqError;

	//Motors
	CANTalon *frontRight = new CANTalon(2);
	CANTalon *frontLeft = new CANTalon(1);
	CANTalon *backRight = new CANTalon(6);
	CANTalon *backLeft = new CANTalon(5);
	CANTalon *intake = new CANTalon(3);
	CANTalon *sprocket = new CANTalon(4);

	//Mirror Stuff
	Servo *mirror = new Servo(0);
	bool mirrorFlat = false;

	//Joysticks
	Joystick *stick = new Joystick(0);
	Joystick *stick2 = new Joystick(1);

	//RobotDrive stuff
	RobotDrive *drive = new RobotDrive(frontLeft, backLeft, frontRight,backRight);
	float modifiedLeft = 0;
	float modifiedRight = 0;
	bool reversed = true;
	bool lastPressedReverse = false;
	/*AnalogGyro *gyro = new AnalogGyro(0);
	float gyroOffset = 0;
	float gyroDegs = 0;*/

	//Accelerometer stuff
	Accelerometer *accel = new BuiltInAccelerometer();
	float tempAccel[3] = { 0, 0, 0 };
	float averageAccel[3] = { 0, 0, 0 };
	int accelStep = 0;

	//Encoder stuff
	Encoder *sprocketEncoder = new Encoder(0, 1);
	int SprocketDegs = 0;
	int SprocketOffset = 0;
	bool sprocketUsed = false;

	//Autonomous
	int autoSelected;
	float autoDriveTime;
	float autoPower = 1.0;
	float autoTime = 0;
	float routineTime = 0;
	Timer *autoTimer = new Timer();
	Timer *routineTimer = new Timer();

	void Accel(){
		tempAccel[0] += accel->GetX();
		tempAccel[1] += accel->GetY();
		tempAccel[2] += accel->GetZ();
		if (accelStep == 9) {
			averageAccel[0] = tempAccel[0] / 10;
			averageAccel[1] = tempAccel[1] / 10;
			averageAccel[2] = tempAccel[2] / 10;
			SmartDashboard::PutNumber("Accelerometer X", averageAccel[0]);
			SmartDashboard::PutNumber("Accelerometer Y", averageAccel[1]);
			SmartDashboard::PutNumber("Accelerometer Z", averageAccel[2]);
			tempAccel[0] = 0;
			tempAccel[1] = 0;
			tempAccel[2] = 0;
		}
		accelStep = (accelStep + 1) % 10;
	}

	void Drive(){
		if (stick->GetRawButton(11) && stick->GetRawButton(12) && !lastPressedReverse) {
			reversed = !reversed;
			mirrorFlat = false;
		}
		if (reversed) {
			drive->TankDrive(modifiedRight, modifiedLeft);
		} else {
				drive->TankDrive(-1 * modifiedLeft, -1 * modifiedRight);
		}
	}

	//Sets sprocket to a given position
	void SetSprocket(int pos, int override = 0) {
		if (SprocketDegs > 320 && override == 0) {
			sprocket->Set(-1);
		} else {
			sprocket->Set(std::min(std::max(((float) (SprocketDegs - pos)) / 40.0,-1.0), 1.0));
		}
	}

	int portcullisStep = 0;
	/*
	void Portcullis() {
		sprocketUsed = true;
		reversed = false;
		switch (portcullisStep) {
		case 0:
			modifiedLeft -= 0.5;
			modifiedRight -= 0.5;
			if (averageAccel[0] < -0.18) {
				portcullisStep = 1;
			}
			SetSprocket(295);
			break;
		case 1:
			modifiedLeft -= 0.5;
			modifiedRight -= 0.5;
			SetSprocket(88);
			mirrorFlat = true;
			break;
		}
	}
*/
	void Portcullis2() {
		if (portcullisStep == 0){
			routineTimer->Reset();
			routineTimer->Start();
		}
		portcullisStep = 1;
		routineTime = routineTimer->Get();
		sprocketUsed = true;
		reversed = false;
		if (routineTime < 2){
			modifiedLeft -= 0.5;
			modifiedRight -= 0.5;
			SetSprocket(295,1);
		} else if (routineTime < 2.5){
			modifiedLeft -= 0.5;
			modifiedRight -= 0.5;
			SetSprocket(88);
			mirrorFlat = true;
		} else {
			modifiedLeft -= 0.7;
			modifiedRight -= 0.7;
			SetSprocket(88);
			mirrorFlat = true;
		}
	}

	int chevalStep = 0;
	/*
	void Cheval() {
		sprocketUsed = true;
		reversed = false;
		switch (chevalStep) {
		case 0:
			modifiedLeft -= 0.5;
			modifiedRight -= 0.5;
			SetSprocket(138);
			if (averageAccel[0] < -0.05) {
				chevalStep = 1;
			}
			break;
		case 1:
			modifiedLeft -= 0.5;
			modifiedRight -= 0.5;
			SetSprocket(250);
			if (averageAccel[0] < -0.12) {
				chevalStep = 2;
			}
			break;
		case 2:
			modifiedLeft -= 0.5;
			modifiedRight -= 0.5;
			SetSprocket(350,1);
			if (averageAccel[0] < -0.17) {
				chevalStep = 3;
			}
			break;
		case 3:
			modifiedLeft -= 0.75;
			modifiedRight -= 0.75;
			sprocket->Set(0);
			//if (averageAccel[0] < -0.15) {
			//	chevalStep = 2;
			//}
			break;
		}
	}
	*/
	void Cheval2() {
		if (chevalStep == 0){
			routineTimer->Reset();
			routineTimer->Start();
		}
		chevalStep = 1;
		routineTime = routineTimer->Get();
		sprocketUsed = true;
		reversed = false;
		if (routineTime < 1){
			modifiedLeft -= 0.5;
			modifiedRight -= 0.5;
			SetSprocket(138);
		} else if (routineTime < 1.5){
			modifiedLeft -= 0.3;
			modifiedRight -= 0.3;
			SetSprocket(420,1);
		} else if (routineTime < 3.5){
			modifiedLeft -= 0.75;
			modifiedRight -= 0.75;
			sprocket->Set(0);
			//if (averageAccel[0] < -0.15) {
			//	chevalStep = 2;
			//}
		}
	}

	void RobotInit() {
		SmartDashboard::PutNumber("Autonomous Selected:",autoSelected);
		SmartDashboard::PutNumber("Autonomous Drive Time:",autoDriveTime);
		SmartDashboard::PutNumber("Autonomous Power:",autoPower);
		frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
		imaqError = IMAQdxOpenCamera("cam1", IMAQdxCameraControlModeController, &session);
		if(imaqError != IMAQdxErrorSuccess) {
			DriverStation::ReportError("IMAQdxOpenCamera error: " + std::to_string((long)imaqError) + "\n");
		}
		imaqError = IMAQdxConfigureGrab(session);
		if(imaqError != IMAQdxErrorSuccess) {
			DriverStation::ReportError("IMAQdxConfigureGrab error: " + std::to_string((long)imaqError) + "\n");
		}
		drive->SetSafetyEnabled(false);
		sprocketEncoder->Reset();
		//gyro->InitGyro();
		//gyro->Reset();
	}

	void AutonomousInit() {
		autoSelected = SmartDashboard::GetNumber("Autonomous Selected:",0);
		autoDriveTime = SmartDashboard::GetNumber("Autonomous Drive Time:",2);
		autoPower = SmartDashboard::GetNumber("Autonomous Power:",1.0);
		//std::cout << "Auto selected: "<<autoSelected<<std::endl;
		SprocketOffset = sprocketEncoder->Get();
		autoTimer->Reset();
		autoTimer->Start();
		portcullisStep = 0;
		chevalStep = 0;
	}

	void AutonomousPeriodic() {
		autoTime = autoTimer->Get();
		if (autoTime<12.0) {
			drive->TankDrive(0.0,0.0);
		} else if (autoTime<14.0){
			drive->TankDrive(-1.0,-1.0);
		} else {
			drive->TankDrive(0.0,0.0);
		}
		return;
/*
		routineTime = routineTimer->Get();
		SprocketDegs = (sprocketEncoder->Get() - SprocketOffset) * -1;
		while (SprocketDegs < 0)
			SprocketDegs += Revolution;
		SprocketDegs = SprocketDegs % Revolution;

		switch (autoSelected){
		case 0: //Drive Forwards
			if (autoTime<autoDriveTime){
				drive->TankDrive(-autoPower,-autoPower);
			} else {
				drive->TankDrive(0.0,0.0);
			}
			break;
		case 1: //Do Nothing
			SetSprocket(450,1);
			break;
		case 2: //Portcullis
			if (autoTime<0.25){
				sprocket->Set(-1);
				drive->TankDrive(0.0,0.0);
			} else if (autoTime<2){
				SetSprocket(300,1);
				drive->TankDrive(0.0,0.0);
			} else if (autoTime<4){
				SetSprocket(270);
				drive->TankDrive(0.5,0.5);
			} else if (autoTime<8){
				modifiedLeft = 0.0;
				modifiedRight = 0.0;
				Portcullis2();
				Drive();
			} else {
				sprocket->Set(0.0);
				drive->TankDrive(0.0,0.0);
			}
			break;
		case 3: //Cheval de Frise
			if (autoTime<0.25){
				sprocket->Set(-1);
				drive->TankDrive(0.0,0.0);
			} else if (autoTime<2){
				SetSprocket(300,1);
				drive->TankDrive(0.0,0.0);
			} else if (autoTime<4){
				SetSprocket(270);
				drive->TankDrive(0.5,0.5);
			} else if (autoTime<6){
				modifiedLeft = 0.0;
				modifiedRight = 0.0;
				//Accel();
				Cheval2();
				Drive();
			} else {
				sprocket->Set(0.0);
				drive->TankDrive(0.0,0.0);
			}
			break;
		}
		SmartDashboard::PutNumber("Sprocket Encoder", SprocketDegs);
		*/
	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {
		/*if (stick->GetRawButton(10)){
			gyroOffset = gyro->GetAngle();
		}
		gyroDegs = gyro->GetAngle()-gyroOffset;
		while (gyroDegs<0){
			gyroDegs+=360;
		}
		while (gyroDegs>360){
			gyroDegs-=360;
		}*/
		IMAQdxGrab(session, frame, true, NULL);
		if(imaqError != IMAQdxErrorSuccess) {
			DriverStation::ReportError("IMAQdxGrab error: " + std::to_string((long)imaqError) + "\n");
		} else {
			//imaqDrawShapeOnImage(frame, frame, { 10, 10, 100, 100 }, DrawMode::IMAQ_DRAW_VALUE, ShapeMode::IMAQ_SHAPE_OVAL, 0.0f);
			if (reversed){
				imaqFlip(frame,frame,FlipAxis_enum::IMAQ_VERTICAL_AXIS);
			} else {
				imaqFlip(frame,frame,FlipAxis_enum::IMAQ_HORIZONTAL_AXIS);
			}
			CameraServer::GetInstance()->SetImage(frame);
		}
		//SmartDashboard::PutNumber("Gyro",gyroDegs);
		/*
		switch (stick->GetPOV()){
		case -1:
			if (mirrorFlat) {
				mirror->SetAngle(90);
			} else if (reversed) {
				mirror->SetAngle(45);
			} else {
				mirror->SetAngle(135);
			}
			break;
		case 90:
			mirror->SetAngle(135);
			break;
		case 180:
			mirror->SetAngle(90);
			break;
		case 270:
			mirror->SetAngle(45);
			break;
		}*/
		if (mirrorFlat || stick2->GetRawButton(10)) {
			mirror->SetAngle(90);
		} else if (reversed) {
			mirror->SetAngle(45);
		} else {
			mirror->SetAngle(135);
		}
		SprocketDegs = (sprocketEncoder->Get() - SprocketOffset) * -1;
		while (SprocketDegs < 0)
			SprocketDegs += Revolution;
		SprocketDegs = SprocketDegs % Revolution;
		SmartDashboard::PutNumber("Sprocket Encoder", SprocketDegs);
		modifiedLeft = stick->GetY() + 0.75 * (stick2->GetThrottle() - stick2->GetX());
		modifiedRight = stick->GetThrottle() + 0.75 * (stick2->GetThrottle() + stick2->GetX());
		//modifiedLeft = stick->GetY();
		//modifiedRight = stick->GetThrottle();
		//Get average acceleration over 10 readings
		//Accel();

		//Automated Routines
		if (stick2->GetRawButton(2)) {
			Cheval2();
		} else if (stick2->GetRawButton(1)) {
			Portcullis2();
		} else {
			switch (stick->GetPOV()){
			case -1:
				sprocketUsed = false;
				break;
			case 0:
				sprocketUsed = true;
				SetSprocket(125);
				break;
			case 90:
				sprocketUsed = true;
				SetSprocket(250);
				break;
			case 180:
				sprocketUsed = true;
				SetSprocket(375,1);
				break;
			case 270:
				sprocketUsed = true;
				SetSprocket(0);
				break;
			}
			chevalStep = 0;
			portcullisStep = 0;
		}

		//Intake
		if (stick->GetRawButton(6) || stick2->GetRawButton(6)) {
			intake->Set(1);
		} else if (stick->GetRawButton(8) || stick2->GetRawButton(8)) {
			intake->Set(-1);
		} else {
			intake->Set(0);
		}

		//Sprocket
		if (stick->GetRawButton(10)) {
			SprocketOffset = sprocketEncoder->Get();
		}
		if (stick->GetRawButton(5) || stick2->GetRawButton(5)) {
			sprocket->Set(1);
		} else if (stick->GetRawButton(7) || stick2->GetRawButton(7)) {
			sprocket->Set(-1);
		} else if (!sprocketUsed) {
			mirrorFlat = false;
			sprocket->Set(0);
		}

		//Drive Code
		Drive();
		lastPressedReverse = stick->GetRawButton(11) && stick->GetRawButton(12);
		SmartDashboard::PutBoolean("Mirror flat?",mirrorFlat);
		SmartDashboard::PutBoolean("Reversed?",reversed);
		SmartDashboard::PutNumber("Auto Time",autoTimer->Get());
		SmartDashboard::PutNumber("Modified Left", modifiedLeft);
		SmartDashboard::PutNumber("Modified Right", modifiedRight);
	}

	void TestPeriodic() {
		//lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
