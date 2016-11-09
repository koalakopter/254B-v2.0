#pragma config(Sensor, in1,    armShaft,       sensorPotentiometer)
#pragma config(Sensor, dgtl1,  rightshaft,     sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  leftshaft,      sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  MemeLight,      sensorLEDtoVCC)
#pragma config(Motor,  port1,           RearRight,     tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           FrontRight,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           FrontLeft,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           LMidArm,       tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           LTopArm,       tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           RTopArm,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           RMidArm,       tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           BotArms,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           Winch,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          RearLeft,      tmotorVex393_HBridge, openLoop, reversed)
#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"

float drTarg = 0.0;
float dlTarg = 0.0;
float aTarg = 0;
float getArmVal;
float armOffset = 123456;
float aErr;
int armPos;
float kpCalc;
float kiCalc;
float kdCalc;
float realArm;
bool manual;
bool julian = false;

void drive(int xaxis, int yaxis, int turning)
{
	int forwardDeadzone = 15;

	if (yaxis > 127 - forwardDeadzone && xaxis < forwardDeadzone) xaxis = 0;

	motor[FrontRight]  = -xaxis + yaxis + turning;
	motor[FrontLeft] = xaxis + yaxis + turning;
	motor[RearLeft] = -xaxis - yaxis + turning;
	motor[RearRight] = xaxis - yaxis + turning;
}

void wincherino(int up, int down)
{
	int speed = 0;

	if (up && !down)
		speed = 127;

	else if (!up && down)
		speed = -127;

	motor[Winch] = speed;
}

float getArm() {
	return SensorValue[armShaft] - armOffset;
}

task arm()
{
	while (true)
	{
		bool up = vexRT[Btn5U];
		bool down = vexRT[Btn5D];

		//toggles if arm can go past 90 degrees
		//should be renamed but whatever
		//if true, go past 90
		if(vexRT[Btn7U])
		{
			julian = !julian;
			SensorValue[dgtl5] = julian;
		}

		//if you're pressing up button
		if (up && !down)
		{
			manual = true;
			//while you hold up button, the target is set to the max value (depending if it can go past 90 deg)
			while(vexRT[Btn5U] && manual)
			{
				if(julian)
					aTarg = 2900;
				else
					aTarg = 2200;
			}
			//sets the target to the current position
			//(-200 to account for the speed its already going [not necessary, maybe change value])
			aTarg = getArm() + 200;
			manual = false;
		}

		//works the same as the previous block of code but for going down, and sets value to 0 or bottom
		if (!up && down)
		{
			aTarg = 0;
			manual = true;
			while(vexRT[Btn5D] && manual)
			{
			}
			manual = false;
			aTarg = getArm() - 100;
		}

		//probably don't need these, but just to make sure target doesn't go out of bounds, as that would seriously fuck things up
		if(aTarg < 0)
			aTarg = 0;

		if(aTarg > 1900 && !julian)
			aTarg = 1900;
	}
}

//sets the motors
void armPo(int po) {
	motor[LTopArm] = motor[LMidArm] = motor[RTopArm] = motor[RMidArm] = motor[BotArms] = po;
}

//presets for the arm. just sets the target. not much to it.
task armPresets()
{
	while(true)
	{
		bool top = vexRT[Btn8R];
		bool mid = vexRT[Btn8L];

		if(top)
		{
			manual = false;
			aTarg = 2400;
		}

		if(mid)
		{
			manual = false;
			aTarg = 250;
		}
	}
}

task armPID() {
	//some pid code that i just copied
	//change values below if it's not working properly

	//change to edit how fast it will move towards the target
	float aKp = .07; //.07
	//change to i dont know (i think to predict when it's getting close and slow down)
	float aKi = .000025; //.000025
	//change to i dont know (related to kp)
	float aKd = .0003; //.0003
	float aInt = 0;
	float aDer = 0;
	float aLErr = 0;
	while(true) {
		aErr = aTarg - getArm();
		realArm = getArm();
		aInt += aErr / 10;
		if (abs(aErr) < 5) {
			aInt = 0;
		}
		aDer = aErr - aLErr;
		aLErr = aErr;
		kpCalc = aKp * aErr;
		kiCalc = aKi * aInt;
		kdCalc = aKd * aDer;
		armPos = (aKp*aErr)+(aKi*aInt)+(aKd*aDer);

		//limits the armpos to 127 and -127 for future calculations
		if(armPos > 127) armPos = 127;
		if(armPos < -127) armPos = -127;

		//based on if you're manually raising/lowering the arm or if it's a preset, it will make it go slower
		if(manual && armPos < 0)
			armPos = (int)(armPos * 0.8);
		else if(manual && armPos > 0)
			armPos = (int)(armPos * 0.6);
		else if(armPos > 0)
			armPos = (int)(armPos * 0.5);

		armPo(armPos);
	}
}

task usercontrol()
{
	//starts all the tasks
	if (armOffset == 123456) armOffset = SensorValue[armShaft];
	startTask(armPID);
	startTask(arm);
	startTask(armPresets);
	while (true)
	{
		getArmVal = getArm();
		drive(vexRT[Ch3], vexRT[Ch1], vexRT[Ch4]);
		wincherino(vexRT[Btn8U], vexRT[Btn8D]);
	}
}

void driveForwardAuton()
{
	drive(127, 0, 0);
	motor[port4] = motor[port5] = motor[port6] = motor[port7] = 127;
	delay(1430);
	drive(0, 0, 0);
	motor[port4] = motor[port5] = motor[port6] = motor[port7] = 10;
	//drive forward and knock the stars down
	motor[port2] = motor[port10] = -127;
	delay(3000);
	drive(0, 0, -127);
	delay(400);
	drive(-127, 0, 0);
	motor[port4] = motor[port5] = motor[port6] = motor[port7] = -10;
	delay(3000);
	drive(0, 0, 0);
}

void pre_auton()
{
	bStopTasksBetweenModes = true;
}

task autonomous()
{
	armOffset = SensorValue[armShaft];
	driveForwardAuton();
}
