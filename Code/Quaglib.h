#include <stdio.h>
#include <phidget21.h>

//---------------------------------------------------------General------------------------------------------------------------------------------------//

int AttachHandler(CPhidgetHandle board, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (board, &name);
	CPhidget_getSerialNumber(board, &serialNo);
	printf("%s %10d attached!\n", name, serialNo);

	return 0;
}

int DetachHandler(CPhidgetHandle board, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (board, &name);
	CPhidget_getSerialNumber(board, &serialNo);
	printf("%s %10d detached!\n", name, serialNo);

	return 0;
}

int ErrorHandler(CPhidgetHandle board, void *userptr, int ErrorCode, const char *Description)
{
	printf("Error handled. %d - %s\n", ErrorCode, Description);
	return 0;
}

//---------------------------------------------------------Motor control board------------------------------------------------------------------------//

int MotorInputChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, int State)
{
	//printf("Motor %d now in state: %d\n", Index, State);
	return 0;
}

int VelocityChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value)
{
	//printf("Motor %d  at %f speed\n", Index, Value);
	return 0;
}

int CurrentChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value)
{
	//printf("Motor %d at %f current draw\n", Index, Value);
	return 0;
}

int display_motor_properties(CPhidgetMotorControlHandle phid)
{
	int serialNo, version, numInputs, numMotors;
	const char* ptr;

	CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
	CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
	CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);
	
	CPhidgetMotorControl_getInputCount(phid, &numInputs);
	CPhidgetMotorControl_getMotorCount(phid, &numMotors);

	printf("%s\n", ptr);
	printf("Serial Number: %10d\nVersion: %8d\n", serialNo, version);
	printf("# Inputs: %d\n# Motors: %d\n", numInputs, numMotors);

	return 0;
}

//---------------------------------------------------------Interface kit board------------------------------------------------------------------------//

int dInputIndex;
int dInputState;

int sensorArray[8];

//callback that will run if an input changes.
//Index - Index of the input that generated the event, State - boolean (0 or 1) representing the input state (on or off)
int InterfaceInputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State)
{
	dInputIndex = Index;
	dInputState = State;
	if (Index == 5)
	{
		printf("Left bumper now in state: %d\n", State);
	}
	if (Index == 6)
	{
		printf("Right bumper now in state: %d\n", State);
	}
	return 0;
}

//callback that will run if an output changes.
//Index - Index of the output that generated the event, State - boolean (0 or 1) representing the output state (on or off)
int OutputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State)
{
	return 0;
}

//callback that will run if the sensor value changes by more than the OnSensorChange trigger.
//Index - Index of the sensor that generated the event, Value - the sensor read value
int SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int Value)
{
	sensorArray[Index] = Value;
	//printf("----------------------------------------\n");
	//printf("Sonar : %d\n", sensorArray[5]);
	//printf("Front IR : %d\n", sensorArray[7]);
	//printf("Servo IR : %d\n", sensorArray[6]);
	//printf("Left Under Light : %d\n", sensorArray[0]);
	//printf("Right Under Light : %d\n", sensorArray[1]);
	//printf("Left Front Light : %d\n", sensorArray[2]);
	//printf("Right Front Light : %d\n", sensorArray[3]);
	//printf("Sensor: %d > Value: %d\n", Index, Value);
	return 0;
}

//Display the properties of the attached phidget to the screen.  We will be displaying the name, serial number and version of the attached device.
//Will also display the number of inputs, outputs, and analog inputs on the interface kit as well as the state of the ratiometric flag
//and the current analog sensor sensitivity.
int display_interface_properties(CPhidgetInterfaceKitHandle phid)
{
	int serialNo, version, numInputs, numOutputs, numSensors, triggerVal, ratiometric, i;
	const char* ptr;

	CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
	CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
	CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

	CPhidgetInterfaceKit_getInputCount(phid, &numInputs);
	CPhidgetInterfaceKit_getOutputCount(phid, &numOutputs);
	CPhidgetInterfaceKit_getSensorCount(phid, &numSensors);
	CPhidgetInterfaceKit_getRatiometric(phid, &ratiometric);

	printf("%s\n", ptr);
	printf("Serial Number: %10d\nVersion: %8d\n", serialNo, version);
	printf("# Digital Inputs: %d\n# Digital Outputs: %d\n", numInputs, numOutputs);
	printf("# Sensors: %d\n", numSensors);
	printf("Ratiometric: %d\n", ratiometric);

	for(i = 0; i < numSensors; i++)
	{
		CPhidgetInterfaceKit_getSensorChangeTrigger (phid, i, &triggerVal);

		printf("Sensor#: %d > Sensitivity Trigger: %d\n", i, triggerVal);
	}

	return 0;
}

//---------------------------------------------------------Servo control board------------------------------------------------------------------------//

int PositionChangeHandler(CPhidgetServoHandle SERV, void *usrptr, int Index, double Value)
{
	//printf("Servo Motor %d 's current position: %f\n", Index, Value);
	return 0;
}

//Display the properties of the attached phidget to the screen.  We will be displaying the name, serial number and version of the attached device.
int display_servo_properties(CPhidgetServoHandle phid)
{
	int serialNo, version, numMotors;
	const char* ptr;

	CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
	CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
	CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

	CPhidgetServo_getMotorCount (phid, &numMotors);

	printf("%s\n", ptr);
	printf("Serial Number: %10d\nVersion: %8d\n# Motors: %d\n", serialNo, version, numMotors);

	return 0;
}

//---------------------------------------------------------Setup--------------------------------------------------------------------------------------//

//---------------------------------------------------------Motor directions--------------------------------------------------------------------------//

int GoForward(CPhidgetMotorControlHandle motoControl)
{
	CPhidgetMotorControl_setAcceleration(motoControl, 0, 100.00);
	CPhidgetMotorControl_setVelocity(motoControl, 0, 100.00);
	CPhidgetMotorControl_setAcceleration(motoControl, 1, 100.00);
	CPhidgetMotorControl_setVelocity(motoControl, 1, 100.00);
	return 0;
}

int GoBackward(CPhidgetMotorControlHandle motoControl)
{
	CPhidgetMotorControl_setAcceleration(motoControl, 0, -100.00);
	CPhidgetMotorControl_setVelocity(motoControl, 0, -100.00);
	CPhidgetMotorControl_setAcceleration(motoControl, 1, -100.00);
	CPhidgetMotorControl_setVelocity(motoControl, 1, -100.00);
	return 0;
}

int RotateLeft(CPhidgetMotorControlHandle motoControl)
{
	CPhidgetMotorControl_setAcceleration(motoControl, 0, -100.00);
	CPhidgetMotorControl_setVelocity(motoControl, 0, -100.00);
	CPhidgetMotorControl_setAcceleration(motoControl, 1, 100.00);
	CPhidgetMotorControl_setVelocity(motoControl, 1, 100.00);
}

int RotateRight(CPhidgetMotorControlHandle motoControl)
{
	CPhidgetMotorControl_setAcceleration(motoControl, 0, 100.00);
	CPhidgetMotorControl_setVelocity(motoControl, 0, 100.00);
	CPhidgetMotorControl_setAcceleration(motoControl, 1, -100.00);
	CPhidgetMotorControl_setVelocity(motoControl, 1, -100.00);
	return 0;
}

int Stop(CPhidgetMotorControlHandle motoControl)
{
	CPhidgetMotorControl_setVelocity(motoControl, 0, 0.00);
	CPhidgetMotorControl_setAcceleration(motoControl, 0, 0.00);
	CPhidgetMotorControl_setVelocity(motoControl, 1, 0.00);
	CPhidgetMotorControl_setAcceleration(motoControl, 1, 0.00);
	return 0;
}

//---------------------------------------------------------END----------------------------------------------------------------------------------------//
