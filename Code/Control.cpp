#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <phidget21.h>
#include <libpowerbutton.h>
#include <time.h>
#include "quaglib.h"

//Declare a motor control, interface kit and servo handle
CPhidgetMotorControlHandle motoControl = 0;
CPhidgetInterfaceKitHandle ifKit = 0;
CPhidgetServoHandle servo = 0;

int rightThreshold;				//Thresholds for under lights
int leftThreshold;

int leftLightThresh;			//Threshold for front lights
int rightLightThresh;
const int gapThresh = 150;		//The threshold between wall and gap
const int underGapThresh = 100;	//Distance to wall under a gap
const float tpcm = 0.049375;
const float dps = 20.25;

const int siteRadius = 50;		//The distance of influence of a site

int forwardIrSweepReadings[20];
int angledIrSweepReadings[20]; 	//index 0 = 60deg, index 19 = 160deg
int sonarSweepReadings[20]; 	//Same as above
int readingsIr360[72];
int readingsSonar360[72];
int readingsIr270[54];
int readingsSonar270[54];

int firstRun = 1;
int roboBearing;
int currentGoal = 0;				//Holds the index to the current site
int numSitesCompleted = 0;
int numPossCompleted = 0;

const int numPosSites = 20;		//Number of possibilites to save
int numFoundSites = 0;
int siteInfo[numPosSites][3];
int siteCoord[numPosSites][2];	//Hold coords of each possible site and state
								//0 = Not visited, 1 = visited, 2 = Not site
int maneuverCount = 0;

/*	ANALOGUE				DIGITAL
0 = under left light		--
1 = under right light		--
2 = front left light		--
3 = front right light		rear left touch
4 = --						rear right touch
5 = sonar					front left touch
6 = angled IR				front right touch
7 = forward IR				--
*/

//-------------------------------------------------------------------------------------------------
//----------------------------------------SET UP CONNECTIONS---------------------------------------

int setup()
{
	int result;
	const char *err;

	//create the motor control, interface kit and servo object
	CPhidgetMotorControl_create(&motoControl);
	CPhidgetInterfaceKit_create(&ifKit);
	CPhidgetServo_create(&servo);

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)motoControl, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)motoControl, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)motoControl, ErrorHandler, NULL);
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)ifKit, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)ifKit, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)ifKit, ErrorHandler, NULL);
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)servo, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)servo, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)servo, ErrorHandler, NULL);


	//Registers a callback that will run if an input changes.
	//Requires the handle for the Phidget, the function that will be called, and a arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetMotorControl_set_OnInputChange_Handler (motoControl, MotorInputChangeHandler, NULL);
	CPhidgetInterfaceKit_set_OnInputChange_Handler (ifKit, InterfaceInputChangeHandler, NULL);
	CPhidgetServo_set_OnPositionChange_Handler(servo, PositionChangeHandler, NULL);

	//Registers a callback that will run if a motor changes.
	//Requires the handle for the Phidget, the function that will be called, and a arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetMotorControl_set_OnVelocityChange_Handler (motoControl, VelocityChangeHandler, NULL);

	//Registers a callback that will run if the sensor value changes by more than the OnSensorChange trig-ger.
	//Requires the handle for the IntefaceKit, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetInterfaceKit_set_OnSensorChange_Handler (ifKit, SensorChangeHandler, NULL);

	//Registers a callback that will run if the current draw changes.
	//Requires the handle for the Phidget, the function that will be called, and a arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetMotorControl_set_OnCurrentChange_Handler (motoControl, CurrentChangeHandler, NULL);

	//Registers a callback that will run if an output changes.
	//Requires the handle for the Phidget, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetInterfaceKit_set_OnOutputChange_Handler (ifKit, OutputChangeHandler, NULL);

	//Registers a callback that will run when the motor position is changed.
	//Requires the handle for the Phidget, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetServo_set_OnPositionChange_Handler(servo, PositionChangeHandler, NULL);

	//open the motor control, interface kit and servo for device connections
	CPhidget_open((CPhidgetHandle)motoControl, -1);
	CPhidget_open((CPhidgetHandle)ifKit, -1);
	CPhidget_open((CPhidgetHandle)servo, -1);

	//------------ATTACH MOTOR CONTROL-------------
	//get the program to wait for a motor control device to be attached
	printf("Waiting for MotorControl to be attached....\n");
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)motoControl, 10000)))
	{
		CPhidget_getErrorDescription(result, &err);
		printf("Problem waiting for attachment: %s\n", err);
		return 0;
	}

	//Display the properties of the attached motor control device
	display_motor_properties(motoControl);

	//-------------ATTACH INTERFACE KIT------------
	//get the program to wait for an interface kit device to be attached
	printf("Waiting for interface kit to be attached....\n");
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)ifKit, 10000)))
	{
		CPhidget_getErrorDescription(result, &err);
		printf("Problem waiting for attachment: %s\n", err);
		return 0;
	}

	//Display the properties of the attached interface kit device
	display_interface_properties(ifKit);

	//-----------------ATTACH SERVO----------------
	//get the program to wait for an servo device to be attached
	printf("Waiting for Servo controller to be attached....\n");
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)servo, 10000)))
	{
		CPhidget_getErrorDescription(result, &err);
		printf("Problem waiting for attachment: %s\n", err);
		return 0;
	}

	//Display the properties of the attached servo device
	display_servo_properties(servo);

	return 0;
}

//-------------------------------------------------------------------------------------------------
//----------------------------------------SET THRESHOLDS-------------------------------------------

int setThresholds()
{
	//Under lights
	leftThreshold = sensorArray[0] - 5;
	rightThreshold = sensorArray[1] - 5;
	printf("Under light readings: left %d, right %d\n", leftThreshold, rightThreshold);

	//Front lights
	leftLightThresh = sensorArray[2] + 25;
	rightLightThresh = sensorArray[3] + 25;
	printf("Front light readings: left %d, right %d\n", sensorArray[2], sensorArray[3]);
	return 0;
}


//-------------------------------------------------------------------------------------------------
//----------------------------------------DISTANCE TO IR-------------------------------------------

int distanceToIr(double distance)
{
	int distanceIr = 512.08*(pow (((distance-5)/5),-0.61));
	return distanceIr;
}

//-------------------------------------------------------------------------------------------------
//----------------------------------------IR TO DISTANCE-------------------------------------------

int irToDistance(int ir)
{
	float irDistance = (0.0001*(50000.0*(pow ((float) ir, (100.0/61.0)))+(1.38199*pow (10.0, 9.0))))/(pow ((float) ir, (100.0/61.0)));
	return (int) irDistance;
}

//-------------------------------------------------------------------------------------------------
//------------------------------------TRACKING COORDS----------------------------------------------

int updateCoords(float duration, int direction)
{
	int hyp, opp, adj, angle;

	//Working out hyp
	hyp = duration*dps;

	//Working out angle, opp and adj
	if(roboBearing >= 0 && roboBearing <= 90)
	{
		angle = 90 - roboBearing;		
		adj = cos(angle)*hyp;		//x
		opp = sin(angle)*hyp;		//y
	}
	else if(roboBearing > 90 && roboBearing <= 180)
	{
		angle = roboBearing - 90;		
		adj = cos(angle)*hyp;		//x
		opp = -(sin(angle)*hyp);	//y
	}
	else if(roboBearing > 180 && roboBearing <= 270)
	{
		angle = 270 - roboBearing;
		adj = -(cos(angle)*hyp);	//x
		opp = -(sin(angle)*hyp);	//y
	}
	else
	{
		angle = roboBearing - 270;
		adj = -(cos(angle)*hyp);	//x
		opp = sin(angle)*hyp;		//y
	}

	for(int i = 0; i < numFoundSites; i++)
	{
		if(direction == 1)
		{
			siteCoord[i][0] = siteCoord[i][0] - adj;
			siteCoord[i][1] = siteCoord[i][1] - opp;
		}
		else
		{
			siteCoord[i][0] = siteCoord[i][0] + adj;
			siteCoord[i][1] = siteCoord[i][1] + opp;
		}
	}

	return 0;
}


//-------------------------------------------------------------------------------------------------
//------------------------------------- SWEEP READINGS---------------------------------------------

int sweep()
{
	//This will sweep full range (100 deg) recording IR and sonar every 5 deg
	int index = 0;
	CPhidgetServo_setPosition (servo, 0, 60.00);
	usleep(500000);
	for(int i = 60; i <= 160; i += 5)
	{
		if((dInputIndex == 5 && dInputState == 1) || (dInputIndex == 6 && dInputState == 1))
		{
			CPhidgetServo_setPosition (servo, 0, 110);
			printf("Bumper pressed, sweeping stopped...\n");
			break;
		}
		CPhidgetServo_setPosition (servo, 0, i);
		usleep(350000);
		forwardIrSweepReadings[index] = sensorArray[7];
		angledIrSweepReadings[index] = sensorArray[6];
		sonarSweepReadings[index] = sensorArray[5];
		//printf("IR %d position reading: %d\n", i, irSweepReadings[index]);
		index += 1;
	}
	CPhidgetServo_setPosition (servo, 0, 110.00);
	return 0;
}

//-------------------------------------------------------------------------------------------------
//--------------------------------------ROTATE TO BEARING------------------------------------------

int rotateToFace(int servoAngle)
{
	//Takes in a servoAngle, NOT bearing
	//110 is the direction the robot is facing
	int degTurn, turnTime;

	printf("Robo bearing is: %d Servo angle is: %d\n", roboBearing, servoAngle);
	degTurn = -110 + servoAngle;
	turnTime = abs(degTurn) * 13611;	//4.85secs for 360

	if(degTurn <= 0)		//left
	{
		RotateLeft(motoControl);
		printf("Turning left for: %d micro seconds...\n", turnTime);
		usleep(turnTime);
		Stop(motoControl);
		roboBearing = roboBearing - degTurn;
	}

	else if(degTurn > 0)	//right
	{
		RotateRight(motoControl);
		printf("Turning right for: %d micro seconds...\n", turnTime);
		usleep(turnTime);
		Stop(motoControl);
		roboBearing = (roboBearing + degTurn) % 360;
	}

	if(roboBearing < 0)
	{
		roboBearing = 360 + roboBearing;
	}

	printf("Current robot bearing: %d\n", roboBearing);

	return 0;
}

//-------------------------------------------------------------------------------------------------
//--------------------------------------------FREQUENCY--------------------------------------------

float frequency()
{
	float freq;
	int wait = 6;
	int index = 0;

	int leftFlashCount[wait];
	int rightFlashCount[wait];
	int leftPrevLight = 0;
	int rightPrevLight = 0;
	float leftVar;
	float rightVar;

	//Initialise flash counts
	for (int i = 0; i < wait; i++)
	{
		leftFlashCount[i] = 0;
		rightFlashCount[i] = 0;
	}

	time_t currentTime = time(NULL);
	time_t loopTime;
	time_t endTime = time(NULL) + wait;


	while((currentTime < endTime) && (index < wait))
	{
		currentTime = time(NULL);
		loopTime = time(NULL) + 1;
		while(currentTime < loopTime)
		{
			//--------------Left light flash count---------------
			//Increase flash count and set prevLight to a high value (light on)
			if((sensorArray[2] > leftLightThresh) && (leftPrevLight == 0))
			{
				leftPrevLight = 1;
				leftFlashCount[index] += 1;
				printf("Left for second %d increasing to: %d\n", index, leftFlashCount[index]);
			}
			//Set prevLight to a low value (light off)
			else if((sensorArray[2] <= leftLightThresh) && (leftPrevLight == 1))
			{
				leftPrevLight = 0;
			}
			
			//Increase flash count and set prevLight to a high value (light on)
			if((sensorArray[3] > rightLightThresh) && (rightPrevLight == 0))
			{
				rightPrevLight = 1;
				rightFlashCount[index] += 1;
				printf("Right for second %d increasing to: %d\n", index, rightFlashCount[index]);
			}
			//Set prevLight to a low value (light off)
			else if((sensorArray[3] <= rightLightThresh) && (rightPrevLight == 1))
			{
				rightPrevLight = 0;
			}
	
			currentTime = time(NULL);
			//printf("Time: %d\n", currentTime);
		}
		printf("Index: %d, Left: %d, Right: %d\n", index, leftFlashCount[index], rightFlashCount[index]);
		index += 1;
	}

	//--------GET VARIANCE--------
	float lmean = 0, rmean = 0;
	float lsquare = 0, rsquare = 0;
	for(int i = 0; i < wait; i++)
	{
		lmean += leftFlashCount[i];
		rmean += rightFlashCount[i];
		lsquare += leftFlashCount[i]*leftFlashCount[i];
		rsquare += rightFlashCount[i]*rightFlashCount[i];
	}

	lmean = lmean/wait;
	rmean = rmean/wait;
	lsquare = lsquare/wait;
	rsquare = rsquare/wait;

	leftVar = sqrt((lmean*lmean) - lsquare);
	rightVar = sqrt((rmean*rmean) - rsquare);

	//-------CHOOSE YOUR DESTINY----------
	if(leftVar <= rightVar && lmean != 0)
	{
		freq = lmean;
	}
	else if(rightVar <= leftVar && rmean != 0)
	{
		freq = rmean;
	}
	else
	{
		if(lmean >= rmean)
		{
			freq = lmean;
		}
		else if(rmean >= lmean)
		{
			freq = rmean;
		}
	}
	printf("Frequency reading at: %fHz (%f | %f)\n", freq, lmean, rmean);
	return freq;
}

//-------------------------------------------------------------------------------------------------
//-------------------------------JUST DANCE, GONNA BE OKAY-----------------------------------------

int dance(float freq)
{
	if(freq <= 0.75)
	{
		printf("Frequency is 0.5Hz\nDancin'..\n");
		GoBackward(motoControl);
		sleep(1);
		RotateLeft(motoControl);
		sleep(4);
		usleep(850000);
		Stop(motoControl);
		updateCoords(1,0);

		sleep(5);
		rotateToFace(290);
	}
	else if(freq > 0.75 && freq <= 1.5)
	{
		printf("Frequency is 1Hz\nDancin'..\n");
		GoBackward(motoControl);
		sleep(1);
		RotateRight(motoControl);
		sleep(4);
		usleep(850000);
		Stop(motoControl);
		updateCoords(1,0);

		sleep(5);
		rotateToFace(290);
	}
	else if(freq > 1.5 && freq <= 2.5)
	{
		printf("Frequency is 2Hz\nDancin'..\n");
		GoBackward(motoControl);
		usleep(500000);
		Stop(motoControl);
		usleep(500000);
		GoBackward(motoControl);
		usleep(500000);
		Stop(motoControl);
		updateCoords(1,0);

		sleep(5);
		rotateToFace(290);
	}
	else if(freq > 2.5 && freq <= 5)
	{
		printf("Frequency is 4Hz\nDancin'..\n");
		GoBackward(motoControl);
		sleep(1);
		RotateLeft(motoControl);
		sleep(2);
		usleep(425000);
		Stop(motoControl);
		updateCoords(1,0);
	}
	else if(freq > 5 && freq <= 7)
	{
		printf("Frequency is 6Hz\nDancin'..\n");
		GoBackward(motoControl);
		sleep(1);
		RotateRight(motoControl);
		sleep(2);
		usleep(425000);
		Stop(motoControl);
		updateCoords(1,0);
	}
	else if(freq > 7 && freq <= 9)
	{
		printf("Frequency is 8Hz\nDancin'..\n");
		GoBackward(motoControl);
		sleep(1);
		usleep(500000);
		Stop(motoControl);
		usleep(500000);
		GoForward(motoControl);
		usleep(500000);
		Stop(motoControl);
		updateCoords(0.5,0);

		sleep(5);
		rotateToFace(290);
	}
	else
	{
		printf("Frequency is MENTAL!!!!\nCan't dance to that!\n");
		GoBackward(motoControl);
		usleep(500000);
		Stop(motoControl);
	}
	return 0;
}

//-------------------------------------------------------------------------------------------------
//---------------------------------------IR SWEEP ANALYSIS-----------------------------------------

int* irAnalysis()
{
	//Look for a v-shape or inverted v-shape
	int midPoint = -1;
	int inverted = 0;
	int irAnalysis[2];
	int* irPointer;

	irPointer = irAnalysis;
	
	if(forwardIrSweepReadings[0] < forwardIrSweepReadings[1])
	{
		inverted = 1;
	}
		
	for(int i = 1; i < 18; i++)
	{
		if(inverted == 0)
		{
			if(i > i+1)
			{
				midPoint = i;
				break;
			}
		}
		else
		{
			if(i <= i+1)
			{
				midPoint = i;
				break;
			}
		}
	}

	irAnalysis[0] = inverted;
	irAnalysis[1] = midPoint;
	//*****NEED TO RETURN IF INVERTED OR NOT*****
	return irPointer;
}

//-------------------------------------------------------------------------------------------------
//---------------------------------------GET COORDINATE--------------------------------------------

int* getCoord(int distance, int bearing)
{
	//Returns coords in relation to (0,0)
	//Zero bearing is direction robot started at
	//Distance is in cm
	int* pointer;
	int adj = 0;
	int opp = 0;
	float rads = 0;
	int coordinates[2];
	
	pointer = coordinates;
	
	if(bearing > 0 && bearing <= 90)
	{
		rads = (bearing * 3.14159265) / 180;
		coordinates[0] = distance * sin(rads); //adj
		coordinates[1] = distance * cos(rads); //opp
	}
	else if(bearing > 90 && bearing <= 180)
	{
		bearing = bearing - 90;
		rads = (bearing * 3.14159265) / 180;
		coordinates[0] = distance * cos(rads); //adj
		coordinates[1] = -(distance * sin(rads)); //opp
	}
	else if(bearing > 180 && bearing <= 270)
	{
		bearing = bearing - 180;
		rads = (bearing * 3.14159265) / 180;
		coordinates[0] = -(distance * sin(rads)); //opp
		coordinates[1] = -(distance * cos(rads)); //adj
	}
	else
	{
		bearing = bearing - 270;
		rads = (bearing * 3.14159265) / 180;
		coordinates[0] = -(distance * cos(rads)); //adj
		coordinates[1] = distance * sin(rads); //opp
	}
	
	return pointer;
}

//-------------------------------------------------------------------------------------------------
//-----------------------------------------SWEEP 270-----------------------------------------------

int sweep270()
{
	int index = 0;
	int* coordPointer;
	int distance = 0;			//Holds IR reading in cm
	int prevDistance = 0;
	int distanceThresh = 200;	//200cm

	int siteLoc = 110;
	int siteSize = 0;			//Holds the site size
	int siteSizeMin = 4;		//Set min for possible site (num*5 = degrees)
	int siteSizeMax = 20;

	//------FILL IN 270 ARRAY------
	//Turn left
	rotateToFace(20);

	//Get 360 readings array
	printf("---------------------------------------------------------------------------------------------------\n");
	printf("Position\t Bearing\t Sonar\t\t IR\t\t Distance(cm)\n");
	printf("---------------------------------------------------------------------------------------------------\n");
	for(int i = 0; i < 3; i++)
	{
		sweep();
		for(int i2 = 0; i2 < 17; i2++)
		{
			printf("%d\t\t %d\t\t %d\t\t %d\t\t %d\n", i, (index*5), sonarSweepReadings[i2], forwardIrSweepReadings[i2], irToDistance(forwardIrSweepReadings[i2]));
			readingsIr270[index] = forwardIrSweepReadings[i2];
			readingsSonar270[index] = sonarSweepReadings[i2];
			index++;
		}

		rotateToFace(200);
	}
	//Fill in the last two as don't overlap
	readingsIr270[52] = forwardIrSweepReadings[18];
	readingsSonar270[52] = sonarSweepReadings[18];
	readingsIr270[53] = forwardIrSweepReadings[19];
	readingsSonar270[53] = sonarSweepReadings[19];

	rotateToFace(20);

	//-----ANALYSE------
	index = 0;
	for(int i = 0; i < 53; i++)
	{
		distance = irToDistance(readingsIr270[i]);
		//printf("Bearing: %d, Distance %d\n", ((315+(i*5))%360), distance);
		
		//On first case set both to index 0
		if(i == 0)
		{
			prevDistance = distance;
		}
		else
		{
			prevDistance = irToDistance(readingsIr270[i-1]);
		}
		
		//Reading closer than threshold
		if(distance <= distanceThresh)
		{
			//if first reading of possible site
			if(siteSize == 0)
			{
				siteSize = 1;
				//printf("Start of possible site at bearing %d\n", ((315+(i*5))%360));//This is were shit gets funny
			}
			
			//else if continuation of possibility
			else if((siteSize > 0) && (distance >= prevDistance - 15 && distance <= prevDistance + 15))
			{
				//Check it's still of correct size
				if(siteSize < siteSizeMax)
				{
					siteSize++;
					//printf("Size of object increased to %d\n", siteSize);
				}
				//Too big to be a site, reset
				else
				{		
					printf("Too big to be a site\n");
					siteSize = 0;
				}
			}
			
			//else new possible site start, record location of last
			else
			{
				if(siteSize > 0 && siteSize < siteSizeMin)
				{
					printf("Site too small\n");
					siteSize == 0;
				}	
				else if(siteSize <= siteSizeMax && siteSize >= siteSizeMin)
				{
					//Find the true bearing
					siteLoc = (roboBearing-140) + ((i-siteSize) + (siteSize/2))*5;
					
					//If negative subtract from 360 for real bearing
					if(siteLoc < 0)
					{
						siteLoc = 360 + siteLoc;
					}
					
					//printf("Saving possible site at bearing: %d\n", siteLoc);
					
					//Check if saved, or add as new possible site
					coordPointer = getCoord(distance, siteLoc);

					for(int i2 = 0; i2 < numFoundSites; i2++)
					{
						int dist = sqrt((siteCoord[i2][0] - coordPointer[0])*(siteCoord[i2][0] - coordPointer[0]) + (siteCoord[i2][1] - coordPointer[1])*(siteCoord[i2][1] - coordPointer[1]));
						
						if(dist <= siteRadius)
						{
							//Update the coord as mean of possibilites
							siteCoord[i2][0] = (siteCoord[i2][0] + coordPointer[0])/2;
							siteCoord[i2][1] = (siteCoord[i2][1] + coordPointer[1])/2;
						}
						else
						{
							numFoundSites++;

							if(numFoundSites == numPosSites)
							{
								//printf("Too many sites!\n");
								return 0;
							}
							siteCoord[numFoundSites-1][0] = coordPointer[0];
							siteCoord[numFoundSites-1][1] = coordPointer[1];
						}
					}

					printf("Site at bearing %d has coordinates: [%d, %d]\n", siteLoc, siteCoord[index][0], siteCoord[index][1]);
				}
				siteSize = 1;
				//printf("Start of possible site at bearing %d\n", ((315+(i*5))%360));
			}
		}
			
		//Reading is further than threshold	
		else
		{
			//Save location of current site
			if(siteSize >= siteSizeMin && siteSize <= siteSizeMax)
			{
				//Find the true bearing
					siteLoc = (roboBearing-140) + ((i-siteSize) + (siteSize/2))*5;
					
					//If negative subtract from 360 for real bearing
					if(siteLoc < 0)
					{
						siteLoc = 360 + siteLoc;
					}
					
					//printf("Saving possible site at bearing: %d\n", siteLoc);
					
					//Save the coordinates
					coordPointer = getCoord(distance, siteLoc);

					for(int i2 = 0; i2 < numFoundSites; i2++)
					{
						int dist = sqrt((siteCoord[i2][0] - coordPointer[0])*(siteCoord[i2][0] - coordPointer[0]) + (siteCoord[i2][1] - coordPointer[1])*(siteCoord[i2][1] - coordPointer[1]));
						
						if(dist <= siteRadius)
						{
							//Update the coord as mean of possibilites
							siteCoord[i2][0] = (siteCoord[i2][0] + coordPointer[0])/2;
							siteCoord[i2][1] = (siteCoord[i2][1] + coordPointer[1])/2;
							siteInfo[i2][0] = siteLoc;
							siteInfo[i2][1] = distance;
						}
						else if((distance < distanceThresh) && (distance != 0))
						{
							numFoundSites++;
							if(numFoundSites == numPosSites)
							{
								//printf("Too many sites!\n");
								return 0;
							}
							siteCoord[numFoundSites-1][0] = coordPointer[0];
							siteCoord[numFoundSites-1][1] = coordPointer[1];
							siteInfo[numFoundSites-1][0] = siteLoc;
							siteInfo[numFoundSites-1][1] = distance;
						}
					}

					printf("Site at bearing %d has coordinates: [%d, %d]\n", siteLoc, siteCoord[index][0], siteCoord[index][1]);
			}
			else if(siteSize > 0 && siteSize < siteSizeMin)
			{
				//printf("Site too small\n");
				siteSize = 0;
			}
		}
	}
	return 0;
}

//-------------------------------------------------------------------------------------------------
//----------------------------------------DOCK CONTROL---------------------------------------------

int dockControl()
{
	int gapLoc1 = 110;			//To hold the deg at which the gap was located, default
	int gapLoc2 = 0;			//This will hold a second gap loc, if found
	int horizontal1 = 0;		//This will hold the reading of the horizontal wall
	int horizontal2 = 0;		//This will hold the reading of the horizontal wall
	int gapSize = 0;			//The size of the gap located
	int wallCount = 0;			//Keeps track of the number of wall readings
	float freq = 0;				//This will hold the frequency of the light pressed
	
	Stop(motoControl);
	
	sweep();
	
	if((dInputIndex == 5 && dInputState == 1) || (dInputIndex == 6 && dInputState == 1))
	{
		goto pressed;	//Check for bumper presses
	}

	//----------------ANALYSE----------------------
	//Analyse the array..
	for(int i = 0; i < 19; i++)
	{
		//--------Wall reading----------
		if(angledIrSweepReadings[i] >= gapThresh)
		{
			wallCount += 1;

			//Wall after a gap
			if(gapSize > 0)
			{
				//60deg + start of gap + Mid point of gap
				gapLoc1 = 60 + ((i-gapSize) + (gapSize/2))*5;

				//If gap at far left of sensing range (not bounded by walls)
				if(i - gapSize == 0)
				{
					printf("Possible gap found at Location: %d. Continue search for other potentials..\n", gapLoc1);
					gapSize = 0;
				}
			
				//If there is a wall either side of the gap, break
				else if(angledIrSweepReadings[i-(gapSize+1)] <= gapThresh)
				{
					printf("True gap found at Location: %d\n", gapLoc1);
					goto respond;
				}
			}
		}
			
		//-------Open space reading--------
		if(angledIrSweepReadings[i] < gapThresh)
		{
			gapSize += 1;
			
			//If gap on far right of sensing range (not bounded by walls)
			if(i == 19)
			{
				if(gapLoc1 != 110)
				{
					gapLoc2 = 60 + ((i-gapSize) + (gapSize/2))*5;
					printf("Possible second gap found at Location: %d\n", gapLoc2);
				}
				else
				{
					gapLoc1 = 60 + ((i-gapSize) + (gapSize/2))*5;
					printf("Possible first gap found at Location: %d\n", gapLoc1);
				}
			}
		}
	}

	//-------------FURTHER ANALYSIS----------------
	//If single gap detected and is open space
	if(gapLoc2 == 0 && (forwardIrSweepReadings[((gapLoc1 - 60)/5)] < underGapThresh))
	{
		//printf("Forward IR Reading under gap: %d\n", forwardIrSweepReadings[((gapLoc1 - 60)/5)]);
		//turn towards wall and scan again
		if(gapLoc1 < 110)
		{
			gapLoc1 = 155;
			printf("Open gap found to the left, turning right to face wall...\n");
		}
		else if(gapLoc1 > 110)
		{
			gapLoc1 = 65;
			printf("Open gap found to the right, turning left to face wall...\n");
		}
	}
	
	//If two gaps are detected
	else if(gapLoc2 != 0)
	{
		//If the IR reading below loc2 reads closer wall than loc1 choose loc2
		if(forwardIrSweepReadings[((gapLoc1 - 60)/5)] < forwardIrSweepReadings[((gapLoc2 - 60)/5)])
		{
			gapLoc1 = gapLoc2;
		}
	}

	//If no walls detected, proceed closer
	else if(wallCount == 0)
	{
		printf("Nothing detected, proceed forward\n");
		GoForward(motoControl);
		usleep(500000);
		Stop(motoControl);
		updateCoords(0.5,1);
		return 0;
	}
	
	//If no gaps detected, back-up
	else if(wallCount == 20)
	{
		printf("No gap found, backing up\n");
		GoBackward(motoControl);
		usleep(500000);
		updateCoords(0.5,0);
		Stop(motoControl);
		return 0;
	}

	
	//---------------RESPOND-----------------------	
	//Rotate to face the direction of the gap
	respond:
	rotateToFace(gapLoc1);

	//Move towards gap
	pressed:
	for(int i = 0; i < 5; i++)
	{
		GoForward(motoControl);
		usleep(100000);
		updateCoords(0.1,1);

		//---------------BUMPER PRESS-------------------
		//If front left or right bumper is pressed, wait to see if light is on
		if((dInputIndex == 5 && dInputState == 1) || (dInputIndex == 6 && dInputState == 1))
		{
			Stop(motoControl);
			printf("Light readings at: %d and %d\n", sensorArray[2], sensorArray[3]);
			for(int i2 = 0; i2 < 19; i2++)
			{
				//If light response is found move out zone
				if((sensorArray[2] > rightLightThresh) || (sensorArray[3] > leftLightThresh))	//***VALUE CHANGE***
				{
					printf("Light pressed! Light readings: %d and %d\n", sensorArray[2], sensorArray[3]);
					GoForward(motoControl);
					usleep(250000);
					Stop(motoControl);
					freq = frequency();
					dance(freq);
					GoForward(motoControl);
					sleep(1);
					updateCoords(1,1);
					siteInfo[currentGoal][2] = 1; //Set site as visited
					sweep270();
					return 0;
				}
				usleep(50000);
			}

			if(dInputIndex == 5 && dInputState == 1)
			{
				//No response so get a better angle
				printf("No light response, getting better angle\n");
				GoBackward(motoControl);
				usleep(500000);
				RotateLeft(motoControl);
				usleep(500000);
				GoBackward(motoControl);
				usleep(500000);
				RotateRight(motoControl);
				usleep(500000);
				Stop(motoControl);
				break;
			}
			else if(dInputIndex == 6 && dInputState == 1)
			{
				//No response so get a better angle
				printf("No light response, getting better angle\n");
				GoBackward(motoControl);
				usleep(500000);
				RotateRight(motoControl);
				usleep(500000);
				GoBackward(motoControl);
				usleep(500000);
				RotateLeft(motoControl);
				usleep(500000);
				Stop(motoControl);
				break;
			}
		}
	}
	Stop(motoControl);
	
	return 0;
}

//-------------------------------------------------------------------------------------------------
//----------------------------------GET BEARING AND DIST TO COORD----------------------------------

int* getCoordInfo(int index)
{
	//Given destination coords, get bearing and distance
	int* pointer;
	int distance = 0;
	float bearing = 0;
	int info[2];
	int coord[2];
	
	pointer = info;
	coord[0] = siteCoord[index][0];
	coord[1] = siteCoord[index][1];
	printf("Site %d: [%d, %d]\n", index, coord[0], coord[1]);
	
	//-------------GET DISTANCE--------------

	distance = sqrt((coord[0]*coord[0])+(coord[1]*coord[1]));
	
	//--------------GET BEARING--------------
	
	if(coord[0] == 0)
	{
		goto dick;
	}

	bearing = fabs(atan(fabs((float) (coord[1])/abs(coord[0]))));
	printf("Bearing after 1st calc: %f using %d and %d\n", bearing, coord[0], coord[1]);
	bearing = (bearing*180.0)/3.14159265;
	printf("Bearing after 2nd calc: %f\n", bearing);
	
	if(coord[0] > 0 && coord[1] <= 0) //Bottom right
	{
		bearing = 90 + bearing;
		printf("Bottom right\n");
	}
	else if(coord[0] < 0 && coord[1] <= 0) //Bottom left
	{
		bearing = 180 + bearing;
		printf("Bottom left\n");
	}
	else if(coord[0] < 0 && coord[1] >= 0) //Top left
	{
		bearing =  270 + bearing;
		printf("Top left\n");
	}
	else
	{
		printf("Top right\n");
	}

	//-------------------SET COORD INFO------------------
	dick:
	info[0] = (int) bearing;
	info[1] = distance;
	printf("Site %d: Bearing: %d, Distance: %d\n", index, info[0], info[1]);

	return pointer;
}

//-------------------------------------------------------------------------------------------------
//---------------------------------GET ANGLE BETWEEN ROBOT AND SITE--------------------------------

int getAngleBetween()
{
	printf("Robo bearing and site info going into angle calc: %d, %d\n", roboBearing, siteInfo[currentGoal][0]);
	int difAngle = roboBearing - siteInfo[currentGoal][0];
	int direction;
	int servoAngle;
	
	//setting direction (0: anticlockwise, 1: clockwise)
	if(difAngle < 0 || (roboBearing < 90 && (270 < siteInfo[currentGoal][0] < 360)))
	{
		if(difAngle < -180)
		{
			difAngle = 360 + difAngle;
		}
		direction = 1;
	}
	else
	{
		if(difAngle > 180)
		{
			difAngle = 360 - difAngle;
		}
		direction = 0;
	}

	//converting into servo angle
	if(direction == 1)
	{
		servoAngle = 110 - difAngle;
	}
	else
	{
		servoAngle = 110 + difAngle;
	}
	printf("getAngleBetween: %d\n", servoAngle);
	return servoAngle;
}

//-------------------------------------------------------------------------------------------------
//--------------------------------------RANDOM SEARCH----------------------------------------------

int randomSearch()
{
	int time;
	int underAv = 0;

	//Explore
	GoForward(motoControl);

	//Left and right under light sensor
	while((sensorArray[0] >= leftThreshold) || (sensorArray[1] >= rightThreshold))
	{
		printf("Docking. Under light readings: %d and %d\n", sensorArray[0], sensorArray[1]);
		dockControl();
	}

	//Front IR sensor
	while(sensorArray[7] > 400)
	{
		printf("FRONTAL COLLISION IMMINENT!\n");
		RotateLeft(motoControl);
		usleep(200000);
		if((dInputIndex == 3 || dInputIndex == 4) && dInputState == 1)
		{
			GoForward(motoControl);
			usleep(500000);
			break;
		}
	}

	//Front left bumper
	if(dInputIndex == 5 && dInputState == 1)
	{
		printf("Front left bumper triggered!\n");

		time = 0;
		while(time < 1)
		{
			GoBackward(motoControl);
			usleep(250000);
			time++;

			//Rear touch sensors
			if((dInputIndex == 3 || dInputIndex == 4) && dInputState == 1)
			{
				printf("Rear bumper triggered!\n");
				GoForward(motoControl);
				usleep(500000);
				break;
			}
		}

		time = 0;
		while(time < 1)
		{
			RotateRight(motoControl);
			usleep(250000);
			time++;

			//Rear bumpers
			if((dInputIndex == 3 || dInputIndex == 4) && dInputState == 1)
			{
				printf("Rear bumper triggered!\n");
				GoForward(motoControl);
				usleep(500000);
				break;
			}
		}
		time = 0;
	}

	//Front right bumper	
	else if(dInputIndex == 6 && dInputState == 1)
	{
		printf("Front right bumper triggered!\n");

		time = 0;
		while(time < 1)
		{
			GoBackward(motoControl);
			usleep(250000);
			time++;

			//Rear bumpers
			if((dInputIndex == 3 || dInputIndex == 4) && dInputState == 1)
			{
				printf("Rear bumper triggered!\n");
				GoForward(motoControl);
				usleep(500000);
				break;
			}
		}			

		time = 0;
		while(time < 1)
		{
			RotateLeft(motoControl);
			usleep(250000);
			time++;

			//Rear bumpers
			if((dInputIndex == 3 || dInputIndex == 4) && dInputState == 1)
			{
				printf("Rear bumper triggered!\n");
				GoForward(motoControl);
				usleep(500000);
				break;
			}
		}
		time = 0;
	}
	return 0;
}

//-------------------------------------------------------------------------------------------------
//--------------------------------------------MAP 360----------------------------------------------

int map360()
{
	int index = 0;
	int* coordPointer;
	int distance = 0;			//Holds IR reading in cm
	int prevDistance = 0;
	int distanceThresh = 200;	//200cm
	
	int siteLoc = 110;
	int siteSize = 0;			//Holds the site size
	int siteSizeMin = 3;		//Set min for possible site (num*5 = degrees)
	int siteSizeMax = 20;


	//Initialise site location area to origin
	for(int i = 0; i < numPosSites; i++)
	{
		siteCoord[i][0] = 0;	//x
		siteCoord[i][1] = 0;	//y
		siteInfo[i][2] = 0;		//State (0 == not visited)
	}

	//Get 360 readings array
	printf("---------------------------------------------------------------------------------------------------\n");
	printf("Position\t Bearing\t Sonar\t\t IR\t\t Distance(cm)\n");
	printf("---------------------------------------------------------------------------------------------------\n");
	for(int i = 0; i < 4; i++)
	{
		sweep();
		for(int i2 = 1; i2 < 19; i2++)
		{
			printf("%d\t\t %d\t\t %d\t\t %d\t\t %d\n", i, (index*5), sonarSweepReadings[i2], forwardIrSweepReadings[i2], irToDistance(forwardIrSweepReadings[i2]));
			if(index == 71)
			{
				index = 0;
			}
			readingsIr360[index] = forwardIrSweepReadings[i2];
			readingsSonar360[index] = sonarSweepReadings[i2];
			index++;
		}
		rotateToFace(200);
	}
	
	//---------ANALYSE--------------
	
	index = 0;
	for(int i = 0; i < 71; i++)
	{
		distance = irToDistance(readingsIr360[i]);
		//printf("Bearing: %d, Distance %d\n", ((315+(i*5))%360), distance);
		
		//On first case set both to index 0
		if(i == 0)
		{
			prevDistance = distance;
		}
		else
		{
			prevDistance = irToDistance(readingsIr360[i-1]);
		}
		
		//Reading closer than threshold
		if(distance <= distanceThresh)
		{
			//if first reading of possible site
			if(siteSize == 0)
			{
				siteSize = 1;
				//printf("Start of possible site at bearing %d\n", ((315+(i*5))%360));
			}
			
			//else if continuation of possibility
			else if((siteSize > 0) && (distance >= prevDistance - 15 && distance <= prevDistance + 15))
			{
				//Check it's still of correct size
				if(siteSize < siteSizeMax)
				{
					siteSize++;
					//printf("Size of object increased to %d\n", siteSize);
				}
				//Too big to be a site, reset
				else
				{		
					//printf("Too big to be a site\n");
					siteSize = 0;
				}
			}
			
			//else new possible site start, record location of last
			else
			{
				if(siteSize > 0 && siteSize < siteSizeMin)
				{
					//printf("Site too small\n");
					siteSize == 0;
				}	
				else if(siteSize <= siteSizeMax && siteSize >= siteSizeMin)
				{
					//Find the true bearing
					siteLoc = (roboBearing-50) + ((i-siteSize) + (siteSize/2))*5;
					
					//If negative subtract from 360 for real bearing
					if(siteLoc < 0)
					{
						siteLoc = 360 + siteLoc;
					}
					
					//printf("Saving possible site at bearing: %d\n", siteLoc);
					
					//Save the coordinates
					coordPointer = getCoord(distance, siteLoc);
					if((distance < distanceThresh) && (distance != 0))
					{
						siteInfo[index][0] = siteLoc;
						siteInfo[index][1] = distance;
						siteCoord[index][0] = coordPointer[0];
						siteCoord[index][1] = coordPointer[1];
						printf("Site at bearing %d has coordinates: [%d, %d]\n", siteLoc, siteCoord[index][0], siteCoord[index][1]);
						numFoundSites++;
						index++;
					}

					if(index == numPosSites)
					{
						printf("Too many sites!\n");
						return 0;
					}
				}
				siteSize = 1;
				//printf("Start of possible site at bearing %d\n", ((315+(i*5))%360));
			}
		}
			
		//Reading is further than threshold	
		else
		{
			//Save location of current site
			if(siteSize >= siteSizeMin && siteSize <= siteSizeMax)
			{
				//Find the true bearing
					//siteLoc = 310 + ((i-siteSize) + (siteSize/2))*5;
					siteLoc = (roboBearing-50) + ((i-siteSize) + (siteSize/2))*5;
					
					//If negative subtract from 360 for real bearing
					if(siteLoc < 0)
					{
						siteLoc = 360  + siteLoc;
					}
					
					//printf("Saving possible site at bearing: %d\n", siteLoc);
					
					//Save the coordinates
					coordPointer = getCoord(distance, siteLoc);
					if((distance < distanceThresh) && (distance != 0))
					{
						siteInfo[index][0] = siteLoc;
						siteInfo[index][1] = distance;
						siteCoord[index][0] = coordPointer[0];
						siteCoord[index][1] = coordPointer[1];
						printf("Site at bearing %d has coordinates: [%d, %d]\n", siteLoc, siteCoord[index][0], siteCoord[index][1]);
						numFoundSites++;
						index++;
					}

					if(index == numPosSites)
					{
						//printf("Too many sites!\n");
						return 0;
					}
			}
			else if(siteSize > 0 && siteSize < siteSizeMin)
			{
				//printf("Site too small\n");
				siteSize = 0;
			}
		}
	}	
	return 0;
}

//-------------------------------------------------------------------------------------------------
//-----------------------------------MANEUVER AROUND OBJECT----------------------------------------

int maneuver()
{
	int turnTo;
	int clearThresh = 200;
	int servoAngle;
	
	Stop(motoControl);
	GoBackward(motoControl);
	usleep(500000);
	Stop(motoControl);
	updateCoords(0.25,0);
	sweep();

	//Check to see if object passed by while sweeping (othr robot)
	if(sensorArray[7] < 250)
	{
		printf("Object passed!\n");
		return 0;
	}
	
	//*******COULD USE THE ANGLED IR TO DIFFERENTATE BETWEEEN ROBOT AND WALL*********

	//Turning left if detect sloping away on left side
	if(forwardIrSweepReadings[6] < forwardIrSweepReadings[9])
	{	
		turnTo = -1;
		for(int i = 9; i >= 0; i--)
		{
			//Set turnTo index as the first view of open space
			if(forwardIrSweepReadings[i] < clearThresh)
			{
				turnTo = i;
				break;
			}
		}
		//If no open space detected, turn 90deg left
		if(turnTo == -1)
		{
			servoAngle = 20;	
		}
		else
		{
			servoAngle = 60 + (turnTo-2)*5;
		}
		//Turn angle, IR face object
		CPhidgetServo_setPosition (servo, 0, 160.00);
		rotateToFace(servoAngle);
	}

	//Turning right
	else
	{	
		turnTo = -1;
		for(int i = 9; i <= 19; i++)
		{
			//Set turnTo index as the first view of open space
			if(forwardIrSweepReadings[i] < clearThresh)
			{
				turnTo = i;
				break;
			}
		}
		//If no open space detected, turn 90deg right
		if(turnTo == -1)
		{
			servoAngle = 200;	
		}
		else
		{
			servoAngle = 60 + (turnTo+2)*5;
		}
		//Found parallel, turn IR to face object
		CPhidgetServo_setPosition (servo, 0, 60.00);
		rotateToFace(servoAngle);
	}
	
	//Proceed forward until clear
	while(sensorArray[7] > 200)
	{
		GoForward(motoControl);
		updateCoords(0.5, 1);
		usleep(500000);
	}
	//Continue to clear back of robot
	GoForward(motoControl);
	updateCoords(1, 1);
	sleep(1);
	Stop(motoControl);
	
	//Turn back to face site
	CPhidgetServo_setPosition (servo, 0, 110.00);
	servoAngle = getAngleBetween();
	rotateToFace(getAngleBetween());
	
	return 0;
}

//-------------------------------------------------------------------------------------------------
//-----------------------------------IDENTIFY NEXT TARGET------------------------------------------

int nextSite()
{
	//The closest unvisited site will be the next target
	int* pointer;
	int direction = 1;		//0 turn clkwise, 1 turn anti-clkwise
	int closestSite = 0; 	//Default index 0

	printf("Identifying next site\n");
	
	//Update the angle and distance to each site at current location and bearing
	for(int i = 0; i < numFoundSites; i++)
	{
		pointer = getCoordInfo(i);
		siteInfo[i][0] = pointer[0];	//Bearing
		siteInfo[i][1] = pointer[1];	//Distance
		
		if((siteInfo[i][1] <= siteInfo[closestSite][1]) && (siteInfo[i][2] == 0))
		{
			closestSite = i;
		}
	}

	currentGoal = closestSite;
	
	return 0;
}
//-------------------------------------------------------------------------------------------------
//-------------------------------VERIFY COORD WHEN NOT DOCKED--------------------------------------

int verify()
{
	int midPoint;
	int inverted;
	int* analysisPointer;
	sweep();

	analysisPointer = irAnalysis();
	midPoint = analysisPointer[1];
	inverted = analysisPointer[0];

	//If not inverted and gap above midpoint drive to centre of v-shape
	if(inverted != 1 && angledIrSweepReadings[midPoint] <= 150)
	{
		rotateToFace(60 + midPoint*5);
		
	}
	//If not inverted but the midpoint is wall, not goal
	else if(inverted != 1 && angledIrSweepReadings[midPoint] > 150)
	{
		siteInfo[currentGoal][2] == 2; //Not a site
		return 0;
	}
	//If inverted, maneuver around to other side
	else if(inverted == 1)
	{
		maneuverCount++;
		maneuver();
	}

	return 0;
}
//-------------------------------------------------------------------------------------------------
//------------------------------------------CONTROL------------------------------------------------

int control()
{
	int servoAngle;
	int distTo;
	int maneuverCount = 0;
	int* analysisPointer;
	time_t endTime, startTime;

	//These are to be updated at the end of control cycle or trigger press
	//Used to calculate straight distance travelled
	startTime = time(NULL);
	
	if(siteInfo[currentGoal][2] == 1 || siteInfo[currentGoal][2] == 2 || firstRun == 1)
	{
		for (int i = 0; i < numFoundSites; i++)
		{
			//Count actual sites visited
			if(siteInfo[i][2] == 1)
			{
				numSitesCompleted++;
				numPossCompleted++;
			}
			//Count possible sites visited
			else if(siteInfo[i][2] == 2)
			{
				numPossCompleted++;
			}
		}
		//If all possible sites have been visited, stop
		if(numSitesCompleted == numFoundSites)
		{
			if(numSitesCompleted != 5)
			{
				Stop(motoControl);
			}
			else
			{
				randomSearch();
			}
		}
		else
		{
			nextSite();		//Set new goal site
			firstRun = 0;
			printf("Current goal coordinates set to [%d, %d] at distance %d and bearing %d\n", siteCoord[currentGoal][0], siteCoord[currentGoal][1], siteInfo[currentGoal][1], siteInfo[currentGoal][0]);
			printf("Rotating to face current goal...\n");
			servoAngle = getAngleBetween();
			rotateToFace(servoAngle);
		}
	}
	
	//printf("Proceeding...\n");
	GoForward(motoControl);
	
	//-----------------SITE LOCATED-------------------
	//Left and right under light sensor
	while((sensorArray[0] >= leftThreshold) || (sensorArray[1] >= rightThreshold))
	{
		printf("Docking. Under light readings: %d and %d\n", sensorArray[0], sensorArray[1]);
		endTime = time(NULL);
		updateCoords(endTime - startTime, 1);
		dockControl();
	}
	
	//----------------OBJECT DETECTED-----------------
	//Front IR sensor
	if(sensorArray[7] > 400)
	{
		printf("Object detected!\n");		

		//If further out than zone treat as obstacle
		if(siteInfo[currentGoal][1] > siteRadius)
		{
			printf("Not at destination, maneuvering around object..\n");
			while(sensorArray[7] > 400)
			{
				maneuver();
				maneuverCount++;
				if(maneuverCount = 5);
				{
					printf("Not a site\n");
					siteInfo[currentGoal][2] == 2; //Not a site
					rotateToFace(290);
					break;
				}
			}
		}
		
		//Search area for possible site
		else
		{
			printf("At destination, scanning to verify validity..\n");
			GoBackward(motoControl);
			usleep(250000);
			Stop(motoControl);
			updateCoords(0.25,0);
		
			distTo = sqrt((siteCoord[currentGoal][0]*siteCoord[currentGoal][0]) + (siteCoord[currentGoal][1]*siteCoord[currentGoal][1]));

			//While in the radius of the goal coord look for v-shapes
			while(distTo <= siteRadius && siteInfo[currentGoal][2] == 0)
			{
				maneuverCount++;
				verify();
				distTo = sqrt((siteCoord[currentGoal][0]*siteCoord[currentGoal][0]) + (siteCoord[currentGoal][1]*siteCoord[currentGoal][1]));
				if(maneuverCount = 5);
				{
					printf("Not a site\n");
					siteInfo[currentGoal][2] == 2; //Not a site
					break;
				}
				if((sensorArray[0] >= leftThreshold) || (sensorArray[1] >= rightThreshold))
				{
					break;
				}
			}						
		}
	}

	//-----------------COLLISION DETECTION------------
	//Front left bumper
	if(dInputIndex == 5 && dInputState == 1)
	{
		printf("Front left bumper triggered!\n");
		maneuver();
	}

	//Front right bumper	
	else if(dInputIndex == 6 && dInputState == 1)
	{
		printf("Front right bumper triggered!\n");
		maneuver();
	}
	endTime = time(NULL);
	updateCoords(endTime - startTime, 1);
	return 0;
}

//-------------------------------------------------------------------------------------------------
//----------------------------------------CLOSE PROGRAM--------------------------------------------

int close()
{
	Stop(motoControl);

	//this is a signal to terminate the program so we will close the phidget and delete the object we created
	printf("Closing...\n");
	CPhidget_close((CPhidgetHandle)motoControl);
	CPhidget_delete((CPhidgetHandle)motoControl);
	CPhidget_close((CPhidgetHandle)ifKit);
	CPhidget_delete((CPhidgetHandle)ifKit);
	CPhidget_close((CPhidgetHandle)servo);
	CPhidget_delete((CPhidgetHandle)servo);

	return 0;
}

//-------------------------------------------------------------------------------------------------
//--------------------------------------------MAIN-------------------------------------------------

int main(int argc, char* argv[])
{
	setup();

	//Reset all to ready position
	Stop(motoControl);
	power_button_reset();
	CPhidgetServo_setPosition (servo, 0, 110.00);
	roboBearing = 0;

	setThresholds();

	while(power_button_get_value() == 0)
	{
		Stop(motoControl);
	}
	while(power_button_get_value() == 1)
	{
		if(firstRun == 1)
		{
			map360();
		}
		control();
	}
	sleep(1);
	close();
	return 0;
}

//Distance in cm to IR: 512.08((x-5)/5)^-0.61
