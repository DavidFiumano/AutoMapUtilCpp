/*
 * AutoMapMove.cpp
 *
 *  Created on: Sep 22, 2015
 *      Author: david
 */
#include "AutoMap.h"

void AutoMap::setRobotPosition(int set[8])
{
	robotPosition[1] = set[1]; //x upper left
	robotPosition[2] = set[2]; //y upper left
	robotPosition[3] = set[3]; //x upper right
	robotPosition[4] = set[4]; //y upper right
	robotPosition[5] = set[5]; //x lower left
	robotPosition[6] = set[6]; //y lower left
	robotPosition[7] = set[7]; //x lower right
	robotPosition[8] = set[8]; //y lower right
}

void AutoMap::setRobotAngle(int setAngle)
{
	robotPosition[9] = setAngle;
}

void AutoMap::MonitorPos()
{
	inchesTraveled1 = encoder1->GetDistance();//gets distance traveled by upper left wheel
	inchesTraveled2 = encoder2->GetDistance();//gets distance traveled by upper right wheel
	inchesTraveled3 = encoder3->GetDistance();//gets distance traveled by lower left wheel
	inchesTraveled4 = encoder4->GetDistance();//gets distance traveled by lower right wheel
	centimetersTraveled1 = 2.54 * inchesTraveled1;
	centimetersTraveled2 = 2.54 * inchesTraveled2;
	centimetersTraveled3 = 2.54 * inchesTraveled3;
	centimetersTraveled4 = 2.54 * inchesTraveled4;
	if(checkTurn() == true)
	{
		MeasureTurn(centimetersTraveled1, centimetersTraveled2, centimetersTraveled3, centimetersTraveled4);
	}else if(centimetersTraveled1 > 0 && centimetersTraveled2 > 0  && centimetersTraveled3 > 0 && centimetersTraveled4 > 0){ //TODO implement reasonable margin of error
		//GET NEW POSITION OF EACH CORNER
		//rise = centimetersTraveled1 * cos(robotPosition[9]); //ratio by which to alter xPosition
		//run = centimetersTraveled1 * cos(robotPosition[9]); //ratio by which to alter yPosition
		robotPosition[1] = robotPosition[1] + (centimetersTraveled1 * (pow(decSlope, -1))); //gets new x pos of upper left
		robotPosition[2] = robotPosition[2] + (centimetersTraveled1 * decSlope); //gets new y pos of upper left
		//rise = centimetersTraveled1 * cos(robotPosition[9]);
		//run = centimetersTraveled1 * cos(robotPosition[9]);
		robotPosition[3] = robotPosition[3] + (centimetersTraveled1 * (pow(decSlope, -1))); //gets new x pos of upper right
		robotPosition[4] = robotPosition[4] + (centimetersTraveled1 * decSlope); //gets new y pos of upper right
		//rise = centimetersTraveled1 * cos(robotPosition[9]);
		//run = centimetersTraveled1 * cos(robotPosition[9]);
		robotPosition[5] = robotPosition[5] + (centimetersTraveled1 * (pow(decSlope, -1))); //gets new x pos of lower left
		robotPosition[6] = robotPosition[6] + (centimetersTraveled1 * decSlope); //gets new y pos of lower left
		//rise = centimetersTraveled1 * cos(robotPosition[9]);
		//run = centimetersTraveled1 * cos(robotPosition[9]);
		robotPosition[7] = robotPosition[7] + (centimetersTraveled1 * (pow(decSlope, -1))); //gets new x pos of lower right
		robotPosition[8] = robotPosition[8] + (centimetersTraveled1 * decSlope); //gets new y pos of lower right


		encoder1->Reset();
		encoder2->Reset();
		encoder3->Reset();
		encoder4->Reset();
	}else{
		printf("Something funny happened measuring the position \n");
		printf("One or more of the encoders gave a negative value in a way that doesn't indicate the robot is turning \n");
	}
}

void AutoMap::MeasureTurn(float encoder1, float encoder2, float encoder3, float encoder4)
{
	cornerAngle1 = (360*encoder1)/(2*3.14*robotDimensions[3]); //determines the angle of each Angle
	cornerAngle2 = (360*encoder2)/(2*3.14*robotDimensions[3]);
	cornerAngle3 = (360*encoder3)/(2*3.14*robotDimensions[3]);
	cornerAngle4 = (360*encoder4)/(2*3.14*robotDimensions[3]);

	cornerSlope1 = atan(cornerAngle1); //gets decimal value of the slope
	cornerSlope2 = atan(cornerAngle2);
	cornerSlope3 = atan(cornerAngle3);
	cornerSlope4 = atan(cornerAngle4);

	decSlope = (cornerSlope1 + cornerSlope2 + cornerSlope3 + cornerSlope4)/4; //sets decSlope = to the mean of the angles
}

void AutoMap::CorrectMovement(int centimetersTraveledLow, int centimetersTraveledAverage)
{

}

void AutoMap::MoveRobotToPosition(int xPosMove, int yPosMove)
{

}

void AutoMap::FindPath(int xFind, int yFind)
{

}

void AutoMap::moveDistance(int dist)
{

}

void AutoMap::moveTime(int time)
{

}

void AutoMap::turnAngle(int angle)
{

}

void AutoMap::turnTime(int time)
{

}

bool AutoMap::checkTurn() //exists in case users have a bot that isn't perfectly rectangular
{
	if(inchesTraveled1 < 0 && inchesTraveled3 > 0)
	{
		robotIsTurning = true;
	}else if(inchesTraveled2 < 0 && inchesTraveled4 > 0){
		robotIsTurning = true;
	}else if(inchesTraveled1 > 0 && inchesTraveled3 < 0){
		robotIsTurning = true;
	}else if(inchesTraveled2 > 0 && inchesTraveled4 < 0){
		robotIsTurning = true;
	}else if (centimetersTraveled1 > 0 && centimetersTraveled2 > 0 && centimetersTraveled3 > 0 && centimetersTraveled4 > 0){
		robotIsTurning = false;
	}else if (centimetersTraveled1 < 0 && centimetersTraveled2 < 0 && centimetersTraveled3 < 0 && centimetersTraveled4 < 0){
		robotIsTurning = false;
	}else{
		printf("Something unexpected happened while checking turn, we'll assume the robot isn't turning");
		robotIsTurning = false;
	}
	return robotIsTurning;
}
