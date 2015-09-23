/*
 * AutoMapMove.cpp
 *
 *  Created on: Sep 22, 2015
 *      Author: david
 */
#include "AutoMap.h"

void AutoMap::setRobotPosition(int setX, int setY)
{
	robotPosition[1] = setX;
	robotPosition[2] = setY;
}

void AutoMap::setRobotAngle(int setAngle)
{
	robotAngle = setAngle;
}

void AutoMap::MonitorPos()
{
	//TODO check if turning
	//TODO check if at point of action
	AMEncoderObj = new Encoder(channelA, channelB, false, Encoder::EncodingType::k1X);
	//encoder starts counting
	inchesTraveled = AMEncoderObj->GetDistance();
	centimetersTraveled = 2.54 * inchesTraveled;
	if (centimetersTraveled < 0)
	{
		centimetersTraveled = centimetersTraveled * -1;
	}

	rise = centimetersTraveled * cos(robotAngle);
	run = centimetersTraveled * cos(robotAngle);

	robotPosition[2] = robotPosition[2] + (centimetersTraveled * (rise/run));
	robotPosition[1] = robotPosition[1] + (centimetersTraveled * (run/rise));

	AMEncoderObj->Reset();
	//monitors position and checks if robot is at action point every tick
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

bool AutoMap::checkTurn()
{

	return robotIsTurning;
}
