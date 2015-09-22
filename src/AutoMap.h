/*
 * Automap.h
 *
 *  Created on: Jul 9, 2015
 *      Author: david
 */

#ifndef AUTOMAP_H_
#define AUTOMAP_H_
#include <fstream>
#include <stdio.h>
#include <algorithm>
#include <vector>
#include <Encoder.h>
#include <cmath>


class AutoMap
{
public:

	Encoder * AMEncoderObj;

	DigitalSource * channelA;
	DigitalSource * channelB;

	uint32_t analogChannelA;
	uint32_t analogChannelB;

	struct pointOfInterest
		{
			int xValue;
			int yValue;
			int lengthOfZone;
			int widthOfZone;
		};



	pointOfInterest pointConverter;


	//TODO, replace with array template classes
	std::vector<std::vector<pointOfInterest>> Objectives;
	std::vector<std::vector<int>> Obstacles;
	std::vector<std::vector<int>>::iterator iHateTheCompiler; //hey, it's true

	long int fieldLength;
	long int fieldWidth;
	long int fieldArea;
	//both of the following will be used in path finding to ensure the robot goes where it needs to be
	int robotLength;//used in determining position of robot
	//cm
	int robotWidth;//used in determining position of robot
	//cm
	//set x and y position using these if you don't wish to use a file to do so
	int robotAngle; //angle relative to other end of the field (assumed 180)
	//potentially make a program to determine this for noobish teams? pick point straight ahead and compare slopes for angles?
	int robotPosition[3];
	//xVal of upper left corner, yVal of the same, angle of robot relative to ends of the field
	//robat!
	int setAngle;
	int objectsStored;

	bool xPosDetermined;
	bool yPosDetermined;


	static pointOfInterest genPoint(int X, int Y, int L, int W);
	int main();
	//eliminate upon completion of the project
	void MoveRobot(int time);
	//Moves robot at certain speed for so and so seconds
	void MoveRobotToPosition(int xPosMove, int yPosMove);

	void FindPath(int xFind, int yFind);

	void AutoMapInit(int robotLength, int robotWidth, uint32_t * channelA, uint32_t channelB);
	void AutoMapInit(int robotLength, int robotWidth, DigitalSource * channelA, DigitalSource * channelB); //gets all neccessary information needed to
	//will eventually take ports and other things too
	//MUST run first
	void LoadInitialFieldState(); //Loads the initial robotPosition into a set of arrays
	//MUST BE RUN OUTSIDE OF PERIODIC
	//only usable once
	/*
	 * empty space defined as a 0
	 * 1 where the robot's upper left corner would be
	 */
	void setRobotPosition(int setX, int setY); //sets position of robot, usable any time
	void setRobotAngle(int setAngle);
	void MonitorPos(); //monitors position of robot
	int GetPos(); //returns position of robot in x,y coordinates

	int inchesTraveled;
	int centimetersTraveled;



private:

	float rise;
	float run;

	bool bufferParsed;
	bool parseError;
	bool objectParsed;

	int checkControl;
	int barrierCheck[2];
	int barrierLength;
	int parseCounter; //used to parse
	int objectCreateCounter;//used for parsing objects
	int rowsCounted = 0;
	int collumnsCounted = 0;
	int verticalControl = 0;

	bool pushGuard;

	int xVal[1];
	int yVal[1];

	int objectLength = 0;
	int objectWidth = 0;

	long int parseControl;

	char parseBuffer[1];

	int barriersStored;

	int robotParseMechanizism = 0;
	int robotParseSeeker;




	void streamBufferParser();
};

#endif /* AUTOMAP_H_ */
