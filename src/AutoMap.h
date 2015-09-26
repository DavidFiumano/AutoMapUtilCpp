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
#include <PIDSource.h>


class AutoMap
{
public:



	enum ObjectiveType {SOLID, ZONE};
	enum slope{HORIZONTAL, UNDEFINED, OTHER};

	PIDSource * AMPIDObj;

	Encoder * AMEncoderObj;

	DigitalSource * channelA;
	DigitalSource * channelB;

	uint32_t analogChannelA;
	uint32_t analogChannelB;

	struct guard
	{
		int x;
		int y;
		int skipLength;
	};

	struct objective
	{
		std::string nameOfObjective;
		int xPosOfUpperLeft;
		int yPosOfUpperLeft;
		int xPosOfUpperRight;
		int yPosOfUpperRight;
		int xPosOfLowerLeft;
		int yPosOfLowerLeft;
		int xPosOfLowerRight;
		int yPosOfLowerRight;
		int Length;
		int Width;
		/* Length -> *
Width->  *		     *
		 *		     */
	};

	struct pointOfInterest
	{
		/*int xValue;
		int yValue;
		int lengthOfZone;
		int widthOfZone;*/
		std::string nameOfObjective;
		int xPosOfUpperLeft;
		int yPosOfUpperLeft;
		int xPosOfUpperRight;
		int yPosOfUpperRight;
		int xPosOfLowerLeft;
		int yPosOfLowerLeft;
		int xPosOfLowerRight;
		int yPosOfLowerRight;
		int Length;
		int Width;
			/* Length -> *
	Width->  *		     *
			 *		     */
	};

	struct objectiveRegister
	{
		std::string objectiveName;
		int memoryPosition;
	};
	struct obstacle
	{
		int xPosOfStart;
		int yPosOfStart;
		int lineLength;
		int xPosOfEnd;
		int yPosOfEnd;
		int SLOPE;
		int rise;
		int run;
		bool slopeIsUndefined;
	};

	std::fstream ObjectiveList;

	pointOfInterest pointConverter;


	//TODO, replace with array template classes
	std::vector<pointOfInterest> Objectives;
	std::vector<obstacle> Obstacles;

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

	pointOfInterest newObjective;


	bool xPosDetermined;
	bool yPosDetermined;

	bool robotIsTurning;

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


	void createObjective(std::string name, int nameLength, int xPosOfUpperLeftCorner, int yPosOfUpperLeftCorner, int objectLength, int objectWidth, bool loadIntoFile, ObjectiveType); //must be called outside of periodic
	bool checkTurn(); //checks if the robot turns
	void moveTime(int time); //seconds
	void turnTime(int time); //seconds
	void turnAngle(int angle); //take a guess
	void moveDistance(int dist); //centimeters
	void setRobotPosition(int setX, int setY); //sets position of robot, usable any time
	void setRobotAngle(int setAngle);
	void MonitorPos(); //monitors position of robot
	int FindPoint(std::string name);
	int GetPos(); //returns position of robot in x,y coordinates

	obstacle createObstacle(int xStart, int yStart, int Length, int xFinal, int yFinal, slope SLOPE);

	int inchesTraveled;
	int centimetersTraveled;

	char * internalRegister;



private:

	float rise;
	float run;

	objectiveRegister newRegister;

	std::string newRegisterName;
	std::string newRegisterAddress;

	bool bufferParsed;
	bool parseError;
	bool objectParsed;
	bool objectiveFound;


	int lineCount = 0;
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
	char writeTranslator[1];

	int barriersStored;

	int robotParseMechanizism = 0;
	int robotParseSeeker;

	int objectiveListSize;

	int objectiveFind;
	int lines = 0;

	std::string buffer;
	std::string id;

	guard guardCreate;

	std::vector<guard> Guards;

	int guardCounter;
	int guards;
	int guardCheck;

	pointOfInterest genPoint(int X, int Y, int L, int W, std::string nameString);
	void createGuard(int x, int y, int length, int width);
};

#endif /* AUTOMAP_H_ */
