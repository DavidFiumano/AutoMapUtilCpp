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
#include <PIDController.h>
#include <pthread.h>

class AutoMap
{
public:

	enum ObjectiveType {SOLID, ZONE};
	enum slope{HORIZONTAL, UNDEFINED, OTHER};

	Encoder * encoder1;
	Encoder * encoder2;
	Encoder * encoder3;
	Encoder * encoder4;

	struct slopeInfo
	{
		float rise;
		float run;
	};

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
		int numericalSlope;
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
	int robotDimensions[4];//used in determining position of robot
	//length, width, distance from corner to center, cm, circumference of circle
	int robotPosition[9];
	//x and y values of each of the four corners, angle of each of the corners
	//robat!
	int objectsStored;

	double slopeOfLine;

	pointOfInterest newObjective;

	bool robotIsTurning;

	uint32_t encoder1ChannelA; //front left
	uint32_t encoder1ChannelB;
	uint32_t encoder2ChannelA; //front right
	uint32_t encoder2ChannelB;
	uint32_t encoder3ChannelA; //back left
	uint32_t encoder3ChannelB;
	uint32_t encoder4ChannelA; //back right
	uint32_t encoder4ChannelB;

	float cornerAngle1;
	float cornerAngle2;
	float cornerAngle3;
	float cornerAngle4;

	int inchesTraveled1;
	int inchesTraveled2;
	int inchesTraveled3;
	int inchesTraveled4;
	float centimetersTraveled1;
	float centimetersTraveled2;
	float centimetersTraveled3;
	float centimetersTraveled4;

	int main();
	//eliminate upon completion of the project
	AutoMap(int robotLength, int robotWidth, uint32_t encoderChannels[8]);
	void MoveRobot(int time);
	//Moves robot at certain speed for so and so seconds
	void MoveRobotToPosition(int xPosMove, int yPosMove);

	void FindPath(int xFind, int yFind);

	void AutoMapInit(int robotLength, int robotWidth, uint32_t encoderChannels[8]);
	void AutoMapInitDigital(int robotLength, int robotWidth, DigitalSource * channelA, DigitalSource * channelB); //gets all neccessary information needed to
	//will eventually take ports and other things too
	//MUST run first
	void LoadInitialFieldState(); //Loads the initial robotPosition into a set of arrays
	//MUST BE RUN OUTSIDE OF PERIODIC
	//only usable once
	//if the robot is tilted to the side, then it shouldn't be included in the field state file

	void createObjective(std::string name, int nameLength, int xPosOfUpperLeftCorner, int yPosOfUpperLeftCorner, int objectLength, int objectWidth, bool loadIntoFile, ObjectiveType); //must be called outside of periodic
	bool checkTurn(); //checks if the robot turns
	void moveTime(int time); //seconds
	void turnTime(int time); //seconds
	void turnAngle(int angle); //take a guess
	void moveDistance(int dist); //centimeters
	void setRobotPosition(int setX[8]); //sets position of robot, usable any time
	void setRobotAngle(int setAngle);
	void MonitorPos(); //monitors position of robot
	int FindPoint(std::string name);
	int GetPos(); //returns position of robot in x,y coordinates
	void CorrectMovement(int centimetersTraveledLow, int centimetersTraveledAverage);
	void MeasureTurn(float encoder1, float encoder2, float encoder3, float encoder4);
	void createObstacle(int xStart, int yStart, int Length, int xFinal, int yFinal, slope SLOPE);

private:

	float rise;
	float run;

	float circleCircumference;

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

	float cornerSlope1;
	float cornerSlope2;
	float cornerSlope3;
	float cornerSlope4;

	float decSlope;
	std::string buffer;
	std::string id;

	guard guardCreate;

	std::vector<guard> Guards;

	int guardCounter;
	int guards;
	int guardCheck;

	pointOfInterest genPoint(int X, int Y, int L, int W, std::string nameString);
	void createGuard(int x, int y, int length, int width);

	bool lengthDetermined;

	slopeInfo slopeReturn;

};

#endif /* AUTOMAP_H_ */
