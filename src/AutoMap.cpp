/*
 * Automap.cpp
 *
 *  Created on: Jul 9, 2015
 *      Author: david
 */
#include "AutoMap.h"
//TODO path finding
//TODO movement and moving on set paths
//TODO get nathaniel to make file generator for c++ and java
//TODO TODO move class (closed loop control based)
//TODO TODO wrappers for move class
//TODO TODO TODO change anything that is stored as an open vector to a struct





int main()
{
	return 0;
}

void AutoMap::AutoMapInit(int robotLength, int robotWidth, DigitalSource * channelA, DigitalSource * channelB)
{

	fieldLength = 1646;
	//update this every time it's needed as well as any mention of 1646
	fieldWidth = 823;
	//update this every time it's needed as well as any mention of 1646
	fieldArea = fieldWidth*fieldLength;

	//INITIAL VALUES SET
	//PARSING CSV
	bufferParsed = false;
	parseError = false;
	parseControl = 1;
	//POSITIONING
	robotAngle = 180;

	objectsStored = 0;

	pushGuard = false;

	ObjectiveList.open("Objectives.txt"); //opens ObjectiveList
}

void AutoMap::LoadInitialFieldState()
{
	AutoMap::pointOfInterest genPoint(int X, int Y, int L, int W, int nameString);
	std::ifstream Map;
	//creates an ifstream object named "Map"
	char streamBuffer[fieldArea];
	Map.open("InitialFieldState.fs");

	//opens the file
    if(Map.is_open()) //checks if map opened properly
    {
    	printf(".fs file opened successfully \n");
    	printf("loading .fs into an array...");
    	Map.read(streamBuffer, fieldArea); //places entire .fs into streamBuffer so that other programmers can alter it and/or parse it themselves
    	Map.seekg(0, Map.beg); //resets the position we read from next useful for parsing()
       	char parseBuffer[parseControl];
       	barriersStored = 0;
       	parseCounter = 0;
    	while(bufferParsed == false && parseError == false)
    	{
    		rowsCounted = fieldLength;
    		Map.seekg(parseCounter);
    		collumnsCounted++;//adds to the xvalue that is counted at them momment
    		if(parseCounter == fieldLength)//sets the amount of horizontal rows counted
    		{
    			rowsCounted--; //adds to the yvalue that is counted at the momment
    		}
    		if(parseCounter == fieldWidth)//sets the amount of vertical collumns = to 0
    		{
    			collumnsCounted = 0;
    		}
    		Map.seekg(parseCounter);
    		Map.read(parseBuffer, parseControl);//reads map 1 character of map to parse control
    		//may need to use seekg here
    		if(parseBuffer[1] == ' ')
    		{

    			//blank space, do nothing
    		}else if(parseBuffer[1] == '='){
    			barrierLength = 0;
    			barriersStored++;
    			barrierLength++;
    			Map.seekg(parseCounter + 1);
    			Map.read(parseBuffer, parseControl);
    			if (parseBuffer[1] ==  '=')
    			{
    				Map.seekg(parseCounter + fieldWidth); //moves to the position directly below the
    				Map.read(parseBuffer, 1);
    				if(parseBuffer[1] == '=')
    				{
    					while(parseBuffer[1] == '=')
    					{
    						Map.read(parseBuffer, 1);
    						barrierLength++;
    					}
    					Obstacles.push_back(createObstacle(collumnsCounted, rowsCounted, barrierLength, collumnsCounted, rowsCounted - barrierLength, UNDEFINED));
    					/*Obstacles[barriersStored].push_back(collumnsCounted); //xPos
    					Obstacles[barriersStored].push_back(rowsCounted);//yPos
    					Obstacles[barriersStored].push_back(barrierLength);//Length
    					Obstacles[barriersStored].push_back(collumnsCounted);//xPos of end
    					Obstacles[barriersStored].push_back(rowsCounted - barrierLength);//yPos of end
    					Obstacles[barriersStored].shrink_to_fit();*/
    				}else if(parseBuffer[1] != '='){
    					Map.seekg(parseCounter + barrierLength);
        				while(parseBuffer[1] == '=')
        				{
        					Map.seekg(parseCounter + barrierLength);
        					Map.read(parseBuffer, 1);
        					barrierLength++;
        				}
        				Obstacles.push_back(createObstacle(collumnsCounted, rowsCounted, barrierLength, collumnsCounted + barrierLength, rowsCounted, HORIZONTAL));
        				Obstacles.shrink_to_fit();
    				}
    				checkControl = barriersStored;
    				while(checkControl != 0)
    				{
    					if(Obstacles[barriersStored].xPosOfEnd == Obstacles[checkControl].xPosOfEnd && Obstacles[barriersStored].yPosOfEnd == Obstacles[checkControl].yPosOfEnd)
    					{
    						Obstacles.pop_back();
    						checkControl = 0;
    					}else{
    					checkControl--;
    					}
    				}
    				barrierLength = 0;
    			}
    		}else if(parseBuffer[1] == 'O'){
    			//non-solid point of interest (go inside it)
    			//used for zones going
    			objectCreateCounter = Map.tellg();
    			guardCheck = guards;
    			objectParsed = false;
    			while(guardCheck != 0)
    			{
    				if(Guards[guardCheck].x == collumnsCounted && Guards[guardCheck].y == rowsCounted)
    				{
    					parseCounter = parseCounter + Guards[guardCheck].skipLength;
    					objectParsed = true;
    				}
    			}
    			//1st val is xPos, 2nd is yPos, 3rd is length, 4th is width
    			//used for parsing operations requiring a while loop

    			while(objectParsed == false)
    			{
    				//create vector
    				//store x and y value in vector (create seperate values for position of read and position of object found)
    				objectCreateCounter++;
    				Map.seekg(objectCreateCounter); //sets position to next character to read
    				Map.read(parseBuffer, parseControl);
    				if(parseBuffer[1] == 'O')
    				{
    					objectCreateCounter++;
    					Map.seekg(objectCreateCounter);
    					Map.read(parseBuffer, parseControl);
    					objectLength++;
    				}else{
    					objectCreateCounter = objectCreateCounter + fieldWidth;
    					Map.seekg(objectCreateCounter);
    					Map.read(parseBuffer, parseControl);
    					if(parseBuffer[1] != 'O')
    					{
    						objectParsed = true;
    					}else{
    						objectWidth++;
    					}
    				}
    				objectCreateCounter = 0;
    				//if next is o, read it, if next is not, add field width to reader and check there
    				//if nothing found there, then finish creating object
    				id = std::to_string(objectsStored);
    				pointConverter = AutoMap::genPoint(collumnsCounted, rowsCounted, objectLength, objectWidth, id);
    				Objectives.push_back(pointConverter);
    				Objectives.shrink_to_fit();
    			}
    		}else if(parseBuffer[1] == 's'){
    				//STORES AS POI
    				objectParsed = false;
    				while(guardCheck != 0)
    				{
    					if(Guards[guardCheck].x == collumnsCounted && Guards[guardCheck].y == rowsCounted)
    					{
    						parseCounter = parseCounter + Guards[guardCheck].skipLength;
    						objectParsed = true;
    					}
    				}
    				objectCreateCounter++;
    				Map.seekg(objectCreateCounter); //sets position to next character to read
    				Map.read(parseBuffer, parseControl);
    				if(parseBuffer[1] == 's')
    				{
    					Map.seekg(objectCreateCounter);
    			    	Map.read(parseBuffer, parseControl);
    			    	objectCreateCounter++;
    			    	objectLength++;
    				}else{
    			    	objectCreateCounter = objectCreateCounter + fieldWidth;
    			    	Map.seekg(objectCreateCounter);
    			    	Map.read(parseBuffer, parseControl);
    			    	if(parseBuffer[1] != 's')
    			    	{
    			    		objectParsed = true;
    			    	}else{
    			    		objectWidth++;
    			    	}
    			    }
    			    objectCreateCounter = 0;
    			    id = std::to_string(objectsStored);
    			    Objectives.push_back(AutoMap::genPoint(collumnsCounted, rowsCounted, objectLength, objectWidth, id));
   			    //STORES AS BARRIER
    			    Obstacles.push_back(AutoMap::createObstacle(collumnsCounted, rowsCounted, objectLength, collumnsCounted + objectLength, rowsCounted, HORIZONTAL)); //creates top line
    			    Obstacles.push_back(AutoMap::createObstacle(collumnsCounted, rowsCounted, objectWidth, collumnsCounted, rowsCounted - objectWidth, UNDEFINED)); //creats left line
    			    Obstacles.push_back(AutoMap::createObstacle(collumnsCounted, rowsCounted - objectWidth, objectLength, collumnsCounted + objectLength, rowsCounted - objectWidth, HORIZONTAL)); //creates bottom line
    			    Obstacles.push_back(AutoMap::createObstacle(collumnsCounted + objectLength, rowsCounted, objectWidth, collumnsCounted + objectLength, rowsCounted - objectWidth, UNDEFINED)); //create right line
    			    Obstacles.shrink_to_fit();
    			    Objectives.shrink_to_fit();
    		}else if(parseBuffer[1] == '+'){
    			//paths are treated as if they are over non-objective empty space

    		}else if(parseBuffer[1] == '~'){
    			//the robot
    			robotParseSeeker = robotLength + parseCounter;
    			robotParseMechanizism--;
    			robotParseMechanizism = rowsCounted;
    			if(robotParseMechanizism == 0)
    			{
    				robotPosition[1] = collumnsCounted;
    				robotPosition[2] = rowsCounted;
    				robotPosition[3] = 180;
    			}else{
    				Map.seekg(robotParseSeeker);
    			}

    		}else if(parseBuffer[1] == 'A'){
    			//action point, calls command referred to by pointer
    		}else if (parseBuffer[1] == 'N'){
    			printf(".fs didn't parse fully, for some reason it didn't load streamBuffer into parseBuffer \n");
    			printf("Is the file too short or corrupt? Recreate the .fs \n");
    			Obstacles.clear();
    			Objectives.clear();

    			parseError = true;
    			//if parseBuffer didn't update
    		}else{
    				printf(".fs enountered an unknown character, has the .fs been altered or corrupted? \n");
    				printf("Recreate .fs \n");
    				Obstacles.clear();
    				Objectives.clear();

    				parseError = true;
    			}
    			    //check the cases and parse them into arrays or vectors as required
    		    parseCounter++;
    		parseBuffer[1] = 'N';
    		}
	}else{
    	printf("Something went wrong opening the .fs for loading, is it open in another program? \n");
    	printf("Due to an error opening the .fs, the initial field state is not loaded \n");
    	delete[] streamBuffer;
    	Objectives.clear();
    }
    Map.close();
    int counter = 0;
    while(counter <= barriersStored)
    {
    	Obstacles.shrink_to_fit();
    	counter--;
    }
    Obstacles.shrink_to_fit();
    counter = objectsStored;
    while(counter <= objectsStored)
    {
    	Objectives.shrink_to_fit();
    	counter--;
    }
    Objectives.shrink_to_fit();
}

void AutoMap::createObjective(std::string name, int nameLength, int xPosOfUpperLeftCorner, int yPosOfUpperLeftCorner, int objectLength, int objectWidth, bool loadIntoFile, ObjectiveType objectiveType)
{
	if(ObjectiveList.is_open() &&  loadIntoFile == true)
	{
		ObjectiveList.seekp(ObjectiveList.end);
		objectsStored++;
		//stores object name
		ObjectiveList << name << "\n" << objectsStored << "\n";
		ObjectiveList << xPosOfUpperLeftCorner << "\n" << yPosOfUpperLeftCorner << "\n" << objectLength << "\n" << objectWidth << "\n";
		/*writeTranslator[1] = objectsStored; //can only store up to 255 objects
		ObjectiveList.write(writeTranslator, 1);*/
		Objectives.push_back(AutoMap::genPoint(collumnsCounted, rowsCounted, objectLength, objectWidth, name));
		ObjectiveList.seekg(ObjectiveList.end);
		objectiveListSize = ObjectiveList.tellg();
		//STORE AS OBSTACLE
		if(objectiveType == SOLID)
		{
			/* Length -> *
			 * 			 *
	Width->	 * 			 *
			 * 			 *
			 *	 	 	 */
			barriersStored++;
		    Obstacles.push_back(AutoMap::createObstacle(collumnsCounted, rowsCounted, objectLength, collumnsCounted + objectLength, rowsCounted, HORIZONTAL)); //creates top line
		    Obstacles.push_back(AutoMap::createObstacle(collumnsCounted, rowsCounted, objectWidth, collumnsCounted, rowsCounted - objectWidth, UNDEFINED)); //creats left line
		    Obstacles.push_back(AutoMap::createObstacle(collumnsCounted, rowsCounted - objectWidth, objectLength, collumnsCounted + objectLength, rowsCounted - objectWidth, HORIZONTAL)); //creates bottom line
		    Obstacles.push_back(AutoMap::createObstacle(collumnsCounted + objectLength, rowsCounted, objectWidth, collumnsCounted + objectLength, rowsCounted - objectWidth, UNDEFINED)); //create right line
		}else{
			objectsStored++;
		}
	}else if (loadIntoFile == true){
		printf("For some reason the ObjectiveList didn't open. Is it open in some other program? \n");
		printf("Objective not created. Sorry, I tried. \n");
	}else if (ObjectiveList.is_open() && loadIntoFile == false){
		id = std::to_string(objectsStored);
		newObjective = AutoMap::genPoint(xPosOfUpperLeftCorner, yPosOfUpperLeftCorner, objectLength, objectWidth, id);
		Objectives.push_back(newObjective);
	}else{
		printf("Not sure exactly what happened, but it was an error!");
	}
    Obstacles.shrink_to_fit();
    Objectives.shrink_to_fit();
}

int AutoMap::FindPoint(std::string name)
{
	objectiveFind = objectsStored;
	while(objectiveFind != 0 && objectiveFound == false)
	{
		if(name.compare(Objectives[objectiveFind].nameOfObjective) != 0)
		{
			objectiveFind--;
		}else if(name.compare(Objectives[objectiveFind].nameOfObjective) == 0)
		{
			objectiveFound = true;
			return objectiveFind;
		}
	}
	return -1; //if this is found, there was no objective named that, printf an error
}

AutoMap::pointOfInterest AutoMap::genPoint(int X, int Y, int L, int W, std::string nameString)
{
	pointOfInterest pointReturn;
	pointReturn.nameOfObjective = nameString;
	pointReturn.Length = L;
	pointReturn.Width = W;
	pointReturn.xPosOfUpperLeft = X;
	pointReturn.yPosOfUpperLeft = Y;
	pointReturn.xPosOfUpperRight = pointReturn.xPosOfUpperLeft + pointReturn.Length;
	pointReturn.yPosOfUpperRight = pointReturn.yPosOfUpperLeft;
	pointReturn.xPosOfLowerLeft = pointReturn.xPosOfUpperLeft;
	pointReturn.yPosOfLowerLeft = pointReturn.yPosOfUpperLeft - pointReturn.Width;
	pointReturn.xPosOfLowerRight = pointReturn.xPosOfUpperLeft + pointReturn.Length;
	pointReturn.yPosOfLowerRight = pointReturn.yPosOfUpperLeft - pointReturn.Width;
	return pointReturn;
}

AutoMap::obstacle AutoMap::createObstacle(int xStart, int yStart, int Length, int xFinal, int yFinal, slope SLOPE)
{
	obstacle returnObs;
	returnObs.xPosOfStart = xStart;
	returnObs.yPosOfStart = yStart;
	returnObs.xPosOfEnd = xFinal;
	returnObs.yPosOfEnd = yFinal;
	if(SLOPE == HORIZONTAL)
	{
		returnObs.SLOPE = 0;
		returnObs.slopeIsUndefined = false;
	}else if(SLOPE == UNDEFINED)
	{
		returnObs.slopeIsUndefined = true;
		returnObs.SLOPE = 0;
	}else if(SLOPE == OTHER)
	{
		returnObs.rise = (returnObs.yPosOfStart - returnObs.yPosOfEnd);
		returnObs.run = (returnObs.xPosOfStart - returnObs.xPosOfEnd);
		returnObs.SLOPE = rise/run;
	}
	return returnObs;
}

void AutoMap::createGuard(int x, int y, int length, int width)
{
	guardCounter = 0;
	while (guardCounter != width - 1)
	{
		guardCreate.x = x;
		guardCreate.y = y;
		guardCreate.skipLength = length;
		Guards.push_back(guardCreate);
		guardCounter++;
		guards++;
	}

}
