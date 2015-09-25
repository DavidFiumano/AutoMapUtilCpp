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
/*
 * write to file using ofstream
 * use commas as deliminating characters
 * vector<char> ObjectReadBuffer;
 * While(
 */





int main()
{
	return 0;
}

AutoMap::pointOfInterest genPoint(int X, int Y, int L, int W)
{
	AutoMap::pointOfInterest pointReturn;
	pointReturn.xValue = X;
	pointReturn.yValue = Y;
	pointReturn.lengthOfZone = L;
	pointReturn.widthOfZone = W;
	return pointReturn;
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

	while(getline(ObjectiveList, buffer))
	{
		lines++;
	}

	buffer.clear();
	buffer.shrink_to_fit();

	int lineCounter = lines;

	while(lineCounter != 0)
	{
		getline(ObjectiveList, newRegisterName, ':');
		getline(ObjectiveList, newRegisterAddress);
		if(lineCounter != lines)//if this has been run before
		{
			ObjectiveRegister[lineCounter].memoryPosition = atoi(newRegisterAddress.c_str());
			ObjectiveRegister[lineCounter].objectiveName = newRegisterName;
		}else{
			newRegister.objectiveName = newRegisterName;
			newRegister.memoryPosition = atoi(newRegisterAddress.c_str());
			ObjectiveRegister.push_back(newRegister);
		}
		lineCounter--;
	}
	lineCounter = lines;
	while(lineCounter > 0)
	{
		ObjectiveRegister[lineCounter].objectiveName.shrink_to_fit();
		lineCounter--;
	}
	ObjectiveRegister.shrink_to_fit();
}

void AutoMap::LoadInitialFieldState()
{
	std::ifstream Map;
	//creates an ifstream object named "Map"
	char streamBuffer[fieldArea];
	Map.open("InitialFieldState.fs");

	pointOfInterest genPoint(int X, int Y, int L, int W);

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
    					Obstacles[barriersStored].push_back(collumnsCounted); //xPos
    					Obstacles[barriersStored].push_back(rowsCounted);//yPos
    					Obstacles[barriersStored].push_back(barrierLength);//Length
    					Obstacles[barriersStored].push_back(collumnsCounted);//xPos of end
    					Obstacles[barriersStored].push_back(rowsCounted - barrierLength);//yPos of end
    					Obstacles[barriersStored].shrink_to_fit();
    				}else if(parseBuffer[1] != '='){
    					Map.seekg(parseCounter + barrierLength);
    					Obstacles[barriersStored].push_back(collumnsCounted);//xPos
    					Obstacles[barriersStored].push_back(rowsCounted);//yPos
        				while(parseBuffer[1] == '=')
        				{
        					Map.seekg(parseCounter + barrierLength);
        					Map.read(parseBuffer, 1);
        					barrierLength++;
        				}
        				Obstacles[barriersStored].push_back(barrierLength);//Length
        				Obstacles[barriersStored].push_back(collumnsCounted + barrierLength);//xPos
        				Obstacles[barriersStored].push_back(rowsCounted);//yPos will always be the same
        				Obstacles[barriersStored].shrink_to_fit();
    				}
    				checkControl = barriersStored;
    				while(checkControl != 0)
    				{
    					if(Obstacles[barriersStored][4] == Obstacles[checkControl][4] && Obstacles[barriersStored][5] == Obstacles[checkControl][5])
    					{
    						Obstacles[barriersStored].clear();
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

    			//1st val is xPos, 2nd is yPos, 3rd is length, 4th is width
    			objectParsed = false; //used for parsing operations requiring a while loop
    			while(objectParsed == false)
    			{
    				//create vector
    				//store x and y value in vector (create seperate values for position of read and position of object found)
    				objectCreateCounter++;
    				Map.seekg(objectCreateCounter); //sets position to next character to read
    				Map.read(parseBuffer, parseControl);
    				if(parseBuffer[1] == 'O')
    				{
    					Map.seekg(objectCreateCounter);
    					Map.read(parseBuffer, parseControl);
    					objectCreateCounter++;
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
    				pointConverter = genPoint(AutoMap::collumnsCounted, AutoMap::rowsCounted, AutoMap::objectLength, AutoMap::objectWidth);
    				Objectives[objectsStored].push_back(pointConverter);
    				Objectives[objectsStored].shrink_to_fit();
    			}
    		}else if(parseBuffer[1] == 's'){
    				//STORES AS POI

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
    			    pointConverter = genPoint(collumnsCounted, rowsCounted, objectLength, objectWidth);
    			    Objectives[objectsStored].push_back(pointConverter);
   			    //STORES AS BARRIER
    			    barrierLength = 0;
    			    barriersStored++;
    			    Map.seekg(parseCounter + 1);
    			    Map.read(parseBuffer, parseControl);
    			    if (parseBuffer[1] ==  's')
    			    {
    			    	Map.seekg(parseCounter + fieldWidth); //moves to the position directly below the
    			    	Map.read(parseBuffer, 1);
    			    	if(parseBuffer[1] == 's')
    			    	{
    			    		while(parseBuffer[1] == 's')
    			    		{
    			    			Map.read(parseBuffer, 1);
    			    			barrierLength++;
    			    		}
    			    		Obstacles[barriersStored].push_back(collumnsCounted); //xPos
    			    		Obstacles[barriersStored].push_back(rowsCounted);//yPos
    			    		Obstacles[barriersStored].push_back(barrierLength);//Length
    			    		Obstacles[barriersStored].push_back(collumnsCounted);//xPos of end
    			    		Obstacles[barriersStored].push_back(rowsCounted - barrierLength);//yPos of end
    			       		}else if(parseBuffer[1] != 's'){
    			       			Map.seekg(parseCounter + barrierLength);
    			       			Obstacles[barriersStored].push_back(collumnsCounted);//xPos
    			       			Obstacles[barriersStored].push_back(rowsCounted);//yPos
    			       			while(parseBuffer[1] == 's')
    			       			{
    			       				Map.read(parseBuffer, 1);
    			       				barrierLength++;
    			       			}
    			       			Obstacles[barriersStored].push_back(barrierLength);//Length
    			       			Obstacles[barriersStored].push_back(collumnsCounted + barrierLength);//xPos
    			       			Obstacles[barriersStored].push_back(rowsCounted);//yPos will always be the same
    			       		}
    		       			checkControl = barriersStored;
    		       			while(checkControl != 0)
    		       			{
    		       				if(Obstacles[barriersStored][4] == Obstacles[checkControl][4] && Obstacles[barriersStored][5] == Obstacles[checkControl][5])
    		       				{
    		       				Obstacles[barriersStored].clear();
    		       				checkControl = 0;
    		       				}else{
    		    				checkControl--;
    			       		}
    		  			}
    		    Obstacles[barriersStored].shrink_to_fit();
    		    Objectives[objectsStored].shrink_to_fit();
    			barrierLength = 0;
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
    	Obstacles[counter].shrink_to_fit();
    	counter--;
    }
    Obstacles.shrink_to_fit();
    counter = objectsStored;
    while(counter <= objectsStored)
    {
    	Objectives[counter].shrink_to_fit();
    	counter--;
    }
    Objectives.shrink_to_fit();
}

void AutoMap::createObjective(std::string name, int nameLength, int xPosOfUpperLeftCorner, int yPosOfUpperLeftCorner, int objectLength, int objectWidth, bool loadIntoFile, ObjectiveType objectiveType)
{
	pointOfInterest genPoint(int x, int y, int length, int width);
	if(ObjectiveList.is_open() &&  loadIntoFile == true)
	{
		ObjectiveList.seekp(ObjectiveList.end);
		objectsStored++;
		//stores object name
		ObjectiveList << name << "\n" << objectsStored << "\n";
		/*writeTranslator[1] = objectsStored; //can only store up to 255 objects
		ObjectiveList.write(writeTranslator, 1);*/
		ObjectiveRegister[objectsStored].objectiveName.assign(name);
		newObjective = genPoint(xPosOfUpperLeftCorner, yPosOfUpperLeftCorner, objectLength, objectWidth);
		Objectives[objectsStored].push_back(newObjective);
		ObjectiveList.seekg(ObjectiveList.end);
		objectiveListSize = ObjectiveList.tellg();
		if(objectiveType == SOLID)
		{
			/* Length -> *
			 * 			 *
	Width->	 * 			 *
			 * 			 *
			 *	 	 	 */

			//STORE TOP
			barriersStored++;
			Obstacles[barriersStored].push_back(xPosOfUpperLeftCorner);
			Obstacles[barriersStored].push_back(yPosOfUpperLeftCorner);
			Obstacles[barriersStored].push_back(objectLength);
			Obstacles[barriersStored].push_back(xPosOfUpperLeftCorner + objectLength);
			Obstacles[barriersStored].push_back(yPosOfUpperLeftCorner);
			//STORE LEFT SIDE
			barriersStored++;
			Obstacles[barriersStored].push_back(xPosOfUpperLeftCorner);
			Obstacles[barriersStored].push_back(yPosOfUpperLeftCorner);
			Obstacles[barriersStored].push_back(objectWidth);
			Obstacles[barriersStored].push_back(xPosOfUpperLeftCorner);
			Obstacles[barriersStored].push_back(yPosOfUpperLeftCorner - objectWidth);
			//STORE BOTTOM
			barriersStored++;
			Obstacles[barriersStored].push_back(xPosOfUpperLeftCorner);
			Obstacles[barriersStored].push_back(yPosOfUpperLeftCorner - objectWidth);
			Obstacles[barriersStored].push_back(objectLength);
			Obstacles[barriersStored].push_back(xPosOfUpperLeftCorner + objectLength);
			Obstacles[barriersStored].push_back(yPosOfUpperLeftCorner - objectWidth);
			//STORE RIGHT SIDE
			barriersStored++;
			Obstacles[barriersStored].push_back(xPosOfUpperLeftCorner + objectLength);
			Obstacles[barriersStored].push_back(yPosOfUpperLeftCorner);
			Obstacles[barriersStored].push_back(objectWidth);
			Obstacles[barriersStored].push_back(xPosOfUpperLeftCorner + objectLength);
			Obstacles[barriersStored].push_back(yPosOfUpperLeftCorner - objectWidth);
		}else{
			objectsStored++;

		}
	}else if (loadIntoFile == true){
		printf("For some reason the ObjectiveList didn't open. Is it open in some other program? \n");
		printf("Objective not created. Sorry, I tried. \n");
	}else if (loadIntoFile == false){
		newObjective = genPoint(xPosOfUpperLeftCorner, yPosOfUpperLeftCorner, objectLength, objectWidth);
		Objectives[objectsStored].push_back(newObjective);
	}else{
		printf("Not sure exactly what happened, but it was an error!");
	}
    int counter = 0;
    while(counter <= barriersStored)
    {
    	Obstacles[counter].shrink_to_fit();
    	counter--;
    }
    Obstacles.shrink_to_fit();
    counter = objectsStored;
    while(counter <= objectsStored)
    {
    	Objectives[counter].shrink_to_fit();
    	counter--;
    }
    Objectives.shrink_to_fit();
    ObjectiveRegister.shrink_to_fit();
}

int AutoMap::FindPoint(std::string name)
{
	objectiveFind = objectsStored;
	while(objectiveFind != 0)
	{
		if(ObjectiveRegister[objectiveFind].objectiveName == name)
		{
			return ObjectiveRegister[objectiveFind].memoryPosition;
			objectiveFind = 0;
		}else{
		objectiveFind--;
		return 0;
		}
	}
}

