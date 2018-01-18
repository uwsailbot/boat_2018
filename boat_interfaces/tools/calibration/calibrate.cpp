#include <fstream>
#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <cstring>
#include <limits>

using namespace std;

const char PROGNAME[] = "compasscal"; //name of calibration program, no path, must be in same directory
const char FILEPATH[] = "/home/cliff/catkin_ws/src/phidgets_drivers/phidgets_imu/launch/imu.launch"; //launch file
const char BACKUP_FILEPATH[] = "/home/cliff/catkin_ws/src/phidgets_drivers/phidgets_imu/launch/imu_old.launch";
const int SIZE = 13;

bool getVals (const char progname[], char* arguments[], const int size);
bool parseLine (const char line[], char* arguments[], const int size);
bool readFile (const char filepath[], char * lines[], int& lineCount);
bool writeToFile (const char filepath[], const char* const lines[], const int lineCount, const char* const arguments[], const int size);
bool replaceValueInLine (const char line[], char changedLine[], const char newValue[]);
bool makeBackup (const char filepath[], const char backupFilepath[]);

int main (){
	char* arguments[SIZE];
	cout << "Running calibration program." << endl;
	if (!getVals (PROGNAME, arguments, SIZE)) {
		cerr << "Error running calibration program." << endl;
		return -1;
	}
	cout << "Beginning process to save these value to launch file:" << endl;
	cout << "Creating backup..." << endl;
	if (makeBackup (FILEPATH, BACKUP_FILEPATH)) {
		cout << "Backup successful, created at " << BACKUP_FILEPATH << endl;
	} else {
		cerr << "Backup failed." << endl;
	}
	char* lines[256];
	int lineCount = 0;
	if (!readFile (FILEPATH, lines, lineCount)) {
		cerr << "Error reading file." << endl;
		return -2;
	}
	if (!writeToFile (FILEPATH, lines, lineCount, arguments, SIZE)) {
		cerr << "Error writing to file." << endl;
		return -3;
	}
	return 0;
}

bool getVals (const char progname[], char* arguments[], const int size){
	if (size < 0) {
		cerr << "Error: size = 0?" << endl;
		return false;
	}
	if (arguments == 0) {
		cerr << "Error: arguments == null?" << endl;
		return false;
	}
	if (progname == 0) {
		cerr << "Error: progname == null?" << endl;
		return false;
	}


	//prepare pipes to communicate between child and parent
	int c2p[2];
	int p2c[2];
	if (pipe (c2p) < 0) {
		return false;
	}
	if (pipe (p2c) < 0) {
		return false;
	}

	//split into child and parent
	pid_t pID = fork ();
	if (pID < 0) {
		cerr << "Error: fork failed!" << endl;
		return false;
	} else if (pID == 0) {
		//child

		// pipe => stdin
		// this pipe allows parent to send inputs to child
		// apparently doing this also stops the child from taking input from the console
		if (dup2 (p2c[0], 0) < 0) {
			cerr << "Error with child pipes. (1)" << endl;
			return false;
		}
		// stdout => pipe,
		// this pipe allows child to send output to parent
		// apparently doing this also stops the child from outputting to the console
		if (dup2 (c2p[1], 1) < 0) {
			cerr << "Error with child pipes. (2)" << endl;
			return false;
		}
		if (close (c2p[0]) < 0 || close (c2p[1]) < 0 || close (p2c[0]) < 0 || close (p2c[1]) < 0) {
			cerr << "Error with child pipes. (3)" << endl;
			return false;
		}
		if (execl (progname, progname, NULL)) {
			cerr << "Error trying to execute calibration program." << endl;
			return false;
		}
	} else {
		//parent
		if (close (p2c[0]) < 0 || close (c2p[1]) < 0) {
			cerr << "Error with parent pipes." << endl;
			return false;
		}

		//we choose 3-axis by entering '3'
		if (write (p2c[1], "3\n", 2) < 0) {
			cerr << "Error sending input to select 3-axis mode." << endl;
			return false;
		}
		cout << "Choosing 3-axis option for calibration" << endl;


		//start sampling by pressing enter.
		cout << "Press [Enter] to start sampling." << endl;
		cin.ignore (numeric_limits<streamsize>::max (), '\n' );
		if (write (p2c[1], "\n", 1) < 0) {
			cerr << "Error sending input to start sampling." << endl;
			return false;
		}
		cout << "Starting sampling; drive your boat around in circles or something." << endl;
		cout << endl;
		sleep (3);
		//stop sampling by pressing enter.
		cout << "Press [Enter] to stop sampling." << endl;
		cin.ignore (numeric_limits<streamsize>::max (), '\n' );
		if (write (p2c[1], "\n", 1) < 0) {
			cerr << "Error sending input to stop sampling." << endl;
			return false;
		}
		cout << "Stopped sampling." << endl;


		//end program and get results
		if (write (p2c[1], "\n", 1) < 0) {
			cerr << "Error sending input to get results." << endl;
			return false;
		}

		//now we read everything.
		//has to read two times, first time will go up to Estimates,
		//second time will start with "Compass calibration finished ..." and the the variables
		char line[1001];
		for (int i = 0; i < 2; i++) {
			if (read (c2p[0], line, 1000) < 0) {
				cerr << "Error reading output message from calibration program." << endl;
				return false;
			}
			cout << line;
		}
		cout << "Finished running calibration program." << endl;
		if (close (p2c[1]) < 0 || close (c2p[0]) < 0) {
			cout << "Error closing pipes." << endl;
			return false;
		}

		if (!parseLine (line, arguments, size)) {
			return false;
		}

	}
	return true;

}


/* Sample input for parsing:

   Compass calibration finished
    Vars: 0.004434, -0.084885, 0.319036, 0.266376, 8.716262, 269.402878, 398.528180, -0.014981, -0.100216, -0.453510, 0.764028, -4.634166, 1.167096

   These arguments are in the right order to be passed direcly into the PhidgetSpatial_setCompassCorrectionParameters function.

   You may wish to use a more accurate value for magnetic field strength as the number provided here is only an estimate.

   Calibration values have been written to firmware on your spatial.
   These values will be maintained from now on, across power cycles, until explicitely reset or changed.

   Enter to exit
 */

bool parseLine (const char* line, char* arguments[], const int size){
	if (size < 0) {
		cerr << "Error: size == 0?" << endl;
		return false;
	}

	int lineIndex = 0;//index of character on line as we read through everything

	char curVal[32]; //accumulates characters for one value, before it get turns to float.
	int curIndex = 0; //current index for curVal;

	int count = 0; //number of values found
	bool startedReadingNumbers = false; //once we hit the list of values this should become true

	while (line[lineIndex] && lineIndex < 1001 && count < size) {
		switch (line[lineIndex]) {
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
		case '0':
		case '.':
		case '-':
			if (!startedReadingNumbers) {
				startedReadingNumbers = true;
			}
			curVal[curIndex] = line[lineIndex];
			curIndex++;
			break;
		case ' ':
		case ',':
		case '\n':
			if (startedReadingNumbers && curIndex > 0) {
				curVal[curIndex] = 0;
				//check if is number
				char *endptr;
				float val;
				val = strtof (curVal, &endptr);
				if (endptr == curVal) {
					cerr << "Error parsing values: " << curVal << " is not a number" << endl;
					return false;
				}
				//str copy to final array
				arguments[count] = new char[curIndex + 1];
				for (int j = 0; j < curIndex + 1; j++) {
					(arguments[count])[j] = curVal[j];
				}

				count++;
				curIndex = 0;
			}
			break;
		default:
			if (startedReadingNumbers) {
				cerr << "Error parsing values: encountered unexpected character " << curVal[curIndex] << "(from: " << curVal << ")" << endl;
				return false;
			}
			break;
		}
		lineIndex++;
	}
	if (count < size) {
		cerr << "Warning: Less than " << size << " values parsed." << endl;
	}
	return true;
}


bool readFile (const char filepath[], char* lines[], int& lineCount){
	if (filepath == 0) {
		cerr << "Could not read file: filepath = null?" << endl;
		return false;
	}

	const int maxLinesLength = 255;
	char line[maxLinesLength];

	cout << "Opening file at " << filepath << " (reading)" << endl;
	ifstream infile (filepath);
	if (!infile.is_open ()) {
		cerr << "Could not open file " << filepath << endl;
		return false;
	}

	bool done = false;
	int lineNum = 0; //let's do zero based for convenience
	while (!done) {
		//get line
		if (!infile.getline (line, maxLinesLength)) {
			if (infile.eof ()) {
				done = true;
			} else {
				false;
			}
		}

		//copy to array
		int i = 0;
		lines[lineNum] = new char[maxLinesLength];
		while (line[i] && i < maxLinesLength) {
			lines[lineNum][i] = line[i];
			i++;
		}
		lines[lineNum][i] = 0;
		lineNum++;
	}
	lineCount = lineNum;
	infile.close ();
	return true;
}

bool writeToFile (const char filepath[], const char* const lines[], const int lineCount, const char* const arguments[], const int size){
	if (size < 0) {
		cerr << "Could not save to file: size = 0?" << endl;
		return false;
	}
	if (arguments == 0) {
		cerr << "Could not save to file: arguments == null?" << endl;
		return false;
	}
	if (filepath == 0) {
		cerr << "Could not save to file: filepath == null?" << endl;
		return false;
	}

	cout << "Opening file at " << filepath << " (writing)" << endl;

	//open file
	ofstream outfile (filepath);
	if (!outfile.is_open ()) {
		cerr << "Failed: cannot open file!" << endl;
		return false;
	}

	int argumentsIndex = 0;
	//write each on one line
	for (int i = 0; i < lineCount; i++) {
		if (!strstr (lines[i], "<param name=\"cc_")) {
			//if not a compass calibration value, then leave it as is
			outfile << lines[i] << endl;
		} else {
			//if it is a compass calibration value, change the value
			if (argumentsIndex >= size) {
				cerr << "Error: more cc_ params in file than there are arguments for!" << endl;
				return false;
			}

			char changedLine[256];
			if (!replaceValueInLine (lines[i], changedLine, arguments[argumentsIndex])) {
				cerr << "Error replacing value in launch file on line " << (i + 1) << ": " << lines[i] << endl;
				return false;
			}
			outfile << changedLine << endl;
			argumentsIndex++;
		}
	}
	if (argumentsIndex < size - 1) {
		cerr << "Warning: less cc_ params in file than there are arguments for!" << endl;
	}
	outfile.close ();
	return true;
}


bool replaceValueInLine (const char line[], char changedLine[], const char newValue[]){
	if (line == 0 || changedLine == 0 || newValue == 0) {
		cerr << "Error: null values? line = " << line << ", changedLine = " << changedLine << ", newValue = " << newValue << endl;
	}
	int i = 0; //index for line
	int i2 = 0; //index for changedLine

	//first, copy over chars before the values we want to changed.
	int valueStrCounter = 0;//used to check if we found "value=\""
	while (line[i] && i < 256 && valueStrCounter < 7) {
		switch (line[i]) {
		case 'v':
			if (valueStrCounter == 0) {
				valueStrCounter++;
			} else {
				valueStrCounter = 0;
			}
			break;
		case 'a':
			if (valueStrCounter == 1) {
				valueStrCounter++;
			} else {
				valueStrCounter = 0;
			}
			break;
		case 'l':
			if (valueStrCounter == 2) {
				valueStrCounter++;
			} else {
				valueStrCounter = 0;
			}
			break;
		case 'u':
			if (valueStrCounter == 3) {
				valueStrCounter++;
			} else {
				valueStrCounter = 0;
			}
			break;
		case 'e':
			if (valueStrCounter == 4) {
				valueStrCounter++;
			} else {
				valueStrCounter = 0;
			}
			break;
		case '=':
			if (valueStrCounter == 5) {
				valueStrCounter++;
			} else {
				valueStrCounter = 0;
			}
			break;
		case '\"':
			if (valueStrCounter == 6) {
				valueStrCounter++;
			} else {
				valueStrCounter = 0;
			}
			break;
		case ' ':
		case '\t':
			break;
		default:
			valueStrCounter = 0;
		}

		changedLine[i2] = line[i];
		i++;
		i2++;
	}
	if (valueStrCounter < 7) {
		cerr << "Could not find the term: value\" in line" << endl;
		return false;
	}

	//second, write in the new values
	int i3 = 0; //index for newValue
	while (newValue[i3] && i3 < 32 && i2 < 256) {
		changedLine[i2] = newValue[i3];
		i2++;
		i3++;
	}

	//finally, copy over the rest from line, starting from the next "
	while (line[i] && line[i] != '\"' && i < 256) {
		i++;
	}
	while (line[i] && i < 256 && i2 < 256) {
		changedLine[i2] = line[i];
		i++;
		i2++;
	}
	changedLine[i2] = 0;
	cout << "old: " << line << endl;
	cout << "new: " << changedLine << endl;
	return true;
}


bool makeBackup (const char filepath[], const char backupFilepath[]){
	ifstream src (filepath, ios::binary);
	ofstream dest (backupFilepath,  ios::binary);
	if (!src.is_open () || !dest.is_open ()) {
		return false;
	}

	dest << src.rdbuf ();
	src.close ();
	dest.close ();
	return true;
}
