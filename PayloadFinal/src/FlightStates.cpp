#include "FlightStates.h"

flightState currentFS = STANDBY;

double currentTime, transitionTime;
double minTimes[] = {10.0, 30.0};

void States::unarmed() {
	currentFS = UNARMED;
	//Hard lock on doing nothing - transmits sensor stuff and that's it
}

void States::standby(double altitude, double initialAltitude, double velocity) {
	currentFS = STANDBY;
	//Perform PreLaunch Operations

	if (altitude - initialAltitude >= 100 && velocity >= 10) {
		currentFS = ASCENT;
		transitionTime = millis()/1000;
	}
}

void States::ascent(double altitude, double initialAltitude, double velocity) {
	currentFS = ASCENT;
	//Perform Ascent Operations

	currentTime = millis()/1000;
	if ((currentTime - transitionTime) >= minTimes[0] && (altitude - initialAltitude) >= 1000 && fabs(velocity) <= 5) {
		currentFS = DESCENT;
		transitionTime = millis()/1000;
	}
	
}

void States::descent(double altitude, double initialAltitude, double velocity, double accelx, double accely, double accelz, double distance) {
	currentFS = DESCENT;
	//Perform Descent Operations

	currentTime = millis()/1000;
	if ((currentTime - transitionTime) >= minTimes[1] && altitude - initialAltitude < 100) {
		if (distance < 8.0) {
			actuateServo(false);
			delay(500);
		}
	}
	if (fabs(velocity) <= 5 && (accelx + accely + accelz) < 10.0 && altitude < 50) {
		landedOrientx = orientx;
		landedOrienty = orienty;
		landedOrientz = orientz;
		currentFS = LEVELLING;
	}
}

void States::levelling(double radialOrient, double tangentialOrient) {
    currentFS = LEVELLING;
	//Perform Levelling Operations
	double resultCurrent = sqrt(pow((radialOrient), 2) + pow(tangentialOrient, 2)); // Resultant vector REMEMBER TO ADD BACK 90 TO RADIAL FOR SLED CONFIGURATION
	
	if (resultCurrent >= 1.0) {
		calibrateLeveler(radialOrient, tangentialOrient);

		if (oriented1 != 0 && oriented2 != 0 && oriented3 != 0) {
			if (oriented1 == 1) {
				driveMotor(1, 1);
			}
			else if (oriented1 == 2) {
				driveMotor(1, 2);
			}
			if (oriented2 == 1) {
				driveMotor(2, 1);
			}
			else if (oriented2 == 2) {
				driveMotor(2, 2);
			}
			if (oriented3 == 1) {
				driveMotor(3, 1);
			}
			else if (oriented3 == 2) {
				driveMotor(3, 2);
			}
			if (hasChanged(resultCurrent, resultPrevious) != 1) {
				resetCalibration();
			}
		}
	}
	else {	// If Levelled:
		driveMotor(1, 0);
		driveMotor(2, 0);
		driveMotor(3, 0);

		leveledx = orientx;
		leveledy = orienty;
		leveledz = orientz;

		String packet = "";
		packet += "LEVELLING RESULTS";
		packet += ",";
		packet += landedOrientx;
		packet += ",";
		packet += landedOrienty;
		packet += ",";
		packet += landedOrientz;
		packet += ",";
		packet += leveledx;
		packet += ",";
		packet += leveledy;
		packet += ",";
		packet += leveledz;
		packet += "LEVELLING RESULTS END";
				
		File dataFile = SD.open("datalog.txt", FILE_WRITE);
		if (dataFile) {
			dataFile.println(packet);
			dataFile.close();
		}
		else {
			Serial.println("Could not open datalog.txt");
		}
		if(CAM1_EXIST) {myCAMSaveToSDFile(myCAM1, "cam1");}
		if(CAM2_EXIST) {myCAMSaveToSDFile(myCAM2, "cam2");}
		if(CAM3_EXIST) {myCAMSaveToSDFile(myCAM3, "cam3");}
		delay(1000);
		currentFS = FINISHED;
	}	
}

void States::finished() {
	currentFS = FINISHED;
	//Decrease transmission rate
	//Run until powered off
}

void States::actuateServo(bool locked) {
	Servo release1;
	Servo release2;
	if (locked) {
		release1.detach();
		release2.detach();
		digitalWrite(RELEASE_POWER1, LOW);
		digitalWrite(RELEASE_POWER2, LOW);	
	}
	else {
		digitalWrite(RELEASE_POWER1, HIGH);
		digitalWrite(RELEASE_POWER2, HIGH);
		release1.attach(RELEASE1);
		release2.attach(RELEASE2);
		release1.write(160);
		release2.write(160);
		delay(100);
		release1.detach();
		release2.detach();
		digitalWrite(RELEASE_POWER1, LOW);
		digitalWrite(RELEASE_POWER2, LOW);
	}
}

void States::myCAMSaveToSDFile(ArduCAM myCAM, char str[8]) {
	byte buf[256];
	static int i = 0;
	static int k = 0;
	uint8_t temp = 0,temp_last=0;
	uint32_t length = 0;
	bool is_header = false;
	File outFile;
	//Flush the FIFO
	myCAM.flush_fifo();
	//Clear the capture done flag
	myCAM.clear_fifo_flag();
	//Start capture
	myCAM.start_capture();
	Serial.println(F("start Capture"));
	double camStartTime = millis()/1000;
	double camNowTime;
	while(!myCAM.get_bit(ARDUCHIP_TRIG , CAP_DONE_MASK)) {
		camNowTime = millis()/1000;
		if (camNowTime - camStartTime >= 10000) {
			return;
		}
	}
	Serial.println(F("Capture Done."));  
	length = myCAM.read_fifo_length();
	Serial.print(F("The fifo length is :"));
	Serial.println(length, DEC);
	if (length >= MAX_FIFO_SIZE) { //8M
		Serial.println(F("Over size."));
		return;
	}
	if (length == 0 ) { //0 kb
		Serial.println(F("Size is 0."));
		return;
	}
	//Construct a file name
	strcat(str, ".jpg");
	//Open the new file
	outFile = SD.open(str, O_WRITE | O_CREAT | O_TRUNC);
	if(!outFile){
		Serial.println(F("File open failed"));
		return;
	}
	myCAM.CS_LOW();
	myCAM.set_fifo_burst();
	while ( length--) {
		temp_last = temp;
		temp =  SPI.transfer(0x00);
		//Read JPEG data from FIFO
		if ((temp == 0xD9) && (temp_last == 0xFF)) { //If find the end ,break while,
			buf[i++] = temp;  //save the last  0XD9     
			//Write the remain bytes in the buffer
			myCAM.CS_HIGH();
			outFile.write(buf, i);    
			//Close the file
			outFile.close();
			Serial.println(F("Image save OK."));
			is_header = false;
			i = 0;
		}  
		if (is_header == true) { 
			//Write image data to buffer if not full
			if (i < 256)
			buf[i++] = temp;
			else {
			//Write 256 bytes image data to file
			myCAM.CS_HIGH();
			outFile.write(buf, 256);
			i = 0;
			buf[i++] = temp;
			myCAM.CS_LOW();
			myCAM.set_fifo_burst();
			}        
		}
		else if ((temp == 0xD8) & (temp_last == 0xFF)) {
			is_header = true;
			buf[i++] = temp_last;
			buf[i++] = temp;   
		} 
	} 
}

void States::setCurrentState(uint8_t stateID) {
	if (stateID == 0) {
		this->currentState = UNARMED;
	} 
	else if (stateID == 1) {
		this->currentState = STANDBY;
	}
	else if (stateID == 2) {
		this->currentState = ASCENT;
	}
	else if (stateID == 3) {
		this->currentState = DESCENT;
	}
	else if (stateID == 4) {
		this->currentState = LEVELLING;
	}
	else if (stateID == 5) {
		this->currentState = FINISHED;
	}
}

/*void States::whichState(flightState newState) {
	switch (newState) {
	case UNARMED:
		//Just chillin'.
		cout << "UNARMED" << endl;
		unarmed();
		break;
	case STANDBY:
		//I'M READY!!!
		cout << "STANDBY" << endl;
		standby(smoothAltitude, initialAltitude);
		break;
	case ASCENT:
		//Here we go!
		cout << "ASCENT" << endl;
		ascent(velocity);
		break;
	case DESCENT:
		//Down, down, down...
		cout << "DESCENT" << endl;
		descent(velocity, gForce);
		break;
	case LANDING:
		//Getting kinda lonely, someone come find me.
		cout << "LANDING" << endl;
		landing();
		break;
	}
}*/
