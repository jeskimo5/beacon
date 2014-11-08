#include <TinyGPS++.h>
#include <AltSoftSerial.h>
#include "C:\Users\Jesse\Documents\Arduino\libraries\mavlink\common\mavlink.h"




//static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 38400;

// The TinyGPS++ object
TinyGPSPlus gps;

//Mavlink Setup
byte gs_id = 250;

byte gs_state = MAV_STATE_BOOT;
mavlink_heartbeat_t mav_lastBeat;
bool hasGotBeat;

byte mav_id;
//Timers
long heartTimer;
long followMeTimer;
long gpsRetrieveTimer;
long gpsSendTimer;
// The serial connection to the GPS device
AltSoftSerial ss;
uint16_t oldSec = 0;



void setup()
{
	//Start Timers
	heartTimer = millis();
	followMeTimer = millis();
	gpsRetrieveTimer = millis();
	gpsSendTimer = millis();
	hasGotBeat = false;
	Serial.begin(57600);
	ss.begin(GPSBaud);
	//send our heartbeat
	sendMavlinkHeartBeat();
	//get initial response heartbeat
	//comm_receive();
	//wait for a quality gps signal

	//

	gs_state = MAV_STATE_ACTIVE;
}

void loop()
{


	// This sketch displays information every time a new sentence is correctly encoded.
	
	checkGPS();
	sendMavlinkHeartBeat();
	/*if (millis() > 5000 && gps.charsProcessed() < 10)
	{
	Serial.println(F("No GPS detected: check wiring."));
	while (true);
	}*/
}
void checkGPS(){
	if (500 <= millis() - gpsRetrieveTimer){
		while (ss.available() > 0){
			if (gps.encode(ss.read())){
					displayInfo();
					gpsRetrieveTimer = millis();
			}
		}
		
	}
}
void sendMavlinkFollowMe(){
	

	//// put your main code here, to run repeatedly:
	//// Define the system type (see mavlink_types.h for list of possible types)
	//int system_type = MAV_TYPE_GCS;
	//int autopilot_type = MAV_AUTOPILOT_GENERIC;

	//// Initialize the required buffers
	//mavlink_message_t msg;
	//uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	//// Pack the message
	//mavlink_msg_heartbeat_pack(gs_id, 200, &msg, system_type, autopilot_type, MAV_MODE_FLAG_AUTO_ENABLED, (byte)0, gs_state);

	////mavlink_msg_mission_set_current_pack()

	//// Copy the message to send buffer
	//uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

	//// Send the message (.write sends as bytes)
	//Serial.write(buf, len);

}


void sendMavlinkHeartBeat(){
	//send it once per second
	if (1000 <= millis() - heartTimer)
	{
	
	// put your main code here, to run repeatedly:
	// Define the system type (see mavlink_types.h for list of possible types)
	int system_type = MAV_TYPE_GCS;
	int autopilot_type = MAV_AUTOPILOT_INVALID;

	// Initialize the required buffers
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];


	// Pack the message
	mavlink_msg_heartbeat_pack(gs_id, 200, &msg, system_type, autopilot_type,0,0, gs_state);

	// Copy the message to send buffer
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

	// Send the message (.write sends as bytes)
	Serial.write(buf, len);
	//Serial.println("here you go");
	//Serial.write("hi", len);
	heartTimer = millis();
	}
}
void comm_receive() {
	mavlink_message_t msg;
	mavlink_status_t status;

	//receive data over serial 
	while (Serial.available() > 0) {
		uint16_t c = Serial.read();

		//try to get a new message 
		if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			// Handle message
			switch (msg.msgid) {

			case MAVLINK_MSG_ID_SET_MODE: {
											  // set mode
			}
				break;
			case MAVLINK_MSG_ID_HEARTBEAT: {
											   //get the information about our copter
											  // mavlink_msg_heartbeat_decode(&msg, &he);

			}
				break;


			default:
				//Do nothing
				break;
			}
		}
		// And get the next one
	}
}
void displayInfo()
{
	Serial.print(F("Location: "));
	if (gps.location.isValid())
	{
		Serial.print(gps.location.lat(), 6);
		Serial.print(F(","));
		Serial.print(gps.location.lng(), 6);
		mavlink_msg_location
	}
	else
	{
		Serial.print(F("INVALID"));
	}

	Serial.print(F("  Date/Time: "));
	if (gps.date.isValid())
	{
		Serial.print(gps.date.month());
		Serial.print(F("/"));
		Serial.print(gps.date.day());
		Serial.print(F("/"));
		Serial.print(gps.date.year());
	}
	else
	{
		Serial.print(F("INVALID"));
	}

	Serial.print(F(" "));
	if (gps.time.isValid())
	{
		if (gps.time.hour() < 10) Serial.print(F("0"));
		Serial.print(gps.time.hour());
		Serial.print(F(":"));
		if (gps.time.minute() < 10) Serial.print(F("0"));
		Serial.print(gps.time.minute());
		Serial.print(F(":"));
		if (gps.time.second() < 10) Serial.print(F("0"));
		Serial.print(gps.time.second());
		Serial.print(F("."));
		if (gps.time.centisecond() < 10) Serial.print(F("0"));
		Serial.print(gps.time.centisecond());
	}
	else
	{
		Serial.print(F("INVALID"));
	}
	Serial.print(" ");
}
