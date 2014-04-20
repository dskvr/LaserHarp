//TODO// Objectify those strings, yeah. With absolute precision.
#include <Streaming.h>
#include "SPI.h"
#include "Adafruit_WS2801.h"
/*++++++++++++++++++++++++++++++++++++++++++++++++++
Constraints
//*************** LASER SWITCH PINS ***************/
#define DEBUG										false
#define DEBUG_STATS							false
#define NUMBER_STRINGS  				7
#define NUMBER_PHOTOCELLS  			7
#define NUMBER_RANGE_FINDERS		2
/*++++++++++++++++++++++++++++++++++++++++++++++++++
PINS PINS PINS
//*************** LASER SWITCH PINS ***************/
#define PINS_LASER_ONE 					28
#define PINS_LASER_TWO 					30	
#define PINS_LASER_THREE 				32
#define PINS_LASER_FOUR 				28
#define PINS_LASER_FIVE 				30
#define PINS_LASER_SIX	 				32
#define PINS_LASER_SEVEN				28
//*************** STRING RANGE FINDERS ***************//
#define PINS_RANGE_TRIG_ONE 		23 // Trigger Pin
#define PINS_RANGE_TRIG_TWO 		25 // Trigger Pin
#define PINS_RANGE_ECHO_ONE 		22
#define PINS_RANGE_ECHO_TWO 		24
//*************** USERRANGE TRIG & ECHO ***************//
#define PINS_USER_RANGE_TRIG 		27
#define PINS_USER_RANGE_ECHO		26
//*************** PHOTOCELLS ***************//
#define PINS_PC_ONE 						8
#define PINS_PC_TWO 						9 
#define PINS_PC_THREE 					10
#define PINS_PC_FOUR 						8
#define PINS_PC_FIVE 						9 
#define PINS_PC_SIX		 					10
#define PINS_PC_SEVEN		 				10
#define PINS_PC_AMBIENT		 			10
//*************** KNOB PINS ***************//
#define PINS_KNOB_OCTAVE 				0
#define PINS_KNOB_NOTE					1
#define PINS_KNOB_SCALE  				2
//*************** LIGHT PINS ***************//
#define PINS_LIGHTS_DATA 				0
#define PINS_LIGHTS_CLOCK				1
//*************** MODE BUTTON PIN ***************//
#define PINS_MODE 							11
/*++++++++++++++++++++++++++++++++++++++++++++++++++
MIDI
//*************** LIMITS ***************/
#define MIDI_NOTE_LOW 					1
#define MIDI_NOTE_HIGH 					115
//*************** MIDI VALUES ***************/
#define OFF 										0
#define ON 											1
#define WAIT 										2
#define CONTINUOUS 							11
#define NOTEON 									0x90
#define NOTEOFF 								0x80
/*++++++++++++++++++++++++++++++++++++++++++++++++++
RANGE
//*************** LIMITS ***************/
#define THRESHOLD_PING_RANGE	50
/*++++++++++++++++++++++++++++++++++++++++++++++++++
INTERACTIVITY
//*************** LIMITS ***************/
#define IDLE_THRESHOLD 				30000
#define RANGE_MAX 						114 //CHANGE!
#define RANGE_MIN 						1	  //CHANGE!
#define LASER_PC_THRESH 			600 //CHANGE!
#define TOTAL_MODES 					3
#define DEFAULT_BASE_NOTE 		60
#define DEFAULT_SCALE					0
//*************** DATA CACHE ***************//
int stringRange[NUMBER_STRINGS];
int stringNote[NUMBER_STRINGS];
int PCValues[NUMBER_STRINGS];
int rangeRawValue[NUMBER_STRINGS];
int rangeMidiValue[NUMBER_STRINGS];
int midiStage[NUMBER_STRINGS];
boolean stringTriggered[NUMBER_STRINGS];
boolean holding[NUMBER_STRINGS];

int knobRawValue[3];
//*************** User Tracking ***************//
long lastInteraction;
int lastButtonState;
boolean isIdle;
boolean active = false;
boolean booted = false;
//*************** Stepper for range tweens ***************//
boolean stepping[NUMBER_STRINGS];
//*************** Midi Variables ***************//
int action = ON; //1 =note off ; 2=note on ; 3= wait
byte incomingByte;
byte note;
byte velocity;
int pot;
//*************** Pins ***************//
uint8_t pinsPC[7];
uint8_t pinsKnob[3];
//*************** Music Math  ***************//
int baseNote = DEFAULT_BASE_NOTE;
int harpScale = DEFAULT_SCALE;
//*************** Ambient Light Sensor (532nm sensitive)  ***************//
int PCThreshAdjusted, subtractAmbience;
//*************** Switch Modes!  ***************//
int mode = 0; // 0 = momentary, 2 = on/off (sampler)
//*************** Debugging  ***************//
long debugLast = millis();
int debugEvery = 7*1000;

void setup() {
	setupStepper();
	setupPhotocells();
	setupRange();
	setScale();
	setupMidi();
  Serial.begin(31250);  //start serial with midi baudrate 31250
	setupKnobs();
	interrupts();	
	// setupLasers();	
}

void update(){}

void loop() {		
	if(booted==false) { startupLasers(); booted==true; }
	pollKnobs();   //Check for octave, key, scale, mode values
	pollSensors(); //Check photocells and find range.
	sendMidi();    //Send midi signal. 
	if(DEBUG) debugReport();
}
/**************************************
**
LASERS
with Sharks.
**
***************************************/
int totalLasers = NUMBER_STRINGS;
long cooldownLast = millis();
int cooldownEvery = 60*10;
int cooldownPeriod = 60;
long startupLast = millis();
boolean coolingDown = false;

boolean laserState[NUMBER_STRINGS];

int lasers[NUMBER_STRINGS]; 
		
void startupLasers(){
	lasers[0] = PINS_LASER_ONE;
	lasers[1] = PINS_LASER_TWO;
	lasers[2] = PINS_LASER_THREE;
	lasers[3] = PINS_LASER_FOUR;
	lasers[4] = PINS_LASER_FIVE;
	lasers[5] = PINS_LASER_SIX;
	lasers[6] = PINS_LASER_SEVEN;
	
	for(int l = 0;l < totalLasers; l++) {
		pinMode(lasers[l], OUTPUT);
	}
	
	lasersOn(500);
}

void shutdownLasers(){
	
}

//*************** LASERS: TURN A LASER ON ***************//
void laserOn(int laser){
	if(DEBUG) LOG(1, "(ON) Laser", laser);
	digitalWrite(lasers[laser], HIGH);
	laserState[laser] = true;
}
//*************** LASERS: TURN A LASER OFF  ***************//
void laserOff(int laser){
	if(DEBUG) LOG(1, "(OFF) Laser", laser);
	digitalWrite(lasers[laser], LOW);
	laserState[laser] = false;
}
//*************** LASERS: TURN ALL ON ***************//
void lasersOn(){
	for(int l = 0; l < totalLasers; l++) {
		laserOn(l);
	}
	lasersCooldownSet("startupLast", millis());
}
//*************** LASERS: TURN ALL ON WITH DELAY ***************//
void lasersOn(int d){
	for(int l = 0; l < totalLasers; l++) {
		laserOn(l);
		delay(d);
	}
	lasersCooldownSet("startupLast", millis());
}
//*************** LASERS: TURN ALL OFF ***************//
void lasersOff(){
	for(int l = 0; l < totalLasers; l++) {
		laserOff(l); 
	}
	lasersCooldownSet("cooldownLast", millis());
}

//*************** LASERS: TURN ALL OFF WITH DELAY ***************//
void lasersOff(int d){
	for(int l = 0; l < totalLasers; l++) {
		laserOff(l);
		delay(d);
	}
	lasersCooldownSet("cooldownLast", millis());
}

//*************** CHECK LASER COOLDOWN CYCLE  ***************//
void lasersCheckCooldown(){
	long now = millis();
	if(now-startupLast > cooldownEvery && !coolingDown) {
		lasersCooldown();
	} else if(now-cooldownLast > cooldownPeriod && coolingDown) {
		lasersCooled();		
	} else {
		//do nothing
	}
}

void lasersCooldownSet(const char* type, long millis) {
	if(type == "cooldownLast") 	cooldownLast = millis; 
	if(type == "startupLast") 	startupLast = millis; 
}

void lasersCooldown(){
	//blink 3 times then cooldown
	for(int blink=0;blink<3;blink++){
		lasersOff( 20 );  delay(250);
		lasersOn( 10 );   delay(500);
	}
	if(DEBUG) LOG("Cooling Down", "Lasers", 7);
	lasersOff(500); delay(400);
	cooldownLast = millis();
} 

void lasersCooled(){
	lasersOn( 250 );
	if(DEBUG) LOG("Cooling Finished", "Lasers", 7);
}

/**************************************
**
Photocells
The tiny big section for most desired feature
**
***************************************/
void setupPhotocells(){
	pinsPC[0] = PINS_PC_ONE;
	pinsPC[1] = PINS_PC_TWO;
	pinsPC[2] = PINS_PC_THREE;
	pinsPC[3] = PINS_PC_FOUR;
	pinsPC[4] = PINS_PC_FIVE;
	pinsPC[5] = PINS_PC_SIX;
	pinsPC[6] = PINS_PC_SEVEN;
}

/**************************************
**
RANGE FINDER
The tiny big section for most desired feature
**
***************************************/

//*************** RANGE FINDERS: SETUP PINS ***************//

// SIMPLIFY!, memory hog...
uint8_t pinsRangeTrigger[NUMBER_STRINGS];
uint8_t pinsRangeEcho[NUMBER_STRINGS];

//*************** RANGE FINDERS: SETUP PINS FOR INPUT AND OUTPUT ***************//
long rangeMillisSince[NUMBER_STRINGS], rangeMillisLast[NUMBER_STRINGS];

/* This setups up the range finders */
void setupRange(){
	for(int s=0;s<NUMBER_STRINGS;s++){
		if(s<4) {
			pinsRangeTrigger[s] = uint8_t(PINS_RANGE_TRIG_ONE);
			pinsRangeEcho[s] = uint8_t(PINS_RANGE_ECHO_ONE);
		} else {
			pinsRangeTrigger[s] = uint8_t(PINS_RANGE_TRIG_TWO);
			pinsRangeEcho[s] = uint8_t(PINS_RANGE_ECHO_TWO);
		}
		pinMode(pinsRangeTrigger[s], OUTPUT);
		pinMode(pinsRangeEcho[s], INPUT);
		rangeMillisLast[s] = millis();
	}
}

/* This pings the range finders, which sends an ultra sonic wave, pauses and then recieves it microseconds later */
void pingRange(int s){
	rangeMillisSince[s] = millis() - rangeMillisLast[s];
	if(rangeMillisSince[s] > THRESHOLD_PING_RANGE) {
		if(DEBUG) LOG(pinsRangeTrigger[s], "(PING) Range for String/Laser ", s);
		digitalWrite(pinsRangeTrigger[s], LOW); 
	 	delayMicroseconds(2); 
	 	digitalWrite(pinsRangeTrigger[s], HIGH);
	 	delayMicroseconds(10); 
	 	digitalWrite(pinsRangeTrigger[s], LOW);
	 	long duration = pulseIn(pinsRangeEcho[s], HIGH);
	  rangeRawValues[s] = microsecondsToCentimeters(duration);
		if(DEBUG) LOG(pinsRangeEcho[s], "(PING) Range for String/Laser ", s);
		rangeMillisLast[s] = millis();
	}
	rangeMidiValues[s] = convertDistanceToMidi(rangeRawValues[s]);
}

int getStringRange(int s){
	return limitRange(rangeRawValue[s]);
}

int convertDistanceToMidi(int distance){
	return NORMALIZE(distance, 1, RANGE_MAX, MIDI_NOTE_LOW, MIDI_NOTE_HIGH);
}

int limitRange(int range){
	return (range > RANGE_MAX) ? RANGE_MAX : range;
}

/**************************************
**

SAFETY
Without them the lasers are just pretty (and debateably dangerous)

**
***************************************/
int stringTrigDur[NUMBER_STRINGS];
int stringActivatedTime[NUMBER_STRINGS];
long safetyOn[NUMBER_STRINGS];
int stringOn;

#define SAFETY_LASER_DURATION  10000

void safetySecond(int s, boolean reset){
	
	long now = millis();
	
	if(stringTriggered[s] && laserState[s]) {
		stringTrigDur[s] = now-stringActivatedTime[s];
	}	else if(!stringTriggered[s]) {
		stringTrigDur[s] = 0;
	}
	
	if(safetyOn[s]) {
		if(now-safetyOn[s]>1*1000) { //Put it on after 1 second
			if(now-safetyOn[s]>1*1000+5 && stringTriggered[s] && laserState[s]) laserOff(s); //Shortly after, check if there is something still obstructing the laser.
			else laserOn(s); //First... turn the laser on
			
			if(now-safetyOn[s]>5*1000+100 && !stringTriggered[s] && laserState[s]){ //If the obstructure is no longer there, it continues as normal. 
				laserOn(s);
				safetyOn[s]=0;
			}
			//Otherwise, it will try it again and again and again until the obstruction has been removed. 
		}
	}
	
	if(stringTrigDur[s] > SAFETY_LASER_DURATION) {  //Something has blocked the laser for 10 seconds, turn on the Safety
		laserOff(s); 
		safetyOn[s] = now;
	}
}


/**************************************
**

MODE
Without them the lasers are just pretty (and debateably dangerous)

**
***************************************/

void setupMode(){
	pinMode(PINS_MODE, INPUT);
}

void pollMode(){
	int buttonState = digitalRead(PINS_MODE);
	// compare the buttonState to its previous state
	  if (buttonState != lastButtonState) {
	    // if the state has changed, increment the counter
	    if (buttonState == HIGH) {
				// setMode();
				//DO!
	    } 
			//Ohterwise do nothing.
	  }
	  // save the current state as the last state, 
	  //for next time through the loop
	  lastButtonState = buttonState;
}

//*************** Midi Config ***************//

void Midi_Send(byte cmd, byte data1, byte data2) {
  Serial.write(cmd);
  Serial.write(data1);
  Serial.write(data2);
	if(DEBUG) {
		LOG(cmd, "Sending MIDI cmd");
		LOG(data1, "Sending MIDI data1");
		LOG(data2, "Sending MIDI data2");
	}
}

void sendMidi(){
	switch( mode ){
		case 1: //toggle
			for(int s = 0; s < NUMBER_STRINGS; s++){
				if(stringTriggered[s]) {
					if(DEBUG) LOG(s, "String Triggered (Toggle Mode)");
					if(!midiStage[s]) { //is presently off
						Midi_Send(NOTEON, stringNote[s], rangeMidiValue[s]); 
						Midi_Send(0xB0, 22+s, stringRange[s]); 
						midiStage[s] = 1; //on
						stringTriggered[s] = false;
						if(DEBUG) LOG(stringNote[s], "(toggle) Sending Note ON", s);
					} else { //is presently on
						Midi_Send(NOTEOFF, stringNote[s], rangeMidiValue[s]);
						midiStage[s] = 0; //off
						stringTriggered[s] = false;
						if(DEBUG) LOG(stringNote[s], "(toggle) Sending Note OFF", s);
					}
				} else {
					midiStage[s] = 2; //wait
					if(DEBUG) LOG(stringNote[s], "(toggle) Waiting...", s);
				}
			}
			break;
		default: //momentary
			for(int s = 0; s < NUMBER_STRINGS; s++){
				if(stringTriggered[s] && midiStage[s]==0) {
					Midi_Send(NOTEON, stringNote[s], rangeMidiValue[s]);
					Midi_Send(0xB0, 22+s, rangeMidiValue[s]);
					midiStage[s] = ON;
					stringTriggered[s] = false;
					if(DEBUG) LOG(stringNote[s], "(Momentary) Sending Note ON", s);
				} else if( !stringTriggered[s] && midiStage[s]) { //is not trigged but current stage is ON or WAIT. 
					Midi_Send(NOTEOFF, stringNote[s], rangeMidiValue[s]);
					midiStage[s] = OFF;
					if(DEBUG) LOG(stringNote[s], "(Momentary) Sending Note OFF", s);
				} else if(stringTriggered[s] && midiStage[s] == ON) {
					midiStage[s] = WAIT;
					Midi_Send(0xB0, 22+s, rangeMidiValue[s]);
					if(DEBUG) LOG(stringNote[s], "(Momentary) Waiting...", s);
				}
			}
	}
}

void setupMidi(){
	for(int s=0;s>NUMBER_STRINGS;s++){ stringTriggered[s] = false; }
}

void midiLoopback(){ }

long lastPoll = millis();
		
void pollSensors(){
	if(DEBUG) LOG(millis()-lastPoll, "(Polling Sensors) Cycling every");
	pollAmbientLight();
	for(int s=0;s<NUMBER_STRINGS;s++) {
		if( pollString(s) ) { //does a bunch of math, sets some stuff and returns true/false for each string.
			pingRange(s); //This will create latency, so it's only running every 50 milliseconds. 38 microseconds * 7 is a fraction of a Milli, but still.
			setStringRange(s);
			safetySecond(s, false);
			// if(stringTriggered[s]) 
		} else {
			safetySecond(s, true);
		}
	}
	lastPoll = millis();
	//
}

boolean pollString(int s){
	boolean triggered;
	long now = millis();
	PCValues[s] = analogRead(pinsPC[s]);
	PCThreshAdjusted = LASER_PC_THRESH-subtractAmbience; //subtract ambience from Threshold (which is taken in complete darkness)
	stringTriggered[s] = ( PCValues[s] < PCThreshAdjusted );
	if(!stringTriggered[s]) 	{ stringActivatedTime[s] = 0; stepStop(s); } //It's not triggered.
	else if(!stringActivatedTime[s] && stringTriggered[s]){ //It's just been activated, for the very first time (as long as we know.)
		stringActivatedTime[s] = now; 
		lastInteraction = now;
	}
	return stringTriggered[s];
}

int knobValue[3];
int knobRawValueCache[3];
int knobPollEvery = 100;
int knobPollLast = millis();



void setupKnobs(){
	pinMode(PINS_KNOB_OCTAVE, OUTPUT);
	pinMode(PINS_KNOB_NOTE, OUTPUT);
	pinMode(PINS_KNOB_SCALE, OUTPUT);
	setupKnobLights();
}

void pollKnobs(){
	long now = millis();
	if(now-knobPollLast>knobPollEvery) {
		boolean tripped = false;
		
		knobRawValue[0] = analogRead(PINS_KNOB_OCTAVE);
		knobRawValue[1] = analogRead(PINS_KNOB_NOTE);
		knobRawValue[2] = analogRead(PINS_KNOB_SCALE);
		
		int n = (knobRawValue[0] != knobRawValueCache[0]);
		int o = (knobRawValue[1] != knobRawValueCache[1]);
		int s = (knobRawValue[2] != knobRawValueCache[2]);
		
		tripped = (n || o || s);
		
		if(tripped) {		
			if(DEBUG) LOG(3, "(Knobs) Value has changed");
			int note = (n) ? NORMALIZE(knobRawValue[0], 0, 1024, 1, 12) : knobValue[0];
			int octave = (o) ? NORMALIZE(knobRawValue[1], 0, 1024, 0, 7) : knobValue[1];
			int scale = (s) ? NORMALIZE(knobRawValue[2], 0, 1024, 0, 12) : knobValue[2];

			setBaseNote(octave*12+note);
			setHarpScale(scale);
	
			knobValue[0] = note; 
			knobValue[1] = octave; 
			knobValue[2] = scale;
			
			refreshLights();
	
			memcpy(knobRawValueCache, knobRawValue, 3);
		}
	}
}

Adafruit_WS2801 knobLights = Adafruit_WS2801(3, PINS_LIGHTS_DATA, PINS_LIGHTS_CLOCK);

void setupKnobLights(){
	knobLights.begin();
	knobLights.show();
}

void refreshLights(){ //don't need note...

	//note
	int nR = NORMALIZE(baseNote, MIDI_NOTE_LOW, MIDI_NOTE_HIGH, 0, 255); 
	int nB = NORMALIZE(baseNote, MIDI_NOTE_LOW, MIDI_NOTE_HIGH, 255, 0); 
	uint32_t nRGB = color(nR, 0, nB); //From Blue-Red, entire scale. Changes color when octave changes color.
	
	//octave
	int oR = NORMALIZE(knobValue[1], 0, 7, 0, 255);
	int oB = NORMALIZE(knobValue[1], 0, 7, 255, 0); 
	uint32_t oRGB = color(oR, 0, oB); //From Blue-Red
	
	//scale
	int sR = NORMALIZE(knobValue[2], 0, 12, 50, 10);
	int sB = NORMALIZE(knobValue[2], 0, 12, 120, 40);
	int sG = NORMALIZE(knobValue[2], 0, 12, 100, 255);
	uint32_t sRGB = color(sR, sG, sB); //From Don't know. to who knows.
	
	//Happy Pixels
	knobLights.setPixelColor(0, nRGB);
	knobLights.setPixelColor(1, oRGB);
	knobLights.setPixelColor(2, sRGB);
	knobLights.show();
	
}

uint32_t color(byte r, byte g, byte b)
{
  uint32_t c;
  c = r;
  c <<= 8;
  c |= g;
  c <<= 8;
  c |= b;
  return c;
}

//*********************

// Chords

// **** **** **********

void setBaseNote(int n){
 if(DEBUG) LOG(n, "(Notes) Setting Base Note");
 baseNote = n;
 if(baseNote > MIDI_NOTE_HIGH) { baseNote = MIDI_NOTE_HIGH; }
 if(baseNote < MIDI_NOTE_LOW) { baseNote = MIDI_NOTE_LOW; }\
 if(DEBUG) LOG(baseNote, "(Notes) Base note set");
}

void setHarpScale(char scale){
	harpScale = scale;
	setScale();
	if(DEBUG) LOG(scale, "(Notes) Scale has been set.");
}

void setScale(){	
	if(DEBUG) LOG("harpscale", "(Notes) Setting scale.");
	switch(harpScale){
		
		case 0: //W – W – H – W – W – W – H //ionian/major
			stringNote[0] = baseNote;
			stringNote[1] = stringNote[0] + 2;
			stringNote[2] = stringNote[1] + 2;
			stringNote[3] = stringNote[2] + 1;
			stringNote[4] = stringNote[3] + 2;
			stringNote[5] = stringNote[4] + 2;
			stringNote[6] = stringNote[5] + 2;
		break;
		case 1: //w h w w h w w //MINOR Aeolian
			stringNote[0] = baseNote;
			stringNote[1] = stringNote[0] + 2;
			stringNote[2] = stringNote[1] + 1;
			stringNote[3] = stringNote[2] + 2;
			stringNote[4] = stringNote[3] + 2;
			stringNote[5] = stringNote[4] + 1;
			stringNote[6] = stringNote[5] + 2;
		break;
		case 2: //harmonic minor W - H - W - W - H - W+H - H 
			stringNote[0] = baseNote;
			stringNote[1] = stringNote[0] + 2;
			stringNote[2] = stringNote[1] + 1;
			stringNote[3] = stringNote[2] + 2;
			stringNote[4] = stringNote[3] + 2;
			stringNote[5] = stringNote[4] + 1;
			stringNote[6] = stringNote[5] + 3;
		case 3: //melodic minor W - H - W - W - W - W - H
			stringNote[0] = baseNote;
			stringNote[1] = stringNote[0] + 2;
			stringNote[2] = stringNote[1] + 1;
			stringNote[3] = stringNote[2] + 2;
			stringNote[4] = stringNote[3] + 2;
			stringNote[5] = stringNote[4] + 2;
			stringNote[6] = stringNote[5] + 2;
		case 4: //w h w w w h w - dorian
			stringNote[0] = baseNote;
			stringNote[1] = stringNote[0] + 2;
			stringNote[2] = stringNote[1] + 1;
			stringNote[3] = stringNote[2] + 2;
			stringNote[4] = stringNote[3] + 2;
			stringNote[5] = stringNote[4] + 2;
			stringNote[6] = stringNote[5] + 1;
		break;
		case 5: //h w w w h w w Phyrigian
			stringNote[0] = baseNote;
			stringNote[1] = stringNote[0] + 1;
			stringNote[2] = stringNote[1] + 2;
			stringNote[3] = stringNote[2] + 2;
			stringNote[4] = stringNote[3] + 2;
			stringNote[5] = stringNote[4] + 1;
			stringNote[6] = stringNote[5] + 2;
		break;
		case 6: //w w w h w w h Lydian
			stringNote[0] = baseNote;
			stringNote[1] = stringNote[0] + 2;
			stringNote[2] = stringNote[1] + 2;
			stringNote[3] = stringNote[2] + 2;
			stringNote[4] = stringNote[3] + 1;
			stringNote[5] = stringNote[4] + 2;
			stringNote[6] = stringNote[5] + 2;
		break;
		case 7: //W - W - W - H - W - H - W Lydian Flat Seven
			stringNote[0] = baseNote;
			stringNote[1] = stringNote[0] + 2;
			stringNote[2] = stringNote[1] + 2;
			stringNote[3] = stringNote[2] + 2;
			stringNote[4] = stringNote[3] + 1;
			stringNote[5] = stringNote[4] + 2;
			stringNote[6] = stringNote[5] + 1;
		break;
		case 8: //Mixolydian w w h w w h w
			stringNote[0] = baseNote;
			stringNote[1] = stringNote[0] + 2;
			stringNote[2] = stringNote[1] + 2;
			stringNote[3] = stringNote[2] + 1;
			stringNote[4] = stringNote[3] + 2;
			stringNote[5] = stringNote[4] + 2;
			stringNote[6] = stringNote[5] + 1;
		break;	
		case 9: //w w h w w h w - Locrian
			stringNote[0] = baseNote;
			stringNote[1] = stringNote[0] + 2;
			stringNote[2] = stringNote[1] + 2;
			stringNote[3] = stringNote[2] + 1;
			stringNote[4] = stringNote[3] + 2;
			stringNote[5] = stringNote[4] + 2;
			stringNote[6] = stringNote[5] + 1;
		break;
		case 10: //W - W - W+H - W - W+H - Penta Major
			stringNote[0] = baseNote;
			stringNote[1] = stringNote[0] + 2;
			stringNote[2] = stringNote[1] + 2;
			stringNote[3] = stringNote[2] + 3;
			stringNote[4] = stringNote[3] + 2;
			stringNote[5] = stringNote[4] + 3;
			stringNote[6] = stringNote[5] + 4;
		break;
		case 11: //W+H - W - W - W+H + W - Penta Minor
			stringNote[0] = baseNote;
			stringNote[1] = stringNote[0] + 3;
			stringNote[2] = stringNote[1] + 2;
			stringNote[3] = stringNote[2] + 2;
			stringNote[4] = stringNote[3] + 3;
			stringNote[5] = stringNote[4] + 2;
			stringNote[6] = stringNote[5] + 5;
		break;
		case 12: //W+H - H - H - H - H - W+H + W - blues
			stringNote[0] = baseNote;
			stringNote[1] = stringNote[0] + 3;
			stringNote[2] = stringNote[1] + 1;
			stringNote[3] = stringNote[2] + 1;
			stringNote[4] = stringNote[3] + 1;
			stringNote[5] = stringNote[4] + 1;
			stringNote[6] = stringNote[5] + 3;
		break;
		case 13: //whole tone
			stringNote[0] = baseNote;
			stringNote[1] = stringNote[0] + 2;
			stringNote[2] = stringNote[1] + 2;
			stringNote[3] = stringNote[2] + 2;
			stringNote[4] = stringNote[3] + 2;
			stringNote[5] = stringNote[4] + 2;
			stringNote[6] = stringNote[5] + 2;
		break;
	}
}


/**************************************
**

USABILITY AND INTERACTIONS
The tiny big section for most desired feature

**
***************************************/
boolean cacheIdle;

boolean checkIdle(){
	long now = millis();
	//
	if(now - lastInteraction > IDLE_THRESHOLD) //Is anybody using the damn thing?
		{ isIdle = true; }
	else 
		{ isIdle = false; } 
	//
	if(isIdle && !cacheIdle) { //If isIdle was just set as true, and it's not already idling. 
		lasersOff(999);
		if(DEBUG) LOG(lastInteraction, "(Idle) Presently Idle.");
		return true;
	} else if (cacheIdle && !isIdle) { //if it was idle last time we checked, but it's not anymore.
		lasersOn(200);
		if(DEBUG) LOG(now, "(Idle) Idle Broken, activity detected.");
		return false;
	} //else do nothing because it's idle or running.
	//
	cacheIdle = isIdle; //So we have something to match against.
}

//*********************

// Ambient Light

// **** **** **********

// int subtractAmbience = 0;
int ambientPollEvery = 1000*3;
int ambientPollLast = millis();

void pollAmbientLight(){
	long now = millis();
	if(now-ambientPollLast>ambientPollEvery) {
		subtractAmbience = analogRead(PINS_PC_AMBIENT);
		 if(DEBUG) LOG(subtractAmbience, "(Ambient Light) Subtracting ambient light in 532nm wavelength.");
	}
}

//*********************

// Utilites

// **** **** **********

int getRange(int s){ return digitalRead(stringRange[s]); }

int NORMALIZE(int set, int rangeLow, int rangeHigh, int toMin, int toMax) { 
	// toMin = toMin || 0;
	// toMax = toMax || 1000;
	int result = toMin + (set-rangeLow)*(toMax-toMin)/(rangeHigh-rangeLow);
	return result;
}


char button(char button_num)
{
  return (!(digitalRead(button_num)));
}

/**************************************
**
STEPPER
Oh shit, I wanted to avoid this. le sigh
**
***************************************/

int stepIndex[8];
int stepNumber = 14;
int stepDuration = 1000;
int stepEvery = stepDuration/stepNumber;
// boolean stepping[NUMBER_STRINGS];
//
int stepTo[NUMBER_STRINGS];
int stepFrom[NUMBER_STRINGS];
int stepCurrent[NUMBER_STRINGS];
int stepDistance[NUMBER_STRINGS];
int stepMillis[NUMBER_STRINGS];

#define STEPPER_THRESHOLD 50 //Must be more than this big of a gap to 

void setupStepper(){
	memset(stepping, false, NUMBER_STRINGS);
}

int step(int s, int from, int to){
	long now = millis();
	if(stepping[s]){
		if(now-stepMillis[s] > stepEvery) { //is it time to do a step
			if(stepIndex[s] < stepNumber) { //have we finished our steps
				if(DEBUG) LOG(s, "(Stepper) Step"); 
				stepDistance[s] = (stepFrom[s] - stepTo[s]) / stepNumber;
				stepCurrent[s] = stepDistance[s] * (stepIndex[s]+1);
				stepMillis[s] = now;
				stepIndex[s]++;
				return stepCurrent[s];
			} else { //give them their number, and make sure it doesn't happen again.
				if(DEBUG) LOG(s, "(Stepper) Steps Complete"); 
				stepping[s] = false;
				stepReset(s, "all");
				stepIndex[s] = -1; //this flags that it has already been completed.
				return to;
			}
		}
	} else { //First step!
		if(!stepStart(s, from, to)) return to; //If the gap isn't big enough, should have never gotten here but still.
	}
}

boolean stepStart(int s, int from, int to){
	if(stepIndex[s] == -1) return false;
	if(DEBUG) { LOG(s, "(Stepper) Initialized for String/Laser"); } 
	stepReset(s, "all");
	int difference = to-from;
	if(difference < 0) difference=difference*1;
	if(difference > STEPPER_THRESHOLD) {
		stepFrom[s] = from;
		stepTo[s] = to;
		stepping[s] = true;
		return true;
	} else {
		stepReset(s, "all");
	}
}

void stepStop(int s){
	stepReset(s, "all");
}

void stepReset(int s, const char* type){
	if(DEBUG) { LOG(s, "(Stepper) Reset"); } 
	if(type == "current") 	stepCurrent[s] = 0;
	if(type == "begin") 		stepFrom[s] = 0;
	if(type == "target") 		stepTo[s] = 0;
	if(type == "distance") 	stepDistance[s] = 0;
	if(type == "stepping") 	stepping[s] = false;
	if(type == "all") {
		stepCurrent[s] = 0;
		stepFrom[s] = 0;
		stepTo[s] = 0;
		stepIndex[s] = 0;
		stepMillis[s] = 0;
		stepping[s] = false;
	}
}

void stepResetAll(int s){
	for(int s=0;s<NUMBER_STRINGS;s++) { stepReset(s, "all"); }
}

void stepSet(const char* type, int target, int s){
	if(type == "current") 	stepCurrent[s] = target;
	if(type == "begin") 		stepFrom[s] = target;
	if(type == "target") 		stepTo[s] = target;
	if(type == "distance") 	stepDistance[s] = target;
	if(type == "stepping") 	stepping[s] = target;
}


//*********************

// Debugging

// **** **** **********

void LOG(int in ){
	Serial.println(in);
}

void LOG(int in, const char* label){
	Serial.print(label); Serial.print(": "); Serial.println(in);
}

void LOG(const char* in, const char* label){
	Serial.print(label); Serial.print(": "); Serial.println(in);
}

void LOG(const char* in, const char* label, int labelID){
	Serial.print(label); Serial.print(" "); Serial.print(labelID); Serial.print(": "); Serial.println(in);
}

void LOG(int in, const char* label, int labelID){
	Serial.print(label); Serial.print(" "); Serial.print(labelID); Serial.print(": "); Serial.println(in);
}

void debugReport(){
	
	long now = millis();
	if(now-debugLast > debugEvery) {	
		Serial.println("!!!++++++++++++++++++++++++++++++++++++++++++!!!");
		Serial.println("!!!++++++   LASER HARP 2014             +++++!!!");
		Serial.println("!!!++++++++++++++++++++++++++++++++++++++++++!!!");
		Serial.println(" ");	Serial.println(" ");	Serial.println(" ");	Serial.println(" ");	Serial.println(" ");	Serial.println(" ");
		Serial.println("LIVE STATS");	Serial.println(" ");
		for(int s=0;s<NUMBER_STRINGS;s++) {
			Serial.print("~~~~~~~~~~ String ");
			Serial.print(s);
			Serial.println("       -~~~~~~~~~~~~~~~~~~~~~~");
			Serial.print("Note: "); Serial.println(stringNote[s]);
			Serial.print("Midi Stage: "); Serial.println(midiStage[s]);
			Serial.print("Triggered?: "); Serial.println(stringTriggered[s]);
			Serial.print("Holding: "); Serial.println(stringTriggered[s]);
			Serial.print("Photocell: "); Serial.println(PCValues[s]);
			Serial.print("Range: "); Serial.println(stringRange[s]);
			Serial.print("Range (Raw Values): "); Serial.println(rangeRawValue[s]);
		}
		
		for(int k=0;k<3;k++) {
			Serial.print("Knob: "); Serial.println(knobRawValue[k]);
		}
		
		Serial.print("Last Interaction: "); Serial.println(lastInteraction);
					
		Serial.println(" ");	Serial.println(" ");	Serial.println(" ");	Serial.println(" ");	Serial.println(" ");	Serial.println(" ");
		Serial.println("PINS");	Serial.println(" ");
		Serial.print("TOTAL STRINGS:  "); 		Serial.println(NUMBER_STRINGS);
		Serial.print("TOTAL RANGES: ");				Serial.println(NUMBER_RANGE_FINDERS);
		Serial.print("LASER 1: ");						Serial.println(PINS_LASER_ONE);
		Serial.print("LASER 2: ");						Serial.println(PINS_LASER_TWO);
		Serial.print("LASER 3: ");						Serial.println(PINS_LASER_THREE);
		Serial.print("LASER 4: ");						Serial.println(PINS_LASER_FOUR);
		Serial.print("LASER 5: ");						Serial.println(PINS_LASER_FIVE);
		Serial.print("LASER 6: ");						Serial.println(PINS_LASER_SIX);
		Serial.print("LASER 7: ");						Serial.println(PINS_LASER_SEVEN);
		Serial.print("RANGE 2 ECHO: ");				Serial.println(PINS_RANGE_ECHO_TWO);
		Serial.print("PHOTOCELL 1: ");				Serial.println(PINS_PC_ONE);
		Serial.print("PHOTOCELL 2: ");				Serial.println(PINS_PC_TWO);
		Serial.print("PHOTOCELL 3: ");				Serial.println(PINS_PC_THREE);
		Serial.print("PHOTOCELL 4: ");				Serial.println(PINS_PC_FOUR);
		Serial.print("PHOTOCELL 5: ");				Serial.println(PINS_PC_FIVE);
		Serial.print("PHOTOCELL 6: ");				Serial.println(PINS_PC_SIX);
		Serial.print("PHOTOCELL 7: ");				Serial.println(PINS_PC_SEVEN);
		Serial.print("RANGE 1 TRIGGER: ");		Serial.println(PINS_RANGE_TRIG_ONE);
		Serial.print("RANGE 2 TRIGGER: ");		Serial.println(PINS_RANGE_TRIG_TWO);
		Serial.print("RANGE 1 ECHO: ");				Serial.println(PINS_RANGE_ECHO_ONE);

		Serial.print("MODE: ");								Serial.println(PINS_MODE);
				
		debugLast = now;
	}
}