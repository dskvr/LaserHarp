#include <Streaming.h>
/*++++++++++++++++++++++++++++++++++++++++++++++++++
Constraints
//*************** LASER SWITCH PINS ***************/
#DEFINE NUMBER_STRINGS = 7;
#DEFINE NUMBER_SENSORS = 2;
	#DEFINE NUMBER_RANGE_FINDERS = 2;
	/*++++++++++++++++++++++++++++++++++++++++++++++++++
	PINS PINS PINS
	//*************** LASER SWITCH PINS ***************/
	#define PINS_LASER_ONE 			28
	#define PINS_LASER_TWO 			30
	#define PINS_LASER_THREE 		32
	//*************** STRING RANGE FINDERS ***************//
	//								TRIGGER ***************//
	#define PINS_RANGE_TRIG_ONE 	23 // Trigger Pin
	#define PINS_RANGE_TRIG_TWO 	25 // Trigger Pin
	//								ECHO ***************//
	#define PINS_RANGE_ECHO_ONE 	22
	#define PINS_RANGE_ECHO_TWO 	24
	//*************** USERRANGE TRIG & ECHO ***************//
	#define PINS_USER_RANGE_TRIG 	27
	#define PINS_USER_RANGE_ECHO	26
	//*************** PHOTOCELLS ***************//
	#define PINS_PC_ONE 					8
	#define PINS_PC_TWO 					9 
	#define PINS_PC_THREE 				10
	#define PINS_PC_FOUR 					8
	#define PINS_PC_FIVE 					9 
	#define PINS_PC_SIX		 				10
	#define PINS_PC_SEVEN		 				10
	//*************** KNOB PINS ***************//
	#define PINS_KNOB_ONE 				0
	#define PINS_KNOB_TWO  				1
	#define PINS_KNOB_THREE  			2
	//*************** MODE BUTTON PIN ***************//
	#define PINS_MODE 						11
/*++++++++++++++++++++++++++++++++++++++++++++++++++
MIDI
//*************** HELPERS ***************/
#define OFF 1
#define ON 2
#define WAIT 3
#define CONTINUOUS 11
#define NOTEON 0x90
#define NOTEOFF 0x80
//*************** LIMITS ***************/
#define MIDI_NOTE_LOW 0
#define MIDI_NOTE_HIGH 115
/*++++++++++++++++++++++++++++++++++++++++++++++++++
RANGE
//*************** LIMITS ***************/
#define THRESHOLD_PING_RANGE	50
/*++++++++++++++++++++++++++++++++++++++++++++++++++
INTERACTIVITY
//*************** LIMITS ***************/
#define IDLE_THRESHOLD 				30000
#define RANGE_MAX 						114 //CHANGE!
#define LASER_PC_THRESH 			600 //CHANGE!
#define TOTAL_MODES 					3
#define DEFAULT_BASE_NOTE 		60
#define DEFAULT_SCALE					0
//*************** DATA CACHE ***************//
int stringRange[NUMBER_STRINGS];
int stringNote[NUMBER_STRINGS];
int knobValues[NUMBER_STRINGS];
int midiStage[NUMBER_STRINGS];
int PCValues[NUMBER_STRINGS];
int rangeValues[NUMBER_STRINGS];
bool stringTriggered[NUMBER_STRINGS];
bool holding[NUMBER_STRINGS];
//*************** User Tracking ***************//
long lastInteraction;
int lastButtonState;
boolean isIdle;
boolean active = false;
//*************** Midi Variables ***************//
int action = ON; //1 =note off ; 2=note on ; 3= wait
byte incomingByte;
byte note;
byte velocity;
int pot;

// Mode Two Stuff
int modeTwoCacheOne;
int modeTwoCacheTwo;
int modeTwoCacheThree;
int harmonicOne[8];
int modeTwoPos = 0;
int mode = 0; // 0 = harp, 1 = harp notes, 2 = sampler

void setup() {
	setupRange();
	setScale();
	setupMidi();
  Serial.begin(31250);  //start serial with midi baudrate 31250
	setupLaser();	
}

void loop () {		
		pollKnobs();   //Check for octave, key, scale, mode values
		pollSensors(); //Check photocells and find range.
		sendMidi();    //Send midi signal. 
}
/**************************************
**
LASERS
with Sharks.
**
***************************************/
int totalLasers = NUMBER_STRINGS;
int cooldownLast = millis();
int cooldownEvery = 60*10;
int cooldownPeriod = 60;
int startupLast = millis();
bool coolingDown = false;

bool laserState[totalLasers];

int lasers[totalLasers]; 
		lasers[0] = PINS_LASER_ONE;
		lasers[1] = PINS_LASER_TWO;
		lasers[2] = PINS_LASER_THREE;
		lasers[3] = PINS_LASER_FOUR;
		lasers[4] = PINS_LASER_FIVE;
		lasers[5] = PINS_LASER_SIX;
		lasers[6] = PINS_LASER_SEVEN;
		
void startupLasers(){
	for(int l = 0;l < totalLasers; l++) {
		pinMode(lasers[l], OUTPUT);
	}
	lasersOn(500);
}

void shutdownLasers(){
	
}

void setupLaser(){
	startupLasers();
}

//*************** LASERS: TURN A LASER ON ***************//
void laserOn(int laser){
	digitalWrite(lasers[laser], HIGH);
	laserState[laser] = true;
}
//*************** LASERS: TURN A LASER OFF  ***************//
void laserOff(int laser){
	digitalWrite(lasers[laser], LOW);
	laserState[laser] = false;
}
//*************** LASERS: TURN ALL ON ***************//
void lasersOn(){
	for(int l = 0; l < totalLasers; l++) {
		laserOn(l);
	}
}
//*************** LASERS: TURN ALL ON WITH DELAY ***************//
void lasersOn(int d){
	for(int l = 0; l < totalLasers; l++) {
		laserOn(l);
		delay(d);
	}
}
//*************** LASERS: TURN ALL OFF ***************//
void lasersOff(){
	for(int l = 0; l < totalLasers; l++) {
		laserOff(l); 
	}
}

//*************** LASERS: TURN ALL OFF WITH DELAY ***************//
void lasersOff(int d){
	for(int l = 0; l < totalLasers; l++) {
		laserOff(l);
		delay(d);
	}
}

//*************** CHECK LASER COOLDOWN CYCLE  ***************//
void checkCooldown(){
	int now = millis();
	if(now-cooldownLast > cooldownEvery && !coolingDown) {
		//blink 3 times then cooldown
		for(int blink=0;blink<3;blink++){
			lasersOff( 20 );  delay(250);
			lasersOn( 70 );   delay(500);
		}
	} elseif(now-startupLast > cooldownPeriod && coolingDown) {
		lasersOn( 250 );
	} else {
		//do nothing
	}
}

void coolDown(){
	
}

/**************************************
**
RANGE FINDER
The tiny big section for most desired feature
**
***************************************/

//*************** SETUP ***************//
int rangeMillisLast[NUMBER_STRINGS];

//*************** RANGE FINDERS: SETUP PINS ***************//
int pinsRangeTrigger[NUMBER_STRINGS];
		pinsRangeTrigger[0] = PINS_RANGE_TRIG_ONE;
		pinsRangeTrigger[1] = PINS_RANGE_TRIG_ONE;
		pinsRangeTrigger[2] = PINS_RANGE_TRIG_ONE;
		pinsRangeTrigger[3] = PINS_RANGE_TRIG_ONE;
		pinsRangeTrigger[4] = PINS_RANGE_TRIG_ONE;
		pinsRangeTrigger[5] = PINS_RANGE_TRIG_ONE;
		pinsRangeTrigger[6] = PINS_RANGE_TRIG_ONE;
		
int pinsRangeEcho[NUMBER_STRINGS];
		pinsRangeEcho[0] = PINS_RANGE_ECHO_ONE;
		pinsRangeEcho[1] = PINS_RANGE_ECHO_ONE;
		pinsRangeEcho[2] = PINS_RANGE_ECHO_ONE;
		pinsRangeEcho[3] = PINS_RANGE_ECHO_ONE;
		pinsRangeEcho[4] = PINS_RANGE_ECHO_ONE;
		pinsRangeEcho[5] = PINS_RANGE_ECHO_ONE;
		pinsRangeEcho[6] = PINS_RANGE_ECHO_ONE;

//*************** RANGE FINDERS: SETUP PINS FOR INPUT AND OUTPUT ***************//
long rangeMillisSince[s], rangeMillisLast[s];

void setupRange(){
	for(int s=0;s<NUMBER_STRINGS;s++){
		pinMode(pinsRangeTrigger, OUTPUT);
		pinMode(pinsRangeEcho, INPUT);
		rangeMillisLast[s] = millis();
	}
}

void pingRange(int s){
	rangeMillisSince[s] = millis() - rangeMillisLast[s];
	if(rangeMillisSince[s] > THRESHOLD_PING_RANGE) {
		long duration;
		digitalWrite(pinsRangeTrigger[s], LOW); 
	 	delayMicroseconds(2); 
	 	digitalWrite(pinsRangeTrigger[s], HIGH);
	 	delayMicroseconds(10); 
	 	digitalWrite(pinsRangeTrigger[s], LOW);
	 	duration = pulseIn(pinsRangeEcho[s], HIGH);
		rangeMillisLast[s] = millis();
	}
}

/**************************************
**

SAFETY
Without them the lasers are just pretty (and debateably dangerous)

**
***************************************/
int stringTrigDur[NUMBER_STRINGS];
int stringActivated[NUMBER_STRINGS];
long safetyOn[NUMBER_STRINGS];
int stringOn;

#define SAFETY_LASER_DURATION  10000

void safetySecond(int s, bool reset){
	
	int now = millis();
	
	if(stringTriggered[s] && laserState[s]) {
		stringTrigDur[s] = now-stringActivated[s];
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
				setMode();
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
}

void sendMidi(){
	
	switch( mode ){
		
		case 1: //toggle
			for(int s = 0; s < NUMBER_STRINGS; s__){
				if(stringTriggered[s]) {
					if(midiStage[s]) { //is presently off
						Midi_Send(NOTEON, stringNote[s], stringRange[s]); 
						Midi_Send(0xB0, 22+s, stringRange[s]); 
						midiStage[s] = 1; //on
					} else { //is presently on
						Midi_Send(NOTEOFF, stringNote[s], stringRange[s]);
						midiStage[s] = 0; //off
					}
				} else {
					midiStage[s] = 2; //wait
				}
			}
			break;
		default: //momentary
			for(int s = 0; s < NUMBER_STRINGS; s__){
				if(stringTriggered[s] && !midiStage[s]) {
					Midi_Send(NOTEON, stringNote[s], stringRange[s]);
					Midi_Send(0xB0, 17, stringRange[s]);
					midiStage[s] = 1;
					// Midi_Send(0x90, stringNote.one, 70);
					stringTriggered[s] = false;
				} else if( !stringTriggered[s] && midiStage[s] ) { 
					Midi_Send(NOTEOFF, stringNote[s], stringRange[s]);
					midiStage[s] = 0;
					// Midi_Send(0x80, stringNote.one, 70);
				} else if(stringTriggered[s]) {
					midiStage[s] = 2;
					Midi_Send(0xB0, 17, stringRange[s]);
					// Midi_Send(NOTEON, stringNote[s], stringRange[s]);
				}
			}
	}
}

void setupMidi(){
	for(int s=0;s>NUMBER_STRINGS;s++){ holding[s] = false; }
}

void midiLoopback(){ }
		
void pollSensors(){
	boolean tripped = false;
	
	for(int s=0;s<NUMBER_STRINGS;s++) {
		if( isStringActive(s) ) { //String has been tripped. 
			pingRange(s); //This will create latency, so it's only running every 50 milliseconds. 38 microseconds * 7 is a fraction of a Milli, but still.
			setStringRange(s);
			tripped = true;
			safetySecond(s, false);
		} else {
			stringTriggered[s] = false;
			safetySecond(s, false);
		}
	}
	//
	if(tripped) 	{ active = true; lastInteraction = millis(); }
	else 					{ active = false; }
	//
}
int setStringRange(int s){
	stringRange[s] = getStringRange(s);
}

int getStringRange(int s){
	rangeValues[s]=limitRange(rangeValues[s]);
	return NORMALIZE(rangeValues[s], 1, RANGE_MAX, MIDI_NOTE_LOW, MIDI_NOTE_HIGH);
}

int limitRange(int range){
	return (range > RANGE_MAX) ? RANGE_MAX : range;
}

int pinsPC[7];
		pinsPC[0] = PINS_PC_ONE;
		pinsPC[1] = PINS_PC_TWO;
		pinsPC[2] = PINS_PC_THREE;
		pinsPC[3] = PINS_PC_FOUR;
		pinsPC[4] = PINS_PC_FIVE;
		pinsPC[5] = PINS_PC_SIX;
		pinsPC[6] = PINS_PC_SEVEN;
		
boolean isStringActive(int s){
	PCValues[s] = analogRead(pinsPC[s])
	if(!stringActivated[s]) stringActivated[s] = millis();
	bool result = ( PCValues[s] < LASER_PC_THRESH );
	if(stringActivated[s] && !result) 	stringActivated[s] = 0;
	else if(result) 										stringTriggered[s] = true;
	return result;
}

int knobValuesCache;
int knobPollEvery = 200;
int knobPollLast = millis();

void pollKnobs(){
	int now = millis();
	if(now-knobPollLast>knobPollEvery) {
		bool tripped = false;
		for(int s=0;s<NUMBER_KNOBS;s++) {
			knobValues[s] = analogRead(pinsKnob[s]) / 8; // convert value to value 0-127
		}
		int note = (knobValues[0] != knobValuesCache[0] && tripped=true) ? NORMALIZE(knobValue[0], 0, 1024, 0, 12) : knobValuesCache[0];
		int octave = (knobValues[1] != knobValuesCache[1] && tripped=true) ? NORMALIZE(knobValue[1], 0, 1024, 0, 8) : knobValuesCache[1];
		int scale = (knobValues[2] != knobValuesCache[2] && tripped=true) ? NORMALIZE(knobValue[2], 0, 1024, 0, 4) : knobValuesCache[2];
	
		if(tripped) {
			setBaseNote(octave*12+note);
			setHarpScale(scale);
		}
		knobValuesCache = knobValues;
	}
}

//*********************

// Chords

// **** **** **********

int baseNote = DEFAULT_BASE_NOTE;
int harpScale = DEFAULT_SCALE;

void setBaseNote(int n){
 baseNote = n;
}

void setHarpScale(char scale){
	harpScale = scale;
	setScale();
}

void setScale(){	
	switch(harpScale){
		case 0: //Major
			stringNote[0] = baseNote;
			stringNote[1] = baseNote + 4;
			stringNote[2] = baseNote + 4 + 3;
			stringNote[3] = baseNote + 4 + 3;
			stringNote[4] = baseNote + 4 + 3;
			stringNote[5] = baseNote + 4 + 3;
			stringNote[6] = baseNote + 4 + 3;
		break;
		case 1: //Major7th
			stringNote[0] = baseNote;
			stringNote[1] = baseNote + 4;
			stringNote[2] = baseNote + 4 + 6;
			stringNote[3] = baseNote + 4 + 6;
			stringNote[4] = baseNote + 4 + 6;
			stringNote[5] = baseNote + 4 + 6;
			stringNote[6] = baseNote + 4 + 6;
		break;
		case 2: //Minor 
			stringNote[0] = baseNote;
			stringNote[1] = baseNote + 3;
			stringNote[2] = baseNote + 3 + 4;
			stringNote[3] = baseNote + 3 + 4;
			stringNote[4] = baseNote + 3 + 4;
			stringNote[5] = baseNote + 3 + 4;
			stringNote[6] = baseNote + 3 + 4;
		break;
		case 3: //Diminished
			stringNote[0] = baseNote;
			stringNote[1] = baseNote + 3;
			stringNote[2] = baseNote + 3 + 3;
			stringNote[3] = baseNote + 3 + 3;
			stringNote[4] = baseNote + 3 + 3;
			stringNote[5] = baseNote + 3 + 3
			stringNote[6] = baseNote + 3 + 3;
		break;
		case 4: //Augmented
			stringNote[0] = baseNote;
			stringNote[1] = baseNote + 4;
			stringNote[2] = baseNote + 4 + 6;
			stringNote[3] = baseNote + 4 + 6;
			stringNote[4] = baseNote + 4 + 6;
			stringNote[5] = baseNote + 4 + 6
			stringNote[6] = baseNote + 4 + 6;
		break;	
	}
}


/**************************************
**

USABILITY AND INTERACTIONS
The tiny big section for most desired feature

**
***************************************/
void checkIdle(){
	boolean cacheIdle = isIdle;
	int now = millis();
	if(now - lastInteraction > IDLE_THRESHOLD) 
		{ isIdle = true; }
	else 
		{ isIdle = false; } 
	if(isIdle == true) {
		lasersOff(999);
	} else if (cacheIdle == true && isIdle == false) {
		lasersOn(200);
	}
	
}

//*********************

// Ambient Light

// **** **** **********

int subtractAmbience = 0;
int ambientPollEvery = 1000*3;
int ambientPollLast = millis();

void pollAmbientLight(){
	int now = millis();
	if(now-ambientPollLast>ambientPollEvery) {
		subtractAmbience = analogRead(PINS_AMBIENT);
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

//*********************

// Debugging

// **** **** **********

int debugLast = millis();
int debugEvery = 7*1000;

void debug(){
	
	int now = millis();
	if(now-debugLast > debugEvery) {	
		Serial.println("!!!++++++++++++++++++++++++++++++++++++++++++!!!");
		Serial.println("!!!++++++   LASER HARP 2014             +++++!!!");
		Serial.println("!!!++++++++++++++++++++++++++++++++++++++++++!!!");
		Serial.println(" ");	Serial.println(" ");	Serial.println(" ");	Serial.println(" ");	Serial.println(" ");	Serial.println(" ");
		Serial.println("LIVE STATS");	Serial.println(" ");
		for(int s=0;s<NUMBER_STRINGS;s++) {
			Serial.print('~~~~~~~~~~ String ');
			Serial.print(s);
			Serial.println('       -~~~~~~~~~~~~~~~~~~~~~~');
			Serial.print("Note: "); Serial.println(stringNote[s]);
			Serial.print("Midi Stage: "); Serial.println(midiStage[s]);
			Serial.print("Triggered?: "); Serial.println(stringTriggered[s]);
			Serial.print("Holding: "); Serial.println(holding[s]);
			Serial.print("Photocell: "); Serial.println(PCValues[s]);
			Serial.print("Range: "); Serial.println(stringRange[s]);
			Serial.print("Range (Raw Values): "); Serial.println(rangeValues[s]);
		}
		
		for(int k=0;k<3;k++) {
			Serial.print("Knob: "); Serial.println(knobValues[k]);
		}
		
		Serial.print("Last Interaction: "); Serial.println(lastInteraction);
					
		Serial.println(" ");	Serial.println(" ");	Serial.println(" ");	Serial.println(" ");	Serial.println(" ");	Serial.println(" ");
		Serial.println("PINS");	Serial.println(" ");
		Serial.print("TOTAL STRINGS:  "); Serial.println(NUMBER_STRINGS);
		Serial.print("TOTAL RANGES: ");		Serial.println(NUMBER_RANGE_FINDERS);
		Serial.print("LASER 1: ");						Serial.println(PINS_LASER_ONE);
		Serial.print("LASER 2: ");						Serial.println(PINS_LASER_TWO);
		Serial.print("LASER 3: ");						Serial.println(PINS_LASER_THREE);
		Serial.print("LASER 4: ");						Serial.println(PINS_LASER_FOUR);
		Serial.print("LASER 5: ");						Serial.println(PINS_LASER_FIVE);
		Serial.print("LASER 6: ");						Serial.println(PINS_LASER_SIX);
		Serial.print("LASER 7: ");						Serial.println(PINS_LASER_SEVEN);
		Serial.print("RANGE 1 TRIGGER: ");		Serial.println(PINS_RANGE_TRIG_ONE);
		Serial.print("RANGE 2 TRIGGER: ");		Serial.println(PINS_RANGE_TRIG_TWO);
		Serial.print("RANGE 1 ECHO: ");				Serial.println(PINS_RANGE_ECHO_ONE);
		Serial.print("RANGE 2 ECHO: ");				Serial.println(PINS_RANGE_ECHO_TWO);
		Serial.print('PHOTOCELL 1: '););			Serial.println(PINS_PC_ONE);
		Serial.print('PHOTOCELL 2: ');				Serial.println(PINS_PC_TWO);
		Serial.print('PHOTOCELL 3: ');				Serial.println(PINS_PC_THREE);
		Serial.print('PHOTOCELL 4: '););			Serial.println(PINS_PC_FOUR);
		Serial.print('PHOTOCELL 5: ');				Serial.println(PINS_PC_FIVE);
		Serial.print('PHOTOCELL 6: ');				Serial.println(PINS_PC_SIX);
		Serial.print('PHOTOCELL 7: ');				Serial.println(PINS_PC_SEVEN);
	
		Serial.print("MODE: ");
		Serial.println(PINS_MODE);
	
		debugLast = now();
	}

}


// Setup Status Lights
// pinMode(STAT1,OUTPUT);   
// pinMode(STAT2,OUTPUT);
// 
// for(int i = 0;i < 10;i++) // flash MIDI Sheild LED's on startup
// {
//   digitalWrite(STAT1,HIGH);  
//   digitalWrite(STAT2,LOW);
//   delay(30);
//   digitalWrite(STAT1,LOW);  
//   digitalWrite(STAT2,HIGH);
//   delay(30);
// }
// 
// digitalWrite(STAT1,HIGH);   
// digitalWrite(STAT2,HIGH);

// void blink(){
//   digitalWrite(STAT1, HIGH);
//   delay(100);
//   digitalWrite(STAT1, LOW);
//   delay(100);
// }
//#define BUTTON1  2
//#define BUTTON2  3
//#define BUTTON3  4

//#define STAT1  7
//#define STAT2  6
