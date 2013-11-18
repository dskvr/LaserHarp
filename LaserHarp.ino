#include <Streaming.h>


// Math Constants 
 //Dimensional



//*************** PIN CONFIG ***************//

struct SV {
	int one;
	int two;
	int three;
};

struct SB {
	boolean one;
	boolean two;
	boolean three;
};

//* PINS *//
SV pinsLDR;
SV pinsRangeEcho;
SV pinsRangeTrigger;

//* DATA CACHE *//
SV stringRange;
SV stringNote;
SV stringDown;
SV knobValues;
SV holding;
SV midiStage;

//* VALUES *//
SV ldrValues;
SV rangeValues;

SB buttonStatus;
SB stringTriggeredCache;
SB stringTriggered;

//*************** Range Pins ***************//


//*************** Knob Cache ***************//

struct knobValues {
	int one;
	int two;
};


//*************** Harp Params ***************//

int baseNote = 60;
int harpScale = 0;

//*************** RANGE PINS ***************//

#define PINS_RANGE_ECHO_ONE 22
#define PINS_RANGE_ECHO_TWO 24
#define PINS_RANGE_ECHO_THREE 26

#define PINS_RANGE_TRIG_ONE 23 // Trigger Pin
#define PINS_RANGE_TRIG_TWO 25 // Trigger Pin
#define PINS_RANGE_TRIG_THREE 27 // Trigger Pin

//*************** LDR PINS ***************//

#define PINS_LDR_ONE 8
#define PINS_LDR_TWO 9 
#define PINS_LDR_THREE 10

//*************** MODE BUTTON PIN ***************//

#define PINS_MODE 11

//*************** LASER SWITCH PINS ***************//

#define PINS_SWITCH_ONE 28
#define PINS_SWITCH_TWO 30
#define PINS_SWITCH_THREE 32

//*************** Midi Config ***************//

#define KNOB1  0
#define KNOB2  1

#define BUTTON1  2
#define BUTTON2  3
#define BUTTON3  4

#define STAT1  7
#define STAT2  6

#define OFF 1
#define ON 2
#define WAIT 3
#define CONTINUOUS 11

#define NOTEON 0x90
#define NOTEOFF 0x80

#define MIDI_NOTE_LOW 0
#define MIDI_NOTE_HIGH 115

//*************** LDR CONFIG ***************//

#define LASER_LDR_THRESH 600 //CHANGE!

#define IDLE_THRESHOLD 30000

#define TOTAL_MODES 3


#define RANGE_MAX 114 //CHANGE!

long rangeMillisSince, rangeMillisLast;
	
//Thresholds


//*************** Midi Variables ***************//

byte incomingByte;
byte note;
byte velocity;
int pot;

//*************** Midi Loopback Cache ***************//

byte byte1;
byte byte2;
byte byte3;

boolean active = false;

//*************** User Tracking ***************//

long lastInteraction;
int lastButtonState;
boolean isIdle; 

// Mode Two Stuff
int modeTwoCacheOne;
int modeTwoCacheTwo;
int modeTwoCacheThree;
int harmonicOne[8];
int modeTwoPos = 0;

int mode = 0; // 0 = harp, 1 = harp notes, 2 = sampler

int action = ON; //1 =note off ; 2=note on ; 3= wait

void setup() {
	
	lasersOn(200);

	// setupLaser();	
	setupRange();
	// setupLDR();
	//
	// setupNotes();
	setScale();
	// setMode();
	//
	setupMidi();

  //start serial with midi baudrate 31250
  Serial.begin(31250);
//  Serial.begin(9600);
	
}

void loop () {
	
	// Midi_Send(0x90, 0x5A, 0x45);
	// delay(100);
	// Midi_Send(0x80, 0x5A, 0x45);
	// delay(300);
	
	// if(analogRead(8) < 700) {
	// 			Midi_Send(0x90, 0x5A, 0x45);
	// 			// Midi_Send(0x90, (char) stringNote.one, (char) stringRange.one);
	// 		} else {
	// 			Midi_Send(0x80, 0x5A, 0x45);
	// 			// Midi_Send(0x80, (char) stringNote.one, (char) stringRange.one);
	// 		}
	
		// 
		// //*************** Poll Inputs ***************//
		// checkIdle();
		pollSensors();
		pollKnobs();
		
		// // pollMode();
		// // pollButtons();
		// 
		// //*************** Prepare 
		// setScale();
		// 
		// //*************** Conditional Midi ***************//
		// if(active) {
		sendMidi(); 
		// }
		// 
	//*************** MIDI OUT ***************//
  
	// pot = analogRead(0);
	// 	  note = 40;  // convert value to value 0-127
	// 	  if(button(BUTTON1) || button(BUTTON2) || button(BUTTON3))
	// 	  {  
	// 	    Midi_Send(0x90,note,0x45);
	// 	    while(button(BUTTON1) || button(BUTTON2) || button(BUTTON3));
	// 	  }
	
  //*************** MIDI LOOPBACK ******************//
	//midiLoopback();

  //*************** MIDI IN ***************//
  // if (Serial.available() > 0) {
  //   // read the incoming byte:
  //   incomingByte = Serial.read();
  // 
  //   // wait for as status-byte, channel 1, note on or off
  //   if (incomingByte== 144) // Note on
  //   { 
  //     action = OFF;
  //   }
  //   else if (incomingByte== 128) // Note off
  //   { 
  //     action = ON;
  //   }
  //   else if (note==0 && action != WAIT) // note on, wait for note value
  //   { 
  //     note=incomingByte;
  //   }
  //   else if (note!=0 && action != WAIT)  // velocity
  //   { 
  //     velocity=incomingByte;
  //     if(action == ON){ 
  //       Midi_Send(0x90,note,velocity); 
  //     }
  //     if(action == OFF){ 
  //       Midi_Send(0x80,note,velocity); 
  //     }
  //     note=0;
  //     velocity=0;
  //     action=WAIT;
  //   }
  //   else{
  //   }
  // }

}

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

//
// SETUP FUNCTIONS
//
void setupLaser(){
	pinMode(PINS_SWITCH_ONE, OUTPUT);
	pinMode(PINS_SWITCH_TWO, OUTPUT);
	pinMode(PINS_SWITCH_THREE, OUTPUT);
	
	lasersOn(500);
}

void laserOn(int laser){
	switch(laser) {
		case 1 : digitalWrite(PINS_SWITCH_ONE, HIGH); break;
		case 2 : digitalWrite(PINS_SWITCH_TWO, HIGH); break;
		case 3 : digitalWrite(PINS_SWITCH_THREE, HIGH); break;		
	}
}

void laserOff(int laser){
	switch(laser) {
		case 1 : digitalWrite(PINS_SWITCH_ONE, LOW); break;
		case 2 : digitalWrite(PINS_SWITCH_TWO, LOW); break;
		case 3 : digitalWrite(PINS_SWITCH_THREE, LOW); break;		
	}
}

void lasersOn(){
	for(int l = 0; l < 3; l++) {
		laserOn(l);
	}
}

void lasersOn(int d){
	for(int l = 0; l < 3; l++) {
		laserOn(l);
		if(l != 2) { delay(d); }
	}
}

void lasersOff(){
	for(int l = 0; l < 3; l++) {
		laserOff(l); 
	}
}

void lasersOff(int d){
	for(int l = 0; l < 3; l++) {
		laserOff(l);
		if(l != 2) { delay(d); }
	}
}

void setupRange(){
	
	// Setup Trigger Pins
	pinMode(PINS_RANGE_TRIG_ONE, OUTPUT);
	pinMode(PINS_RANGE_TRIG_TWO, OUTPUT);
	pinMode(PINS_RANGE_TRIG_THREE, OUTPUT);
	
	// Setup input pins for strings.
	pinMode(PINS_RANGE_ECHO_ONE, INPUT);
	pinMode(PINS_RANGE_ECHO_TWO, INPUT);
	pinMode(PINS_RANGE_ECHO_THREE, INPUT);	
	
	rangeMillisLast = millis();
}

void setupLDR(){
	// PINS_LDR_ONE = 8;
	// pinsLDR.two = 9;
	// pinsLDR.three = 10;
}

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

void setMode(){ 
	mode = mode + 1;
	if(mode >= TOTAL_MODES) { mode = 0; }
	
	if(mode == 1) {
		modeTwoUpdate();
	}
}

void modeTwoUpdate(){
	
	for(int n = 0; n < 8; n++) {
		if(n == 0) {
			modeTwoPos = stringNote.one;
			harmonicOne[n] = modeTwoPos;
		}

		if(n == (1 || 3 || 4 || 7)) {
			modeTwoPos = modeTwoPos + 2;
			harmonicOne[n] = modeTwoPos;
		}

		if(n == (2 || 5 || 8)) {
			modeTwoPos = modeTwoPos + 1;
			harmonicOne[n] = modeTwoPos;
		}
	}
	
}

void pingRangeSensors(){
		
	rangeMillisSince = millis() - rangeMillisLast;
	if(rangeMillisSince > 50) {
		long duration;
	
		// one
		digitalWrite(PINS_RANGE_TRIG_ONE, LOW); 
	 	delayMicroseconds(2); 
	 	digitalWrite(PINS_RANGE_TRIG_ONE, HIGH);
	 	delayMicroseconds(10); 
	 	digitalWrite(PINS_RANGE_TRIG_ONE, LOW);
	 	duration = pulseIn(PINS_RANGE_ECHO_ONE, HIGH);

	 	//Calculate the distance (in cm) based on the speed of sound.
	 	rangeValues.one = duration/58.2;
		duration = 0;

		// // two
		// 		digitalWrite(PINS_RANGE_TRIG_TWO, LOW); 
		// 	 	delayMicroseconds(2); 
		// 	 	digitalWrite(PINS_RANGE_TRIG_TWO, HIGH);
		// 	 	delayMicroseconds(10); 
		// 	 	digitalWrite(PINS_RANGE_TRIG_TWO, LOW);
		// 	 	duration = pulseIn(PINS_RANGE_ECHO_TWO, HIGH);
		// 
		// 	 	//Calculate the distance (in cm) based on the speed of sound.
		// 	 	rangeValues.two = duration/58.2;
		// 		duration = 0;
		// 
		// 		// three
		// 		digitalWrite(PINS_RANGE_TRIG_THREE, LOW); 
		// 	 	delayMicroseconds(2); 
		// 	 	digitalWrite(PINS_RANGE_TRIG_THREE, HIGH);
		// 	 	delayMicroseconds(10); 
		// 	 	digitalWrite(PINS_RANGE_TRIG_THREE, LOW);
		// 	 	duration = pulseIn(PINS_RANGE_ECHO_THREE, HIGH);
		// 
		// 	 	//Calculate the distance (in cm) based on the speed of sound.
		// 	 	rangeValues.three = duration/58.2;
	
		rangeMillisLast = millis();
	}
}

void setupMidi(){
	
	holding.one = false;
	holding.two = false;
	holding.three = false;
	
	pinMode(STAT1,OUTPUT);   
  pinMode(STAT2,OUTPUT);

  pinMode(BUTTON1,INPUT);
  pinMode(BUTTON2,INPUT);
  pinMode(BUTTON3,INPUT);

  digitalWrite(BUTTON1,HIGH);
  digitalWrite(BUTTON2,HIGH);
  digitalWrite(BUTTON3,HIGH);

  for(int i = 0;i < 10;i++) // flash MIDI Sheild LED's on startup
  {
    digitalWrite(STAT1,HIGH);  
    digitalWrite(STAT2,LOW);
    delay(30);
    digitalWrite(STAT1,LOW);  
    digitalWrite(STAT2,HIGH);
    delay(30);
  }

  digitalWrite(STAT1,HIGH);   
  digitalWrite(STAT2,HIGH);

}

void midiLoopback(){
	// if(Serial.available() > 0)
	//   {
	//     byte1 = Serial.read();
	//     byte2 = Serial.read();
	//     byte3 = Serial.read();
	// 
	//     Midi_Send(byte1, byte2, byte3);
	//   }
}

void sendMidi(){
	
	if(mode == 0) { 
		
		if(stringTriggered.one && !midiStage.one) {
			// Midi_Send(NOTEON, stringNote.one, 69);
			Midi_Send(NOTEON, stringNote.one, stringRange.one);
			Midi_Send(0xB0, 17, stringRange.one);
			midiStage.one = 1;
			// Midi_Send(0x90, stringNote.one, 70);
			stringTriggered.one = false;
		} else if( !stringTriggered.one && midiStage.one ) { 
			Midi_Send(NOTEOFF, stringNote.one, stringRange.one);
			midiStage.one = 0;
			// Midi_Send(0x80, stringNote.one, 70);
		} else if(stringTriggered.one) {
			midiStage.one = 2;
			Midi_Send(0xB0, 17, stringRange.one);
			// Midi_Send(NOTEON, stringNote.one, stringRange.one);
		} 
	
		// if(stringTriggered.two && !midiStage.two) {
		// 	// Midi_Send(NOTEON, stringNote.two, 69);
		// 	Midi_Send(NOTEON, stringNote.two, stringRange.two);
		// 	Midi_Send(0xB0, 2, stringRange.two);
		// 	midiStage.two = 1;
		// 	// Midi_Send(0x90, stringNote.two, 70);
		// 	stringTriggered.two = false;
		// } else if( !stringTriggered.two && midiStage.two ) { 
		// 	Midi_Send(NOTEOFF, stringNote.two, stringRange.two);
		// 	midiStage.two = 0;
		// 	// Midi_Send(0x80, stringNote.two, 70);
		// } else if(stringTriggered.two) {
		// 	midiStage.two = 2;
		// 	Midi_Send(0xB0, 2, stringRange.two);
		// 	// Midi_Send(NOTEON, stringNote.two, stringRange.two);
		// }
		// 	
		// if(stringTriggered.three && !midiStage.three) {
		// 	// Midi_Send(NOTEON, stringNote.three, 69);
		// 	Midi_Send(NOTEON, stringNote.three, stringRange.three);
		// 	Midi_Send(0xB0, 3, stringRange.three);
		// 	midiStage.three = 1;
		// 	// Midi_Send(0x90, stringNote.three, 70);
		// 	stringTriggered.three = false;
		// } else if( !stringTriggered.three && midiStage.three ) { 
		// 	Midi_Send(NOTEOFF, stringNote.three, stringRange.three);
		// 	midiStage.three = 0;
		// 	// Midi_Send(0x80, stringNote.three, 70);
		// } else if(stringTriggered.three) {
		// 	midiStage.three = 2;
		// 	Midi_Send(0xB0, 3, stringRange.three);
		// 	// Midi_Send(NOTEON, stringNote.one, stringRange.one);
		// }
	
	} else if(mode == 1) {
		
		int rangeOne = NORMALIZE(stringRange.one, 0, RANGE_MAX, 0, 8);
		
		if(stringTriggered.one && !midiStage.one) {
			// Midi_Send(NOTEON, stringNote.one, 69);
			Midi_Send(NOTEON, harmonicOne[rangeOne], 69);
			modeTwoCacheOne = rangeOne;
			midiStage.one = 1;
			// Midi_Send(0xB0, 17, stringRange.one);
			// Midi_Send(0x90, stringNote.one, 70);
			stringTriggered.one = false;
		} else if( !stringTriggered.one && midiStage.one ) { 
			Midi_Send(NOTEOFF, harmonicOne[rangeOne], 69);
			Midi_Send(NOTEOFF, harmonicOne[modeTwoCacheOne], 69);
			midiStage.one = 0;
			// Midi_Send(0x80, stringNote.one, 70);
		} else if(stringTriggered.one) {
			if(modeTwoCacheOne != rangeOne) {
				Midi_Send(NOTEOFF, harmonicOne[modeTwoCacheOne], 69);
				Midi_Send(NOTEON, harmonicOne[rangeOne], 69);
			}
			midiStage.one = 2;
			// Midi_Send(0xB0, 17, stringRange.one);
			// Midi_Send(NOTEON, stringNote.one, stringRange.one);
		}
		
	} else if(mode == 2) {
		
	}
	
	// if(stringTriggered.two && !holding.two) {
	// 	Midi_Send(0x90, (char) stringNote.two, (char) stringRange.two);
	// 	stringTriggered.two = false;
	// 	holding.two = true;
	// }	else {
	// 	if(holding.two) { 
	// 		Midi_Send(0x80, (char) stringNote.two, (char) stringRange.two);
	// 		holding.two = false;
	// 	}
	// }
	// 
	// if(stringTriggered.three && !holding.three) {
	// 	Midi_Send(0x90, (char) stringNote.three, (char) stringRange.three);
	// 	stringTriggered.three = false;
	// 	holding.three = true;
	// }	else {
	// 	if(holding.one) { 
	// 		Midi_Send(0x80, (char) stringNote.three, (char) stringRange.three);
	// 		holding.three = false;
	// 	}
	// }

	// while(stringTriggered.one || stringTriggered.two || stringTriggered.three );
	
}

//**********************

// Watchers

// **** **** **********
void pollSensors(){
	
	boolean tripped = false;
	
	pingRangeSensors();

	if( ldrValues.one = analogRead(PINS_LDR_ONE) < LASER_LDR_THRESH ) { //String has been tripped. 
		rangeValues.one = (rangeValues.one > RANGE_MAX) ? RANGE_MAX : rangeValues.one;
		stringRange.one = NORMALIZE(rangeValues.one, 1, RANGE_MAX, MIDI_NOTE_LOW, MIDI_NOTE_HIGH);
		stringTriggered.one = true;
		tripped = true;
	} else {
		stringTriggered.one = false;
	}
	
	if( ldrValues.two = analogRead(PINS_LDR_TWO) < LASER_LDR_THRESH ) { //String has been tripped. 
		rangeValues.two = (rangeValues.two > RANGE_MAX) ? RANGE_MAX : rangeValues.two;
		stringRange.two = NORMALIZE(rangeValues.two, 1, RANGE_MAX, MIDI_NOTE_LOW, MIDI_NOTE_HIGH);
		stringTriggered.two = true;
		tripped = true;
	} else {
		stringTriggered.two = false;
	}
	
	if( ldrValues.three = analogRead(PINS_LDR_THREE) < LASER_LDR_THRESH ) { //String has been tripped. 
		rangeValues.three = (rangeValues.three > RANGE_MAX) ? RANGE_MAX : rangeValues.three;
		stringRange.three = NORMALIZE(rangeValues.three, 1, RANGE_MAX, MIDI_NOTE_LOW, MIDI_NOTE_HIGH);
		stringTriggered.three = true;
		tripped = true;
	} else {
		stringTriggered.three = false;
	}
	
	// if( ldrValues.two = analogRead(pinsLDR.two) < LASER_LDR_THRESH ) {
	// 	stringTriggered.two = true;
	// 	stringRange.two = NORMALIZE(digitalRead(pinsRangeEcho.two), 0, RANGE_MAX, 1, 127);
	// 	tripped = true;
	// } else {
	// 	stringTriggered.two = false;
	// }
	// 
	// if( ldrValues.three = analogRead(pinsLDR.three) < LASER_LDR_THRESH ) {
	// 	stringTriggered.three = true;
	// 	stringRange.three = NORMALIZE(digitalRead(pinsRangeEcho.three), 0, RANGE_MAX, 1, 127);
	// 	tripped = true;
	// } else {
	// 	stringTriggered.three = false;
	// }
	
	if(tripped) 	{ active = true; lastInteraction = millis(); }
	else 					{ active = false; }
	
}

// void pollKnobs(){
// 	knobValues.one = analogRead(KNOB1) / 8; // convert value to value 0-127
// 	knobValues.two = analogRead(KNOB2) / 8; // convert value to value 0-127
// }

void pollButtons(){
	buttonStatus.one = button(BUTTON1);
	buttonStatus.two = button(BUTTON2);
	buttonStatus.three = button(BUTTON3);
}

//*********************

// Chords

// **** **** **********

void setBaseNote(int n){
 baseNote = n;
}

void setHarpScale(char scale){
	harpScale = 0;
}

void setScale(){
	
	if(harpScale == 0) { //Major 
		stringNote.one = baseNote;
		stringNote.two = baseNote + 4;
		stringNote.three = baseNote + 4 + 3;
	} else if (harpScale == 1) { //Major7th
		stringNote.one = baseNote;
		stringNote.two = baseNote + 4;
		stringNote.three = baseNote + 4 + 6;
	}	else if (harpScale == 2) { //Minor 
		stringNote.one = baseNote;
		stringNote.two = baseNote + 3;
		stringNote.three = baseNote + 3 + 4;
	} else if (harpScale == 3) { //Diminished
		stringNote.one = baseNote;
		stringNote.two = baseNote + 3;
		stringNote.three = baseNote + 3 + 3;
	} else if (harpScale == 4 ) { //Aug
		stringNote.one = baseNote;
		stringNote.two = baseNote + 4;
		stringNote.three = baseNote + 4 + 4;
	}

}

//*********************

// Utilites

// **** **** **********

int getRange(int s){  
	if(s == 1) { return digitalRead(stringRange.one); } 
	if(s == 2) { return digitalRead(stringRange.two); } 
	if(s == 3) { return digitalRead(stringRange.three); } 	
}

int NORMALIZE(int set, int rangeLow, int rangeHigh, int toMin, int toMax) { 
	// toMin = toMin || 0;
	// toMax = toMax || 1000;
	int result = toMin + (set-rangeLow)*(toMax-toMin)/(rangeHigh-rangeLow);
	return result;
}

void Midi_Send(byte cmd, byte data1, byte data2) {
  Serial.write(cmd);
  Serial.write(data1);
  Serial.write(data2);
}

void blink(){
  digitalWrite(STAT1, HIGH);
  delay(100);
  digitalWrite(STAT1, LOW);
  delay(100);
}

char button(char button_num)
{
  return (!(digitalRead(button_num)));
}




