#define PIN_OCTAVE 13
#define PIN_BASENOTE 14
#define PIN_SCALE 15

#define MAX_OCTAVES 7

#define NV_C 		0
#define NV_Db		1
#define NV_D 		2
#define NV_Eb 	3
#define NV_E 		4
#define NV_F 		5
#define NV_Gb 	6
#define NV_G 		7
#define NV_Ab 	8
#define NV_A 		9
#define NV_Bb 	10
#define NV_B 		11

#define SCALE_OPTIONS

struct theory {
	int octave;
	int scale;
	int noteValue;
};

theory knobs = {0,0,60};
theory knobCache = {0,0,0};

void pollKnobs() {
	knobCache = knobs;
	octaveKnob();
	baseNoteKnob();
	scaleKnob();
	setBasenote();
	setNotes();
	if( (knobCache.octave != knobs.octave || knobCache.noteValue != knobs.noteValue ) && mode == 1) {
		modeTwoUpdate();
	}
}

void octaveKnob(){
	knobs.octave = NORMALIZE(analogRead(PIN_OCTAVE), 0, 1023, 0, 7);
}

void baseNoteKnob(){
	knobs.noteValue = NORMALIZE(analogRead(PIN_OCTAVE), 0, 1023, 0, 11);
}

void scaleKnob(){
	knobs.scale = NORMALIZE(analogRead(PIN_OCTAVE), 0, 1023, 0, 5);
}

void setBasenote(){
	baseNote = ( knobs.octave * 12 ) + knobs.noteValue;
	if(baseNote > MIDI_NOTE_HIGH) { baseNote = MIDI_NOTE_HIGH; }
	if(baseNote < MIDI_NOTE_LOW) { baseNote = MIDI_NOTE_LOW; }
}

void setNotes(){
	setScale();
}