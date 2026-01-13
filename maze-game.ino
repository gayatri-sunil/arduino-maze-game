// --- Pins (unchanged) ---
#define ANALOG_X_PIN A2 
#define ANALOG_Y_PIN A3 
#define ANALOG_BUTTON_PIN A4
#define SERVO_1_PIN 3
#define SERVO_2_PIN 4

#include <Servo.h>

Servo myservo1; // X servo on SERVO_1_PIN
Servo myservo2; // Y servo on SERVO_2_PIN

// ====================== Gentle ramp to EXACT same positions ======================
// Update cadence for servo outputs
const unsigned long UPDATE_PERIOD_MS = 20;   // 50 Hz

// Slew-rate limit: max change per UPDATE_PERIOD_MS (in servo .write() units)
const float MAX_STEP_PER_TICK_X = 0.9f;  // was 1
const float MAX_STEP_PER_TICK_Y = 0.9f; 

// Start EXACTLY at your neutral positions
float xSlew = 105;    // X neutral (matches your code)
float ySlew = 80;    // Y neutral (matches your code)
unsigned long lastUpdate = 0;

struct button { byte pressed = 0; };
struct analog_t { short x, y; button button; };

// ------------------ Forward decls ------------------
byte readAnalogAxisLevel(int pin);
bool isAnalogButtonPressed(int pin);
void gentleWriteExact(byte xTarget, byte yTarget);

// ----------- Recording / Playback with Run-Length Encoding -----------
struct Sample {
  byte x;        // servo angle (your xCmd: 105, 90, 110)
  byte y;        // servo angle (your yCmd: 80, 70, 90)
  byte duration; // number of "ticks" this state lasts
};

enum Mode { LIVE, RECORDING, PLAYBACK };
Mode mode = LIVE;

// Each Sample is 3 bytes -> 300 * 3 = 900 bytes (safe on Uno)
const int MAX_SAMPLES = 300;
Sample buffer[MAX_SAMPLES];
int recLen = 0;          // number of used entries in buffer
int paybackLength = 0;   // how many entries to play

// We record/playback in fixed "ticks"
const unsigned long TICK_MS = 20;
unsigned long lastRecordTick = 0;
unsigned long lastPlaybackTick = 0;

// Playback state
int playIndex = 0;          // which Sample we are playing
byte playTickInSample = 0;  // how many ticks we have spent in this Sample
byte playbackXCmd = 105;     // current playback command X
byte playbackYCmd = 80;     // current playback command Y

// Button edge detection + debounce
bool prevButton = false;
unsigned long lastButtonTime = 0;
const unsigned long DEBOUNCE_MS = 200;

// ------------------ Setup ------------------
void setup() 
{ 
  pinMode(ANALOG_BUTTON_PIN, INPUT_PULLUP); 
  myservo1.attach(SERVO_1_PIN);
  myservo2.attach(SERVO_2_PIN);
  Serial.begin(9600);

  // Set initial servo positions EXACTLY like your original (105, 80)
  myservo1.write((int)xSlew);
  myservo2.write((int)ySlew);

  lastRecordTick = millis();
  lastPlaybackTick = millis();
} 
	 
// ------------------ Main loop ------------------
void loop() 
{ 
  analog_t a; 
	
  a.x = readAnalogAxisLevel(ANALOG_X_PIN);
  a.y = readAnalogAxisLevel(ANALOG_Y_PIN);

  // EXACT same mapping as your reference
  short yCmd = map(a.y, 0, 40, 88, 72);
  short xCmd = map(a.x, 0, 40, 92, 108);


  a.button.pressed = isAnalogButtonPressed(ANALOG_BUTTON_PIN); 

  unsigned long now = millis();

  // ------------- Button edge detection (same mode sequence) -------------
  bool rising = a.button.pressed && !prevButton && (now - lastButtonTime >= DEBOUNCE_MS);
  prevButton = a.button.pressed;

  if (rising) {
    lastButtonTime = now;
    if (mode == LIVE) {
      // Start a new recording
      mode = RECORDING;
      recLen = 0;
      paybackLength = 0;
      lastRecordTick = now;
      Serial.println("LIVE -> RECORDING");
    } else if (mode == RECORDING) {
      // Stop recording, start playback
      mode = PLAYBACK;
      paybackLength = recLen;
      recLen = 0;
      playIndex = 0;
      playTickInSample = 0;
      lastPlaybackTick = now;
      if (paybackLength > 0) {
        playbackXCmd = buffer[0].x;
        playbackYCmd = buffer[0].y;
      }
      Serial.println("RECORDING -> PLAYBACK");
    }
  }

  // ------------- Recording logic (RLE) -------------
  if (mode == RECORDING) {
    Serial.print("RECORDING len=");
    Serial.println(recLen);

    // Record in discrete ticks
    if (now - lastRecordTick >= TICK_MS) {
      lastRecordTick += TICK_MS;

      if (recLen == 0) {
        // first sample
        buffer[0].x = (byte)xCmd;
        buffer[0].y = (byte)yCmd;
        buffer[0].duration = 1;
        recLen = 1;
      } else {
        Sample &last = buffer[recLen - 1];

        // If same command and duration not maxed, just extend duration
        if (last.x == (byte)xCmd && last.y == (byte)yCmd && last.duration < 255) {
          last.duration++;
        } else {
          // Need a new entry
          if (recLen < MAX_SAMPLES) {
            buffer[recLen].x = (byte)xCmd;
            buffer[recLen].y = (byte)yCmd;
            buffer[recLen].duration = 1;
            recLen++;
          } else {
            // Buffer full: you can auto-switch to playback if desired
            Serial.println("BUFFER FULL during RECORDING");
          }
        }
      }
    }
  }

  // ------------- Playback logic (RLE) -------------
  if (mode == PLAYBACK) {
    Serial.print("PLAYBACK index=");
    Serial.print(playIndex);
    Serial.print(" / ");
    Serial.println(paybackLength);

    if (playIndex < paybackLength) {
      if (now - lastPlaybackTick >= TICK_MS) {
        lastPlaybackTick += TICK_MS;

        Sample &s = buffer[playIndex];

        // Command is the sample's x,y
        playbackXCmd = s.x;
        playbackYCmd = s.y;

        playTickInSample++;

        if (playTickInSample >= s.duration) {
          playIndex++;
          playTickInSample = 0;
        }
      }
    } else {
      // Finished playback -> go back to LIVE
      mode = LIVE;
      Serial.println("PLAYBACK finished -> LIVE");
    }
  }

  // ------------- Decide what to send to servos -------------
  byte servoXCmd;
  byte servoYCmd;

  if (mode == PLAYBACK) {
    servoXCmd = playbackXCmd;
    servoYCmd = playbackYCmd;
  } else {
    // LIVE or RECORDING: follow joystick mapping
    servoXCmd = (byte)xCmd;
    servoYCmd = (byte)yCmd;
  }

  // Always drive servos gently toward the chosen command
  gentleWriteExact(servoXCmd, servoYCmd);

  // Small delay is OK; motion timing handled by UPDATE_PERIOD_MS
  delay(5); 
} 
	 
// ------------------ Helpers ------------------
byte readAnalogAxisLevel(int pin) 
{ 
  return map(analogRead(pin), 10, 1023, 0, 40);
} 
	 
bool isAnalogButtonPressed(int pin) 
{ 
  byte val = map(analogRead(pin), 0, 1023, 0, 255);
  return val < 128; 
} 

// Slew-only: ramp to target without changing final value
void gentleWriteExact(byte xTarget, byte yTarget) {
  auto slewStep = [](float current, float target, float maxStep) -> float {
    float delta = target - current;
    if (delta >  maxStep) delta =  maxStep;
    if (delta < -maxStep) delta = -maxStep;
    return current + delta;
  };

  static int lastOutX = -1000;
  static int lastOutY = -1000;

  unsigned long now = millis();
  if (now - lastUpdate >= UPDATE_PERIOD_MS) {
    lastUpdate = now;

    xSlew = slewStep(xSlew, (float)xTarget, MAX_STEP_PER_TICK_X);
    ySlew = slewStep(ySlew, (float)yTarget, MAX_STEP_PER_TICK_Y);

    int outX = (int)round(xSlew);
    int outY = (int)round(ySlew);

    // Only write to the servo if the integer value actually changed
    if (outX != lastOutX) {
      myservo1.write(outX);
      lastOutX = outX;
    }
    if (outY != lastOutY) {
      myservo2.write(outY);
      lastOutY = outY;
    }
  }
}
