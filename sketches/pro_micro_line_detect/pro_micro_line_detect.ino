#define NO_SERIAL_MSGS

const int PIN_L_CH_IN  = 9;
const int PIN_L_CH_LED = 3;

const int PIN_R_CH_IN  = 8;
const int PIN_R_CH_LED = 5;

const int PIN_MODE_BTN = 2;

const int PIN_L_MOTOR = 10;
const int PIN_R_MOTOR = 6;

enum SenseState {START, CALIBRATE_WHITE, READING_WHITE, CALIBRATE_BLACK, READING_BLACK, READY, GO, STOP};
volatile SenseState sense_state; 

// 
int left_ch_black = 1023;
int left_ch_white = 0;
int right_ch_black = 1023;
int right_ch_white = 0;

int left_ch_threshold = 512;
int right_ch_threshold = 512;

int left_ch_current_val = 0;
int right_ch_current_val = 0;

bool started_once = false;


void setup() {
  sense_state = START;
  pinMode(PIN_L_CH_IN, INPUT);
  pinMode(PIN_L_CH_LED, OUTPUT);
  pinMode(PIN_R_CH_IN, INPUT);
  pinMode(PIN_R_CH_LED, OUTPUT);

  pinMode(PIN_MODE_BTN, INPUT_PULLUP);

  pinMode(PIN_L_MOTOR, OUTPUT);
  pinMode(PIN_R_MOTOR, OUTPUT);

#ifndef NO_SERIAL_MSGS
  Serial.begin(9600); //This pipes to the serial monitor
#endif
}

void blink(int pin, int dutyCycle, int count) {
  for (int j = 0; j < count; ++j) {
    digitalWrite(pin,LOW);
    delay(dutyCycle/2);
    digitalWrite(pin,HIGH);
    delay(dutyCycle/2);
  }
}

void blinkStart() {
  digitalWrite(PIN_L_CH_LED,HIGH);
  digitalWrite(PIN_R_CH_LED,HIGH);

  blink(PIN_L_CH_LED, 800, 3);
  blink(PIN_R_CH_LED, 800, 3);
}

void blinkReady() {
  digitalWrite(PIN_L_CH_LED,LOW);
  digitalWrite(PIN_R_CH_LED,HIGH);
  delay(500);
  if (sense_state == GO) {
    return;
  }
  digitalWrite(PIN_L_CH_LED,HIGH);
  digitalWrite(PIN_R_CH_LED,LOW);
  delay(500);
}


void blinkCalibration() {
  digitalWrite(PIN_L_CH_LED, HIGH);
  digitalWrite(PIN_R_CH_LED, HIGH);
  
  for (int j = 224; j >= 0; j -= 32) {
    if ((sense_state == READING_WHITE) || (sense_state == READING_BLACK)) {
      return;
    }
    delay(80);
    analogWrite(PIN_L_CH_LED,j);
    analogWrite(PIN_R_CH_LED,j);
  }

  for (int j = 32; j < 256; j += 32) {
    if ((sense_state == READING_WHITE) || (sense_state == READING_BLACK)) {
      return;
    }
    delay(80);
    analogWrite(PIN_L_CH_LED,j);
    analogWrite(PIN_R_CH_LED,j);
  }

  delay(80);
  digitalWrite(PIN_L_CH_LED, HIGH);
  digitalWrite(PIN_R_CH_LED, HIGH);
}

void blinkDoneRead() {
  // longer blink
  digitalWrite(PIN_L_CH_LED,LOW);
  digitalWrite(PIN_R_CH_LED,LOW);
  delay(800);
  digitalWrite(PIN_L_CH_LED,HIGH);
  digitalWrite(PIN_R_CH_LED,HIGH);

  // short blink
  delay(200);
  digitalWrite(PIN_L_CH_LED,LOW);
  digitalWrite(PIN_R_CH_LED,LOW);
  delay(200);
  digitalWrite(PIN_L_CH_LED,HIGH);
  digitalWrite(PIN_R_CH_LED,HIGH);
  delay(300);

}

void loop() {
  switch (sense_state) {
    case START:
      blinkStart();
      if (!started_once) {
        attachInterrupt(digitalPinToInterrupt(PIN_MODE_BTN),buttonHandler,FALLING);
        started_once = true;
      } 
      sense_state = CALIBRATE_WHITE;
      break;
    case CALIBRATE_WHITE:
#ifndef NO_SERIAL_MSGS
      Serial.print("Waiting white ");
#endif
      reportSensorState();
      blinkCalibration();
      break;  
    case READING_WHITE:
      getBaselineReading();
      sense_state = CALIBRATE_BLACK;
      break;
    case CALIBRATE_BLACK:
#ifndef NO_SERIAL_MSGS
      Serial.print("Waiting black ");
#endif
      reportSensorState();
      blinkCalibration();
      break;
    case READING_BLACK:
      getBaselineReading();
      setThreshold();
#ifndef NO_SERIAL_MSGS
      Serial.print("Thresholds set: ");
      Serial.print(left_ch_threshold);
      Serial.print(", ");
      Serial.println(right_ch_threshold);
#endif
      sense_state = READY;
      break;
    case READY:
      blinkReady();
      break;
    case GO:
      reportSensorState();
      setStatusLEDs();
      setMotorSpeed();
      break;
    case STOP:
      digitalWrite(PIN_L_CH_LED,LOW);
      digitalWrite(PIN_R_CH_LED,LOW);
      digitalWrite(PIN_L_MOTOR,LOW);
      digitalWrite(PIN_R_MOTOR,LOW);
#ifndef NO_SERIAL_MSGS
      Serial.println("Stopped");
#endif
      delay(500);
      break;
  }
}

int readChannel(int ch_pin, int avg_count, int sample_interval) {
  int val = analogRead(ch_pin); 
  val = 0; // discard first reading
  for (int j = 0; j < avg_count; ++j) {
    delay(sample_interval);
    val += analogRead(ch_pin);
  }
  val /= avg_count;
  return val;
}

void getBaselineReading() {
  int const avg_count = 2;
  int const sample_delay_ms = 20;
  int const val_left = readChannel(PIN_L_CH_IN,avg_count,sample_delay_ms);
  int const val_right = readChannel(PIN_R_CH_IN,avg_count,sample_delay_ms);
 
  switch(sense_state) {
    case READING_WHITE:
      left_ch_white = val_left;
      right_ch_white = val_right;
#ifndef NO_SERIAL_MSGS
      Serial.print("Calibrated white: ");
#endif
      break;
    case READING_BLACK:
      left_ch_black = val_left;
      right_ch_black = val_right;
#ifndef NO_SERIAL_MSGS
      Serial.print("Calibrated black: ");
#endif
      break;
  }
#ifndef NO_SERIAL_MSGS
  Serial.print(val_left);
  Serial.print(", ");
  Serial.println(val_right);
#endif

  blinkDoneRead();

}

void setThreshold() {
  left_ch_threshold = (left_ch_black - left_ch_white)/2 + left_ch_white;
  right_ch_threshold = (right_ch_black - right_ch_white)/2  + right_ch_white;
}

void reportSensorState() {
  right_ch_current_val = readChannel(PIN_R_CH_IN,2,20);
  left_ch_current_val = readChannel(PIN_L_CH_IN,2,20);

#ifndef NO_SERIAL_MSGS
  Serial.print(left_ch_current_val);
  Serial.print(", ");
  Serial.println(right_ch_current_val);
#endif
}

void setStatusLEDs() {
  // white (PD active, input low) = LED off
  digitalWrite(PIN_L_CH_LED,left_ch_current_val < left_ch_threshold);
  digitalWrite(PIN_R_CH_LED,right_ch_current_val < right_ch_threshold);
}


void setMotorSpeed() {
  int const speed_val = 96;

  // white (PD active, input low) = opposite motor is on
  //   LEFT    | RIGHT   |  state
  //  ---------+---------+----------
  //   white   | white   | both driving
  //   white   | black   | right ON only
  //   black   | white   | left ON only
  //   black   | black   | both stop
   
  int const left_speed = (right_ch_current_val < right_ch_threshold ? speed_val : 0);
  int const right_speed = (left_ch_current_val < left_ch_threshold ? speed_val : 0);
  
  analogWrite(PIN_L_MOTOR,left_speed);
  analogWrite(PIN_R_MOTOR,right_speed);

  delay(50);
}


void buttonHandler() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time < 200) {
    return;
  }
 
  last_interrupt_time = interrupt_time;
  
  switch(sense_state) {
    case CALIBRATE_WHITE:
      sense_state = READING_WHITE;
      break;
    case CALIBRATE_BLACK:
      sense_state = READING_BLACK;
      break;
    case READY:
      sense_state = GO;
      break;
    case GO:
      sense_state = STOP;
      break;
    case STOP:
      sense_state = START;
      break;
  }
  
}
