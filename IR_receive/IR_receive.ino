#include <EEPROM.h>
#include <CircularBuffer.h>

#define CHANNEL_0_PIN 9
#define CHANNEL_1_PIN 10
#define IR_REC_RD_PIN 19
#define tPwlMicros 900
#define tPwhMicros 600
#define SAMPLE_WAIT_TIME_MICROS 350
#define TOP F_CPU / (2 * 256 * 50)
#define NUM_CHANNELS 2
// use built-in LED for cal

/*
Packet Structure:
|--------START FLAG---------|-CALIBRATE?-|-CHANNEL (MOTOR)-|--CAL UP/DN--|------END FLAG------|
| (010101010101)x2+11111111 |     X      |      XXXX       |     XXXX    | 1111100000111111x2 |
calibrate = 1 if calibrating, 0 if actuating
cal up/dn = 0/3 if neither, 1 if dn, 2 if up
channel = 0-9, specifies which motor to use. Numbers >9 reserved

If cal button is pressed, don't do anything except light an LED
If cal is selected and a channel is pressed, send a packet with cal=1, channel=channel and cal up/dn = 0
If cal is selected, channel is selected and up/dn is pressed, send a packet with cal=1, channel=channel and cal up/dn = 1/2
If cal is pressed again, send a packet with cal=0, channel=10, cal up/dn = 0 to specify done calibrating

If cal is not selected and channel is pressed send cal=0, channel=channel, cal_up/dn = 0
*/

struct Packet {
  bool calibrating;
  byte channel;
  byte up_dn;
};

void setup() {
  /*
    _______________ Top = ICR1
    /\    /
   /  \  /
   _________________ Comparison threshold = OCR1A
  /    \/
  Internal timer counts up to ICR1 (if set to use ICR1 as TOP, otherwise 255) then turns around. When it collides with OCR1A, it does something to the output based on settings in TCCR1A/B

  Set up timer 1 here. Timer 0 is used for delay() so don't touch that one
  Timer 1 controls pins 9/10 which I think are referred to as channel A and B respectively
  base Arduino cpu freq is 16MHz and is stored in F_CPU
  Allowed divisors for pins 9/10 are 1,8,64,256,1024

  set ICR1 = F_CPU / (2 * prescalar * desired_pwm_freq)
    F_CPU' = F_CPU / prescalar
    X = F_CPU' / desired_freq = how many cycles of the divided CPU freq does it take to reach 1 FULL cycle of the desired freq (high and low combined)
    X = X / 2 since we want the output to toggle halfway for a perfect 50% duty cycle
    
  set COM1A1 and COM1B1 to 1 so that the output is set low when up counting and high when down-counting and the threshold is met by the sawtooth wave
  set OCR1A = desired duty cycle as a % of ICR1. E.g. for 1000Hz, OCR1A = 450 gives 45% DC
  set OCR1B = desired dc
  */
  Serial.begin(9600);
  pinMode(CHANNEL_0_PIN, OUTPUT);
  pinMode(CHANNEL_1_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(IR_REC_RD_PIN, INPUT);

  // Zero out default settings, keep only reserved/required bits
  TCCR1B &= 0b11111000;
  TCCR1A &= 0b10100000;

  // Set prescalar to 64 (CS12/11/10 = 0, 1, 1)
  TCCR1B |= (1 << CS11) | (1 << CS10);
  // Set PWM mode to phase correct + use ICR1 register as top
  TCCR1B |= (1 << WGM13);
  TCCR1A |= (1 << WGM11);
  // Set top to be the proper value to get the freq we want
  ICR1 = TOP;

  // Set behavior of output to go high/low appropriately
  TCCR1A |= (1 << COM1A1);
  TCCR1A |= (1 << COM1B1);
  // Set duty cycle. Default to 1.5% for both channels (20ms period, 1.5ms pulse)
  OCR1A = int(0.075 * TOP);
  OCR1B = int(0.075 * TOP);

  // uint16_t ch0StoredDC;
  // uint16_t ch1StoredDC;
  // OCR1A = EEPROM.get(0, ch0StoredDC);
  // OCR1B = EEPROM.get(sizeof(uint16_t), ch1StoredDC);
}

int stepSize = TOP * 0.0005;
void parsePacket(bool calibrating, byte channel, byte up_dn) {
  if ((channel < 0) || (channel >= NUM_CHANNELS)) {
    return;
  }
  
  if (calibrating) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (calibrating && up_dn == 1 && channel == 0) {
    int temp = OCR1A - stepSize;
    OCR1A = (temp < 0) ? 0 : temp;
    // EEPROM is byte addressed. Correct address per channel should be channel * sizeof(uint16_t) since OCR1A is 16 bits
    // EEPROM.put(0, OCR1A);
  } else if (calibrating && up_dn == 2 && channel == 0) {
    int temp = OCR1A + stepSize;
    OCR1A = (temp >= TOP) ? TOP : temp;
    // EEPROM.put(0, OCR1A);
  } else if (calibrating && up_dn == 1 && channel == 1) {
    int temp = OCR1B - stepSize;
    OCR1B = (temp < 0) ? 0 : temp;
    // EEPROM.put(sizeof(uint16_t), OCR1B);
  } else if (calibrating && up_dn == 2 && channel == 1) {
    int temp = OCR1B + stepSize;
    OCR1B = (temp >= TOP) ? TOP : temp;
    // EEPROM.put(sizeof(uint16_t), OCR1B);
  } else if (channel == 0) {
    int prev = OCR1A;
    int temp = prev + (50 * stepSize);
    OCR1A = (temp >= TOP) ? TOP : temp;
    delay(1500);
    OCR1A = prev;
  } else if (channel == 1) {
    int prev = OCR1B;
    int temp = prev + (50 * stepSize);
    OCR1B = (temp >= TOP) ? TOP : temp;
    delay(1500);
    OCR1B = prev;
  }
}

byte bitArrToByte(int x[]) {
  const int arrLen = sizeof(x) / sizeof(x[0]);
  if (arrLen != 4) {
    return 0;
  }

  byte res = 0;
  bitWrite(res, 0, x[3]);
  bitWrite(res, 1, x[2]);
  bitWrite(res, 2, x[1]);
  bitWrite(res, 3, x[0]);
  return res;
}

bool allOnes(CircularBuffer<int, 8>* x) {
  int len = x->size();
  for (int i = 0; i < len; i++) {
    if (!(*x)[i]) {
      return false;
    }
  }
  return true;
}

bool endFlagInBuffer(CircularBuffer<int, 16>* x) {
  for (int i = 0; i < 6; i++) {
    if (!(*x)[i]) {
      return false;
    }
  }

  for (int i = 0; i < 5; i++) {
    if ((*x)[i]) {
      return false;
    }
  }

  for (int i = 0; i < 5; i++) {
    if (!(*x)[i]) {
      return false;
    }
  }
  return true;
}

int start_flag_first = 0b1010101010101010;
int start_flag_remainder = 0b1111111110101010;
int end_flag_half = 0b1111100000111111;
int rec_pin_rd;
CircularBuffer<int, 8> startBuff;
CircularBuffer<int, 16> endBuff;
String state = "wait";
int channSamp[4] = {0, 0, 0, 0};
int updnSamp[4] = {0, 0, 0, 0};
bool calibrate = false;
int counter = 0;
int timesEndFlagSeen = 0;
int sample0, sample1, sample2;  // VS1838B is active low, 0 = high
void loop() {
  if (state == "wait") {
    sample0 = !digitalRead(IR_REC_RD_PIN);
    delayMicroseconds(SAMPLE_WAIT_TIME_MICROS);
    sample1 = !digitalRead(IR_REC_RD_PIN);
    delayMicroseconds(SAMPLE_WAIT_TIME_MICROS);
    sample2 = !digitalRead(IR_REC_RD_PIN);
    
    Serial.print("Sample 0/1/2: ");
    Serial.print(sample0);
    Serial.print(" / ");
    Serial.print(sample1);
    Serial.print(" / ");
    Serial.print(sample2);
    Serial.println();

    if (sample0 || sample1 || sample2) {
      state = "lock";
      counter = 0;
    }
  } else if (state == "lock") {
      if (counter > 15) {
        Serial.println("Timed out waiting for lock");
        state = "wait";
      }
      sample0 = !digitalRead(IR_REC_RD_PIN);
      delayMicroseconds(SAMPLE_WAIT_TIME_MICROS);
      sample1 = !digitalRead(IR_REC_RD_PIN);
      delayMicroseconds(SAMPLE_WAIT_TIME_MICROS);
      sample2 = !digitalRead(IR_REC_RD_PIN);
      if (!sample0 && !sample1 && sample2) {
        Serial.println("Early");
        delayMicroseconds(200);
      } else if (!sample0 && sample1 && sample2) {
        Serial.println("A little too early");
        delayMicroseconds(100);
      } else if (!sample0 && sample1 && sample2) {
        Serial.println("A little too early");
        delayMicroseconds(100);
      } else if (!sample0 && sample1 && !sample2) {
        Serial.println("Locked!");
        delayMicroseconds(SAMPLE_WAIT_TIME_MICROS);
        state = "looking";
        startBuff.clear();
        counter = 0;
      } else if (sample0 && !sample1 && !sample2) {
        Serial.println("Late");
      } else {
        Serial.println("Detected 111 which shouldn't happen");
      }
      counter++;

  } else if (state == "look") {
      if (counter > 24) {
        Serial.println("Timed out looking for start flag");
        state = "wait";
      }
      sample0 = !digitalRead(IR_REC_RD_PIN);
      startBuff.unshift(sample0);
      if (sample0) {
        delayMicroseconds(tPwhMicros);
      } else {
        delayMicroseconds(tPwlMicros);
      }
      if (allOnes(&startBuff)) {
        Serial.println("Flag found, packet time");
        state = "build";
        memset(channSamp, 0, sizeof(channSamp));
        memset(updnSamp, 0, sizeof(updnSamp));

      }
      counter++;
  } else if (state == "build") {
      sample0 = !digitalRead(IR_REC_RD_PIN);
      calibrate = bool(sample0);
      if (calibrate) {
        delayMicroseconds(tPwhMicros);
      } else {
        delayMicroseconds(tPwlMicros);
      }

      for (int i = 0; i < 4; i++) {
        sample0 = !digitalRead(IR_REC_RD_PIN);
        channSamp[i] = sample0;
        if (sample0) {
          delayMicroseconds(tPwhMicros);
        } else {
          delayMicroseconds(tPwlMicros);
        }
      }

      for (int i = 0; i < 4; i++) {
        sample0 = !digitalRead(IR_REC_RD_PIN);
        updnSamp[i] = sample0;
        if (sample0) {
          delayMicroseconds(tPwhMicros);
        } else {
          delayMicroseconds(tPwlMicros);
        }
      }
    
      Serial.println("Found all data");
      state = "ending";
      endBuff.clear();
      counter = 0;
      timesEndFlagSeen = 0;
  } else if (state == "ending") {
    if (counter > 24) {
      Serial.println("Timed out waiting for end flag");
      state = "wait";
    }
    sample0 = !digitalRead(IR_REC_RD_PIN);
    endBuff.unshift(sample0);
    if (sample0) {
      delayMicroseconds(tPwhMicros);
    } else {
      delayMicroseconds(tPwlMicros);
    }

    if (endFlagInBuffer(&endBuff)) {
      timesEndFlagSeen++;
    }

    if (timesEndFlagSeen == 2) {
      state = "wait";
      parsePacket(calibrate, bitArrToByte(channSamp), bitArrToByte(updnSamp));
    }
    counter++;
  }
}
