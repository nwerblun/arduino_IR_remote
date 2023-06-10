#include <EEPROM.h>
#include <CircularBuffer.h>

#define CHANNEL_0_PIN 9
#define CHANNEL_1_PIN 10
#define IR_REC_RD_PIN 19
#define NUM_CHANNELS 2

#define TOP F_CPU / (2 * 256 * 50)

#define BUFFER_SIZE 960
#define SAMPLE_BUFFER_SIZE int(1.5 * BUFFER_SIZE / (8335 / 1667))

#define PLL_STEP_SIZE 65536 / int(8335 / 1667)  // 2^16 / (fs / bit rate) where bit rate is 1/600us assuming bit period is 600us
#define FILL_BUFF_SAMP_PERIOD_US 120  // 8.335kHz sample rate
// use built-in LED for cal

/*
Packet Structure:
|------START FLAG-------|-CALIBRATE?-|-CHANNEL (MOTOR)-|--CAL UP/DN--|------END FLAG------|
| (01010101)x?+11111111 |     X      |      XXXX       |     XXXX    | 1111100000111111x2 |
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
  byte updn;
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
void parsePacket(Packet p) {
  if ((p.channel < 0) || (p.channel > 10)) {
    return;
  }
  
  if (p.calibrating) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (p.calibrating && p.updn == 2 && p.channel == 0) {
    Serial.println("Cal channel 0 down");
    int temp = OCR1A - stepSize;
    OCR1A = (temp < 0) ? 0 : temp;
    // EEPROM is byte addressed. Correct address per channel should be channel * sizeof(uint16_t) since OCR1A is 16 bits
    // EEPROM.put(0, OCR1A);
  } else if (p.calibrating && p.updn == 3 && p.channel == 0) {
    Serial.println("Cal channel 0 up");
    int temp = OCR1A + stepSize;
    OCR1A = (temp >= TOP) ? TOP : temp;
    // EEPROM.put(0, OCR1A);
  } else if (p.calibrating && p.updn == 2 && p.channel == 1) {
    Serial.println("Cal channel 1 down");
    int temp = OCR1B - stepSize;
    OCR1B = (temp < 0) ? 0 : temp;
    // EEPROM.put(sizeof(uint16_t), OCR1B);
  } else if (p.calibrating && p.updn == 3 && p.channel == 1) {
    Serial.println("Cal channel 1 up");
    int temp = OCR1B + stepSize;
    OCR1B = (temp >= TOP) ? TOP : temp;
    // EEPROM.put(sizeof(uint16_t), OCR1B);
  } else if (p.channel == 0 && !p.calibrating) {
    Serial.println("Actuating channel 0");
    int prev = OCR1A;
    int temp = prev + (50 * stepSize);
    OCR1A = (temp >= TOP) ? TOP : temp;
    delay(1500);
    OCR1A = prev;
  } else if (p.channel == 1 && !p.calibrating) {
    Serial.println("Actuating channel 1");
    int prev = OCR1B;
    int temp = prev + (50 * stepSize);
    OCR1B = (temp >= TOP) ? TOP : temp;
    delay(1500);
    OCR1B = prev;
  }
}

void fillBuffer(byte* x, int len) {
  int sample;
  int cnt = 0;
  while (cnt < len) {
    sample = !digitalRead(IR_REC_RD_PIN);
    *(x+cnt) = byte(sample);
    delayMicroseconds(FILL_BUFF_SAMP_PERIOD_US);
    cnt++;
  }
}

void printBuffer(byte* x) {
  Serial.println("Buffer Contents: ");
  for (int i = 0; i < BUFFER_SIZE; i++) {
    Serial.print(*(x+i));
  }
  Serial.println();
}

void printSampleBuffer(CircularBuffer<byte, SAMPLE_BUFFER_SIZE>* x) {
  Serial.print("Sample Buffer Contents with size ");
  Serial.print(x->size());
  Serial.println(":");
  for (int i = 0; i < x->size(); i++) {
    Serial.print((*x)[i]);
    Serial.print(" ");
  }
  Serial.println();
}

bool consumeStartFlag(CircularBuffer<byte, SAMPLE_BUFFER_SIZE>* s) {
  int size = s->size();
  byte onesCount = 0;
  for (int i = 0; i < size; i++) {
    if (onesCount == 4) {
      return true;
    }
    if (s->first() == 1) {
      onesCount++;
    } else {
      onesCount = 0;
    }
    s->shift();
  }
  return false;
}

bool consumeEndFlag(CircularBuffer<byte, SAMPLE_BUFFER_SIZE>* s) {
  int size = s->size();
  byte onesCount = 0;
  byte zerosCount = 0;
  bool lookingForOnes = false;
  for (int i = 0; i < size; i++) {
    if (zerosCount == 2 && !lookingForOnes) {
      lookingForOnes = true;
    } else if (onesCount == 14 && lookingForOnes) {
      return true;
    }

    if (s->first() == 1) {
      onesCount++;
      zerosCount = 0;
    } else {
      zerosCount++;
      onesCount = 0;
    }
    s->shift();
  }
  return false;
}

Packet consumeData(CircularBuffer<byte, SAMPLE_BUFFER_SIZE>* s) {
  bool calibrating = bool(s->shift());
  int samp;
  int breakBit;
  byte ch;
  byte updn;
  Packet p;
  for (int i = 0; i < 9; i++) {
    if (i > 0 && i % 2 == 0) {
      breakBit = s->shift();
      if (breakBit == samp) {
        p.channel = 10;
        p.calibrating = false;
        p.updn = 1;
        return p;
      }
    }
    if (i < 8) {
      samp = s->shift();
      bitWrite(updn, i, samp);
    }
  }

  for (int i = 0; i < 9; i++) {
    if (i > 0 && i % 2 == 0) {
      breakBit = s->shift();
      if (breakBit == samp) {
        p.channel = 10;
        p.calibrating = false;
        p.updn = 1;
        return p;
      }
    }
    if (i < 8) {
      samp = s->shift();
      bitWrite(ch, i, samp);
    }
  }
  p.channel = ch;
  p.updn = updn;
  p.calibrating = calibrating;
  return p;
}

void getSamplePoints(byte* x, int len, CircularBuffer<byte, SAMPLE_BUFFER_SIZE>* s) {
  // DireWolf SDR software PLL implementation based on an overflowing counter
  int cntr = 0;
  int prevCntr = 0;
  bool edge = false;
  float alpha = 0.5;
  for (int i = 1; i < len; i++) {
    edge = *(x+i) != *(x+(i-1));
    alpha = (edge) ? 0.6 : 1;
    cntr = (prevCntr * alpha) + PLL_STEP_SIZE;
    if (cntr < 0 && prevCntr >= 0) {
      // Overflow, take samp
      s->push(*(x+i));
    }
    prevCntr = cntr;
  }
}

void printPacket(Packet p) {
  Serial.print("Cal?: ");
  Serial.println(p.calibrating);
  Serial.print("Ch: ");
  Serial.println(p.channel);
  Serial.print("updn: ");
  Serial.println(p.updn);
}

int samp;
Packet recPacket;
byte* sampleBuffer;
CircularBuffer<byte, SAMPLE_BUFFER_SIZE> sampledPointsBuffer;
void loop() {
  // Samples are active low
  samp = !digitalRead(IR_REC_RD_PIN);
  if (samp) {
    Serial.println("---------");
    // ----initialize----
    sampledPointsBuffer.clear();
    sampleBuffer = (byte*) calloc(BUFFER_SIZE, sizeof(byte));
    // -----take samples-----
    fillBuffer(sampleBuffer, BUFFER_SIZE);
    // printBuffer(sampleBuffer);
    // -------Run PLL on sample points and fill sample point buffer----
    getSamplePoints(sampleBuffer, BUFFER_SIZE, &sampledPointsBuffer);
    // printSampleBuffer(&sampledPointsBuffer);
    // ----clear sample buffer mem-------
    free(sampleBuffer);
    // ------Check if flag exists and if so, build data
    if (consumeStartFlag(&sampledPointsBuffer)) {
      Serial.println("We got a valid start");
      // printSampleBuffer(&sampledPointsBuffer);
      recPacket = consumeData(&sampledPointsBuffer);
      // printSampleBuffer(&sampledPointsBuffer);
    } else {
      return;
    }
    // -----Check if end flag exists and do something------
    if (consumeEndFlag(&sampledPointsBuffer)) {
      Serial.println("End found!");
      // printPacket(recPacket);
      parsePacket(recPacket);
    }
  }
}
