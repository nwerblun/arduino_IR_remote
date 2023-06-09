#include <EEPROM.h>
#include <CircularBuffer.h>

#define CHANNEL_0_PIN 9
#define CHANNEL_1_PIN 10
#define IR_REC_RD_PIN 19
#define NUM_CHANNELS 2

#define TOP F_CPU / (2 * 256 * 50)

#define BUFFER_SIZE 1200
#define FILL_BUFF_SAMP_PERIOD_US 150
#define START_FLAG_TIMEOUT_CYCLES 500
#define END_FLAG_TIMEOUT_CYCLES 500
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

  if (p.calibrating && p.updn == 1 && p.channel == 0) {
    Serial.println("Cal channel 0 down");
    int temp = OCR1A - stepSize;
    OCR1A = (temp < 0) ? 0 : temp;
    // EEPROM is byte addressed. Correct address per channel should be channel * sizeof(uint16_t) since OCR1A is 16 bits
    // EEPROM.put(0, OCR1A);
  } else if (p.calibrating && p.updn == 2 && p.channel == 0) {
    Serial.println("Cal channel 0 up");
    int temp = OCR1A + stepSize;
    OCR1A = (temp >= TOP) ? TOP : temp;
    // EEPROM.put(0, OCR1A);
  } else if (p.calibrating && p.updn == 1 && p.channel == 1) {
    Serial.println("Cal channel 1 down");
    int temp = OCR1B - stepSize;
    OCR1B = (temp < 0) ? 0 : temp;
    // EEPROM.put(sizeof(uint16_t), OCR1B);
  } else if (p.calibrating && p.updn == 2 && p.channel == 1) {
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

void fillBuffer(CircularBuffer<bool, BUFFER_SIZE>* x) {
  int samp;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    samp = !digitalRead(IR_REC_RD_PIN);
    x->push(bool(samp));
    delayMicroseconds(FILL_BUFF_SAMP_PERIOD_US);
  }
}

int getOneLength(CircularBuffer<bool, BUFFER_SIZE>* x) {
  int startInd = 0;
  int endInd = 0;
  bool waitingForNegEdge = false;
  bool waitingForPosEdge = true;
  bool prevSample = true;
  bool currSample;
  for (int i = 0; i < 100; i++) {
    currSample = x->first();
    // 0 -> 1
    if (!prevSample && currSample && waitingForPosEdge) {
      startInd = i;
      waitingForNegEdge = true;
      waitingForPosEdge = false;
    // 1 -> 0
    } else if (prevSample && !currSample && waitingForNegEdge) {
      endInd = i;
      return endInd - startInd;
    }
    x->shift();
    prevSample = currSample;
  }
  return -1;
}

int getZeroLength(CircularBuffer<bool, BUFFER_SIZE>* x) {
  int startInd = 0;
  int endInd = 0;
  bool waitingForNegEdge = true;
  bool waitingForPosEdge = false;
  bool prevSample = false;
  bool currSample;
  for (int i = 0; i < 100; i++) {
    currSample = x->first();
    // 1 -> 0
    if (prevSample && !currSample && waitingForNegEdge) {
      startInd = i;
      waitingForNegEdge = false;
      waitingForPosEdge = true;
    // 0 -> 1
    } else if (!prevSample && currSample && waitingForPosEdge) {
      endInd = i;
      return endInd - startInd;
    }
    
    x->shift();
    prevSample = currSample;
  }
  return -1;
}

bool popOneVal(CircularBuffer<bool, BUFFER_SIZE>* x, byte minO, byte minZ, bool consumeMax) {
  // Guaranteed that max-min is 1
  bool first = x->first();
  bool found = false;
  int relevantMin = (first) ? minO : minZ;
  int maxInd = x->size();
  int counter = 0;
  if (maxInd < relevantMin) {
    // Serial.println("There's fewer than the min samples left. Clearing. This is probably an error.");
    x->clear();
    return first;
  }

  while (!found && (counter < maxInd)) {
    if ((*x)[counter] != first) {
      found = true;
    }
    counter++;
  }

  counter--; // Final is where the next bit starts, so the current ends 1 before.
  // Clean X -> Y
  if (found && counter == relevantMin) {
    for (int i = 0; i < relevantMin; i++) {
      x->shift();
    }
    return first;
  } else if (found && (counter == relevantMin + 1)) {
    for (int i = 0; i < relevantMin + 1; i++) {
      x->shift();
    }
    return first;
  } 
  
  if (found && (counter > (relevantMin + 1))) {
    int amt = (consumeMax) ? relevantMin + 1 : relevantMin;
    for (int i = 0; i < amt; i++) {
      // Serial.print(x->first());
      x->shift();
      counter--;
    }
  }
  
  if (found && (counter < relevantMin)) {
    for (int i = 0; i < counter; i++) {
      x->shift();
    }
    return first;
  } else if (found) {
    return first;
  }

  // There's nothing left after this one. 
  if (!found && (maxInd <= (relevantMin + 1))) {
    x->clear();
    return first;
  }
  
  // There's at least 1 more value but everything until the end is all the same value, take the min. I think it'll work out since max-min = 1
  if (!found && (maxInd > (relevantMin + 1))) {
    int amt = (consumeMax) ? relevantMin + 1 : relevantMin;
    for (int i = 0; i < amt; i++) {
      x->shift();
    }
    return first;
  }
}

bool flagSearch(CircularBuffer<bool, BUFFER_SIZE>* x, byte minO, byte minZ) {
  Serial.println("Searching for start flag");
  int onesCount = 0;
  int count = 0;
  bool currSample;
  while (!(x->isEmpty()) && count < START_FLAG_TIMEOUT_CYCLES) {
    if (onesCount == 4) {
      // Serial.println("Found it!");
      return true;
    }

    currSample = popOneVal(x, minO, minZ, true);
    if (currSample) {
      onesCount++;
    } else {
      onesCount = 0;
    }
    count++;
  }
  // Serial.println("Not found");
  return false;
}

bool endFlagSearch(CircularBuffer<bool, BUFFER_SIZE>* x, byte minO, byte minZ) {
  byte onesCount = 0;
  byte zerosCount = 0;
  byte count = 0;
  bool currSample;
  bool fourOnes = false;
  bool nineOnes = false;
  bool threeZeros = false;
  // printBufferContents(x);
  while (!(x->isEmpty()) && count < END_FLAG_TIMEOUT_CYCLES) {
    if (onesCount == 4 && !fourOnes && !nineOnes && !threeZeros) {
      Serial.println("Found 4 1s");
      fourOnes = true;
      onesCount = 0;
      zerosCount = 0;
      // printBufferContents(x);
    } else if (zerosCount == 3 && fourOnes && !nineOnes && !threeZeros) {
      Serial.println("Found 4 0s");
      threeZeros = true;
      onesCount = 0;
      zerosCount = 0;
      // printBufferContents(x);
    } else if (onesCount == 9 && fourOnes && threeZeros && !nineOnes) {
      Serial.println("Found 9 1s");
      nineOnes = true;
      onesCount = 0;
      zerosCount = 0;
    }

    if (fourOnes && threeZeros && nineOnes) {
      return true;
    }

    currSample = popOneVal(x, minO, minZ, !(fourOnes && threeZeros));
    if (currSample) {
      zerosCount = 0;
      onesCount++;
    } else {
      onesCount = 0;
      zerosCount++;
    }
    count++;
  }
  return false;
}

Packet constructPacket(CircularBuffer<bool, BUFFER_SIZE>* x, byte minO, byte minZ) {
  bool cal;
  bool samp;
  // These print statements make it more stable??
  // printBufferContents(x);
  // cal is most often 0, consume max
  samp = popOneVal(x, minO, minZ, true);
  cal = samp;
  // If samp == 1, then the break will be a 0.
  samp = popOneVal(x, minO, minZ, samp);

  if (samp == cal) {
    Serial.println("Break after cal. bit not found");
    Packet res;
    res.calibrating = 0;
    res.channel = 10;
    res.updn = 0;
    return res;
  }
  
  byte ch;
  for (int i = 0; i < 8; i++) {
    if (i > 0 && i % 2 == 0) {
      samp = popOneVal(x, minO, minZ, false);
      if (bool(bitRead(ch, i-1)) == samp) {
        Serial.println("Break in ch bits not found");
        Packet res;
        res.calibrating = 0;
        res.channel = 10;
        res.updn = 0;
        return res;
      }
    }
    samp = popOneVal(x, minO, minZ, false);
    bitWrite(ch, i, int(samp));
  }

  byte updn;
  for (int i = 0; i < 8; i++) {
    if (i > 0 && i % 2 == 0) {
      samp = popOneVal(x, minO, minZ, false);
      if (bool(bitRead(updn, i-1)) == samp) {
        Serial.println("Break in updn bits not found");
        Packet res;
        res.calibrating = 0;
        res.channel = 10;
        res.updn = 0;
        return res;
      }
    }
    samp = popOneVal(x, minO, minZ, false);
    bitWrite(updn, i, int(samp));
  }

  Packet res;
  res.calibrating = cal;
  res.channel = ch;
  res.updn = updn;
  return res;
}

void printBufferContents(CircularBuffer<bool, BUFFER_SIZE>* x) {
  for (int i = 0; i < x->size(); i++) {
    Serial.print((*x)[i]);
  }
    Serial.println();
}

CircularBuffer<bool, BUFFER_SIZE> sampleBuffer;
int samp;
byte oneLength;
byte minOneLength = 1000;
byte maxOneLength = 0;
byte zeroLength;
byte minZeroLength = 1000;
byte maxZeroLength = 0;
bool flagFound;
bool foundEnd;
Packet recPacket;
void loop() {
  // Samples are active low
  samp = !digitalRead(IR_REC_RD_PIN);
  if (samp) {
    Serial.println("---------");
    sampleBuffer.clear();
    minZeroLength = 1000;
    maxZeroLength = 0;
    minOneLength = 1000;
    maxOneLength = 0;
    fillBuffer(&sampleBuffer);

    // ----Find min/max length of a 1 sample---
    for (int i = 0; i < 4; i++) {
      oneLength = getOneLength(&sampleBuffer);

      if (oneLength <= 0) {
        Serial.println("Went through everything and didn't find a 1???");
        return;
      }

      if (oneLength > maxOneLength) {
        maxOneLength = oneLength;
      }

      if (oneLength < minOneLength) {
        minOneLength = oneLength;
      }
    }
    // Serial.print("Min 1 length in samples: ");
    // Serial.println(minOneLength);
    // Serial.print("Max 1 length in samples: ");
    // Serial.println(maxOneLength);

    // ----Find min/max length of a 0 sample---
    for (int i = 0; i < 4; i++) {
      zeroLength = getZeroLength(&sampleBuffer);

      if (zeroLength <= 0) {
        // Serial.println("Went through everything and didn't find a 0???");
        return;
      }

      if (zeroLength > maxZeroLength) {
        maxZeroLength = zeroLength;
      }

      if (zeroLength < minZeroLength) {
        minZeroLength = zeroLength;
      }
    }
    // Serial.print("Min 0 length in samples: ");
    // Serial.println(minZeroLength);
    // Serial.print("Max 0 length in samples: ");
    // Serial.println(maxZeroLength);

    // ------ASSERTIONS-----
    if ((maxOneLength - minOneLength) > 1) {
      // Serial.println("Too much variation in 1 length");
      return;
    }

    if ((maxZeroLength - minZeroLength) > 1) {
      // Serial.println("Too much variation in 0 length");
      return;
    }

    
    if (minOneLength == 1) {
      // Serial.println("1 length is 1 sample. Too small to disambiguate");
      return;
    }

    if (minZeroLength == 1) {
      // Serial.println("0 length is 1 sample. Too small to disambiguate");
      return;
    }

    
    // if (DEBUG_PRINT) {
    //   Serial.println("Buffer contents post 'locking': ");
    // //   printBufferContents(&sampleBuffer);
    // }

    // -----Search remaining buffer vals for the flag----
    flagFound = flagSearch(&sampleBuffer, minOneLength, minZeroLength);
    if (!flagFound) {
      return;
    }

    // if (DEBUG_PRINT) {
    //   Serial.println("Buffer contents post start flag: ");
    // //   printBufferContents(&sampleBuffer);
    // }

    // ------- Build packet----------
    recPacket = constructPacket(&sampleBuffer, minOneLength, minZeroLength);
    Serial.println("Packet contents: ");
    Serial.print("Cal?: ");
    Serial.println(recPacket.calibrating);
    Serial.print("Channel: ");
    Serial.println(recPacket.channel);
    Serial.print("UpDn: ");
    Serial.println(recPacket.updn);
    Serial.print("Remaining vals: ");
    Serial.println(sampleBuffer.size());
    // ------Find end flag---------
    foundEnd = endFlagSearch(&sampleBuffer, minOneLength, minZeroLength);
    if (foundEnd) {
      Serial.println("Found the end flag");
      parsePacket(recPacket);
    }

    // Avoid scanning the same packet again
    delay(200);
  }
}
