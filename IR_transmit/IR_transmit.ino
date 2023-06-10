// #include <EEPROM.h>
#define TEST_SEND_BUTTON_PIN 2
#define CAL_UP_PIN 9
#define CAL_DN_PIN 10
#define CALIBRATE_TOGGLE_PIN 8
#define IR_LED_TX_PIN 3
#define tPwlMicros 550
#define tPwhMicros 650
// It should be 600/600 but for some reason my receiver module decodes it as 620/540. Skew low/high to try and make it closer to 50%
// use built-in LED for cal active for now

#define TOP F_CPU / (8 * 38000)
#define HALF_DUTY_COUNT int(TOP / 2);

/*
Packet Structure:
|----START FLAG-----|-CALIBRATE?-|--CAL UP/DN--|-CHANNEL (MOTOR)-|-----END FLAG-----|
| (01010101)x?+1111 |     X      |    XXXX     |       XXXX      | 0011111111111111 |
calibrate = 1 if calibrating, 0 if actuating
cal up/dn = 0/1 if neither, 2 if dn, 3 if up
channel = 0-9, specifies which motor to use. Numbers >9 reserved

If cal button is pressed, don't do anything except light an LED
If cal is selected and a channel is pressed, send a packet with cal=1, channel=channel and cal up/dn = 0
If cal is selected, channel is selected and up/dn is pressed, send a packet with cal=1, channel=channel and cal up/dn = 1/2
If cal is pressed again, send a packet with cal=0, channel=10, cal up/dn = 0 to specify done calibrating

If cal is not selected and channel is pressed send cal=0, channel=channel, cal_up/dn = 0
*/

struct Packet {
  int start_flag_locking = 0b0101010101010101;
  int start_flag_remainder = 0b1111010101010101;
  int end_flag = 0b1111111111111100;
  bool calibrating = false;
  byte channel = 0b00001010;
  byte updn = 0b00000001;
};

void setup() {
  pinMode(IR_LED_TX_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(TEST_SEND_BUTTON_PIN, INPUT_PULLUP);
  pinMode(CAL_UP_PIN, INPUT_PULLUP);
  pinMode(CAL_DN_PIN, INPUT_PULLUP);
  pinMode(CALIBRATE_TOGGLE_PIN, INPUT_PULLUP);

  Serial.begin(9600);

  // Set pin 3 to 38kHz PWM 50% duty cycle
  // Timer 2 = pins 11/3
  // Timer 0 is used for the delay function, don't touch
  // Zero out default settings, keep only reserved/required bits
  TCCR2B &= 0b11110000;
  TCCR2A &= 0b10100000;

  // Set prescalar to 8
  TCCR2B |= (1 << CS21);
  // Set PWM mode to fast + use OCR2A as top. Note that this effectively disables pin 11. Also the datasheet says OCRA which is a typo.
  TCCR2A |= (1 << WGM20) | (1 << WGM21);
  TCCR2B |= (1 << WGM22);
  // Set top to be the proper value to get the freq we want
  OCR2A = TOP;

  // Set behavior of output to go high/low appropriately
  TCCR2A |= (1 << COM2B1);
  // Set duty cycle
  OCR2B = 0;
}

void sendOne() {
  OCR2B = HALF_DUTY_COUNT;
  delayMicroseconds(tPwhMicros);
  OCR2B = 0;
  // Serial.print(1);
}

void sendZero() {
  OCR2B = 0;
  delayMicroseconds(tPwlMicros);
  // Serial.print(0);
}

void sendByteRaw(byte sendVal) {
  int data;
  for (int i = 0; i < 8; i++) {
    data = bitRead(sendVal, i);
    if (i > 0 && i % 2 == 0) {
      if (bitRead(sendVal, i-1)) {
        sendZero();
      } else {
        sendOne();
      }
    }
    if (data == 1) {
      sendOne();
    } else {
      sendZero();
    }
  }
  
  if (bitRead(sendVal, 7) == 1) {
    sendZero();
  } else {
    sendOne();
  }
}

void sendIntRaw(int sendVal) {
  int data;
  int num_bits = 8 * sizeof(int);
  for (int i = 0; i < num_bits; i++) {
    data = bitRead(sendVal, i);
    if (data == 1) {
      sendOne();
    } else {
      sendZero();
    }
  }
}

Packet getDefaultPacket() {
  Packet def;
  return def;
}

void sendPacket(Packet toBeSent) {
  Serial.println("Sending packet");
  for (int i = 0; i < 4; i++) {
    sendIntRaw(toBeSent.start_flag_locking);
  }
  sendIntRaw(toBeSent.start_flag_remainder);

  
  if (toBeSent.calibrating) {
    sendOne();
  } else{
    sendZero();
  }

  sendByteRaw(toBeSent.updn);
  sendByteRaw(toBeSent.channel);

  sendIntRaw(toBeSent.end_flag);

  Serial.println("\nPacket sent");
}

bool calibrating = false;
byte channel = 10;
byte cal_channel = 10;
int cal_toggle_rd;
int cal_up_rd;
int cal_dn_rd;
int test_chann_button_rd;
Packet toSend;
bool anythingPressed = false;
bool debug = false;
void loop() {
  cal_toggle_rd = digitalRead(CALIBRATE_TOGGLE_PIN);
  test_chann_button_rd = digitalRead(TEST_SEND_BUTTON_PIN);
  cal_up_rd = digitalRead(CAL_UP_PIN);
  cal_dn_rd = digitalRead(CAL_DN_PIN);
  // Remember there's pullups, so LOW = active

  if (debug) {
    String cmd = "";
    if (Serial.available() > 0) {
      cmd = Serial.readString();
      cmd.trim();
      if (cmd == "send_chan_0") {
        test_chann_button_rd = HIGH;
      } else if (cmd == "send_up") {
        cal_up_rd = HIGH;
      } else if (cmd == "send_dn") {
        cal_dn_rd = HIGH;
      } else if (cmd == "send_cal") {
        cal_toggle_rd = HIGH;
      }
    }
  }
  
  // --------READ CALIBRATE BUTTON--------
  if (!(cal_toggle_rd == HIGH)) {
    Serial.println("Cal pressed");
    anythingPressed = true;
    cal_channel = 0b1010;
    if (!calibrating) {
      Serial.print("Calibration activated, cal channel is ");
      Serial.print(cal_channel, DEC);
      Serial.println();

      calibrating = true;
      toSend = getDefaultPacket();
      toSend.calibrating = true;
      digitalWrite(LED_BUILTIN, HIGH);
      sendPacket(toSend);
    } else {
      Serial.println("Calibration deactivated");

      calibrating = false;
      digitalWrite(LED_BUILTIN, LOW);
      sendPacket(getDefaultPacket());
    }
  }
  // --------READ CHANNEL 0 BUTTON--------
  else if (!(test_chann_button_rd == HIGH)) {
    Serial.println("Channel 0 pressed");
    anythingPressed = true;
    toSend = getDefaultPacket();
    toSend.channel = 0b0000;
    toSend.calibrating = calibrating;
    if (calibrating) {
      cal_channel = 0b0000;
      //Serial.println("Calibration channel is now 0");
    }
    sendPacket(toSend);
  }
  // --------READ UP BUTTON--------
  else if (!(cal_up_rd == HIGH)) {
    Serial.println("Up button pressed");
    anythingPressed = true;
    toSend = getDefaultPacket();
    toSend.channel = cal_channel;
    toSend.calibrating = calibrating;
    toSend.updn = 0b0011;
    sendPacket(toSend);
  }
  // --------READ DN BUTTON--------
  else if (!(cal_dn_rd == HIGH)) {
    Serial.println("Down button pressed");
    anythingPressed = true;
    toSend = getDefaultPacket();
    toSend.channel = cal_channel;
    toSend.calibrating = calibrating;
    toSend.updn = 0b0010;
    sendPacket(toSend);
  }

  // Small delay to avoid resending
  delay(100);
}
