// #include <EEPROM.h>
#define TEST_SEND_BUTTON_PIN 2
#define CAL_UP_PIN 9
#define CAL_DN_PIN 10
#define CALIBRATE_TOGGLE_PIN 11
#define IR_LED_TX_PIN 5
// use built-in LED for cal active for now

// Carrier freq = 38kHz (IR standard)
// IR standard is On-Off shift keyed. Receiver module VS1838 expects 600us/900us length pulses for 1/0 respectively
#define CARRIER_HALF_PERIOD_US 13
#define ONE_LENGTH_PERIODS 23  // 600us / (1/38k) = 22.8 periods per 1 one pulse
#define ZERO_LENGTH_PERIODS 34  // 900us / (1/38k) = 34.2 periods per 1 zero pulse

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
  int start_flag_locking = 0b1010101010101010;
  int start_flag_remainder = 0b1111111110101010;
  int end_flag_half = 0b1111100000111111;
  bool calibrating = false;
  byte channel = 0b1010;
  byte up_dn = 0b0000;
};

void setup() {
  pinMode(IR_LED_TX_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(TEST_SEND_BUTTON_PIN, INPUT_PULLUP);
  pinMode(CAL_UP_PIN, INPUT_PULLUP);
  pinMode(CAL_DN_PIN, INPUT_PULLUP);
  pinMode(CALIBRATE_TOGGLE_PIN, INPUT_PULLUP);

  Serial.begin(9600);
}

void sendOne() {
  tone(IR_LED_TX_PIN, 38000);
  delayMicroseconds(600);
  noTone(IR_LED_TX_PIN);
}

void sendZero() {
  noTone(IR_LED_TX_PIN);
  delayMicroseconds(900);
}

void sendByteRaw(byte sendVal) {
  int data;
  for (int i = 0; i < 8; i++) {
    data = bitRead(sendVal, i);
    if (data == 1) {
      sendOne();
    } else {
      sendZero();
    }
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
  sendIntRaw(toBeSent.start_flag_locking);
  sendIntRaw(toBeSent.start_flag_locking);
  sendIntRaw(toBeSent.start_flag_locking);
  sendIntRaw(toBeSent.start_flag_locking);
  sendIntRaw(toBeSent.start_flag_locking);
  sendIntRaw(toBeSent.start_flag_locking);

  // sendIntRaw(toBeSent.start_flag_remainder);
  
  // if (toBeSent.calibrating) {
  //   sendOne();
  // } else{
  //   sendZero();
  // }

  // sendByteRaw(toBeSent.channel);
  
  // sendByteRaw(toBeSent.up_dn);

  // sendIntRaw(toBeSent.end_flag_half);
  // sendIntRaw(toBeSent.end_flag_half);

  Serial.println("Packet sent");
}

bool calibrating = false;
byte channel = 10;
byte cal_channel = 10;
int cal_toggle_rd;
int cal_up_rd;
int cal_dn_rd;
int test_chann_button_rd;
Packet toSend;

bool debug = true;
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
      return;
    } else {
      Serial.println("Calibration deactivated");

      calibrating = false;
      digitalWrite(LED_BUILTIN, LOW);
      sendPacket(getDefaultPacket());
      return;
    }
  }
  // --------READ CHANNEL 0 BUTTON--------
  if (!(test_chann_button_rd == HIGH)) {
    //Serial.println("Channel 0 pressed");

    toSend = getDefaultPacket();
    toSend.channel = 0b0000;
    toSend.calibrating = calibrating;
    if (calibrating) {
      cal_channel = 0b0000;
      //Serial.println("Calibration channel is now 0");
    }
    sendPacket(toSend);
    return;
  }
  // --------READ UP BUTTON--------
  if (!(cal_up_rd == HIGH)) {
    Serial.println("Up button pressed");

    toSend = getDefaultPacket();
    toSend.channel = cal_channel;
    toSend.calibrating = calibrating;
    toSend.up_dn = 0b0010;
    sendPacket(toSend);
    return;
  }
  // --------READ DN BUTTON--------
  if (!(cal_dn_rd == HIGH)) {
    Serial.println("Down button pressed");

    toSend = getDefaultPacket();
    toSend.channel = cal_channel;
    toSend.calibrating = calibrating;
    toSend.up_dn = 0b0001;
    sendPacket(toSend);
    return;
  }

  // Small delay to avoid resending
  delay(100);
}
