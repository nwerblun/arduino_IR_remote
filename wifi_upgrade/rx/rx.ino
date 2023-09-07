#include <SPI.h>
#include <WiFiNINA.h>

#define PWM_CH0_PIN 2
#define PWM_CH1_PIN 3

char ssid[] = "rx_nothing_to_see_here";
char pass[] = "876543210";

int status = WL_IDLE_STATUS;
int led = LED_BUILTIN;

WiFiServer server(80);

void setupPWM(){ 
  // Taken from MatinL on the arduino forums
  // Output 50Hz PWM on timer TCC0 (14-bit resolution)
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(3) |          // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Enable the port multiplexer for the 4 PWM channels: timer TCC0 outputs
  const uint8_t CHANNELS = 2;
  const uint8_t pwmPins[] = {2, 3};
  for (uint8_t i = 0; i < CHANNELS; i++)
  {
     PORT->Group[g_APinDescription[pwmPins[i]].ulPort].PINCFG[g_APinDescription[pwmPins[i]].ulPin].bit.PMUXEN = 1;
  }
  // Connect the TCC0 timer to the port outputs - port pins are paired odd PMUO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;
 // PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;
  //PORT->Group[g_APinDescription[5].ulPort].PMUX[g_APinDescription[5].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;

  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |         // Reverse the output polarity on all TCC0 outputs
                    TCC_WAVE_WAVEGEN_DSBOTTOM;    // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation:
  // 20000 = 50Hz, 10000 = 100Hz, 2500  = 400Hz
  REG_TCC0_PER = 20000;      // Set the frequency of the PWM on TCC0 to 50Hz
  while(TCC0->SYNCBUSY.bit.PER);

  // The CCBx register value corresponds to the pulsewidth in microseconds (us)
  REG_TCC0_CCB0 = 1500;       
  while(TCC0->SYNCBUSY.bit.CCB0);
  REG_TCC0_CCB1 = 1500;       
  while(TCC0->SYNCBUSY.bit.CCB1);


  // Divide the 16MHz signal by 8 giving 2MHz (0.5us) TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV8 |    // Divide GCLK4 by 8
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait until up
  }

  pinMode(PWM_CH0_PIN, OUTPUT);
  pinMode(PWM_CH1_PIN, OUTPUT);
  setupPWM();

  Serial.println("RX Server");
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed so I'm quitting.");
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // Default IP is 192.168.4.1
  Serial.print("Access point named: ");
  Serial.println(ssid);
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("Uhhh AP mode failed. I'm quitting.");
    while (true);
  }
  delay(10000);
  Serial.println("Now listening for connections");
  server.begin();
  printWiFiStatus();
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}

void loop() {
// compare the previous status to the current status
  if (status != WiFi.status()) {
    status = WiFi.status();
    if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }
  WiFiClient client = server.available();
  if (client) {
    Serial.println("New client");
    String currentLine = "";
    while (client.connected()) {
      if (client.available()) {
        
        // Read in command
        char c = client.read();
        Serial.write(c);
        if (c == '\n') {
          Serial.println("Finished command");
          break;
        } else if (c != '\r') {
          currentLine += c;
        }

        // Parse commands
        if (currentLine.endsWith("cal_on")) {
          Serial.println("high");
          digitalWrite(led, HIGH);
        } else if (currentLine.endsWith("cal_off")) {
          Serial.println("low");
          digitalWrite(led, LOW);
        } else if (currentLine.endsWith("pwr")) {

        } else if (currentLine.endsWith("pressure")) {
          
        } else if (currentLine.endsWith("cal_up")) {
          
        } else if (currentLine.endsWith("cal_down")) {
          
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}
