#include <SPI.h>
#include <WiFiNINA.h>

#define CAL_BUTTON_PIN 2
#define PWR_BUTTON_PIN 3
#define PRSR_BUTTON_PIN 4
#define CAL_UP_BUTTON_PIN 5
#define CAL_DN_BUTTON_PIN 6

char ssid[] = "rx_nothing_to_see_here";
char pass[] = "876543210";

int status = WL_IDLE_STATUS;
char hostip[] = "192.168.4.1";
WiFiClient client;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait until up
  }

  pinMode(CAL_BUTTON_PIN, INPUT_PULLUP);
  pinMode(PWR_BUTTON_PIN, INPUT_PULLUP);
  pinMode(PRSR_BUTTON_PIN, INPUT_PULLUP);
  pinMode(CAL_UP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(CAL_DN_BUTTON_PIN, INPUT_PULLUP);

  Serial.println("TX Startup");
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed so I'm quitting.");
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(5000);
  }
  Serial.println("Connected to wifi");
  printWiFiStatus();
  client.connect(hostip, 80);
  Serial.println("Connected to rx");
}

void printWiFiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}


bool calibrating = false;
void loop() {
  // put your main code here, to run repeatedly:
  if (!client.connected()) {
    Serial.println();
    Serial.println("disconnected from server.");
    client.stop();
    while (true);
  }

  // Relay any message from the server
  while (client.available()) {
    char c = client.read();
    Serial.write(c);
  }

  cal_toggle_b = digitalRead(CAL_BUTTON_PIN);
  pwr_btn_b = digitalRead(PWR_BUTTON_PIN);
  prsr_btn_b = digitalRead(PRSR_BUTTON_PIN);
  cal_up_b = digitalRead(CAL_UP_BUTTON_PIN);
  cal_dn_b = digitalRead(CAL_DN_BUTTON_PIN);
  // Remember there's pullups, so LOW = active

  // print seems to include a \n char automatically even without println
  // if cal button is pressed
  if (!(cal_toggle_b == HIGH)) {
    if (calibrating) {
      client.print("cal_off");
    } else {
      client.print("cal_on");
    }
    calibrating = !calibrating;
  } else if (!(pwr_btn_b)) {
      client.print("pwr");
  } else if (!(prsr_btn_b)) {
      client.print("pressure");
  } else if (!(cal_up_b)) {
      client.print("cal_up");
  } else if (!(cal_dn_b)) {
      client.print("cal_down");
  }
  // Small delay to avoid bouncing
  delay(500);
}
