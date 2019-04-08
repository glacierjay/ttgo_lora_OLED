/* Example from Sandeep Mistry
 *  modified by Glacierjay 4/5/2019
 * With mods from AliExpress/LilyGo docs
 * For TTGo ESP32 LoRa-OLED board
 * http://www.lilygo.cn/down_view.aspx?TypeId=11&Id=78&Fid=t14:11:14
 * Based on SX1276 LoRa Radio http://www.semtech.com/apps/product.php?pn=SX1276
 * This program displays incoming LoRa packets on OLED display. It gets packets from
 * several of my POC projects: tbeam_gps and ventana_usb_V5.
 * In Arduino IDE, choose board type "TTGO LoRa32-OLED V1". You might have to 
 * download esp32 
 */

#include <SPI.h>
#include <LoRa.h>       // https://github.com/sandeepmistry/arduino-LoRa
#include <U8g2lib.h>   // https://github.com/olikraus/U8g2_Arduino
// #include <U8x8lib.h>

#define OFF 0   // For LED
#define ON 1

// SPI LoRa Radio
#define LORA_SCK 5        // GPIO5 - SX1276 SCK
#define LORA_MISO 19     // GPIO19 - SX1276 MISO
#define LORA_MOSI 27    // GPIO27 - SX1276 MOSI
#define LORA_CS 18     // GPIO18 - SX1276 CS
#define LORA_RST 14   // GPIO14 - SX1276 RST
#define LORA_IRQ 26  // GPIO26 - SX1276 IRQ (interrupt request)

// I2C OLED Display works with SSD1306 driver
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16

U8G2_SSD1306_128X64_NONAME_F_SW_I2C Display(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ OLED_RST); // Full framebuffer, SW I2C
U8G2LOG u8g2log;

#define U8LOG_WIDTH 32
#define U8LOG_HEIGHT 10
#define BIG_FONT u8g2_font_ncenB10_tr
#define SMALL_FONT u8g2_font_tom_thumb_4x6_mf

const int LoRaFrequency = 903900000; 
const int blueLED = LED_BUILTIN;

#include <WiFi.h>
WiFiClient espClient;
#include <PubSubClient.h>
PubSubClient client(espClient);
#define in_topic "Glacierjay" // use this to accept commands
#define button_topic "Glacierjay/gps" // use to send state of gps
const char* ssid     = "karenjay"; // home ssid
const char* password = "XXXXXX"; // home password
const char* mqtt_server = "192.168.1.20"; // RPi 3 model b+ mexpi that runs mosquitto
long lastMsg = 0;
char msg[50];

void setup_wifi() {
  delay(10);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  randomSeed(micros());
  Serial.print("\nWiFi connected,IP address: ");Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("mqtt received:( ");Serial.print(topic);Serial.print(")");
  payload[length] = '\0'; // terminate it just in case
  Serial.println((char*)payload);
}

void reconnect() {
  while (!client.connected()) {  // Loop until we're reconnected
    String clientId = "TbeamClient-";  // Create a random client ID
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {  // Attempt to connect
      Serial.println("connected to MQTT");
      client.publish(in_topic, "Glacierjay mqtt connected");  // Once connected, publish an announcement...
      client.subscribe(in_topic); // Commands come in to this topic 1=ledOn 2=ledOff 3=sendState
    }
    else {
      Serial.print("MQTT client connect failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);  // Wait 5 seconds before retrying
    }
  }
}



String rssi = "";
String packet = "";
// allocate memory for print buffer
uint8_t u8log_buffer[U8LOG_WIDTH*U8LOG_HEIGHT];

void clear_disp(){
  u8g2log.print("\f"); // crashes the processor some times...
  Display.setFont(BIG_FONT);
  Display.clearBuffer();  
  Display.setCursor(0,12); Display.print("LoRa ReceiverV2");
  Display.sendBuffer();
  Display.setFont(SMALL_FONT);
  delay(2000);

}
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa-mqtt gateway");
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  Display.begin();
  Display.enableUTF8Print();    // enable UTF8 support for the Arduino print() function
  Display.setFont(u8g2_font_tom_thumb_4x6_mf);  // set the font for the terminal window
  u8g2log.begin(Display, U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buffer); // connect to u8g2, assign buffer
  u8g2log.setLineHeightOffset(0); // set extra space between lines in pixel, this can be negative
  u8g2log.setRedrawMode(0);   // 0: Update screen with newline, 1: Update screen for every char 
  clear_disp();

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);  
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);         
  pinMode(blueLED, OUTPUT); // For LED feedback
  
  if (!LoRa.begin(LoRaFrequency)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  
  // The larger the spreading factor the greater the range but slower data rate
  // Send and receive radios need to be set the same
  LoRa.setSpreadingFactor(7);  // ranges from 6-12, default 7 see API docs

   Display.clearBuffer();  
   Display.setCursor(0,12); Display.print("LoRa ReceiverV2");
   Display.sendBuffer();
}

char charBuf[50];
void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {   // received a packet
    digitalWrite(blueLED, ON); // Turn blue LED on
    packet = "";
    while (LoRa.available()) {     // read packet
      packet += (char)LoRa.read(); // Assemble new packet
    }
    rssi = LoRa.packetRssi();
    
    // print the received packet on the OLED. u8g2log emulates a term window, so it scrolls
    if (packet == "clearDisp") clear_disp();
    else {
      u8g2log.print(packet);
      u8g2log.print("\n");
    }
    digitalWrite(blueLED, OFF); // Turn blue LED off
    Serial.print("{type : packet, packet: '");
    Serial.println(packet + "', rssi : " + rssi + " }");
    packet.toCharArray(charBuf, 50);
    client.publish(button_topic, charBuf);
  }
  if (!client.connected())reconnect();
  client.loop();
  long now = millis();
  if (now - lastMsg > 120000) { // publish something every 2 minutes
    lastMsg = now;
    client.publish("Glacierjay/alive", "Glacierjay is live");
  }
}

