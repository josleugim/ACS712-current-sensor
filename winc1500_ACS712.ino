#include <SPI.h>
#include <Adafruit_WINC1500.h>
#include <PubSubClient.h>

#define WINC_CS   8
#define WINC_IRQ  7
#define WINC_RST  4

// Define current sensor pin
#define CURRENT_SENSOR A0

// Define measurement variables
float amplitude_current;
float effective_value;
float effective_voltage = 110; // Set voltage to 230V (Europe) or 110V (US)
float effective_power;
float zero_sensor;

// Setup the WINC1500 connection with the pins above and the default hardware SPI.
Adafruit_WINC1500 WiFi(WINC_CS, WINC_IRQ, WINC_RST);

// Or just use hardware SPI (SCK/MOSI/MISO) and defaults, SS -> #10, INT -> #7, RST -> #5, EN -> 3-5V
//Adafruit_WINC1500 WiFi;


char ssid[] = "";     //  your network SSID (name)
char pass[] = "";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                // your network key Index number (needed only for WEP)

int status = WL_IDLE_STATUS;

// Update these with values suitable for your network.
const char* mqtt_server = "";

// Initialize the WIFI client library
// with the IP address and port of the server
// that you want to connect to (port 80 is default for HTTP):
Adafruit_WINC1500Client wifiClient;

PubSubClient client(wifiClient);
long lastMsg = 0;
char msg[50];
int value = 0;

void setup() {
  // Init serial
  Serial.begin(115200);

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to Wifi network:
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    uint8_t timeout = 10;
    while (timeout && (WiFi.status() != WL_CONNECTED)) {
      timeout--;
      delay(1000);
    }
  }

  Serial.println("Connected to wifi");
  printWifiStatus();

  // initialize server
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  // Calibrate sensor with null current
  zero_sensor = getSensorValue();
  Serial.print("Zero point sensor: ");
  Serial.println(zero_sensor);
  Serial.println("");
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("\nMessage arrived [");
  Serial.print(topic);
  Serial.print("]\n");
  char *value = (char*)payload;
  Serial.println(value);
  String strTopic = topic;
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("arduinoClientWinc1500", "", "")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish("outTopic", "hello world");
      // ... and resubscribe
      // suscribe to the corresponding topics
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Perform power measurement
  float sensor_value = getSensorValue();
  //Serial.print("Sensor value: ");
  //Serial.println(sensor_value);
    
  // Convert to current
  amplitude_current=(float)(sensor_value-zero_sensor)/1024*5/185*1000000;
  effective_value=amplitude_current/1.414;
  
  // Plot data
  //Serial.println("Current amplitude (in mA): ");
  //Serial.println(amplitude_current,1);

  if(amplitude_current > 0){
    client.publish("josleugim/test/current", PrepareDataForMQTT(amplitude_current)); 
  }
  
  //Serial.println("Current effective value (in mA)");
  //Serial.println(effective_value,1);

  if(effective_value > 0) {
    client.publish("josleugim/test/currenteffective", PrepareDataForMQTT(effective_value));
  }
  
  //Serial.println("Effective power (in W): ");
  //Serial.println(abs(effective_value*effective_voltage/1000),1);
  if(abs(effective_value*effective_voltage/1000) > 0) {
    client.publish("josleugim/test/power", PrepareDataForMQTT(abs(effective_value*effective_voltage/1000)));
  }

  Serial.println("");
  
  // Poll every 50ms
  delay(500);
}

// Get the reading from the current sensor
float getSensorValue()
{
  int sensorValue;
  float avgSensor = 0;
  int nb_measurements = 100;
  for (int i = 0; i < nb_measurements; i++) {
    sensorValue = analogRead(CURRENT_SENSOR);
    avgSensor = avgSensor + float(sensorValue);
  }    
  avgSensor = avgSensor/float(nb_measurements);
  return avgSensor;
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

char* PrepareDataForMQTT(float value) {
  String temp_amplitude_current_str;
  char temp_amplitude_current[50];
  temp_amplitude_current_str = String(value);
  temp_amplitude_current_str.toCharArray(temp_amplitude_current, temp_amplitude_current_str.length() + 1);
  return temp_amplitude_current;
}

