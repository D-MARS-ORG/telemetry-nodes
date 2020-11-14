/****************************************
 * Include Libraries
 ****************************************/
#include <WiFi.h>
#include <PubSubClient.h>

#define WIFISSID "_____" // Put your WifiSSID here
#define PASSWORD "_____" // Put your wifi password here
#define TOKEN ""
#define MQTT_CLIENT_NAME "<tail of node ID>"

/****************************************
 * Define Constants
 ****************************************/
#define LOCATION "<location>"
#define NODE_ID "<unique ver4 uuid>"
#define DATA_TYPE "<data type>"
#define MEASUREMENT_UNIT "<measurement unit>"


#define SENSOR 32 // SENSOR was changed from 12 to 32 = touch button 1 on mahavision
#define io2 33 //led output


int txled = io2;
char payload[100];
char topic[150];
// Space to store values to send
char str_sensor[10];

/****************************************
 * Auxiliar Functions
 ****************************************/
WiFiClient ubidots;
PubSubClient client(ubidots);

void callback(char* topic, byte* payload, unsigned int length) {
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  String message(p);
  Serial.write(payload, length);
  Serial.println(topic);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    
    // Attemp to connect
    if (client.connect(MQTT_CLIENT_NAME, TOKEN, "")) {
      Serial.println("Connected");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      // Wait 2 seconds before retrying
      delay(2000);
    }
  }
}

/****************************************
 * Main Functions
 ****************************************/
void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFISSID, PASSWORD);
  
  // Assign the analog pin as INPUT 
  pinMode(SENSOR, INPUT);
  // Assign the analog pin as INPUT 
  pinMode(txled, OUTPUT);
  
  Serial.println();
  Serial.print("Wait for WiFi...");
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  
  Serial.println("");
  Serial.println("WiFi Connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  IPAddress my_broker(10, 0, 0, 19);
  client.setServer(my_broker, 9999);
  
  
  client.setCallback(callback);  
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }

  sprintf(topic, "D-MARS/telemetry");
  sprintf(payload, "%s", ""); // Cleans the payload
  
  float sensor = analogRead(SENSOR); 
  float temp = (sensor-500)/10; //temperature in deg celcius= (reading-offset voltage)/(10mv to 1degree)
  Serial.println(temp);
  /* 4 is mininum width, 2 is precision; float value is copied onto str_sensor*/
  dtostrf(temp, 4, 2, str_sensor);
  
  digitalWrite(io2, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(50);
  // Example:
  // {
  // "location": "Laboratory",
  // "node_id": "417fa14f-c213-4255-92a4-49d766ede1de",
  // "data_type": "Temperature",
  // "measurement_unit": "Celsius",
  // "value": 20.2
  // }
  sprintf(payload, "{\"location\": \"%s\", \"node_id\": \"%s\", 
                    \"data_type\": \"%s\", \"measurement_unit\": \"%s\", 
                    \"value\": %s}",
                    LOCATION,NODE_ID,DATA_TYPE,MEASUREMENT_UNIT,str_sensor);
  Serial.println("Publishing data to D-MARS Telemetry base station. Topic:");
  Serial.println(topic);
  Serial.println(payload);
  client.publish(topic, payload);
  client.loop();
  digitalWrite(io2, LOW);    // turn the LED off by making the voltage LOW
  delay(3000