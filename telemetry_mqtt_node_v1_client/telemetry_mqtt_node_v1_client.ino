/****************************************
 * Include Libraries
 ****************************************/

#include <Adafruit_Sensor.h>
// Tested with BME280 Driver version 1.1.0
#include <Adafruit_BME280.h>
// Tested with SGP30 Driver version 1.2.2
#include <Adafruit_SGP30.h>
#include <Wire.h>

#include <PubSubClient.h>
#include <WiFi.h>

#define WIFISSID "_____" // Put your WifiSSID here
#define PASSWORD "_____" // Put your wifi password here
#define TOKEN ""
#define MQTT_CLIENT_NAME "<tail of node ID>"

/****************************************
 * Define Constants
 ****************************************/
#define LOCATION "<location>"
#define MAX_CONNECTION_RETRIES 5

#define SEA_LEVEL_PRESSURE_HPA (1013.25)

// Fill with UUIDs
#define TEMPERATURE_NODE_ID "____"
#define PRESSURE_NODE_ID "____"
#define ALTITUDE_NODE_ID "____"
#define HUMIDITY_NODE_ID "____"
#define TVOC_NODE_ID "____"
#define ECO2_NODE_ID "____"
#define H2_NODE_ID "____"
#define ETHANOL_NODE_ID "____"

#define TEMPERATURE_DATA_TYPE "Temperature"
#define CELSIUS_MEASUREMENT_UNIT "Celsius"
#define PRESSURE_DATA_TYPE "Pressure"
#define HPA_MEASUREMENT_UNIT "hPa"
#define ALTITUDE_DATA_TYPE "Altitude"
#define METERS_MEASUREMENT_UNIT "Meters"
#define HUMIDITY_DATA_TYPE "Humidity"
#define PERCENTAGE_MEASUREMENT_UNIT "Percentage"
#define TVOC_DATA_TYPE "TVOC"
#define PPB_MEASUREMENT_UNIT "ppb"
#define ECO2_DATA_TYPE "eCO2"
#define PPM_MEASUREMENT_UNIT "ppm"
#define H2_DATA_TYPE "H2"
#define ETHANOL_DATA_TYPE "Ethanol"

#define io2 33 //led output

// I2C protocol
Adafruit_BME280 bme;
Adafruit_SGP30 sgp;

int sgp_baseline_reading_counter = 0;
int txled = io2;
char payload[300];
char topic[50];

/****************************************
 * Auxiliar Functions
 ****************************************/
WiFiClient ubidots;
PubSubClient client(ubidots);

void(* resetFunc) (void) = 0;  // declare reset fuction at address 0

void callback(char* topic, byte* payload, unsigned int length) {
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = 0;
  String message(p);
  Serial.write(payload, length);
  Serial.println(topic);
}


void reconnect() {
  int connect_retries = 0;
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    
    // Attemp to connect
    if (client.connect(MQTT_CLIENT_NAME, TOKEN, "")) {
      Serial.println("Connected");
    } else {
      connect_retries++;
      if(connect_retries == MAX_CONNECTION_RETRIES) {
        resetFunc(); //call reset
      }
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      // Wait 2 seconds before retrying
      delay(2000);
    }
  }
}


void initWifi() {
  WiFi.begin(WIFISSID, PASSWORD);
  
  Serial.println();
  Serial.print("Waiting for WiFi...");
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  
  Serial.println("");
  Serial.println("WiFi Connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  IPAddress my_broker(192, 168, 1, 10);
  client.setServer(my_broker, 9999);
  client.setCallback(callback);  
}


void initBME280(){
  unsigned statusBme;

  // default settings
  statusBme = bme.begin();  

  if (!statusBme) {
    Serial.println("ERROR: Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x");
    Serial.println(bme.sensorID(),16);
    Serial.println("ID of 0xFF probably means a bad address, a BMP 180 or BMP 085");
    Serial.println("ID of 0x56-0x58 represents a BMP 280,");
    Serial.println("ID of 0x60 represents a BME 280.");
    Serial.println("ID of 0x61 represents a BME 680.");
    while (1) delay(10);
  } else {
    Serial.print("SUCCESS: BME280 found at SensorID: 0x");
    Serial.println(bme.sensorID(),16);
  }
}


void initSGP30(){
  unsigned statusSgp;
  statusSgp = sgp.begin();  
  if (!statusSgp) {
    Serial.println("ERROR: Sensor not found");
  }
  else
  {
    Serial.print("SUCCESS: SGP30 found SGP30 serial #");
    Serial.print(sgp.serialnumber[0], HEX);
    Serial.print(sgp.serialnumber[1], HEX);
    Serial.println(sgp.serialnumber[2], HEX);
  }
}


// Example:
// {
// "location": "Laboratory",
// "node_id": "417fa14f-c213-4255-92a4-49d766ede1de",
// "data_type": "Temperature",
// "measurement_unit": "Celsius",
// "value": 20.2

// }
void sendTelemetryDataPoint() {
  sprintf(topic, "D-MARS/telemetry");
  Serial.println("Publishing data to D-MARS Telemetry base station. Topic:");
  Serial.println(topic);
  Serial.println(payload);
  client.publish(topic, payload);
  client.loop();
}


/* return absolute humidity [mg/m^3] with approximation formula
 * @param temperature [°C]
 * @param humidity [%RH]
 */
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}


void processBMEDataPoints() {

  float temperature_value = bme.readTemperature();
  float humidity_value = bme.readHumidity();
  float pressure_value = bme.readPressure() / 100.0F;
  float altitude_value = bme.readAltitude(SEA_LEVEL_PRESSURE_HPA);
  
  Serial.print("Temperature = "); Serial.print(temperature_value); Serial.println(" °C");
  Serial.print("Humidity = "); Serial.print(humidity_value); Serial.println(" %");
  Serial.print("Pressure = "); Serial.print(pressure_value); Serial.println(" hPa");
  Serial.print("Approx. Altitude = "); Serial.print(altitude_value); Serial.println(" m");
  Serial.println();
  
  sprintf(payload, "%s", ""); // Cleans the payload
  sprintf(payload, "{\"location\":\"%s\",\"node_id\":\"%s\",\"data_type\":\"%s\",\"measurement_unit\":\"%s\",\"value\": %s}",
                    LOCATION,TEMPERATURE_NODE_ID,TEMPERATURE_DATA_TYPE,CELSIUS_MEASUREMENT_UNIT,String(temperature_value,2));
  sendTelemetryDataPoint();

  sprintf(payload, "%s", ""); // Cleans the payload
  sprintf(payload, "{\"location\":\"%s\",\"node_id\":\"%s\",\"data_type\":\"%s\",\"measurement_unit\":\"%s\",\"value\": %s}",
                    LOCATION,HUMIDITY_NODE_ID,HUMIDITY_DATA_TYPE,PERCENTAGE_MEASUREMENT_UNIT,String(humidity_value,2));
  sendTelemetryDataPoint();
  
  sprintf(payload, "%s", ""); // Cleans the payload
  sprintf(payload, "{\"location\":\"%s\",\"node_id\":\"%s\",\"data_type\":\"%s\",\"measurement_unit\":\"%s\",\"value\": %s}",
                    LOCATION,PRESSURE_NODE_ID,PRESSURE_DATA_TYPE,HPA_MEASUREMENT_UNIT,String(pressure_value,2));
  sendTelemetryDataPoint();
  
  sprintf(payload, "%s", ""); // Cleans the payload
  sprintf(payload, "{\"location\":\"%s\",\"node_id\":\"%s\",\"data_type\":\"%s\",\"measurement_unit\":\"%s\",\"value\": %s}",
                    LOCATION,ALTITUDE_NODE_ID,ALTITUDE_DATA_TYPE,METERS_MEASUREMENT_UNIT, String(altitude_value,2));
  sendTelemetryDataPoint();
}


void processSGPDataPoints() {
  // If you have a temperature / humidity sensor, 
  // you can set the absolute humidity to enable 
  // the humditiy compensation for the air quality signals
  float temperature = bme.readTemperature(); // [°C]
  float humidity = bme.readHumidity(); // [%RH]
  sgp.setHumidity(getAbsoluteHumidity(temperature, humidity));

  if (! sgp.IAQmeasure()) {
    Serial.println("ERROR: Measurement failed");
    return;
  }

  uint16_t tvoc_value = sgp.TVOC;
  uint16_t eco2_value = sgp.eCO2;

  Serial.print("TVOC "); Serial.print(tvoc_value); Serial.println(" ppb");
  Serial.print("eCO2 "); Serial.print(eco2_value); Serial.println(" ppm");

  sprintf(payload, "%s", ""); // Cleans the payload
  sprintf(payload, "{\"location\":\"%s\",\"node_id\":\"%s\",\"data_type\":\"%s\",\"measurement_unit\":\"%s\",\"value\": %s}",
                    LOCATION,TVOC_NODE_ID,TVOC_DATA_TYPE,PPB_MEASUREMENT_UNIT, String(tvoc_value));
  sendTelemetryDataPoint();

  sprintf(payload, "%s", ""); // Cleans the payload
  sprintf(payload, "{\"location\":\"%s\",\"node_id\":\"%s\",\"data_type\":\"%s\",\"measurement_unit\":\"%s\",\"value\": %s}",
                    LOCATION,ECO2_NODE_ID,ECO2_DATA_TYPE,PPM_MEASUREMENT_UNIT, String(eco2_value));
  sendTelemetryDataPoint();

  if (! sgp.IAQmeasureRaw()) {
    Serial.println("ERROR: Raw Measurement failed");
    return;
  }

  uint16_t rawH2_value = sgp.rawH2;
  uint16_t rawEthanol_value = sgp.rawEthanol;

  Serial.print("Raw H2 "); Serial.print(rawH2_value); Serial.print(" \t");
  Serial.print("Raw Ethanol "); Serial.print(rawEthanol_value); Serial.println("");

  sprintf(payload, "%s", ""); // Cleans the payload
  sprintf(payload, "{\"location\":\"%s\",\"node_id\":\"%s\",\"data_type\":\"%s\",\"measurement_unit\":\"%s\",\"value\": %s}",
                    LOCATION,H2_NODE_ID,H2_DATA_TYPE,PPM_MEASUREMENT_UNIT, String(rawH2_value));
  sendTelemetryDataPoint();

  sprintf(payload, "%s", ""); // Cleans the payload
  sprintf(payload, "{\"location\":\"%s\",\"node_id\":\"%s\",\"data_type\":\"%s\",\"measurement_unit\":\"%s\",\"value\": %s}",
                    LOCATION,ETHANOL_NODE_ID,ETHANOL_DATA_TYPE,PPM_MEASUREMENT_UNIT, String(rawEthanol_value));
  sendTelemetryDataPoint();
 
  delay(1000);

  sgp_baseline_reading_counter++;
  if (sgp_baseline_reading_counter == 30) {
    sgp_baseline_reading_counter = 0;

    uint16_t TVOC_base;
    uint16_t eCO2_base;
    if (! sgp.getIAQBaseline(&eCO2_base, &TVOC_base)) {
      Serial.println("Failed to get baseline readings");
      return;
    }
    Serial.print("****Baseline values: eCO2: 0x"); Serial.print(eCO2_base, HEX);
    Serial.print(" & TVOC: 0x"); Serial.println(TVOC_base, HEX);
  }
}


/****************************************
 * Main Functions
 ****************************************/
void setup() {
  pinMode(txled, OUTPUT);
  Serial.begin(115200);
  // Wait for serial console to open!
  while (!Serial) { delay(10); }

  initWifi();
  initBME280();
  initSGP30();
}


void loop() {
  if (!client.connected()) {
    reconnect();
  }

  processBMEDataPoints();
  processSGPDataPoints();
  
  delay(30000); // 30 Seconds
}
