#include "Arduino.h"
#include <Wire.h>
#include <WiFi.h>

#include <BH1750.h>
#include "AS5600.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "DHT_U.h"
#include "DHT.h"
#include <PubSubClient.h>
#include <SimpleTimer.h>


#define FAN_PIN 25
const char* fanTopic = "Actuators/Fan";

#define LED_PIN 26
const char* ledTopic = "Actuators/LED";

#define WATER_PUMP_PIN 27
const char* waterPumpTopic = "Actuators/WaterPump";

#define AS5600_DIR_PIN 17

#define DHT_DATA_PIN 4
#define DHTTYPE DHT11

#define SEALEVELPRESSURE_HPA (1013.25)

#define ENCODER_PIN 16

BH1750 lightMeter;
float lux = 0;

AS5600 as5600;
float windDir;

// not working
Adafruit_BME280 bme;

DHT dht(DHT_DATA_PIN, DHTTYPE);
float humidity = 0;
float temperature = 0;
float termalSensation = 0;

unsigned long long lastEncoder, resultEncoder, timeoutEncoder;
int encoderCount = 0;
bool encoderBuffer = false;
double rpm = 0;

const char* ssid = "raspberry";
const char* password = "presentation";
const char* mqtt_server = "10.42.0.1";
WiFiClient espClient;
PubSubClient client(espClient);
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];

SimpleTimer timerReading;
SimpleTimer timerSendMQTT;
SimpleTimer timerEncoder;

void setup_wifi() {

  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe(waterPumpTopic);
      client.subscribe(ledTopic);
      client.subscribe(fanTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  
  String payloadStr;
  for (int i = 0; i < length; i++) {
    payloadStr += (char)payload[i];
  }

  if (!strcmp(topic, ledTopic)) 
  {
    if(payloadStr == "true") digitalWrite(LED_PIN, LOW);
    else digitalWrite(LED_PIN, HIGH);
  }

  else if (!strcmp(topic, waterPumpTopic)) 
  {
    if(payloadStr == "true") digitalWrite(WATER_PUMP_PIN, LOW);
    else digitalWrite(WATER_PUMP_PIN, HIGH);
  }

  else if (!strcmp(topic, fanTopic)) 
  {
    if(payloadStr == "true") digitalWrite(FAN_PIN, LOW);
    else digitalWrite(FAN_PIN, HIGH);
  }
}

const char* windDirection(float angle) {
  // Convert the angle to an abbreviated compass direction
  if (angle >= 337.5 || angle < 22.5) {
    return "N";
  } else if (angle >= 22.5 && angle < 67.5) {
    return "NE";
  } else if (angle >= 67.5 && angle < 112.5) {
    return "E";
  } else if (angle >= 112.5 && angle < 157.5) {
    return "SE";
  } else if (angle >= 157.5 && angle < 202.5) {
    return "S";
  } else if (angle >= 202.5 && angle < 247.5) {
    return "SW";
  } else if (angle >= 247.5 && angle < 292.5) {
    return "W";
  } else if (angle >= 292.5 && angle < 337.5) {
    return "NW";
  } else {
    return "Invalid Angle"; // Handle invalid angles
  }
}


void updateEncoder()
{
    bool encoderRead = digitalRead(ENCODER_PIN);

    if((millis() - timeoutEncoder) >= 5000) 
    {
        encoderBuffer = false;
        encoderCount = 0;
        rpm = 0;
        lastEncoder = timeoutEncoder = millis();
    }

    if(encoderRead && !encoderBuffer) encoderBuffer = true;

    if(!encoderRead && encoderBuffer)
    {
        encoderBuffer = false;
        encoderCount++;
        timeoutEncoder = millis();
        Serial.println(encoderCount);
    }

    if(encoderCount >= 10)
    {
        resultEncoder = millis() - lastEncoder;
        rpm = (1000.0 / resultEncoder) * 60;

        encoderBuffer = false;
        encoderCount = 0;
        lastEncoder = millis();
    }
}

void updateReadings()
{
    lux = lightMeter.readLightLevel();
    windDir = abs((as5600.rawAngle()*360.0/4095.0));
    
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();
    if ( isnan(humidity) || isnan(temperature) ) {
        humidity = 0;
        temperature = 0;
    }
    else
        termalSensation = dht.computeHeatIndex(temperature, humidity, false);

}

void sendMQTT()
{
    client.publish("WheaterCondition/WindDirection", windDirection(windDir));
    
    sprintf(msg, "%.4f", rpm);
    client.publish("WheaterCondition/WindVelocity", msg);

    sprintf(msg, "%.4f", lux);
    client.publish("WheaterCondition/Luminity", msg);

    sprintf(msg, "%.4f", humidity);
    client.publish("Climate/Humidity" , msg);

    sprintf(msg, "%.4f", temperature);
    client.publish("Climate/Temperature" , msg);

    sprintf(msg, "%.4f", termalSensation);
    client.publish("Climate/ThermalSensation" , msg);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  lightMeter.begin();

  as5600.begin(AS5600_DIR_PIN);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.

  bme.begin(0x76);

  dht.begin();

  pinMode(ENCODER_PIN, INPUT);
  lastEncoder = timeoutEncoder = millis();

  pinMode(FAN_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(WATER_PUMP_PIN, OUTPUT);
  digitalWrite(FAN_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(WATER_PUMP_PIN, HIGH);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  timerReading.setInterval(500, updateReadings);
  timerSendMQTT.setInterval(1000, sendMQTT);
  timerEncoder.setInterval(5, updateEncoder);
}

void loop() {
    if (!client.connected()) reconnect();

    timerReading.run();
    timerSendMQTT.run();
    timerEncoder.run();
    client.loop();
}

// Serial.println();

// Serial.println("BME 280: ");
// Serial.print("Temperature = ");
// Serial.print(bme.readTemperature());
// Serial.println(" °C");
// Serial.print("Pressure = ");
// Serial.print(bme.readPressure() / 100.0F);
// Serial.println(" hPa");
// Serial.print("Approx. Altitude = ");
// Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
// Serial.println(" m");
// Serial.print("Humidity = ");
// Serial.print(bme.readHumidity());
// Serial.println(" %");

// Serial.println();