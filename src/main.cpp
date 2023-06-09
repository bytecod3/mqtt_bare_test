#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h> 
#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;

#define MQTT_SERVER "192.168.1.117"
#define MQTT_PORT 1883

#define WIFI_SSID "Gakibia-Unit3"
#define WIFI_PASSWORD "password"

WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);

// create queues
QueueHandle_t pressure_queue;

void setup_mqtt(){
  mqtt_client.setServer(MQTT_SERVER, MQTT_PORT);
}

void reconnect(){
  Serial.println("Connecting to MQTT server");

  while (!mqtt_client.connected()) {
      Serial.println("Reconnecting to MQTT Broker..");
      String client_id = "ESP32Client-";
      client_id += String(random(0xffff), HEX);
      
      if (mqtt_client.connect(client_id.c_str())) {
        Serial.println("Connected to MQTT broker");

      }
  }

}

void readBMP(void* pvParameters){
  while (true){

    // read BMP sensor
    float pressure = bmp.readAltitude();
    
    if(xQueueSend(pressure_queue, &pressure, portMAX_DELAY) != pdPASS){
      Serial.println("Queue is full");
    } else{
      // Serial.println("Data sent to queue");
    }

  }
}

void printData(void* pvParameters){

  float p;
  while (true){

    if(xQueueReceive(pressure_queue, &p, portMAX_DELAY) == pdPASS){
      Serial.println(p);
    } else{
      Serial.println("Data receiving failed");
    }

  }

  delay(10);
}


void transmitoverMQTT(void* pvParameters){

  float p;
  char pressure_data[10];

  while (true){

    if(xQueueReceive(pressure_queue, &p, portMAX_DELAY) == pdPASS){
      
      sprintf(pressure_data, "%.2f", p);

      if(mqtt_client.publish("pressure-data", pressure_data)){
        mqtt_client.publish("pressure-data", pressure_data);

      } else{
        Serial.println("Data publish failed");
      }

    } else{
      Serial.println("Data receiving failed");
    }

  }

  delay(10);
}

void blinker(void* pvParameters){
  while(true){
    digitalWrite(4, HIGH);
    vTaskDelay(portTICK_PERIOD_MS / 1000);

    digitalWrite(4, LOW);
    vTaskDelay(portTICK_PERIOD_MS / 1000);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(4, OUTPUT);

  setup_mqtt();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  if(WiFi.status() != WL_CONNECTED){
    Serial.print("Connected");
  }

  // try establish connection to the BMP
  if (!bmp.begin()) {
    Serial.println("[-]Could not find a valid BMP");
    while (1) {}
  }

  Serial.println("[+]BMP initialized");

  // create queue
  pressure_queue = xQueueCreate(2, sizeof(float));

  if(pressure_queue == NULL){
    Serial.println("Failed to create queue");
  } else{
    Serial.println("Queue creation success");
  }

  if(xTaskCreate(
    readBMP,
    "read bmp",
    2048,
    NULL,
    1,
    NULL
  ) == pdPASS){
    Serial.println("Reading task creation success");
  }else{
    Serial.println("Reading task creation failed");
  }

  if(xTaskCreate(
    printData,
    "Print data",
    2048,
    NULL,
    1,
    NULL
  ) == pdPASS){
    Serial.println("Print data task creation success");
  }else{
    Serial.println("Print data task creation failed");
  }

  if(xTaskCreate(
    printData,
    "Print data",
    2048,
    NULL,
    1,
    NULL
  ) == pdPASS){
    Serial.println("Print data task creation success");
  }else{
    Serial.println("Print data task creation failed");
  }

  if(xTaskCreate(
    transmitoverMQTT,
    "transmit MQTT",
    2048,
    NULL,
    1,
    NULL
  ) == pdPASS){
    Serial.println("Transmit data task creation success");
  }else{
    Serial.println("Transmit task creation failed");
  }

  if(xTaskCreate(
    blinker,
    "blinker",
    2048,
    NULL,
    1,
    NULL
  ) == pdPASS){
    Serial.println("Transmit data task creation success");
  }else{
    Serial.println("Transmit task creation failed");
  }




}

void loop() {
  if (!mqtt_client.connected()){
      reconnect();
      // mqtt_client.loop();
    }

}