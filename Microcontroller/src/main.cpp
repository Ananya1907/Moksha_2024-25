// ESP32 Code for Activating the Tractive System
#include <Arduino.h>
#include <TinyGPS++.h> // GPS library (Serial2)
#include <math.h>
#include <esp_now.h>
#include <WiFi.h>

// Define GPIO pins
#define FNR_NEUTRAL_PIN      15 // Input: Gear position in neutral
#define FNR_REVERSE_PIN      16 // Input: Gear position in reverse
#define ACCELERATOR_PIN      9  // Input: Accelerator pedal
#define BRAKE_PEDAL_PIN      17 // Input: Brake pedal
#define START_STOP_PIN       6  // Input: Start/Stop button
#define AIR_RELAY_PIN        39 // Output: AIR relay
#define TSAL_LED_PIN         3  // Output: TSAL indicator
#define RTDS_BUZZER          40 // Output: Ready to drive sound / Reverse sound
#define REVERSE_LED          42 // Output: Reverse Light

TinyGPSPlus GPS;  // GPS via Serial2
#define GPS_RX 8
#define GPS_TX 7
#define MIN_MOV_SPEED 2
float speedbuffer[5];
byte speedindex = 0;

// Define thresholds for inputs
#define BRAKE_TH 3600
#define ACCELERATOR_TH 3600
#define NEUTRAL_TH 3600
#define REVERSE_TH 3600
#define STARTSTOP_TH 3600

// Declare global variables for timing
unsigned long lastReverseTime = 0;
bool reverseBuzzerState = false;

typedef struct message {
    float speed, distance, lastLat, lastLon;
    int fnr;
} message_t;

float currentLat = 0.0;
float currentLon = 0.0;
bool activated = 0;

// Receiver MAC Address (replace with your receiver's MAC)
uint8_t receiverMacAddress[] = {0x84, 0xFC, 0xE6, 0x69, 0xEF, 0x64};
message_t myMessage;

// Callback function
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Last Packet Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371.0; // Radius of the Earth in kilometers
  if(lat2 == lat1 && lon2 == lon1) return 0;
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  
  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(radians(lat1)) * cos(radians(lat2)) *
            sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c; // Distance in kilometers
}

void updateDistance() {
  if (myMessage.lastLat != 0.0 && myMessage.lastLon != 0.0) {
    float segmentDistance = calculateDistance(myMessage.lastLat, myMessage.lastLon, currentLat, currentLon);
    if(segmentDistance > 0.01){
      myMessage.distance += segmentDistance;
      myMessage.lastLat = currentLat;
      myMessage.lastLon = currentLon;
    }
  }
    myMessage.lastLat = currentLat;
    myMessage.lastLon = currentLon;
}

void setup() {
  // Configure input pins
  pinMode(FNR_NEUTRAL_PIN, INPUT_PULLDOWN);
  pinMode(FNR_REVERSE_PIN, INPUT_PULLDOWN);
  pinMode(ACCELERATOR_PIN, INPUT_PULLDOWN);
  pinMode(BRAKE_PEDAL_PIN, INPUT_PULLDOWN);
  pinMode(START_STOP_PIN, INPUT_PULLDOWN);

  // Configure output pins
  pinMode(AIR_RELAY_PIN, OUTPUT);
  pinMode(TSAL_LED_PIN, OUTPUT);
  pinMode(RTDS_BUZZER, OUTPUT);
  pinMode(REVERSE_LED, OUTPUT);

  // Set initial states for outputs
  digitalWrite(AIR_RELAY_PIN, LOW);
  digitalWrite(TSAL_LED_PIN, LOW);
  digitalWrite(RTDS_BUZZER, LOW);
  digitalWrite(REVERSE_LED, LOW);

  Serial.begin(115200); /* Serial monitor */
  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  WiFi.mode(WIFI_STA);

  // Initialize Message values
  myMessage.distance = 0.0;
  myMessage.fnr = 2;
  myMessage.lastLat = 0.0;
  myMessage.lastLon = 0.0;
  myMessage.speed = 0.0;

  // Initialize ESPNow
  if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
  }

  esp_now_register_send_cb(OnDataSent);

  // Register peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
}

void loop() {
  int Reverse, Neutral, brake, StartStop;
  float avg_speed;
  float acc;
  bool fnrReverse, fnrForward, fnrNeutral, brakePressed, startStopPressed;
  bool acceleratorPressed = 0;

  // Read input states
  Reverse = analogRead(FNR_REVERSE_PIN);
  Neutral = analogRead(FNR_NEUTRAL_PIN);
  acc = analogRead(ACCELERATOR_PIN);
  brake = analogRead(BRAKE_PEDAL_PIN);
  StartStop = analogRead(START_STOP_PIN);

  fnrReverse = (Reverse>REVERSE_TH) ? 1:0;
  fnrNeutral = (Neutral>NEUTRAL_TH) ? 1:0;
  fnrForward = (!fnrNeutral && !fnrReverse) || (fnrNeutral && fnrReverse);
  acceleratorPressed = (acc>ACCELERATOR_TH) ? 1:0;
  brakePressed = (brake>BRAKE_TH) ? 1:0;
  startStopPressed = (StartStop>STARTSTOP_TH) ? 1:0;

  // Debugging information
  Serial.print("FNR Reverse: "); Serial.println(fnrReverse);
  Serial.print("FNR Forward: "); Serial.println(fnrForward);
  Serial.print("FNR Neutral: "); Serial.println(fnrNeutral);
  Serial.print("Accelerator Pressed: "); Serial.println(acceleratorPressed);
  Serial.print("Brake Pressed: "); Serial.println(brakePressed);
  Serial.print("Start/Stop: "); Serial.println(startStopPressed);
  Serial.println("***********************************************************");

  // Serial.print("FNR Reverse: "); Serial.println(Reverse);
  // Serial.print("FNR Forward: "); Serial.println(Forward);
  // Serial.print("Accelerator Pressed: "); Serial.println(acc);
  // Serial.print("Brake Pressed: "); Serial.println(brake);
  // Serial.print("Start/Stop: "); Serial.println(StartStop);

  // Logic to activate tractive system
  if (fnrNeutral && !acceleratorPressed && brakePressed && startStopPressed) {
    // Activate AIR
    digitalWrite(AIR_RELAY_PIN, HIGH);

    // Turn on TSAL indicator
    digitalWrite(TSAL_LED_PIN, HIGH);

    // Tractive system has been activated
    activated = 1;

    // Start Ready to Drive sound
    for(int i=0; i<=2; i++){
      digitalWrite(RTDS_BUZZER, HIGH);
      delay(500);
      digitalWrite(RTDS_BUZZER, LOW);
      delay(200);}

    Serial.println("Tractive System Activated: Ready to Drive");
  } 
  else {
    // Deactivate outputs if conditions are not met
    digitalWrite(RTDS_BUZZER, LOW);

    Serial.println("Tractive System Not Activated");
  }

  // FNR
  myMessage.fnr = fnrForward ? 1 : (fnrReverse ? 3 : 2);
  
  // Read GPS data
  while(Serial2.available() > 0){
    // // Debugging
    // char c = Serial2.read();
    // Serial.write(c);
    GPS.encode(Serial2.read());
    if(GPS.location.isUpdated() && GPS.location.isValid()){
      currentLat = GPS.location.lat();
      currentLon = GPS.location.lng();
      if(GPS.speed.isValid()){
        speedbuffer[speedindex] = GPS.speed.kmph();
        speedindex = (speedindex + 1) % 5;
        avg_speed = 0;
        for(byte i=0; i<5; i++) avg_speed += speedbuffer[i];
        avg_speed = avg_speed / 5;

        myMessage.speed = (avg_speed >= MIN_MOV_SPEED) ? avg_speed : 0.0;
      }
    }
  }
  updateDistance();

  // Debugging GPS
  Serial.print("Current Latitude: "); Serial.println(currentLat);
  Serial.print("Current Longitude: "); Serial.println(currentLon);
  Serial.print("Last Latitude: "); Serial.println(myMessage.lastLat);
  Serial.print("Last Longitude: "); Serial.println(myMessage.lastLon);
  Serial.print("Speed: "); Serial.println(myMessage.speed);
  Serial.print("Distance: "); Serial.println(myMessage.distance);
  Serial.println("**********************************************************************");

  // Send GPS and BMS data to Driver Display ESP
  esp_err_t result = esp_now_send(receiverMacAddress, 
                                  (uint8_t *) &myMessage, 
                                  sizeof(message_t));
  
  if (result == ESP_OK) {
      Serial.println("Sent with success");
  } else {
      Serial.println("Error sending the data");
  }

  // Reverse Alarm and LED
  if(fnrReverse && activated && !fnrForward){
    if (millis() - lastReverseTime >= 500) {
      reverseBuzzerState = !reverseBuzzerState;
      digitalWrite(RTDS_BUZZER, reverseBuzzerState);
      lastReverseTime = millis();
    }
    digitalWrite(REVERSE_LED, HIGH);
  } else {
      digitalWrite(REVERSE_LED, LOW);
      digitalWrite(RTDS_BUZZER, LOW);
  }

  delay(200); // Short delay for stability
}
