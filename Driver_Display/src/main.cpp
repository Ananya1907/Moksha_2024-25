/**************************CrowPanel ESP32 HMI Display Example Code************************
Version		 :	1.1
Suitable for :	CrowPanel ESP32 HMI Display
Product link :	https://www.elecrow.com/esp32-display-series-hmi-touch-screen.html
Code	link :	https://github.com/Elecrow-RD/CrowPanel-ESP32-Display-Course-File
Lesson	link :	https://www.youtube.com/watch?v=WHfPH-Kr9XU
Description	 :	The code is currently available based on the course on YouTube, 
				if you have any questions, please refer to the course video: Introduction 
				to ask questions or feedback.
******************************************************************************************/




/******************************************************************************/
#include <LittleFS.h>
#include <SD.h>
#include <SPI.h>
#include "gfx_conf.h"   //Please modify the setting of display driver according to the board you are using in gfx_conf.h.
#include <esp_now.h>
#include <WiFi.h>

//Modify the corresponding pin according to the circuit diagram.
#define SD_MOSI 11
#define SD_MISO 13
#define SD_SCK 12
#define SD_CS 10

SPIClass SD_SPI;

#define SDA_PIN 19
#define SCL_PIN 20

/******************************************************************************/

typedef struct message {
  float speed, distance, lastLat, lastLon;
  int fnr;
} message_t;

message_t myMessage;

// SD test

int sd_init()
{
  //SD_SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
   SD_SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, SD_SPI, 80000000)) {
    Serial.println("Card Mount Failed");
    return 1;
  }
  else {
    Serial.println("Card Mount Successed");
  }
  // listDir(SD, "/", 0);
  Serial.println("TF Card init over.");
  return 0;
}

uint16_t read16(File& f) {
  uint16_t result;
  ((uint8_t*)&result)[0] = f.read();
  ((uint8_t*)&result)[1] = f.read();
  return result;
}

uint32_t read32(File& f) {
  uint32_t result;
  ((uint8_t*)&result)[0] = f.read();
  ((uint8_t*)&result)[1] = f.read();
  ((uint8_t*)&result)[2] = f.read();
  ((uint8_t*)&result)[3] = f.read();
  return result;
}

void displayPhoto16(const char* filename) {
  int16_t x, y;

  digitalWrite(SD_CS, LOW);
  File file = SD.open(filename);

  if (!file) {
    Serial.print("Error opening file: ");
    Serial.println(filename);
    digitalWrite(SD_CS, HIGH);
    return;
  }
  
  if (read16(file) != 0x4D42) {
    Serial.println("File is not a BMP file!");
    file.close();
    digitalWrite(SD_CS, HIGH);
    return;
  }
  
  file.seek(10);
  uint32_t dataOffset = read32(file);
  file.seek(18);
  uint32_t width = read32(file);
  uint32_t height = read32(file);

  Serial.print("BMP Width: "); Serial.println(width);
  Serial.print("BMP Height: "); Serial.println(height);

  if (width > tft.width() || height > tft.height()) {
    Serial.println("BMP too large for screen!");
    file.close();
    digitalWrite(SD_CS, HIGH);
    return;
  }

  // Calculate center coordinates
  x = (tft.width() - width) / 2;
  y = (tft.height() - height) / 2;

  Serial.print("Calculated x: "); Serial.println(x);
  Serial.print("Calculated y: "); Serial.println(y);

  file.seek(dataOffset);

  uint16_t rowSize = (width * 2 + 3) & ~3; // Size of a row in bytes (16 bit)
  uint8_t row[rowSize];

  tft.startWrite(); // Very important with LovyanGFX for RGB panels

  for (int16_t rowNumber = height - 1; rowNumber >= 0; rowNumber--) {
    file.read(row, rowSize);
    uint16_t* pixelData = (uint16_t*)row;
    for (int16_t colNumber = 0; colNumber < width; colNumber++) {
      tft.writePixel(colNumber + x, rowNumber + y, pixelData[colNumber]);
    }
  }

  tft.endWrite();
  file.close();
  digitalWrite(SD_CS, HIGH);
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    memcpy(&myMessage, incomingData, sizeof(myMessage));
    
    Serial.print("FNR: ");
    Serial.println(myMessage.fnr);
    Serial.print("Speed: ");
    Serial.println(myMessage.speed);
    Serial.print("Distance: ");
    Serial.println(myMessage.distance);
}

void overlayDynamicData() { 
  // Clear previous data (use rectangles to clear specific regions)
  tft.fillRect(360, 350, 100, 30, TFT_BLACK); // FNR
  tft.fillRect(310, 255, 60, 30, TFT_BLACK); // Speed
  tft.fillRect(70, 225, 60, 30, TFT_BLACK); // Distance

  // Display dynamic data on top of the speedometer
  tft.setTextColor(TFT_WHITE);

  // FNR
  tft.setCursor(360, 350);
  if(myMessage.fnr == 1) tft.print("FORWARD");
  else { if(myMessage.fnr == 2) tft.print("NEUTRAL");
         else tft.print("REVERSE");
        }

  // Speed
  tft.setCursor(310, 255);
  tft.print(myMessage.speed);

  // Distance
  tft.setCursor(70, 225);
  tft.print(myMessage.distance);
}

void setup()
{
  Serial.begin(115200); /*Serial init*/
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Init Display
  tft.begin();
  tft.setRotation(2);
  tft.setTextSize(2);
  tft.fillScreen(TFT_BLACK);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  if (sd_init() == 0)
  {
    Serial.println("TF init success");
  } else {
    Serial.println("TF init fail");
  }

  //display images you place in SD Card
  displayPhoto16("/frame54.bmp");
  delay(2000);
  tft.fillScreen(TFT_BLACK);
  delay(500);

  displayPhoto16("/Speedometer1.bmp");     // bg image

  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Update the dynamic data continuously in loop
   overlayDynamicData();
   delay(200);
}
