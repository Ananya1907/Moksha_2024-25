# Moksha_2024-25
This repository contains the firmware for the electronic control unit (ECU) and driver display of an electric All-Terrain Vehicle (ATV). The system is built around two main components: a central ESP32-S3 microcontroller and a 7" CrowPanel HMI display, which communicate wirelessly via the ESP-NOW protocol.

## Key Features

### Microcontroller (ESP32-S3)
* Controls essential vehicle systems based on defined logic:
  * Accumulator Isolation Relay (AIR)
  * Tractive System Activation Light (TSAL)
  * Reverse Light & Brake Light
  * Reverse Alarm
  * Ready to Drive Sound (RTDS)
* Reads real-time speed and range information from an attached GPS module.
* Transmits critical vehicle data to the driver display using the ESP-NOW protocol.

### Driver Display (CrowPanel 7")
* Provides the driver with a clean, real-time Human-Machine Interface (HMI).
* Dynamically displays critical vehicle data received from the microcontroller:
  * Speed (km/h)
  * Distance Traveled (Range)
  * FNR State (Forward, Neutral, Reverse)

## Repository Structure
This repository is organized into two main folders, each containing a self-contained PlatformIO project.
* 'Driver_Display/': Contains the firmware for the CrowPanel display, responsible for receiving data and updating the HMI.
* 'Microcontroller': Contains the firmware for the main ESP32-S3 controller, which manages all vehicle logic and data transmission.

## Setup
Each folder is a PlatformIO project. To build or flash the code, open the desired folder (Driver_Display or Microcontroller) in a compatible IDE such as Visual Studio Code with the PlatformIO extension installed.
