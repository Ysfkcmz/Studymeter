
# Studymeter ⏱️
Ein intelligenter, würfelbasierter Pomodoro- und Produktivitäts-Assistent basierend auf ESP32-S3 und FreeRTOS.

## Hardware-Komponenten
* ESP32-S3 Microcontroller
* MPU6050 6-Axis IMU (I2C)
* 0.96" TFT IPS Display (SPI)
* Hall-Sensor & Aktiver Buzzer

## Pinout
* **I2C (MPU6050):** SDA = IO1 | SCL = IO2
* **SPI (TFT):** MOSI = IO11 | SCK = IO12 | CS = IO10 | DC = IO9 | RES = IO13 | BLK = IO21
* **Sonstige:** Buzzer = IO5 | Hall-Sensor = IO4

## Architektur & Software
* **RTOS:** FreeRTOS (Task-Management & Mutex für I2C/SPI)
* **DPM:** Dynamic Power Management (Hardware Sleep-Modus)
* **Framework:** ESP-IDF / C++
