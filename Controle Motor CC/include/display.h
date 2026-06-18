#ifndef DISPLAY_H
#define DISPLAY_H

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <driver/adc.h>

void initDisplay();
void showMessage(const char* msg);
void updateDisplay(float vel, float corrente, float tensao, float setpoint);

#endif
