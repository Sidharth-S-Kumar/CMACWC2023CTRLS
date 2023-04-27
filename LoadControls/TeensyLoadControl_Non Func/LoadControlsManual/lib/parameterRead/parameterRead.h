/*
  Morse.h - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.

#ifndef parameterRead_h
#define parameterRead_h

#include <Arduino.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_ADS1X15.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h> 



class parameterRead
{
  public:
    parameterRead(int vLocal, int iLocal,int adcI2C,int adcPin);
    void dot();
    void dash();
  private:
    int _pin;
};

#endif */