
#include <gfxfont.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#include <tft.hpp>
#include <microhil.hpp>


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int enginespeed = 0;
int old_enginespeed = -1;
int vehiclespeed = 0;
int old_vehiclespeed = -1;
int mode = RPM_MODE;
int old_mode = RPM_MODE;
int encoderPinALast = LOW;
int encoderPinANow = LOW;
int encoderPinA = 18;
int encoderPinB = 4;
int buttonPin = 19;
int signalRPMPin = 30;
int signalWheelPin = 32;
int encoderPos = 0;
int counter;
int abssignal = 0;
unsigned long debounce_button = 0;
unsigned long debounce_encoder = 0;
int debounce_time_button = 200;
int debounce_time_encoder = 5;
int prescaler_rpm = 1;
int prescaler_speed = 64;





void setup() {
  Serial.begin(115200);
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay();
  
  counter =119;

  pinMode(31,INPUT);
  pinMode(33,INPUT);
  
  pinMode(encoderPinA,INPUT_PULLUP);
  pinMode(encoderPinB,INPUT_PULLUP);
  pinMode(buttonPin,INPUT_PULLUP);
  pinMode(signalWheelPin,OUTPUT);
  pinMode(signalRPMPin,OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(encoderPinA), calcEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(buttonPin), button, FALLING);  

  cli();
  //setup INT-Regs for crank signal
  TCCR4A = 0;
  TCCR4B = 0;
  TCCR4B |= (1 << WGM12);
  setPrescalerRPM();
  TCNT4  = 0;
  //setup INT-Regs for wheel speed
  TCCR5A = 0;
  TCCR5B = 0;
  TCCR5B |= (1 << WGM12);
  setPrescalerSpeed();
  TCNT5  = 0;
  sei();  
}

int calcSpeedFrequency()
{
  float speedms = vehiclespeed/3.6;
  float ff = speedms/rollingCircumference*pulsesPerRotation;
  float f = 2*ff;
  int tf;
  tf=round(16000000/(prescaler_speed*f)-1);
  OCR5A = tf;
  TIMSK5 |= (1 << OCIE5A);
}

void calcRPMFrequency()
{
  int f;
  int tf;
  f=enginespeed*2;
  tf=16000000/(prescaler_rpm*f)-1;
  OCR4A = tf;
  TIMSK4 |= (1 << OCIE4A);
}


void button()
{
  if((millis()-debounce_button)>debounce_time_button)
  {
    if(mode == RPM_MODE)
      mode = SPEED_MODE;
    else
      mode = RPM_MODE;
    debounce_button = millis();
  }
  Serial.print("Button ");
  Serial.println(mode);
}


void calcEncoder()
{
  if((millis()-debounce_encoder)>debounce_time_encoder)
  {
    Serial.println("Interrupt");
    Serial.print("EncoderPinALast:");
    Serial.println(encoderPinALast);
    Serial.print("EncoderPinANow:");
    Serial.println(digitalRead(encoderPinA));
    Serial.print("EncoderPinBNow:");
    Serial.println(digitalRead(encoderPinB));    
    encoderPinANow = digitalRead(encoderPinA);
    if ((encoderPinALast == HIGH) && (encoderPinANow == LOW)) {
      if (digitalRead(encoderPinB) == HIGH) {
        encoderPos=1;
      } else {
        encoderPos=-1;
      }
    }
    encoderPinALast = encoderPinANow;
    if(mode == SPEED_MODE)
    {
      int oldvehiclespeed = vehiclespeed;
      vehiclespeed = vehiclespeed + incSpeed * encoderPos;
      if(vehiclespeed > maxSpeed)
        vehiclespeed = maxSpeed;
      if (vehiclespeed < 0)
        vehiclespeed = 0;
      calcSpeedFrequency();
    }
    else
    {
      int oldenginespeed = enginespeed;
      enginespeed = enginespeed + incRPM * encoderPos;
      if(enginespeed > maxRPM)
        enginespeed = maxRPM;
      if(enginespeed < 0)
        enginespeed = 0;
      calcRPMFrequency();
    }
    debounce_encoder=millis();
  }
}

void setPrescalerRPM()
{
  switch(prescaler_rpm){
    case 1:{
       TCCR4B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
       break;}
    case 8:{
       TCCR4B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
       break;}
    case 64:{
       TCCR4B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
       break;}
    case 256:{
       TCCR4B |= (1 << CS12) | (0 << CS11) | (0 << CS10);
       break;}
    case 1024:{
       TCCR4B |= (1 << CS12) | (0 << CS11) | (1 << CS10);
       break;}
    default:
       TCCR4B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
    } 
}

void setPrescalerSpeed()
{
  switch(prescaler_speed){
    case 1:{
       TCCR5B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
       break;}
    case 8:{
       TCCR5B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
       break;}
    case 64:{
       TCCR5B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
       break;}
    case 256:{
       TCCR5B |= (1 << CS12) | (0 << CS11) | (0 << CS10);
       break;}
    case 1024:{
       TCCR5B |= (1 << CS12) | (0 << CS11) | (1 << CS10);
       break;}
    default:
       TCCR5B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
    } 
}

ISR(TIMER4_COMPA_vect) 
{
 digitalWrite(signalRPMPin,crankValue[counter]);
 counter--;
 if(counter==0)
   counter=119;
}  
  
ISR(TIMER5_COMPA_vect) 
{
 digitalWrite(signalWheelPin, !digitalRead(signalWheelPin));
}


void loop() {
  if((enginespeed != old_enginespeed)||(vehiclespeed != old_vehiclespeed)||(mode!=old_mode))
  {
    old_enginespeed = enginespeed;
    old_vehiclespeed = vehiclespeed;
    old_mode = mode;
    display.clearDisplay();
    display.setTextSize(1);      
    display.setTextColor(SSD1306_WHITE); 
    display.cp437(true); 
    display.setCursor(0, 0); 
    display.println(F("Speed"));
    display.setCursor(60, 0);
    display.println(F("RPM"));     
    display.setTextSize(2);
    if(vehiclespeed >= 100)
        display.setCursor(5,15);
    else
        display.setCursor(15, 15);  
    display.println(vehiclespeed);
    display.setCursor(60, 15);    
    if(enginespeed < 10000)
      display.setCursor(70,15);
    else
      display.setCursor(60,15);
    display.println(enginespeed);
    if(mode == RPM_MODE)
      display.drawLine(60, 7, 110, 7, SSD1306_WHITE); 
    else
      display.drawLine(0, 7, 50, 7, SSD1306_WHITE); 
//    Serial.println(vehiclespeed); 
//    Serial.println(enginespeed); 
    
    display.display();
  }         

}