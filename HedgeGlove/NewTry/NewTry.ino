/*
    ArduIMU_Glove.ino
    Author: Alexander Zigelski
*/

//------------------------------------------------------------------------------
// Includes

//#include "Calibration.h"
/*#include "Adafruit_NeoPixel.h"*/
/*
#include "Receive.h"
#include "Send.h"
#include <EEPROM.h> // required by Calibration.cpp
*/
// FreeIMU
#include "FlexSensors.h"

#include <HMC58X3.h>
#include <I2Cdev.h>
#include <MPU60X0.h>
#include "CommunicationUtils.h"
#include "FreeIMU.h"
#include "SimpleTimer.h" 
/* #include "LEDs.h"*/
#include <Wire.h> // required by I2CBus.cpp
#include <SPI.h>  // required by MPU6000::cpp
/*#include "VibrationMotor.h"*/

//------------------------------------------------------------------------------
// Definitions


#define SENSOR_SEND_RATE    10  // sensor readout rate in ms
#define FLEX_SEND_RATE      20  // flex sensor send rates
#define QUATERION_SEND_RATE 20  // quaternion sen rates
#define SAMPLE_TIMER_RATE   10  // sample IMU rates

//------------------------------------------------------------------------------
// Variables


typedef union {
    int intVal;
    struct {
        char lsb;
        char msb;
    };
} IntUnion;

//AHRS ahrs;
SimpleTimer timer;
// Set the FreeIMU object
FreeIMU imu = FreeIMU();
float q[4];


#define BUTTON_PIN   9    // Digital IO pin connected to the button.  This will be
                          // driven with a pull-up resistor so the switch should
                          // pull the pin to ground momentarily.  On a high -> low
                          // transition the button press logic will execute.


#define PIXEL_PIN    13    // Digital IO pin connected to the NeoPixels.

#define PIXEL_COUNT 1

// Parameter 1 = number of pixels in strip,  neopixel stick has 8
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_RGB     Pixels are wired for RGB bitstream
//   NEO_GRB     Pixels are wired for GRB bitstream, correct for neopixel stick
//   NEO_KHZ400  400 KHz bitstream (e.g. FLORA pixels)
//   NEO_KHZ800  800 KHz bitstream (e.g. High Density LED strip), correct for neopixel stick
//Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

bool oldState = LOW;
bool testVar = true;


//------------------------------------------------------------------------------
// Functions

void setup() {

    // Init Serial for use by Send.cpp and Receive.cpp
    Serial.begin(115200);
    delay(100);
    Wire.begin();

    // Init modules
    FlexSensors::init();
    //VibrationMotor::init();
    //LEDs::init();

    
    //LEDs::setRed(1);
    imu.init(true);
    //LEDs::setRed(0);
    // Indicate init complete

    //LEDs::setBlue(1);
    
/*
    //Calibration::init();
    
    // RGB LED
    pinMode(BUTTON_PIN, INPUT);*/

    
    /*strip.begin();
    strip.show(); // Initialize all pixels to 'off'
    colorWipe(strip.Color(0, 100, 0), 50);*/

    
    timer.setInterval(QUATERION_SEND_RATE, sendIMUData);
    timer.setInterval(SAMPLE_TIMER_RATE, getButtonState);
    timer.setInterval(FLEX_SEND_RATE, sendFlexData);
    //timer.setInterval(500, sendTestData);
}

void loop() {
  //timer.run();
  delay(15);
  sendIMUData();
  //Receive::doTasks();  
}

void getButtonState() {
  // Read button state.
  bool newState = digitalRead(BUTTON_PIN);
  
  // Check if state changed from high to low (button press).
  if (newState == LOW && oldState == HIGH) {
    // Check if button is still low after debounce.
    newState = digitalRead(BUTTON_PIN);
    if (newState == LOW) {
      Serial.println("B0");
      if (testVar==true){
        //colorWipe(strip.Color(100, 0, 0), 50);
        testVar=false;
      }
      else {
        //colorWipe(strip.Color(0, 100, 0), 50);
        testVar=true;
      }
    }
  }
  oldState = newState;
}

void sendIMUData() {
  imu.getQ(q);
  Serial.print("S");  
  serialPrintFloatArr(q, 4);
  Serial.println("");
}
 
void sendFlexData() {
    char packet[64];
    int packetLength = 0;
    IntUnion intUnion;
    FlexSensors::read();    // read sensors before sending

    packet[packetLength++] = 'F';
    IntValToChars(packet, &packetLength, (int)FlexSensors::channel[0]);
    packet[packetLength++] = ',';
    IntValToChars(packet, &packetLength, (int)FlexSensors::channel[1]);
    packet[packetLength++] = ',';
    IntValToChars(packet, &packetLength, (int)FlexSensors::channel[2]);
    packet[packetLength++] = ',';
    IntValToChars(packet, &packetLength, (int)FlexSensors::channel[3]);
    packet[packetLength++] = ',';
    IntValToChars(packet, &packetLength, (int)FlexSensors::channel[4]);
    packet[packetLength++] = '\r';
    packet[packetLength++] = '\n';
    
    Serial.write((uint8_t*)packet, packetLength);
}


void IntValToChars(char* const charArray, int* const index, int i) {
    static const char asciiDigits[10] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', };
    div_t n;
    int print = 0;
    if(i < 0) {
        charArray[(*index)++] = '-';
        i = -i;
    }
    if(i >= 10000) {
        n = div(i, 10000);
        charArray[(*index)++] = asciiDigits[n.quot];
        i = n.rem;
        print = 1;
    }
    if(i >= 1000 || print) {
        n = div(i, 1000);
        charArray[(*index)++] = asciiDigits[n.quot];
        i = n.rem;
        print = 1;
    }
    if(i >= 100 || print) {
        n = div(i, 100);
        charArray[(*index)++] = asciiDigits[n.quot];
        i = n.rem;
        print = 1;
    }
    if(i >= 10 || print) {
        n = div(i, 10);
        charArray[(*index)++] = asciiDigits[n.quot];
        i = n.rem;
    }
    charArray[(*index)++] = asciiDigits[i];
}

// Fill the dots one after the other with a color
/*void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}*/

/*
// old code
/*


/*
void startShow(int i) {
  switch(i){
    case 0: colorWipe(strip.Color(0, 0, 0), 50);    // Black/off
            break;
    case 1: colorWipe(strip.Color(255, 0, 0), 50);  // Red
            break;
    case 2: colorWipe(strip.Color(0, 255, 0), 50);  // Green
            break;
    case 3: colorWipe(strip.Color(0, 0, 255), 50);  // Blue
            break;
    case 4: theaterChase(strip.Color(127, 127, 127), 50); // White
            break;
    case 5: theaterChase(strip.Color(127,   0,   0), 50); // Red
            break;
    case 6: theaterChase(strip.Color(  0,   0, 127), 50); // Blue
            break;
    case 7: rainbow(20);
            break;
    case 8: rainbowCycle(20);
            break;
  }
}


void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, c);    //turn every third pixel on
      }
      strip.show();
     
      delay(wait);
     
      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}
*/

//------------------------------------------------------------------------------
// End of file
