#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define SCREEN_WIDTH   128 // OLED display width, in pixels
#define SCREEN_HEIGHT  32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define FIGHTER_WIDTH  16
#define FIGHTER_HEIGHT 9
#define MISSILE_WIDTH  8
#define MISSILE_HEIGHT 4
#define FIGHTER_SPEED  5
#define GAME_SPEED     30

const int rightBtnPin = 2;
const int leftBtnPin = 3;
const int fireBtnPin = 4;

int fighterPositionX = 0;
int fighterPositionY = 0;
int firePositionX = 0;
int firePositionY = 0;
int missilePositionX = 0;
int missilePositionY = 0;

int score = 0;
unsigned long previousTime = 0;
bool start = true;

// Jet Fighter
static const unsigned char PROGMEM jetFighter[] =
{ 0b00000001, 0b10000000,
  /*0b00000011, 0b11000000,*/
  0b00000011, 0b11000000,
  0b00000011, 0b11000000,
  0b00011111, 0b11111000,
  0b11111111, 0b11111111,
  0b01110011, 0b11001110,
  0b00000011, 0b11000000,
  /*0b00000111, 0b11100000,*/
  0b00011111, 0b11111000,
  0b00000001, 0b10000000
};

// Enemy missile
static const unsigned char PROGMEM missile[] = {
  0b00111100,
  0b01111110,
  0b00111100,
  0b00011000
};

// Gyroscope
MPU6050 mpu;

#define INTERRUPT_PIN 5  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
      Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
      Serial.println(F(")..."));
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  pinMode(rightBtnPin, INPUT);
  pinMode(leftBtnPin, INPUT);
  
  if(!oled.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  welcome();

  fighterPositionX = (oled.width()  - FIGHTER_WIDTH ) / 2;
  fighterPositionY = oled.height() - FIGHTER_HEIGHT;
  firePositionY = fighterPositionY;
}

void loop() {

  if (!dmpReady) return;

  float roll;

 if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    roll = ypr[2] * 180/M_PI;
    
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
  
  unsigned long currentTime = millis();

  if(currentTime - previousTime > 60000) {
    previousTime = currentTime;
    start = false;
    showScore();
    score = 0;
  }

  if(start) {
    oled.clearDisplay();
    bool hitTarget = false;
    
    if(digitalRead(rightBtnPin) == HIGH && fighterPositionX < oled.width() - FIGHTER_WIDTH - FIGHTER_SPEED) {
      fighterPositionX += FIGHTER_SPEED;
    } else if(digitalRead(leftBtnPin) == HIGH && fighterPositionX > FIGHTER_SPEED) {
      fighterPositionX -= FIGHTER_SPEED;
    }

    fighterPositionX = map(roll, -25, 25, oled.width() - FIGHTER_WIDTH, 0);
  
    missilePositionY++;
    drawMissile(missilePositionX, missilePositionY);
    if(missilePositionY > oled.height() - MISSILE_HEIGHT - 1) {
      missilePositionY = 0;
      missilePositionX = random(0, oled.width() - MISSILE_WIDTH);
    }
    drawFighterGraphic(fighterPositionX, fighterPositionY);
      
    if(digitalRead(fireBtnPin) == HIGH) {
      firePositionX = fighterPositionX + (FIGHTER_WIDTH / 2) - 1;
      firePositionY--;
  
      if(firePositionY < 0) {
        firePositionY = fighterPositionY;
      }
      
      drawFire(firePositionX, firePositionY);
  
      if(firePositionX > missilePositionX 
        && firePositionX < missilePositionX + MISSILE_WIDTH
        && firePositionY > missilePositionY
        && firePositionY < missilePositionY + MISSILE_HEIGHT) {
          missilePositionY = 0;
          missilePositionX = random(0, oled.width() - MISSILE_WIDTH);
          score++;
      }
      
    } else {
      firePositionY = fighterPositionY;
    }
    
    oled.display();
    delay(GAME_SPEED);
  }
}

void drawFighterGraphic(int x, int y) {
  oled.drawBitmap(x, y, jetFighter, FIGHTER_WIDTH, FIGHTER_HEIGHT, 1);
}

void drawFire(int x, int y) {
  oled.drawPixel(x, y, SSD1306_WHITE);
  oled.drawPixel(x + 1, y, SSD1306_WHITE);
}

void drawMissile(int x, int y) {
  oled.drawBitmap(x, y, missile, MISSILE_WIDTH, MISSILE_HEIGHT, 1);
}

void showScore() {
  oled.clearDisplay();
  oled.setTextSize(2); // Draw 2X-scale text
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(5, 0);
  oled.print(F("score : "));
  oled.print(score);
  oled.display();      // Show initial text
}

void welcome() {
  oled.clearDisplay();
  oled.setTextSize(2); // Draw 2X-scale text
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(18, 10);
  oled.print(F("FireZone"));
  oled.display();      // Show initial text
  delay(2000);
}
