// ---------------------------------------------------------------- //
// Using Arduino Nano 33 BLE
// Tested on 14 December 2022
// Authors: Afonso Alemão, Rui Daniel, Tomás Fonseca
// Sources: https://github.com/arduino-libraries/Arduino_LSM9DS1
// ---------------------------------------------------------------- //

#include <Arduino.h>
#include <Wire.h>

/* The LSM9DS1 embeds 32 slots of 16-bit data FIFO for each of the gyroscope’s three output
  channels, yaw, pitch and roll, and 16-bit data FIFO for each of the accelerometer’s three
  output channels, X, Y and Z. */
class LSM9DS1Class_new {
  public:
    LSM9DS1Class_new(TwoWire& wire);
    virtual ~LSM9DS1Class_new();

    int begin();
    void end();

    /* Controls whether a FIFO is continuously filled, or a single reading is stored. */
    /* Defaults to one-shot. */
    void setContinuousMode();
    void setOneShotMode();

    /* Accelerometer. */
    /* Results are in g (earth gravity). */
    virtual int readAcceleration(float& x, float& y, float& z); 
    /* Number of samples in the FIFO. */
    virtual int accelerationAvailable(); 
    /* Sampling rate of the sensor. */
    virtual float accelerationSampleRate(); 

  private:
    bool continuousMode;
    int readRegister(uint8_t slaveAddress, uint8_t address);
    int readRegisters(uint8_t slaveAddress, uint8_t address, uint8_t* data, size_t length);
    int writeRegister(uint8_t slaveAddress, uint8_t address, uint8_t value);

  private:
    TwoWire* _wire;
};

extern LSM9DS1Class_new IMU_LSM9DS1_new;
#undef IMU
#define IMU IMU_LSM9DS1_new



/* Master : The device which initiates a transfer, generates clock signals and terminates a transfer. */
/* Slave : The device addressed by the master. */

/* I2C operation:
  The I2C protocol involves using two lines to send and receive data: a serial clock pin (SCL) that the
  Arduino Controller board pulses at a regular interval, and a serial data pin (SDA) over which data is 
  sent between the two devices. 
  As the clock line changes from LOW to HIGH (known as the rising edge of the clock pulse), a single bit 
  of information, that will form in sequence the address of a specific device and a command or data, is 
  transferred from the board to the I2C device over the SDA line. 
  When an address is sent, each device in the system compares the first seven bits after a start 
  condition with its address. If they match, the device considers itself addressed by the master. */

/* The I2C embedded inside the LSM9DS1 behaves like a slave device and the following protocol must be 
  adhered to. In the I2C of the accelerometer and gyroscope sensor, after the start condition (ST) a 
  slave address is sent, once a slave acknowledge (SAK) has been returned, an 8-bit sub-address (SUB) is 
  transmitted. The 7 LSb represent the actual register address while the CTRL_REG8 (22h) (IF_ADD_INC) bit 
  defines the address increment. */

/* Slave address. */
#define LSM9DS1_ADDRESS            0x6b 

/* Address that identifies the accelerometer. */
#define LSM9DS1_WHO_AM_I           0x0f 

/* Address that tells if there is new data available. */
#define LSM9DS1_STATUS_REG         0x17 

/* Writting to this register, the accelerometer operates in normal mode and the gyroscope is powered down (operating mode). */
#define LSM9DS1_CTRL_REG6_XL       0x20

/* When the CTRL_REG8 (22h) IF_ADD_INC bit is ‘0’ the address used to read/write data remains the same for every block. 
  When the CTRL_REG8 (22h) IF_ADD_INC bit is ‘1’, the address used to read/write data is increased at every block. */
#define LSM9DS1_CTRL_REG8          0x22 

/* When only accelerometer is activated and the gyroscope is in power down, starting from OUT_X_XL (28h - 29h) multiple 
  reads can be performed. */
#define LSM9DS1_OUT_X_XL           0x28 

LSM9DS1Class_new::LSM9DS1Class_new(TwoWire& wire) :
  continuousMode(false), _wire(&wire)
{
}

LSM9DS1Class_new::~LSM9DS1Class_new()
{
}

int LSM9DS1Class_new::begin()
{
  _wire->begin(); /* Join I2C bus (address optional for master). */

  /* In LSM9DS1_CTRL_REG8, write IF_ADD_INC bit equal to 1 that means register address will be automatically 
  incremented during a multiple byte access with a serial interface (I^2C or SPI). Write last bit (SW_RESET) 
  equal to 1 means device will be reseted. This bit is cleared by hardware after next flash boot. */
  writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG8, 0x05); 
  delay(10);

  /* WHO_AM_I register that identifies the accelerometer. */
  if (readRegister(LSM9DS1_ADDRESS, LSM9DS1_WHO_AM_I) != 0x68) { 
    end();

    return 0;
  }

  /* Writes in register LSM9DS1_CTRL_REG6_XL in order to the accelerometer operate in normal mode and the
  gyroscope is powered down (operating mode), output data rate ODR_XL = 011, that correspondes to 
  ODR selection of 119 Hz and accelerometer full scale selection is given by ±4g (FS_XL = 10). */
  writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL, 0x70); 
  return 1;
}

void LSM9DS1Class_new::setContinuousMode() {
  /* Enable FIFO (see docs https://www.st.com/resource/en/datasheet/DM00103319.pdf). */
  writeRegister(LSM9DS1_ADDRESS, 0x23, 0x02);
  /* Set continuous mode. */
  writeRegister(LSM9DS1_ADDRESS, 0x2E, 0xC0);

  continuousMode = true;
}

void LSM9DS1Class_new::setOneShotMode() {
  /* Disable FIFO (see docs https://www.st.com/resource/en/datasheet/DM00103319.pdf). */
  writeRegister(LSM9DS1_ADDRESS, 0x23, 0x00);
  /* Disable continuous mode. */
  writeRegister(LSM9DS1_ADDRESS, 0x2E, 0x00);

  continuousMode = false;
}

void LSM9DS1Class_new::end() {
  /* Writes in register LSM9DS1_CTRL_REG6_XL in order to power down the accelerometer (ODR_XL = 000). */
  writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL, 0x00); 

  _wire->end();
}

int LSM9DS1Class_new::readAcceleration(float& x, float& y, float& z) {
  int16_t data[3];

  /* Perform multiple reads starting in register LSM9DS1_OUT_X_XL to get the values from the 
  3 output channel from the accelerometer (x, y and z). */
  if (!readRegisters(LSM9DS1_ADDRESS, LSM9DS1_OUT_X_XL, (uint8_t*)data, sizeof(data))) { 
    x = NAN;
    y = NAN;
    z = NAN;
    return 0;
  }

  /* Accelerometer range is set at [-4, +4] g ± 0.122 mg. So, the value returned by the function 
  readAcceleration() is in the range [-4, 4] g. g is the gravitational acceleration = 9.81 m/s^2.
  The raw acceleration data is represented as a 16-bit signed integer (between −32 768 to 32 767), 
  which is then normalized (divided by 32 768) and multiplied by 4 to put in the correct range of [-4, 4].*/
  x = data[0] * 4.0 / 32768.0;
  y = data[1] * 4.0 / 32768.0;
  z = data[2] * 4.0 / 32768.0;

  return 1;
}

int LSM9DS1Class_new::accelerationAvailable() {
  if (continuousMode) {
    /* Read FIFO_SRC. If any of the rightmost 8 bits have a value, there is data. */
    if (readRegister(LSM9DS1_ADDRESS, 0x2F) & 63) {
      return 1;
    }
  } else {
    /* LSM9DS1_STATUS_REG tells if there is new data available. If XLDA bit (last bit) from 
    register LSM9DS1_STATUS_REG is 1 a new set of data is available, otherwise a new set of data 
    is not yet available. We are doing a bitwise AND operation between the contents of this register 
    and 0x01, to get the XLDA bit.  */
    if (readRegister(LSM9DS1_ADDRESS, LSM9DS1_STATUS_REG) & 0x01) { 
      return 1;
    }
  }

  return 0;
}

float LSM9DS1Class_new::accelerationSampleRate() {
  return 119.0F;
}

int LSM9DS1Class_new::readRegister(uint8_t slaveAddress, uint8_t address) {
  /* Master (peripherical that we are using) begins transmission with slave. */
  _wire->beginTransmission(slaveAddress); 
  _wire->write(address);
  if (_wire->endTransmission() != 0) {
    return -1;
  }
  
  /* Request data from only one register. */
  if (_wire->requestFrom(slaveAddress, 1) != 1) {
    return -1;
  }

  /* Reads a byte that was transmitted from a slave device to a master. */
  return _wire->read(); 
}

/* Perform multiple reads starting with reads in address. */
int LSM9DS1Class_new::readRegisters(uint8_t slaveAddress, uint8_t address, uint8_t* data, size_t length) { 

  _wire->beginTransmission(slaveAddress);
  _wire->write(0x80 | address);
  if (_wire->endTransmission(false) != 0) {
    return -1;
  }

  /* Requests number of reads it wants to do (since in this casethe size of the output data from 
  accelerometer is 3, then we will set length as 3 when using this function). */
  if (_wire->requestFrom(slaveAddress, length) != length) { 
    return 0;
  }

  for (size_t i = 0; i < length; i++) {
    *data++ = _wire->read();
  }

  return 1;
}

int LSM9DS1Class_new::writeRegister(uint8_t slaveAddress, uint8_t address, uint8_t value) {
  _wire->beginTransmission(slaveAddress);
  _wire->write(address); 
  _wire->write(value); /* Value written in address. */
  if (_wire->endTransmission() != 0) {
    return 0;
  }

  return 1;
}

LSM9DS1Class_new IMU_LSM9DS1_new(Wire1);

#define echoPin 2 /* Attach pin D2 Arduino to pin Echo of ultrasonic sensor. */
#define trigPin 4 /* Attach pin D4 Arduino to pin Trig of ultrasonic sensor. */
#define A0 0
#define WAITING 0
#define RECEIVING_ECHO 1
#define SENDING_SIGNAL 2
#define DONT_CARE 1
#define CARE 2

float x, y, z;
double degreesX = 0;
double degreesY = 0;
double degreesZ = 0;
bool continuousMode = false;
unsigned long time1, time2;
int count;
double sum;

/* Defines variables. */
double duration; /* Variable for the duration of sound wave travel. */
double distance_ultrasound, distance_infrared, distance; /* Variable for the distance measurement. */
double vout;
volatile short flag;
short flag_ignore;
bool time1Control;

double distance_calibration[14] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

double d_white_sheet;
double d_cardboard;
double d_white_plastic;
double d_green_paper;
double d_blue_paper;
double d_black_notebook;
double error_d_white_sheet;
double error_d_cardboard;
double error_d_white_plastic;
double error_d_green_paper;
double error_d_blue_paper;
double error_d_black_notebook;
double d_choosed;
double error_min;

/* Allows to choose beetween some calibrations that we made. */
int choose_calibration = 2;

/* Considering color calibration: Yes (1) or No (0). */
int color_calibration = 1;

void doInterrupt();
double calibration_considering_color(double vout, double dist_ultrasound);
double calibration_cardboard(double vout);
double calibration_whitePlastic(double vout);
double calibration_blue_paper(double vout);
double calibration_green_paper(double vout);
double calibration_white_sheet(double vout);
double regression (double d1x, double d1y, double d2x, double d2y, double vout);

/* Library that allows to change LED color. */
#include <Arduino_APDS9960.h>

int ledState = LOW;

void setup() {  
  pinMode(trigPin, OUTPUT); /* Sets the trigPin as an OUTPUT. */
  pinMode(echoPin, INPUT); /* Sets the echoPin as an INPUT. */
  Serial.begin(9600); /* Serial Communication is starting with 9600 of baudrate speed. */
  Serial.println("Ultrasonic Sensor Test with Arduino Nano 33 BLE");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");
  
  attachInterrupt(digitalPinToInterrupt(echoPin), doInterrupt, CHANGE);

  /* Set the LEDs pins as outputs. */
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  /* Turn all the LEDs off. */
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);

  /* Initializations. */
  sum = 0;
  count = 0;
  flag = WAITING;
  flag_ignore = CARE;
  time1Control = false;

  /* Clears the trigPin condition. */
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2); 

  /* Sets the trigPin HIGH (ACTIVE) for 10 microseconds. */
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
}

void loop() {
  if (flag == RECEIVING_ECHO) {
    if (time1Control == false) { 
      /* Measure the time in which the echo pin rise up. */
      time1 = micros();
      time1Control = true;
    }

    /* While the echo pin is HIGH it performs measurements with the infrared sensor. */
    sum += analogRead(A0); 
    count++;
    if (count > 10000) {
      flag_ignore = DONT_CARE; 
      flag = SENDING_SIGNAL;
    }
    
  }
  else if (flag == WAITING) {
    /* Serial.println("Waiting for rising of echoPin"); */
  }
  else {
    if (flag_ignore == CARE) {
      /* Measure the time in which the echo pin falls. */
      time2 = micros();

      /* Performs 20 measurements with the infrared sensor. */
      for (int i = 0; i < 20; i++) {
        sum += analogRead(A0); 
        count++;
      }
      vout = sum / count; 

      duration = time2 - time1;
      
      /* Speed of sound wave (multiplied by 10^-6) divided by 2 (go and back). */
      distance_ultrasound = duration * 0.0343 / 2; 

      /* We neglect unrealistic measurements of the ultrasound sensor. */
      if (distance_ultrasound < 150) {
        /* Check if the measure is out of bounds. */
          if (duration < 38000) {
            /* Displays the distance on the Serial Monitor. */
            Serial.print("Distance ultrasound: ");
            Serial.print(distance_ultrasound);
            Serial.println("  cm"); 
          }
          else {
            flag_ignore = DONT_CARE;
            Serial.println("Distance: Out of bounds");
          }

          distance_infrared = calibration_considering_color(vout, distance_ultrasound);

          /* u.m. = units of measure */
          /* Serial.print("vout = ");
          Serial.print(vout);
          Serial.println("  u.m.");*/
          Serial.print("Distance infrared = ");
          Serial.print(distance_infrared);
          Serial.println("  cm"); 

          /* Sensor Fusion. Ultrasonic output has more weight for longer distances 
          and the infrared for shorter distances. */
          if (distance_ultrasound < 5) {
            distance = 0.1 * distance_ultrasound + 0.9 * distance_infrared;
          }
          else if (distance_ultrasound < 10) {
            distance = 0.25 * distance_ultrasound + 0.75 * distance_infrared;
          }
          else if (distance_ultrasound < 15) {
            distance = 0.9 * distance_ultrasound + 0.1 * distance_infrared;
          }
          else if (distance_ultrasound < 20) {
            distance = 0.95 * distance_ultrasound + 0.05 * distance_infrared;
          }
          else {
            distance = distance_ultrasound;
          }

          /* Neglects unrealistic measurements. */
          if (distance < 150 && distance > 0) {
            Serial.print("Distance not considering tilt = ");
            Serial.print(distance);
            Serial.println("  cm");
          }
          
          /* Gets the data from accelerometer. */
          if (IMU.accelerationAvailable()) {
            IMU.readAcceleration(x, y, z); 
          }
          
          /*
          if (x >= 0) {
            x = 100 * x;
            degreesX = map(x, 0, 97, 0, 90);
            Serial.print("Tilting X: ");
            Serial.print(degreesX);
            Serial.println("  degrees");
          }
          if (x < 0) {
            x = 100 * x;
            degreesX = map(x, 0, -100, 0, 90);
            Serial.print("Tilting X: ");
            Serial.print(-degreesX);
            Serial.println("  degrees");
          }
          if (y >= 0) {
            y = 100 * y;
            degreesY = map(y, 0, 97, 0, 90);
            Serial.print("Tilting Y: ");
            Serial.print(degreesY);
            Serial.println("  degrees");
          }
          if (y < 0) {
            y = 100 * y;
            degreesY = map(y, 0, -100, 0, 90);
            Serial.print("Tilting Y: ");
            Serial.print(-degreesY);
            Serial.println("  degrees");
          }
          */
          /* We only need to get the angle with respect to z-axis. */
          if (z >= 0) {
            z = 100 * z;
            degreesZ = map(z, 0, 97, 0, 90);
            Serial.print("Tilting Z: ");
            Serial.print(degreesZ);
            Serial.println("  degrees"); 
          }
          if (z < 0) {
            z = 100 * z;
            degreesZ = map(z, 0, -100, 0, 90);
            Serial.print("Tilting Z: ");
            Serial.print(-degreesZ);
            Serial.println("  degrees");
          }

          /* Distance considering tilt. */
          distance = distance * cos((90 - degreesZ) * DEG_TO_RAD); 

          if (flag_ignore == CARE) {
            /* Neglects unrealistic measurements. */
            if (distance < 150 && distance > 0) {
              Serial.print("Distance = ");
              Serial.print(distance);
              Serial.println("  cm \n ");
            }
          }
          else {
            flag_ignore = CARE;
          }

          /* If we are OUT OF BOUNDS, LED displays a mix of red and blue. */
          if (duration >= 38000) {
            /* If the LED is off turn it on and vice-versa. */
            if (ledState == LOW) {
              ledState = HIGH;
            } else {
              ledState = LOW;
            }

            /* Set the red and the blue LED with the ledState of the variable and turn off the rest. */
            digitalWrite(LEDG, HIGH);
            digitalWrite(LEDR, ledState);
            digitalWrite(LEDB, ledState); 
          }
          else if (distance > 15) {
            /* If distance is greater than 15cm, LED displays green. */

            /* If the LED is off turn it on and vice-versa. */
            if (ledState == LOW) {
              ledState = HIGH;
            } else {
              ledState = LOW;
            }

            /* Set the green LED with the ledState of the variable and turn off the rest. */
            digitalWrite(LEDG, ledState);
            digitalWrite(LEDR, HIGH);
            digitalWrite(LEDB, HIGH); 
          }
          else if (distance > 7) {
            /* If distance is beetwen 7cm and 15cm, LED displays blue. */

            /* If the LED is off turn it on and vice-versa. */
            if (ledState == LOW) {
              ledState = HIGH;
            } else {
              ledState = LOW;
            }

            /* Set the blue LED with the ledState of the variable and turn off the rest. */
            digitalWrite(LEDB, ledState);
            digitalWrite(LEDR, HIGH);
            digitalWrite(LEDG, HIGH); 
          }
          else {
            /* If distance is less or equal to 7cm, LED displays red. */

            /* If the LED is off turn it on and vice-versa. */
            if (ledState == LOW) {
              ledState = HIGH;
            } else {
              ledState = LOW;
            }

            /* Set the red LED with the ledState of the variable and turn off the rest. */
            digitalWrite(LEDR, ledState);
            digitalWrite(LEDB, HIGH);
            digitalWrite(LEDG, HIGH);
          }
        }
        else {
          flag_ignore = CARE;
        }
      }
      
    /* Due to accelerometer sample rate. */
    delay(20); 

    /* Initializations. */
    sum = 0;
    count = 0;
    flag = WAITING;
    time1Control = false;

    /* Clears the trigPin condition. */
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2); 

    /* Sets the trigPin HIGH (ACTIVE) for 10 microseconds. */
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
  }
}

 /* Function that runs when the echoPin changes from HIGH to LOW, or vice-versa. 
  It changes the state of the program. */
void doInterrupt() {
  if (flag == WAITING) {
    flag = RECEIVING_ECHO;
  }
  else {
     flag = SENDING_SIGNAL; 
  }
}

/* Calibration performed for a cardboard. It returns the estimated distance. */
double calibration_cardboard(double vout) {

  /* 7 Dec calibration. */
  double vout_calibration[14];
  if (choose_calibration == 0) {
    double aux[14] = {102.500, 96.000, 76.330, 67.500, 59.250, 52.330, 49.110, 47.000, 44.880, 41.000, 39.000, 38.450, 38.260, 35.740};
    for (int i = 0; i < 14; i++) {
      vout_calibration[i] = aux[i];
    }
  }

  /* 12 Dec calibration. */
  else if (choose_calibration == 1) {
    double aux[14] = {166.500, 161.000, 159.150, 145.790, 141.000, 149.000, 157.160, 106.110, 144.840, 129.370, 140.290, 132.730, 124.340, 146.020};
    for (int i = 0; i < 14; i++) {
      vout_calibration[i] = aux[i];
    }
  }

  /* Demo calibration. */
  else {
    /* In class demonstration we didn't perform this calibration. */
    return -20;
    double aux[14] = {0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0};
    for (int i = 0; i < 14; i++) {
      vout_calibration[i] = aux[i];
    }
  }

  /* Multi-segment curves approach: each segment is a linear regression and store the parameters in a lookup table. */
  if (vout > vout_calibration[6]) {
    if (vout > vout_calibration[3]) {
      if (vout > vout_calibration[2]) {
        if (vout > vout_calibration[1]) {
          double distance = regression(vout_calibration[0], distance_calibration[0], vout_calibration[1], distance_calibration[1], vout);
          if (distance < 0) {
            distance = 0;
          }
          return distance;
        }
        else {
          return regression(vout_calibration[1], distance_calibration[1], vout_calibration[2], distance_calibration[2], vout);
        }
      }
      else {
        return regression(vout_calibration[2], distance_calibration[2], vout_calibration[3], distance_calibration[3], vout);
      }
    }
    else {
      if (vout > vout_calibration[5]) {
        if (vout > vout_calibration[4]) {
          return regression(vout_calibration[3], distance_calibration[3], vout_calibration[4], distance_calibration[4], vout);
        }
        else {
          return regression(vout_calibration[4], distance_calibration[4], vout_calibration[5], distance_calibration[5], vout);
        }
      }
      else {
        return regression(vout_calibration[5], distance_calibration[5], vout_calibration[6], distance_calibration[6], vout);
      }
    }
  }
  else {
    if (vout > vout_calibration[8]) {
      if (vout > vout_calibration[7]) {
        return regression(vout_calibration[6], distance_calibration[6], vout_calibration[7], distance_calibration[7], vout);
      }
      else {
        return regression(vout_calibration[7], distance_calibration[7], vout_calibration[8], distance_calibration[8], vout);
      }
    }
    else {
      if (vout > vout_calibration[9]) {
        return regression(vout_calibration[8], distance_calibration[8], vout_calibration[9], distance_calibration[9], vout);
      }
      else if (vout > vout_calibration[10]) {
        return regression(vout_calibration[9], distance_calibration[9], vout_calibration[10], distance_calibration[10], vout);
      }
      else if (vout > vout_calibration[13] - 1) {
        /* Regression from vout_calibration[10] to vout_calibration[13]. */
        double distance;

        /* 7 Dec calibration. */
        if (choose_calibration == 0) {
          distance = vout * -0.7942 + 43.57;
        }

        /* 12 Dec calibration. */
        else if (choose_calibration == 1) {
          distance = vout * -0.1075 + 27.114;
        }

        /* Demo calibration. */
        else {
          /* In class demonstration we didn't perform this calibration. */
          distance = vout * 0 + 0;
        }

        if (distance < 0) {
          distance = 0;
        }
        return distance;
      }
      else {
        /* With all points. */

        /* 7 Dec calibration. */
        if (choose_calibration == 0) {
          return 7632.4 * pow(vout, -1.75); 
        }

        /* 12 Dec calibration. */
        else if (choose_calibration == 1) {
          return 288818700.49 * pow(vout, -3.538); 
        }

        /* Demo calibration. */
        else {
          /* In class demonstration we didn't perform this calibration. */
          return 0 * pow(vout, 0); 
        }
      }
    }
  }
}

/* Calibration performed for white plastic. It returns the estimated distance. */
double calibration_whitePlastic(double vout) {

  /* 7 Dec calibration. */
  double vout_calibration[14];
  if (choose_calibration == 0) {
    double aux[14] = {169.750, 137.670, 114.290, 95.250, 82.600, 68.450, 59.380, 54.470, 49.580, 46.400, 44.190, 43.960, 42.630, 42.880};
    for (int i = 0; i < 14; i++) {
      vout_calibration[i] = aux[i];
    }
  }

  /* 12 Dec calibration. */
  else if (choose_calibration == 1) {
    double aux[14] = {0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0};
    for (int i = 0; i < 14; i++) {
      vout_calibration[i] = aux[i];
    }
  }

  /* Demo calibration. */
  else {
    /* In class demonstration we didn't perform this calibration. */
    return -20; 
    double aux[14] = {0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0};
    for (int i = 0; i < 14; i++) {
      vout_calibration[i] = aux[i];
    }
  }

  /* Multi-segment curves approach: each segment is a linear regression and store the parameters in a lookup table. */
  if (vout > vout_calibration[6]) {
    if (vout > vout_calibration[3]) {
      if (vout > vout_calibration[2]) {
        if (vout > vout_calibration[1]) {
          double distance = regression(vout_calibration[0], distance_calibration[0], vout_calibration[1], distance_calibration[1], vout);
          if (distance < 0) {
            distance = 0;
          }
          return distance;
        }
        else {
          return regression(vout_calibration[1], distance_calibration[1], vout_calibration[2], distance_calibration[2], vout);
        }
      }
      else {
        return regression(vout_calibration[2], distance_calibration[2], vout_calibration[3], distance_calibration[3], vout);
      }
    }
    else {
      if (vout > vout_calibration[5]) {
        if (vout > vout_calibration[4]) {
          return regression(vout_calibration[3], distance_calibration[3], vout_calibration[4], distance_calibration[4], vout);
        }
        else {
          return regression(vout_calibration[4], distance_calibration[4], vout_calibration[5], distance_calibration[5], vout);
        }
      }
      else {
        return regression(vout_calibration[5], distance_calibration[5], vout_calibration[6], distance_calibration[6], vout);
      }
    }
  }
  else {
    if (vout > vout_calibration[8]) {
      if (vout > vout_calibration[7]) {
        return regression(vout_calibration[6], distance_calibration[6], vout_calibration[7], distance_calibration[7], vout);
      }
      else {
        return regression(vout_calibration[7], distance_calibration[7], vout_calibration[8], distance_calibration[8], vout);
      }
    }
    else {
      if (vout > vout_calibration[9]) {
        return regression(vout_calibration[8], distance_calibration[8], vout_calibration[9], distance_calibration[9], vout);
      }
      else if (vout > vout_calibration[10]) {
        return regression(vout_calibration[9], distance_calibration[9], vout_calibration[10], distance_calibration[10], vout);
      }
      else if (vout > vout_calibration[13] - 1) {
        /* Regression from vout_calibration[10] to vout_calibration[13]. */
        double distance;

        /* 7 Dec calibration. */
        if (choose_calibration == 0) {
          distance = vout * -0.9632 + 55.39;
        }

        /* 12 Dec calibration. */
        else if (choose_calibration == 1) {
          distance = vout * 0 + 0;
        }

        /* Demo calibration. */
        else {
          /* In class demonstration we didn't perform this calibration. */
          distance = vout * 0 + 0;
        }


        if (distance < 0) {
          distance = 0;
        }
        return distance;
      }
      else {
        /* With all points. */

        /* 7 Dec calibration. */
        if (choose_calibration == 0) {
          return 1667.6 * pow(vout, -1.29); 
        }

        /* 12 Dec calibration. */
        else if (choose_calibration == 1) {
          return 0 * pow(vout, 0); 
        }

        /* Demo calibration. */
        else {
          /* In class demonstration we didn't perform this calibration. */
          return 0 * pow(vout, 0); 
        }
      }
    }
  }
}

/* Calibration performed for green paper. It returns the estimated distance. */
double calibration_green_paper(double vout) {

  /* 7 Dec calibration. */
  double vout_calibration[14];
  if (choose_calibration == 0) {
    double aux[14] = {132.250, 113.200, 100.170, 81.140, 73.100, 63.250, 55.730, 51.190, 49.180, 42.470, 42.050, 38.770, 37.040, 35.080};
    for (int i = 0; i < 14; i++) {
      vout_calibration[i] = aux[i];
    }
  }

  /* 12 Dec calibration. */
  else if (choose_calibration == 1) {
    double aux[14] = {170.12,	166.6,	127.39,	119.5,	114.31,	108,	103.91,	101.15,	100.22,	97.24,	93.02,	95,	96.76};
    for (int i = 0; i < 14; i++) {
      vout_calibration[i] = aux[i];
    }
  }

  /* Demo calibration. */
  else {
    /* In class demonstration we didn't perform this calibration. */
    return -20;
    double aux[14] = {0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0};
    for (int i = 0; i < 14; i++) {
      vout_calibration[i] = aux[i];
    }
  }

  /* Multi-segment curves approach: each segment is a linear regression and store the parameters in a lookup table. */
  if (vout > vout_calibration[6]) {
    if (vout > vout_calibration[3]) {
      if (vout > vout_calibration[2]) {
        if (vout > vout_calibration[1]) {
          double distance = regression(vout_calibration[0], distance_calibration[0], vout_calibration[1], distance_calibration[1], vout);
          if (distance < 0) {
            distance = 0;
          }
          return distance;
        }
        else {
          return regression(vout_calibration[1], distance_calibration[1], vout_calibration[2], distance_calibration[2], vout);
        }
      }
      else {
        return regression(vout_calibration[2], distance_calibration[2], vout_calibration[3], distance_calibration[3], vout);
      }
    }
    else {
      if (vout > vout_calibration[5]) {
        if (vout > vout_calibration[4]) {
          return regression(vout_calibration[3], distance_calibration[3], vout_calibration[4], distance_calibration[4], vout);
        }
        else {
          return regression(vout_calibration[4], distance_calibration[4], vout_calibration[5], distance_calibration[5], vout);
        }
      }
      else {
        return regression(vout_calibration[5], distance_calibration[5], vout_calibration[6], distance_calibration[6], vout);
      }
    }
  }
  else {
    if (vout > vout_calibration[8]) {
      if (vout > vout_calibration[7]) {
        return regression(vout_calibration[6], distance_calibration[6], vout_calibration[7], distance_calibration[7], vout);
      }
      else {
        return regression(vout_calibration[7], distance_calibration[7], vout_calibration[8], distance_calibration[8], vout);
      }
    }
    else {
      if (vout > vout_calibration[9]) {
        return regression(vout_calibration[8], distance_calibration[8], vout_calibration[9], distance_calibration[9], vout);
      }
      else if (vout > vout_calibration[10]) {
        return regression(vout_calibration[9], distance_calibration[9], vout_calibration[10], distance_calibration[10], vout);
      }
      else if (vout > vout_calibration[13] - 1) {
        /* Regression from vout_calibration[10] to vout_calibration[13]. */
        double distance;

        /* 7 Dec calibration. */
        if (choose_calibration == 0) {
          distance = vout * -0.4878 + 32.06;
        }

        /* 12 Dec calibration. */
        else if (choose_calibration == 1) {
          distance = vout * 0 + 0;
        }

        /* Demo calibration. */
        else {
          /* In class demonstration we didn't perform this calibration. */
          distance = vout * -0.3181 + 43.675;
        }


        if (distance < 0) {
          distance = 0;
        }
        return distance;
      }
      else {
        /* With all points. */

        /* 7 Dec calibration. */
        if (choose_calibration == 0) {
          return 2221.4 * pow(vout, -1.398); 
        }

        /* Demo calibration. */
        else if (choose_calibration == 1) {
          return 2050391 * pow(vout, -2.638); 
        }

        /* Demo calibration. */
        else {
          return 0 * pow(vout, 0); 
        }
      }
    }
  }
}

/* Calibration performed for blue paper. It returns the estimated distance. */
double calibration_blue_paper(double vout) {

  /* 7 Dec calibration. */
  double vout_calibration[14];
  if (choose_calibration == 0) {
    double aux[14] = {181.750,	163.750,	137.000,	108.250,	88.550,	75.580,	68.790,	56.000,	54.650,	52.250,	49.950,	47.730,	44.170,	43.190};
    for (int i = 0; i < 14; i++) {
      vout_calibration[i] = aux[i];
    }
  }

  /* 12 Dec calibration. */
  else if (choose_calibration == 1) {
    double aux[14] = {224.3,	206.5,	183.27,	158.96,	166.25,	152.87,	129.44,	114.83,	110.62,	106.45,	104.4,	101.93,	100.88,	97.74};
    for (int i = 0; i < 14; i++) {
      vout_calibration[i] = aux[i];
    }
  }

  /* Demo calibration. */
  else {
    double aux[14] = {259.080,	210.440,	159.220,	128.000,	102.600,	87.880,	75.380,	65.470,	59.050,	55.470,	51.590,	49.420,	47.530,	44.800};
    for (int i = 0; i < 14; i++) {
      vout_calibration[i] = aux[i];
    }
  }

  /* Multi-segment curves approach: each segment is a linear regression and store the parameters in a lookup table. */
  if (vout > vout_calibration[6]) {
    if (vout > vout_calibration[3]) {
      if (vout > vout_calibration[2]) {
        if (vout > vout_calibration[1]) {
          double distance = regression(vout_calibration[0], distance_calibration[0], vout_calibration[1], distance_calibration[1], vout);
          if (distance < 0) {
            distance = 0;
          }
          return distance;
        }
        else {
          return regression(vout_calibration[1], distance_calibration[1], vout_calibration[2], distance_calibration[2], vout);
        }
      }
      else {
        return regression(vout_calibration[2], distance_calibration[2], vout_calibration[3], distance_calibration[3], vout);
      }
    }
    else {
      if (vout > vout_calibration[5]) {
        if (vout > vout_calibration[4]) {
          return regression(vout_calibration[3], distance_calibration[3], vout_calibration[4], distance_calibration[4], vout);
        }
        else {
          return regression(vout_calibration[4], distance_calibration[4], vout_calibration[5], distance_calibration[5], vout);
        }
      }
      else {
        return regression(vout_calibration[5], distance_calibration[5], vout_calibration[6], distance_calibration[6], vout);
      }
    }
  }
  else {
    if (vout > vout_calibration[8]) {
      if (vout > vout_calibration[7]) {
        return regression(vout_calibration[6], distance_calibration[6], vout_calibration[7], distance_calibration[7], vout);
      }
      else {
        return regression(vout_calibration[7], distance_calibration[7], vout_calibration[8], distance_calibration[8], vout);
      }
    }
    else {
      if (vout > vout_calibration[9]) {
        return regression(vout_calibration[8], distance_calibration[8], vout_calibration[9], distance_calibration[9], vout);
      }
      else if (vout > vout_calibration[10]) {
        return regression(vout_calibration[9], distance_calibration[9], vout_calibration[10], distance_calibration[10], vout);
      }
      else if (vout > vout_calibration[13] - 1) {
        /* Regression from vout_calibration[10] to vout_calibration[13]. */
        double distance;

        /* 7 Dec calibration. */
        if (choose_calibration == 0) {
          distance = vout * -0.4101 + 32.46;
        }

        /* 12 Dec calibration. */
        else if (choose_calibration == 1) {
          distance = vout * -0.4697 + 61.045;
        }

        /* Demo calibration. */
        else {
          distance = vout * -0.3869 + 32.255;
        }


        if (distance < 0) {
          distance = 0;
        }
        return distance;
      }
      else {
        /* With all points. */

        /* 7 Dec calibration. */
        if (choose_calibration == 0) {
          return 1454.3 * pow(vout, -1.227); 
        }

        /* 12 Dec calibration. */
        else if (choose_calibration == 1) {
          return 196229 * pow(vout, -2.08); 
        }

        /* Demo calibration. */
        else {
          return 776.5 * pow(vout, -1.052); 
        }
      }
    }
  }
}

/* Calibration performed for white sheet. It returns the estimated distance. */
double calibration_white_sheet(double vout) {

  /* 7 Dec calibration. */
  double vout_calibration[14];
  if (choose_calibration == 0) {
    double aux[14] = {229.83,	207.15,	180.08,	158.600,	138.170,	125.050,	115.470,	110.740,	105.640,	103.100,	100.440,	98.070,	95.320,	95.110};
    for (int i = 0; i < 14; i++) {
      vout_calibration[i] = aux[i];
    }
  }

  /* 12 Dec calibration. */
  else if (choose_calibration == 1) {
      double aux[14] = {226.000,    204.600,    171.270,    157.520,    134.400,    119.120,    110.360,    100.760,    97.000,    93.650,    91.660,    89.070,    87.390,    85.070};    
    for (int i = 0; i < 14; i++) {
      vout_calibration[i] = aux[i];
    }
  }

  /* Demo calibration. */
  else {
    double aux[14] = {225.260,	185.120,	149.270,	117.300,	100.130,	84.650,	73.150,	67.080,	60.370,	56.130,	54.000,	51.200,	50.300,	48.890};
    for (int i = 0; i < 14; i++) {
      vout_calibration[i] = aux[i];
    }
  }

  /* Multi-segment curves approach: each segment is a linear regression and store the parameters in a lookup table. */
  if (vout > vout_calibration[6]) {
    if (vout > vout_calibration[3]) {
      if (vout > vout_calibration[2]) {
        if (vout > vout_calibration[1]) {
          double distance = regression(vout_calibration[0], distance_calibration[0], vout_calibration[1], distance_calibration[1], vout);
          if (distance < 0) {
            distance = 0;
          }
          return distance;
        }
        else {
          return regression(vout_calibration[1], distance_calibration[1], vout_calibration[2], distance_calibration[2], vout);
        }
      }
      else {
        return regression(vout_calibration[2], distance_calibration[2], vout_calibration[3], distance_calibration[3], vout);
      }
    }
    else {
      if (vout > vout_calibration[5]) {
        if (vout > vout_calibration[4]) {
          return regression(vout_calibration[3], distance_calibration[3], vout_calibration[4], distance_calibration[4], vout);
        }
        else {
          return regression(vout_calibration[4], distance_calibration[4], vout_calibration[5], distance_calibration[5], vout);
        }
      }
      else {
        return regression(vout_calibration[5], distance_calibration[5], vout_calibration[6], distance_calibration[6], vout);
      }
    }
  }
  else {
    if (vout > vout_calibration[8]) {
      if (vout > vout_calibration[7]) {
        return regression(vout_calibration[6], distance_calibration[6], vout_calibration[7], distance_calibration[7], vout);
      }
      else {
        return regression(vout_calibration[7], distance_calibration[7], vout_calibration[8], distance_calibration[8], vout);
      }
    }
    else {
      if (vout > vout_calibration[9]) {
        return regression(vout_calibration[8], distance_calibration[8], vout_calibration[9], distance_calibration[9], vout);
      }
      else if (vout > vout_calibration[10]) {
        return regression(vout_calibration[9], distance_calibration[9], vout_calibration[10], distance_calibration[10], vout);
      }
      else if (vout > vout_calibration[13] - 1) {
        /* Regression from vout_calibration[10] to vout_calibration[13]. */
        double distance;

        /* 7 Dec calibration. */
        if (choose_calibration == 0) {
          distance = vout * -0.5852 + 43.05;
        }

        /* 12 Dec calibration. */
        else if (choose_calibration == 1) {
          distance = vout * -0.4652 + 54.577;
        }

        /* Demo calibration. */
        else {
          distance = vout * -0.5315 + 40.693;
        }


        if (distance < 0) {
          distance = 0;
        }
        return distance;
      }
      else {
        /* With all points. */

        /* 7 Dec calibration. */
        if (choose_calibration == 0) {
          return 5987.5 * pow(vout, -1.554); 
        }

        /* 12 Dec calibration. */
        else if (choose_calibration == 1) {
          return 46682 * pow(vout, -1.831); 
        }

        /* Demo calibration. */
        else {
          return 1389.8 * pow(vout, -1.188); 
        }
      }
    }
  }
}

double calibration_black_notebook(double vout) {
  /* 7 Dec calibration. */
  if (choose_calibration == 0) {
    return 0;
  }
  /* 12 Dec calibration. */
  else if (choose_calibration == 1) {
    return 214863337.11 * pow(vout,-3.75);
  }

  /* Demo calibration. */
  else {
    return 0;
  }
}

/* Returns distance estimated by the infrared sensor after calibration. */
double calibration_considering_color (double vout, double dist_ultrasound) {
  /* In case of not considering color calibration, use only the white sheet calibration. */
  if (color_calibration) {
    if (choose_calibration != 1) {
      /* In these calibrations, the black notebook absorbed much of the incident infrared radiation. */
      if (vout < 32) {
        return dist_ultrasound;
      }
    }
    
    /* Get the distance estimated by each calibration. */
    d_black_notebook = calibration_black_notebook(vout);
    d_white_sheet = calibration_white_sheet(vout);
    d_cardboard = calibration_cardboard(vout);
    d_white_plastic = calibration_whitePlastic(vout);
    d_green_paper = calibration_green_paper(vout);
    d_blue_paper = calibration_blue_paper(vout);

    /* Calculate the absolute value of the error of the distance estimated by each calibration, 
    compared to the distance measured by ultrasound. The distance choosed is the one that minimizes that value. */

    error_d_white_sheet = d_white_sheet - dist_ultrasound;
    if (error_d_white_sheet < 0) {
      error_d_white_sheet *= -1;
    }

    error_min = error_d_white_sheet;
    d_choosed = d_white_sheet;

    error_d_cardboard = d_cardboard - dist_ultrasound;
    if (error_d_cardboard < 0) {
      error_d_cardboard *= -1;
    }

    if (error_d_cardboard < error_min) {
      error_min = error_d_cardboard;
      d_choosed = d_cardboard;
    }

    error_d_white_plastic = d_white_plastic - dist_ultrasound;
    if (error_d_white_plastic < 0) {
      error_d_white_plastic *= -1;
    }

    if (error_d_white_plastic < error_min) {
      error_min = error_d_white_plastic;
      d_choosed = d_white_plastic;
    }

    error_d_green_paper = d_green_paper - dist_ultrasound;
    if (error_d_green_paper < 0) {
      error_d_green_paper *= -1;
    }

    if (error_d_green_paper < error_min) {
      error_min = error_d_green_paper;
      d_choosed = d_green_paper;
    }

    error_d_blue_paper = d_blue_paper - dist_ultrasound;
    if (error_d_blue_paper < 0) {
      error_d_blue_paper *= -1;
    }

    if (error_d_blue_paper < error_min) {
      error_min = error_d_blue_paper;
      d_choosed = d_blue_paper;
    }

    error_d_black_notebook = d_black_notebook - dist_ultrasound;
    if (error_d_black_notebook < 0) {
      error_d_black_notebook *= -1;
    }

    if (error_d_black_notebook < error_min && choose_calibration != 0) {
      error_min = error_d_black_notebook;
      d_choosed = d_black_notebook;
    }
    if (d_choosed < 0) {
      d_choosed = 0; 
    }
    return d_choosed; 
  }
  else {
    return calibration_white_sheet(vout);
  }
}

/* Calculates the output of a linear regressin given by P1 = (d1x, d1y) and by P2 = (d2x, d2y). */
double regression (double d1x, double d1y, double d2x, double d2y, double vout) {
  double m = (d1y - d2y) / (d1x - d2x);
  double b = d1y - m * d1x;
  return m * vout + b;
}