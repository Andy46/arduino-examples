#include <Arduino.h>
#include <Wire.h>

// Include C libraries
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

// Include the sensor's official library provided by BOSCH at (https://github.com/boschsensortec/BMI160_SensorAPI)
#include <bmi160.h>

/*! bmi160 Device address */
#define BMI160_DEV_ADDR_A       0x68
#define BMI160_DEV_ADDR_B       0x69
#define BMI160_DEV_ADDR         BMI160_DEV_ADDR_A

/* I2C callback functions */
int8_t bmi160i2c_read_cb (uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t bmi160i2c_write_cb (uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len);
void bmi160i2c_delay_ms_cb (uint32_t period);

/* BMI160 sensor */
void init_sensor(void);
void config_sensor(void);
void read_sensor(void);

struct bmi160_dev bmi160;
struct bmi160_sensor_data bmi160_accel;
struct bmi160_sensor_data bmi160_gyro;
struct bmi160_offsets bmi160_offset;

/* MAIN PROGRAM */
void exit(int code)
{
    // Print exit message
    Serial.print("Program exit with code: ");
    Serial.println(code);

    // Stop UART
    Serial.end();

    // Stop I2C
    Wire.end();

    while(1)
    {
        delay(__LONG_MAX__);
    }
}

void setup() 
{
    // Init UART 
    Serial.begin(9600);
    Serial.println("Initialized Serial!");

    // Init I2C
    Wire.begin();
    Wire.setClock(100000); // 100KHz
    Serial.println("Initialized I2C!");
    
    // Init BMI160
    init_sensor();
    config_sensor();
}


float convertRawAcceleration(int16_t aRaw) {
  // since we are using 2 g range
  // -2 g maps to a raw value of -32768
  // +2 g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int16_t gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

void loop() 
{
    read_sensor();
    delay(20);
}

// I2C Read/Write callback functions
int8_t bmi160i2c_write_cb (uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
#ifdef DEBUG
    Serial.println("Writting to I2C!");
    Serial.print("Address: 0x");
    Serial.println(dev_addr, HEX);
#endif

    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    Wire.write(data, len);
    Wire.endTransmission();

    return 0;
}

int8_t bmi160i2c_read_cb (uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len)
{
    uint16_t count = 0;

#ifdef DEBUG
    Serial.println("Reading from I2C!");
    Serial.print("Address: 0x");
    Serial.println(dev_addr, HEX);
    Serial.print("reg: 0x");
    Serial.println(reg_addr, HEX);
    Serial.print("Len: ");
    Serial.println(len);
#endif

    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    Wire.endTransmission(true);
    
    delay(10);

    Wire.requestFrom(dev_addr, len);
    do 
    {
        read_data[count] = Wire.read();
        count++;
    }
    while(Wire.available() && count < len);

    Wire.endTransmission();

    return 0;
}

void bmi160i2c_delay_ms_cb (uint32_t period_ms)
{
    delay(period_ms);
}


/* BMI160 sensor */

void init_sensor(void)
{
    int status = 0;

    /* Set interface address i2c address */
    bmi160.intf = BMI160_I2C_INTF;
    bmi160.id   = BMI160_DEV_ADDR;

    // Set communication callback functions for BMI160 library
    bmi160.read     = bmi160i2c_read_cb;
    bmi160.write    = bmi160i2c_write_cb;
    bmi160.delay_ms = bmi160i2c_delay_ms_cb;

    // Initialize sensor using library
    status = bmi160_init(&bmi160);
    if (status == BMI160_OK)
    {
        Serial.println("BMI160 initialized!");
        Serial.print("Chip ID 0x");
        Serial.println(bmi160.chip_id, HEX);
    }
    else
    {
        Serial.println("Error initializing BMI160!");
        exit(status);
    }
}

void config_sensor(void)
{
    int status = 0;
        
    /* Select the Output data rate, range of accelerometer sensor */
    bmi160.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    bmi160.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
    bmi160.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    bmi160.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    bmi160.gyro_cfg.odr = BMI160_GYRO_ODR_400HZ;
    bmi160.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    bmi160.gyro_cfg.bw = BMI160_GYRO_BW_OSR4_MODE;

    /* Select the power mode of Gyroscope sensor */
    bmi160.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    /* Set the sensor configuration */
    status = bmi160_set_sens_conf(&bmi160);
    if (status == BMI160_OK)
    {
        Serial.println("BMI160 configured!");
        // start_foc();
    }
    else
    {
        Serial.println("Error configuring BMI160!");
        exit(status);
    }
}

void read_sensor(void)
{
    bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL), &bmi160_accel, &bmi160_gyro, &bmi160);
    // bmi160_get_sensor_data((BMI160_GYRO_SEL | BMI160_TIME_SEL), &bmi160_accel, &bmi160_gyro, &bmi160);

    // Serial.print("Accel: ax="); Serial.print(bmi160_accel.x);
    // Serial.print(", ay="); Serial.print(bmi160_accel.y);
    // Serial.print(", az="); Serial.print(bmi160_accel.z);
    // Serial.print(", time="); Serial.println(bmi160_accel.sensortime);
    // Serial.print("Gyro: gx="); Serial.print(bmi160_gyro.x);
    // Serial.print(", gy="); Serial.print(bmi160_gyro.y);
    // Serial.print(", gz="); Serial.print(bmi160_gyro.z);
    // Serial.print(", time="); Serial.println(bmi160_gyro.sensortime);

    Serial.print("Accel: ax="); Serial.print(convertRawGyro(bmi160_accel.x));
    Serial.print(", ay="); Serial.print(convertRawGyro(bmi160_accel.y));
    Serial.print(", az="); Serial.print(convertRawGyro(bmi160_accel.z));
    Serial.print(", time="); Serial.println(bmi160_accel.sensortime);
    Serial.print("Gyro: gx="); Serial.print(convertRawGyro(bmi160_gyro.x));
    Serial.print(", gy="); Serial.print(convertRawGyro(bmi160_gyro.y));
    Serial.print(", gz="); Serial.print(convertRawGyro(bmi160_gyro.z));
    Serial.print(", time="); Serial.println(bmi160_gyro.sensortime);
    Serial.println();
}