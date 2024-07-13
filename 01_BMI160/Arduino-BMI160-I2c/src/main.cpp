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

void loop() 
{
    read_sensor();
    delay(10);
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

    // Set communication callback functions for BMI160 library
    bmi160.read     = bmi160i2c_read_cb;
    bmi160.write    = bmi160i2c_write_cb;
    bmi160.delay_ms = bmi160i2c_delay_ms_cb;

    /* Set interface address i2c address */
    bmi160.intf = BMI160_I2C_INTF;
    bmi160.id   = BMI160_DEV_ADDR;

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

int8_t start_foc()
{
	int8_t status = 0;
	/* FOC configuration structure */
	struct bmi160_foc_conf foc_conf;
	/* Structure to store the offsets */
	struct bmi160_offsets offsets;
	
	/* Enable FOC for accel with target values of z = 1g ; x,y as 0g */
	foc_conf.acc_off_en = BMI160_ENABLE;
	foc_conf.foc_acc_x  = BMI160_FOC_ACCEL_0G;
	foc_conf.foc_acc_y  = BMI160_FOC_ACCEL_0G;
	foc_conf.foc_acc_z  = BMI160_FOC_ACCEL_POSITIVE_G;
	
	/* Enable FOC for gyro */
	foc_conf.foc_gyr_en = BMI160_ENABLE;
	foc_conf.gyro_off_en = BMI160_ENABLE;

	status = bmi160_start_foc(&foc_conf, &bmi160_offset, &bmi160);
	
	if (status == BMI160_OK) {
		Serial.println("\n FOC DONE SUCCESSFULLY ");
		printf("\n OFFSET VALUES AFTER FOC : ");
		Serial.print("\n OFFSET VALUES ACCEL X : "); Serial.println(offsets.off_acc_x);
		Serial.print("\n OFFSET VALUES ACCEL Y : "); Serial.println(offsets.off_acc_y);
		Serial.print("\n OFFSET VALUES ACCEL Z : "); Serial.println(offsets.off_acc_z);
		Serial.print("\n OFFSET VALUES GYRO  X : "); Serial.println(offsets.off_gyro_x);
		Serial.print("\n OFFSET VALUES GYRO  Y : "); Serial.println(offsets.off_gyro_y);
		Serial.print("\n OFFSET VALUES GYRO  Z : "); Serial.println(offsets.off_gyro_z);
	}
    else
    {
        Serial.println("Error starting FOC!");
        exit(status);
    }

	/* After start of FOC offsets will be updated automatically and 
	 * the data will be very much close to the target values of measurement */

	return status;
}

void config_sensor(void)
{
    int status = 0;
        
    /* Select the Output data rate, range of accelerometer sensor */
    bmi160.accel_cfg.odr = BMI160_ACCEL_ODR_12_5HZ;
    bmi160.accel_cfg.range = BMI160_ACCEL_RANGE_16G;
    bmi160.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    bmi160.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    bmi160.gyro_cfg.odr = BMI160_GYRO_ODR_1600HZ;
    bmi160.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    bmi160.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

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


float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;

  return g;
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