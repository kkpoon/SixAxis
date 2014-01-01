#include "Wire.h"

#include "I2Cdev.h"
#include "MPU6050.h"

#define LED_PIN 13
#define ALPHA 0.5
#define MPU6050_SENS_2G 16384
#define MPU6050_SENS_4G 8192
#define MPU6050_SENS_8G 4096
#define MPU6050_SENS_16G 2048
#define MPU6050_TEMP_ZERO -12412 // -512 - (340 * 35)
#define MPU6050_TEMP_PER_DEGREE 340

MPU6050 mpu6050;

// measured data
int16_t ac[3];
int16_t av[3];
int16_t sensor_temperature;

// calculated data
double g[3];

float pitch;
float roll;
float yaw;

double norm(double n, double min_n, double max_n) {
    return (n - min_n) / (max_n - min_n);
}

void readSensors() {
    mpu6050.getMotion6(&ac[0], &ac[1], &ac[2], &av[0], &av[1], &av[2]);
    sensor_temperature = (mpu6050.getTemperature() - MPU6050_TEMP_ZERO) / MPU6050_TEMP_PER_DEGREE;
}

void calculateData() {
    for (int i = 0; i < 3; i++) {
        g[i] = norm(ac[i], 0.0, MPU6050_SENS_2G) * ALPHA + g[i] * (1.0 - ALPHA);
    }

    pitch = -atan2(g[0], g[2]);
    roll = atan2(g[1], g[2]);
}

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    delay(5000);
    // initialize device
    Serial.println("Initializing I2C devices...");

    mpu6050.initialize();

    boolean mpu6050OK = mpu6050.testConnection();
    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(mpu6050OK ? "MPU6050 connection successful" : "MPU6050 connection failed");
    
    mpu6050.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    mpu6050.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

    mpu6050.setXAccelOffset(-1880);
    mpu6050.setYAccelOffset(2260);
    mpu6050.setZAccelOffset(550);
    
    mpu6050.setXGyroOffset(-32);
    mpu6050.setYGyroOffset(-41);
    mpu6050.setZGyroOffset(3);
    
    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, mpu6050OK);

    readSensors();
    for (int i = 0; i < 3; i++) {
        g[i] = norm(ac[i], 0.0, MPU6050_SENS_2G);
    }
}

void loop() {
    readSensors();
    calculateData();

    Serial.print(ac[0]);
    Serial.print(",");
    Serial.print(ac[1]);
    Serial.print(",");
    Serial.print(ac[2]);
    Serial.print(",");
    Serial.print(av[0]);
    Serial.print(",");
    Serial.print(av[1]);
    Serial.print(",");
    Serial.print(av[2]);
    Serial.print(",");
    Serial.print(g[0]);
    Serial.print(",");
    Serial.print(g[1]);
    Serial.print(",");
    Serial.print(g[2]);
    Serial.print(",");
    Serial.print(sensor_temperature);
    Serial.println("");

    delay(1000);
}


