class IMU {
  MPU6050 gyro;
  float angles[3] = {0.0, 0.0, 0.0};
  float prev_angles[3] = {0.0, 0.0, 0.0};
  float omega[3] = {0.0, 0.0, 0.0};
  long prev_time = 0;
  float imu_offset0 = 0;
  float imu_offset1 = 0;
public:
  /*
   * Connects arduino to the MPU6050 IMU
   */
  void setup() {
    // join I2C bus
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.println("Intialize Gyro connection");
    gyro.initialize();
    Serial.println("Testing device connections...");
    Serial.println(gyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    calibrate_eq();
    prev_time = millis();
  }

  /*
    * Function to calculate tilt with Kalman filtered sample data
    * 
    * args :
    *       float*  angles    pointer to array to store tilt_angles value
    *       MPU6050 gyro      intialized object 
    *       int     samp_size number of iterution for doing avg sampling algorithm
    *       float   U         estimate
    */
  void read_tilt_kalman(int samp_size) {
    angles[0] = 0, angles[1] = 0, angles[2] = 0;
    static int16_t U[3] = {0, 0, 0};
    static const double R = 40; /* noise covariance */
    static const double H = 1.00; /* measurement map scalar */
    static const double Q = 10; /* initial extimated covariance */
    static double P = 0; /* initial error covariance */
    static int16_t U_hat[3] = {0, 0, 0}; /* initial estimated state */
    static double K = 0; /* Kalman gain */
    
    gyro.getAcceleration(&U[0], &U[1], &U[2]);
    for (int i = 0; i < samp_size; ++i) {
        gyro.getAcceleration(&U[0], &U[1], &U[2]);
        /* update kalman gain */
        K = P * H / (H * P * H + R);
        /* update estimated */
        U_hat[0] = U_hat[0] + K * (U[0] - H * U_hat[0]);
        U_hat[1] = U_hat[1] + K * (U[1] - H * U_hat[1]);
        U_hat[2] = U_hat[2] + K * (U[2] - H * U_hat[2]);
        /* update covariance */
        P = (1 - K*H)*P + Q;

    }
    /* body tilt angle calculation from filtered imu data */
    angles[0] = atan(U_hat[0] / (pow((pow(U_hat[1], 2) + pow(U_hat[2], 2)), 0.5))) * 180 / M_PI + imu_offset0;
    angles[1] = atan(U_hat[1] / (pow((pow(U_hat[0], 2) + pow(U_hat[2], 2)), 0.5))) * 180 / M_PI + imu_offset1;
    // angles[2] == yaw isn't too reliable, but here is how it would have been calculated
    // angles[2] = atan((pow((pow(U_hat[1], 2) + pow(U_hat[2], 2)), 0.5)) / U_hat[2]) * 180 / M_PI;

    /* print angles[] to Serial for data processing: https://www.hackerscapes.com/how-to-save-data-from-arduino-to-a-csv-file-using-processing/ */
    Serial.print(angles[0]);
    Serial.print(", ");
    Serial.println(angles[1]);
  }

  /*
   * get the angle at an index (0-2)
   */
  float get_angle(int i) {
    for (int i = 0; i < 3; i++) {
      omega[i] = (angles[i] - prev_angles[i]) / (millis() - prevTime);
    }
    prevTime = millis();
    for (int i = 0; i < 3; i++) {
      prev_angles[i] = angles[i];
    }
    read_tilt_kalman(100);
    return angles[i];
  }

  /*
   * Calibrate the imu so that a held position is the equilibrium
   */
  void calibrate_eq() {
    Serial.println("starting calibration in 2 seconds...");
    delay(2000);
    read_tilt_kalman(10000);
    Serial.println("done calibrating.");
    imu_offset0 = -get_angle(0);
    imu_offset1 = -get_angle(1);
  }
};
