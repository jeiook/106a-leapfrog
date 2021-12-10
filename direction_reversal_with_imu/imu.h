class IMU {
  MPU6050 gyro;
  float angles[3] = {0.0, 0.0, 0.0};
  float prev_angles[3] = {0.0, 0.0, 0.0};
  float ang_vel[3] = {0.0, 0.0, 0.0};
  float ang_integral[3] = {0.0, 0.0, 0.0};
  long prev_time = 0;
  float imu_offset0 = 0;
  float imu_offset1 = 0;
  float imu_offset2 = 0;
  float IMU_MAX = 60;

  /* 
   *  Calculates angular velocity (derivative term) and integral
   */
  void calculate_ang_vel() {
    float curr_time = millis();
    ang_vel[0] = (angles[0] - prev_angles[0]) / (float)(curr_time - prev_time) * 1000;
    ang_vel[1] = (angles[1] - prev_angles[1]) / (float)(curr_time - prev_time) * 1000;
    ang_vel[2] = (angles[2] - prev_angles[2]) / (float)(curr_time - prev_time) * 1000;
    ang_integral[0] += angles[0] * (curr_time - prev_time) / 1000;
    ang_integral[1] += angles[1] * (curr_time - prev_time) / 1000;
    ang_integral[2] += angles[2] * (curr_time - prev_time) / 1000;    
    prev_time = curr_time;
    prev_angles[0] = angles[0];
    prev_angles[1] = angles[1];
    prev_angles[2] = angles[2];
  }
  
  /* 
   *  Calculates angular displacement
   */
  void calculate_angle(float ax, float ay, float az) {
    angles[0] = atan2(ax, (pow((pow(ay, 2) + pow(az, 2)), 0.5))) * 180 / M_PI + imu_offset0;
    angles[1] = atan2(ay, (pow((pow(ax, 2) + pow(az, 2)), 0.5))) * 180 / M_PI + imu_offset1;
    angles[2] = atan2((pow((pow(ay, 2) + pow(az, 2)), 0.5)), az) * 180 / M_PI + imu_offset2;
//    if (angles[2] > 90) {
//      angles[2] = 180 - angles[2];
//    }
  }

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
    calibrate();
  }

  /*
   * Reads in raw angular displacement without filtering.
   */
  void read_raw() {
    float ax = gyro.getAccelerationX();
    float ay = gyro.getAccelerationY();
    float az = gyro.getAccelerationZ();

    calculate_angle(ax, ay,az);
    calculate_ang_vel();
    print_angles();
  }

  /*
   * Function to calculate tilt with ave sample data
   * 
   * args :
   *       int     samp_size number of iterution for doing avg sampling algorithm
   */
  void read_tilt_ave(int samp_size) {
    int16_t x, y, z;
    float lax = 0;
    float lay = 0;
    float laz = 0;
    for(int i = 0; i < samp_size; ++i) {
      gyro.getAcceleration(&x, &y, &z);
      lax += x;
      lay += y;
      laz += z;
    }
    lax /= samp_size;lay /= samp_size;laz /= samp_size;
    calculate_angle(lax, lay, laz);
    calculate_ang_vel();
    print_angles();
  }

  /*
   * Function to calculate tilt with exponitial moving ave sample data
   * 
   * args :
   *       int     samp_size number of iterution for doing avg sampling algorithm
   *       float   alpha     weight of the most recent value
   */
  void read_tilt_expo_mov(int samp_size, float alpha) {
    float cur_x, cur_y, cur_z;
    int16_t x, y, z;
    gyro.getAcceleration(&x, &y, &z);
    cur_x = x; cur_y = y; cur_z = z;
    for (int i = 0; i < samp_size; ++i) {
      gyro.getAcceleration(&x, &y, &z);
      cur_x = (alpha * x) + ((1-alpha) * cur_x);
      cur_y = (alpha * y) + ((1-alpha) * cur_y);
      cur_z = (alpha * z) + ((1-alpha) * cur_z);
      x = cur_x; y = cur_y; z = cur_z;
    }
    calculate_angle(cur_x, cur_y, cur_z);
    calculate_ang_vel();
    print_angles();
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
    static float U_hat[3] = {0, 0, 0}; /* initial estimated state */
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
    calculate_angle(U_hat[0], U_hat[1], U_hat[2]);
    calculate_ang_vel();
    print_angles();
  }

  /*
   * get the angular displacement of the stick at an index (0 or 1) 
   * (reading 2 directly is meaningless)
   * units of degrees
   */
  float get_angle(int i) {
    return angles[i];
  }

  /* 
   *  Print angles[] to Serial for data processing: 
   *  https://www.hackerscapes.com/how-to-save-data-from-arduino-to-a-csv-file-using-processing/
   */
  void print_angles() {
    Serial.print("0: ");
    Serial.print(angles[0]);
    Serial.print(", 1: ");
    Serial.print(angles[1]);
    Serial.print(", 2: ");
    Serial.println(angles[2]);
  }

  /*
   * get the angular velocity of the stick at an index (0 - 2)
   * units of degrees/s
   */
  float get_angular_vel(int i) {
    return ang_vel[i];
  }

  /*
   * get the integral of the angular displacement of the stick at an index (0 - 2)
   */
  float get_angular_int(int i) {
    return ang_integral[i];
  }

  /*
   * Calibrates the imu with the equilibrium point and ground state (lying horizontally)
   */
  void calibrate() {
    Serial.println("hold the stick in the equilibrium position.");
    Serial.println("starting calibration in 2 seconds...");
    delay(2000);
    Serial.println("collecting samples...");
    float time_prev = millis();
    read_tilt_expo_mov(5000, 0.5);
    Serial.print("calibration took ");
    Serial.println(millis() - time_prev);
    imu_offset0 = -get_angle(0);
    imu_offset1 = -get_angle(1);
    imu_offset2 = -get_angle(2);
    Serial.print("\n\ndone calibrating!\n\n");
  }

  float max_angle() {
    return IMU_MAX;
  }
};
