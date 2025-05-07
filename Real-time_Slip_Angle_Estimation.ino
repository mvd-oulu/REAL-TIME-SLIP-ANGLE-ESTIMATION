#include <FlexCAN_T4.h>
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can;
#include <stdio.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <BasicLinearAlgebra.h>
#define RAD_TO_DEG 57.2957795 
#define ImuWire Wire
Adafruit_BNO08x bno08xNew;

char vtgHeading[12] = {};
char speedkmh[10] = {};
char latitude[15];
char longitude[15];
char fixQuality[2];

float speedkmhf;
float vtgHeadingf;
float latitudef;
float longitudef;
float latitudeNMEA;
float longitudeNMEA;
float fixQualityf;
float phi = 0;        //Course Angle - VTG Heading
float phi_IMU = 0;    //Course Angle - from IMU
float psi = 0;        //Vehicle Heading - Dual RTK
float v = 0;          //velocity to the actual moving direction
float psi_dot_i = 0;  // Actual rotation rate of vehicle body 
float gyro_x = 0;
float gyro_y = 0;
float gyro_z = 0;
float accel_x = 0;
float accel_y = 0;
float accel_z = 0;
float roll = 0;
float pitch = 0;
float roll_q = 0;
float pitch_q = 0;
float yaw_q = 0;
float gcorrectionHeading = 0.0;
bool headingCorrected = false;
float yaw_game = 0;
float IMUCalibrationstatus = 0;
float L = 2.5;  // Wheelbase of the vehicle 

unsigned long previousMillisk = 0;    // Stores the last time the code was executed
const unsigned long intervalk = 100;  // Interval at which to run the code (100ms for 10Hz) 
unsigned long previousMillisi = 0;
const unsigned long intervali = 10;  //  IMU sensor reading frequency
unsigned long previousMillisg = 0;
const unsigned long intervalg = 50;  //  GNSS measurement update rate 
unsigned long qualityFixTimestamp = 0;
const unsigned long IMU_LAG_MS = 200;  // 200ms lag for IMU data
const size_t IMU_BUFFER_SIZE = 100;    // Buffer size for IMU lag handling

// Buffers for IMU data
struct IMUData {
  int IMUCalibrationstatus;
  float yaw_game;
  float psi_dot_i;
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float yaw_q, pitch_q, roll_q;
  unsigned long timestamp;
};

IMUData imuBuffer[IMU_BUFFER_SIZE];
size_t imuBufferIndex = 0;

// Variables for orientation calculation
float Orientation = 0;
bool RTK_initialized = false;
float Adjusted_Orientation = 0;

// Variables for Kalman filtering
float RV = 0.01;  // Measurement noise covariance
float QV = 1;     // Process noise covariance
float PV = 1;     // Initial estimation error covariance
float yaw_modified_phi_VTG_filtered;

// Variables for slip angle filtering
float slip_angle_raw = 0;       // Placeholder for slip angle data
float slip_angle_filtered = 0;  // Store filtered slip angles
float P = 1;                    // Initial estimation error covariance
float R = 1;                    // Measurement noise covariance
float Q = 0.001;                // Process noise covariance

// Variables for velocity filtering
float Velocity_filtered = 0.0;
float P_v = 1.0;
float Q_v = 0.1;
float R_v = 0.5;

float mod360(float angle) {
  return fmod((angle + 360), 360);
}

static float prev_modified_phi_VTG = 0.0;  // Previous modified_phi_VTG
float modified_phi_VTG = 0.0;
static unsigned long prevTime = 0;  // Previous timestamp

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!bno08xNew.begin_I2C()) {
    while (1) {
    }
  }

  Serial.println("BNO08XNew Found!");

  bno08xNew.enableReport(SH2_ARVR_STABILIZED_RV);
  bno08xNew.enableReport(SH2_ACCELEROMETER);
  bno08xNew.enableReport(SH2_GYROSCOPE_CALIBRATED);
  bno08xNew.enableReport(SH2_GAME_ROTATION_VECTOR);

  Can.begin();
  Can.setBaudRate(500000);  // kbps
  Serial.println("CAN Initialized");
}

void loop() {
  sh2_SensorValue_t sensorValue;
  unsigned long Timestamp = millis();  // Timestamp in milliseconds

  speedkmhf = atof(speedkmh);
  vtgHeadingf = atof(vtgHeading);
  latitudeNMEA = atof(latitude);
  longitudeNMEA = atof(longitude);
  fixQualityf = atof(fixQuality);

  // Convert to decimal degrees
  float latitudef = convertToDecimalDegrees(latitudeNMEA);
  float longitudef = convertToDecimalDegrees(longitudeNMEA);

  if (fixQualityf >= 2) {
    if (qualityFixTimestamp == 0) {
      qualityFixTimestamp = millis();
    }
    // Check if 20s have passed since qualityFixTimestamp
    if (millis() - qualityFixTimestamp >= 20000 && !headingCorrected) {
      gcorrectionHeading = psi;
      headingCorrected = true;
    }
  } else {
    // Reset if fixQualityf is not 2
    qualityFixTimestamp = 0;
    headingCorrected = false;
  }

  //GNSS
  unsigned long currentMillisg = millis();            //non-blocking delay for GNSS
  if (currentMillisg - previousMillisg >= intervalg)  //time interval for updating GNSS signals
  {
    previousMillisg = currentMillisg;

    // Update GNSS measurements
    phi = vtgHeadingf;    // degrees - VTG
    psi = heading;        // degrees - RTK - Orientation
    v = speedkmhf / 3.6;  // m/s
  }

  //IMU
  unsigned long currentMillisi = millis();  //IMU Update rate
  if (currentMillisi - previousMillisi >= intervali) {
    previousMillisi = currentMillisi;

    if (bno08xNew.getSensorEvent(&sensorValue)) {
      IMUCalibrationstatus = sensorValue.status;
      if (sensorValue.sensorId == SH2_ACCELEROMETER) {
        accel_x = sensorValue.un.accelerometer.x;  // m/s2
        accel_y = sensorValue.un.accelerometer.y;
        accel_z = sensorValue.un.accelerometer.z;
      }

      if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        gyro_x = sensorValue.un.gyroscope.x;  //rad/s
        gyro_y = sensorValue.un.gyroscope.y;
        gyro_z = sensorValue.un.gyroscope.z;
      }

      if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
        // Retrieve Quaternion values from sensor event
        float dqw = sensorValue.un.gameRotationVector.real;
        float dqx = sensorValue.un.gameRotationVector.i;
        float dqy = sensorValue.un.gameRotationVector.j;
        float dqz = sensorValue.un.gameRotationVector.k;

        // Normalize the quaternion (to ensure proper magnitude)
        float norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz);
        dqw /= norm;
        dqx /= norm;
        dqy /= norm;
        dqz /= norm;

        // Calculate yaw from quaternion
        float ysqr = dqy * dqy;
        float t3 = 2.0 * (dqw * dqz + dqx * dqy);
        float t4 = 1.0 - 2.0 * (ysqr + dqz * dqz);
        yaw_game = atan2(t3, t4);  // yaw is in radians

        // Convert yaw to degrees
        yaw_game = -yaw_game * RAD_TO_DEG;

        // Optional correction to make sure yaw is in [0, 360] degrees range
        if (yaw_game < 0) {
          yaw_game += 360.0;
        }

        // Add the correction heading to the calculated yaw
        yaw_game += gcorrectionHeading;

        // Ensure yaw is still in [0, 360] degrees
        if (yaw_game >= 360.0) {
          yaw_game -= 360.0;
        } else if (yaw_game < 0.0) {
          yaw_game += 360.0;
        }
      }

      if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
        // Extract quaternion values from the sensorValue if available
        float qw = sensorValue.un.arvrStabilizedRV.real;
        float qx = sensorValue.un.arvrStabilizedRV.i;
        float qy = sensorValue.un.arvrStabilizedRV.j;
        float qz = sensorValue.un.arvrStabilizedRV.k;

        // Convert quaternion to Euler angles
        yaw_q = atan2(2.0 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz);
        pitch_q = asin(2.0 * (qw * qy - qx * qz));
        roll_q = atan2(2.0 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz);

        // actual rotation rate of vehicle
        psi_dot_i = ((sin(roll_q) / cos(pitch_q)) * gyro_y + (cos(roll_q) / cos(pitch_q)) * gyro_z) * 180.0 / PI;
      }
    }
  }

  unsigned long currentMillisk = millis();  //non-blocking delay for data processing
  if (currentMillisk - previousMillisk >= intervalk) {
    previousMillisk = currentMillisk;

    // Handle GNSS data first
    if (fixQualityf >= 1 && !RTK_initialized) {
      Orientation = psi - 90;  // GNSS quality is good; calculate initial orientation using RTK
      RTK_initialized = true;
    }

    // Calculate adjusted RTK orientation
    if (RTK_initialized) {
      Adjusted_Orientation = mod360(Orientation + yaw_game + 0.00);  //add antena alignment here
    }

    // Store IMU data in buffer
    imuBuffer[imuBufferIndex] = { IMUCalibrationstatus, yaw_game, psi_dot_i, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, yaw_q, pitch_q, roll_q, Timestamp };
    imuBufferIndex = (imuBufferIndex + 1) % IMU_BUFFER_SIZE;

    // Retrieve lagged IMU data
    unsigned long currentTimestamp = millis();
    size_t laggedIndex = imuBufferIndex;
    for (size_t i = 0; i < IMU_BUFFER_SIZE; i++) {
      size_t index = (imuBufferIndex + IMU_BUFFER_SIZE - i - 1) % IMU_BUFFER_SIZE;
      if (currentTimestamp - imuBuffer[index].timestamp >= IMU_LAG_MS) {
        laggedIndex = index;
        break;
      }
    }

    IMUData laggedIMU = imuBuffer[laggedIndex];

    unsigned long currentTime = millis();
    float deltaT = (currentTime - prevTime) / 1000.0;
    prevTime = currentTime;

    // Kalman filtering for yaw_modified_phi_VTG_filtered
    if (v > 0.09 && phi != 0) {
      float modified_phi_VTG = phi;
      if (modified_phi_VTG == prev_modified_phi_VTG) {
        yaw_modified_phi_VTG_filtered -= laggedIMU.gyro_z * deltaT * 100;
      } else {
        yaw_modified_phi_VTG_filtered = modified_phi_VTG;
      }

      prev_modified_phi_VTG = modified_phi_VTG;  // Update for next iteration

      // Prediction step
      float xv_pred = yaw_modified_phi_VTG_filtered;
      float Pv_pred = PV + QV;

      // Update step
      float Kv = Pv_pred / (Pv_pred + RV);
      yaw_modified_phi_VTG_filtered = xv_pred + Kv * (modified_phi_VTG - xv_pred);
      PV = (1 - Kv) * Pv_pred;

    } else {
      yaw_modified_phi_VTG_filtered = Adjusted_Orientation;
    }


    if ((yaw_modified_phi_VTG_filtered - Adjusted_Orientation) > 50) {
      slip_angle_raw = yaw_modified_phi_VTG_filtered - Adjusted_Orientation - 360;
    } else if ((yaw_modified_phi_VTG_filtered - Adjusted_Orientation) < -50) {
      slip_angle_raw = yaw_modified_phi_VTG_filtered - Adjusted_Orientation + 360;
    } else {
      slip_angle_raw = yaw_modified_phi_VTG_filtered - Adjusted_Orientation;
    }

    // Kalman filter for slip angle
    // Prediction step
    float x_pred = slip_angle_raw;  // Predicted state
    float P_pred = P + Q;           // Predicted estimation error covariance

    // Update step
    float K = P_pred / (P_pred + R);                       // Kalman gain
    float x_est = x_pred + K * (slip_angle_raw - x_pred);  // Updated state estimate
    P = (1 - K) * P_pred;                                  // Updated estimation error covariance
    slip_angle_filtered = x_est;                           // Store filtered value

    float v_pred = Velocity_filtered;
    float P_pred_v = P_v + Q_v;

    float K_v = P_pred_v / (P_pred_v + R_v);
    Velocity_filtered = v_pred + K_v * (v - v_pred);
    P_v = (1 - K_v) * P_pred_v;

    Serial.print(Timestamp);
    Serial.print(",");
    Serial.print(IMUCalibrationstatus);  //  0,1,2,3
    Serial.print(",");
    Serial.print(fixQualityf);  //0,1,2,3,4
    Serial.print(",");
    Serial.print(headingCorrected);  //0,1
    Serial.print(",");
    Serial.print(longitudef, 8);
    Serial.print(",");
    Serial.print(latitudef, 8);
    Serial.print(",");
    Serial.print(psi, 4);  // RTK degrees
    Serial.print(",");
    Serial.print(yaw_game, 4);  // degrees
    Serial.print(",");
    Serial.print(phi, 4);  //VTG degrees
    Serial.print(",");
    Serial.print(v, 4);  //VTG velocity m/s   
    Serial.print(",");
    Serial.print(psi_dot_i, 4);  //from pitch and roll // degrees
    Serial.print(",");
    Serial.print(accel_x, 4);  // m/s2
    Serial.print(",");
    Serial.print(accel_y, 4);  // m/s2
    Serial.print(",");
    Serial.print(accel_z, 4);  // m/s2
    Serial.print(",");
    Serial.print(gyro_x * RAD_TO_DEG, 4);  // degrees/s
    Serial.print(",");
    Serial.print(gyro_y * RAD_TO_DEG, 4);  // degrees/s
    Serial.print(",");
    Serial.print(gyro_z * RAD_TO_DEG, 4);  // degrees/s 
    Serial.print(",");
    Serial.print(yaw_q * RAD_TO_DEG, 4);  // degrees
    Serial.print(",");
    Serial.print(pitch_q * RAD_TO_DEG, 4);  // degrees
    Serial.print(",");
    Serial.print(roll_q * RAD_TO_DEG, 4);  // degrees   
    Serial.print(",");
    Serial.print(Adjusted_Orientation, 4);  //Adjusted_RTK
    Serial.print(",");
    Serial.print(yaw_modified_phi_VTG_filtered, 4);  
    Serial.print(",");
    Serial.print(slip_angle_raw, 4);  // Raw Slip Angle
    Serial.print(",");
    Serial.println(slip_angle_filtered, 4);  //Slip Angle

    //scaling and data type conversion for CAN messages
    int phi_int = (phi)*10000;
    int gyro_z_deg_int = (gyro_z * RAD_TO_DEG) * 10000;
    int v_int = (v)*10000;
    int psi_int = (psi)*10000;
    int yaw_game_int = (yaw_game)*10000;
    int accel_x_int = (accel_x)*1000;
    int psi_dot_int = (psi_dot_i)*10000;
    int accel_y_int = (accel_y)*1000;
    int accel_z_int = (accel_z)*1000;
    int gyro_x_deg_int = (gyro_x * RAD_TO_DEG) * 10000;
    int gyro_y_deg_int = (gyro_y * RAD_TO_DEG) * 10000;
    int yaw_q_deg_int = (yaw_q * RAD_TO_DEG) * 10000;
    int longitude_int = (longitudef)*10000000;
    int latitude_int = (latitudef)*10000000;
    int slip_angle_filtered_int = (slip_angle_filtered)*10000;
    int slip_angle_raw_int = (slip_angle_raw)*10000;
    int yaw_modified_phi_VTG_filtered_int = (yaw_modified_phi_VTG_filtered)*10000;
    int Adjusted_Orientation_int = (Adjusted_Orientation)*10000;
    int Velocity_filtered_int = (Velocity_filtered)*10000;

    // Message 1: Timestamp, phi
    CAN_message_t Msg1;
    Msg1.id = 0x400;
    Msg1.flags.extended = false;
    Msg1.len = 8;
    Msg1.buf[0] = (Timestamp >> 24) & 0xFF;
    Msg1.buf[1] = (Timestamp >> 16) & 0xFF;
    Msg1.buf[2] = (Timestamp >> 8) & 0xFF;
    Msg1.buf[3] = Timestamp & 0xFF;
    Msg1.buf[4] = (phi_int >> 24) & 0xFF;
    Msg1.buf[5] = (phi_int >> 16) & 0xFF;
    Msg1.buf[6] = (phi_int >> 8) & 0xFF;
    Msg1.buf[7] = phi_int & 0xFF;

    // Message 2: gyro_z_deg, v
    CAN_message_t Msg2;
    Msg2.id = 0x410;
    Msg2.flags.extended = false;
    Msg2.len = 8;
    Msg2.buf[0] = (gyro_z_deg_int >> 24) & 0xFF;
    Msg2.buf[1] = (gyro_z_deg_int >> 16) & 0xFF;
    Msg2.buf[2] = (gyro_z_deg_int >> 8) & 0xFF;
    Msg2.buf[3] = gyro_z_deg_int & 0xFF;
    Msg2.buf[4] = (v_int >> 24) & 0xFF;
    Msg2.buf[5] = (v_int >> 16) & 0xFF;
    Msg2.buf[6] = (v_int >> 8) & 0xFF;
    Msg2.buf[7] = v_int & 0xFF;

    // Message 3: psi, yaw_game
    CAN_message_t Msg3;
    Msg3.id = 0x420;
    Msg3.flags.extended = false;
    Msg3.len = 8;
    Msg3.buf[0] = (psi_int >> 24) & 0xFF;
    Msg3.buf[1] = (psi_int >> 16) & 0xFF;
    Msg3.buf[2] = (psi_int >> 8) & 0xFF;
    Msg3.buf[3] = psi_int & 0xFF;
    Msg3.buf[4] = (yaw_game_int >> 24) & 0xFF;
    Msg3.buf[5] = (yaw_game_int >> 16) & 0xFF;
    Msg3.buf[6] = (yaw_game_int >> 8) & 0xFF;
    Msg3.buf[7] = yaw_game_int & 0xFF;

    // Message 4: fixQualityf, accel_x, IMUCalibrationstatus, psi_dot_i
    CAN_message_t Msg4;
    Msg4.id = 0x430;
    Msg4.flags.extended = false;
    Msg4.len = 8;
    Msg4.buf[0] = fixQualityf;
    Msg4.buf[1] = (accel_x_int >> 8) & 0xFF;
    Msg4.buf[2] = accel_x_int & 0xFF;
    Msg4.buf[3] = IMUCalibrationstatus;
    Msg4.buf[4] = (psi_dot_int >> 24) & 0xFF;
    Msg4.buf[5] = (psi_dot_int >> 16) & 0xFF;
    Msg4.buf[6] = (psi_dot_int >> 8) & 0xFF;
    Msg4.buf[7] = psi_dot_int & 0xFF;

    // Message 5: accel_y, accel_z, gyro_x_deg
    CAN_message_t Msg5;
    Msg5.id = 0x440;
    Msg5.flags.extended = false;
    Msg5.len = 8;
    Msg5.buf[0] = (accel_y_int >> 8) & 0xFF;
    Msg5.buf[1] = accel_y_int & 0xFF;
    Msg5.buf[2] = (accel_z_int >> 8) & 0xFF;
    Msg5.buf[3] = accel_z_int & 0xFF;
    Msg5.buf[4] = (gyro_x_deg_int >> 24) & 0xFF;
    Msg5.buf[5] = (gyro_x_deg_int >> 16) & 0xFF;
    Msg5.buf[6] = (gyro_x_deg_int >> 8) & 0xFF;
    Msg5.buf[7] = gyro_x_deg_int & 0xFF;

    // Message 6: gyro_y_deg, yaw_q_deg
    CAN_message_t Msg6;
    Msg6.id = 0x450;
    Msg6.flags.extended = false;
    Msg6.len = 8;
    Msg6.buf[0] = (gyro_y_deg_int >> 24) & 0xFF;
    Msg6.buf[1] = (gyro_y_deg_int >> 16) & 0xFF;
    Msg6.buf[2] = (gyro_y_deg_int >> 8) & 0xFF;
    Msg6.buf[3] = gyro_y_deg_int & 0xFF;
    Msg6.buf[4] = (yaw_q_deg_int >> 24) & 0xFF;
    Msg6.buf[5] = (yaw_q_deg_int >> 16) & 0xFF;
    Msg6.buf[6] = (yaw_q_deg_int >> 8) & 0xFF;
    Msg6.buf[7] = yaw_q_deg_int & 0xFF;

    // Message 7: longitude, latitude
    CAN_message_t Msg7;
    Msg7.id = 0x460;
    Msg7.flags.extended = false;
    Msg7.len = 8;
    Msg7.buf[0] = (longitude_int >> 24) & 0xFF;
    Msg7.buf[1] = (longitude_int >> 16) & 0xFF;
    Msg7.buf[2] = (longitude_int >> 8) & 0xFF;
    Msg7.buf[3] = longitude_int & 0xFF;
    Msg7.buf[4] = (latitude_int >> 24) & 0xFF;
    Msg7.buf[5] = (latitude_int >> 16) & 0xFF;
    Msg7.buf[6] = (latitude_int >> 8) & 0xFF;
    Msg7.buf[7] = latitude_int & 0xFF;

    // Message 8: slip_angle_filtered, slip_angle_raw
    CAN_message_t Msg8;
    Msg8.id = 0x470;
    Msg8.flags.extended = false;
    Msg8.len = 8;
    Msg8.buf[0] = (slip_angle_filtered_int >> 24) & 0xFF;
    Msg8.buf[1] = (slip_angle_filtered_int >> 16) & 0xFF;
    Msg8.buf[2] = (slip_angle_filtered_int >> 8) & 0xFF;
    Msg8.buf[3] = slip_angle_filtered_int & 0xFF;
    Msg8.buf[4] = (slip_angle_raw_int >> 24) & 0xFF;
    Msg8.buf[5] = (slip_angle_raw_int >> 16) & 0xFF;
    Msg8.buf[6] = (slip_angle_raw_int >> 8) & 0xFF;
    Msg8.buf[7] = slip_angle_raw_int & 0xFF;

    // Message 9: yaw_modified_phi_VTG_filtered, Adjusted_Orientation
    CAN_message_t Msg9;
    Msg9.id = 0x480;
    Msg9.flags.extended = false;
    Msg9.len = 8;
    Msg9.buf[0] = (yaw_modified_phi_VTG_filtered_int >> 24) & 0xFF;
    Msg9.buf[1] = (yaw_modified_phi_VTG_filtered_int >> 16) & 0xFF;
    Msg9.buf[2] = (yaw_modified_phi_VTG_filtered_int >> 8) & 0xFF;
    Msg9.buf[3] = yaw_modified_phi_VTG_filtered_int & 0xFF;
    Msg9.buf[4] = (Adjusted_Orientation_int >> 24) & 0xFF;
    Msg9.buf[5] = (Adjusted_Orientation_int >> 16) & 0xFF;
    Msg9.buf[6] = (Adjusted_Orientation_int >> 8) & 0xFF;
    Msg9.buf[7] = Adjusted_Orientation_int & 0xFF;

    // Message 10: Velocity_filtered
    CAN_message_t Msg10;
    Msg10.id = 0x490;
    Msg10.flags.extended = false;
    Msg10.len = 8;
    Msg10.buf[0] = (Velocity_filtered_int >> 24) & 0xFF;
    Msg10.buf[1] = (Velocity_filtered_int >> 16) & 0xFF;
    Msg10.buf[2] = (Velocity_filtered_int >> 8) & 0xFF;
    Msg10.buf[3] = Velocity_filtered_int & 0xFF;

    // Send CAN Messages
    Can.write(Msg1);
    Can.write(Msg2);
    Can.write(Msg3);
    Can.write(Msg4);
    Can.write(Msg5);
    Can.write(Msg6);
    Can.write(Msg7);
    Can.write(Msg8);
    Can.write(Msg9);
    Can.write(Msg10);
  }
}

float convertToDecimalDegrees(float degMin) {
  int degrees = int(degMin / 100);                    // Extract the degrees part
  float minutes = degMin - (degrees * 100);           // Extract the minutes part
  float decimalDegrees = degrees + (minutes / 60.0);  // Convert minutes to decimal
  return decimalDegrees;
}

void relPosDecode() {

  int carrSoln;
  bool gnssFixOk, diffSoln, relPosValid;

  heading = (int32_t)ackPacket[30] + ((int32_t)ackPacket[31] << 8)
            + ((int32_t)ackPacket[32] << 16) + ((int32_t)ackPacket[33] << 24);
  heading *= 0.0001;

  heading += headingcorr;
  if (heading >= 3600) heading -= 3600;
  if (heading < 0) heading += 3600;
  heading *= 0.1;

  baseline = (int32_t)ackPacket[26] + ((int32_t)ackPacket[27] << 8)
             + ((int32_t)ackPacket[28] << 16) + ((int32_t)ackPacket[29] << 24);
  double baseHP = (signed char)ackPacket[41];
  baseHP *= 0.01;
  baseline += baseHP;

  relPosD = (int32_t)ackPacket[22] + ((int32_t)ackPacket[23] << 8)
            + ((int32_t)ackPacket[24] << 16) + ((int32_t)ackPacket[25] << 24);
  double relPosHPD = (signed char)ackPacket[40];
  relPosHPD *= 0.01;
  relPosD += relPosHPD;
}
