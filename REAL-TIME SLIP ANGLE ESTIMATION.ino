// Serial Ports
#define SerialAOG Serial                //AgIO USB conection
#define SerialRTK Serial3               //RTK radio
HardwareSerial* SerialGPS = &Serial7;   //Main postion receiver (GGA) (Serial2 must be used here with T4.0 / Basic Panda boards - Should auto swap)
HardwareSerial* SerialGPS2 = &Serial2;  //Dual heading receiver
HardwareSerial* SerialGPSTmp = NULL;
//HardwareSerial* SerialAOG = &Serial;

const int32_t baudAOG = 115200;
const int32_t baudGPS = 460800;
const int32_t baudRTK = 9600;  // most are using Xbee radios with default of 115200

// Baudrates for detecting UBX receiver
uint32_t baudrates[]{
  4800,
  9600,
  19200,
  38400,
  57600,
  115200,
  230400,
  460800,
  921600
};

const uint32_t nrBaudrates = sizeof(baudrates) / sizeof(baudrates[0]);

#define ImuWire Wire  //SCL=19:A5 SDA=18:A4

#define RAD_TO_DEG_X_10 572.95779513082320876798154814105

//Swap BNO08x roll & pitch?
//const bool swapRollPitch = false;

const bool invertRoll = true;  //Used for IMU with dual antenna
#define baseLineLimit 5        //Max CM differance in baseline

#define REPORT_INTERVAL 20   //BNO report time, we want to keep reading it quick & offen. Its not timmed to anything just give constant data.
uint32_t READ_BNO_TIME = 0;  //Used stop BNO data pile up (This version is without resetting BNO everytime)

//Status LED's
#define GGAReceivedLED 13         //Teensy onboard LED
#define Power_on_LED 5            //Red
#define Ethernet_Active_LED 6     //Green
#define GPSRED_LED 9              //Red (Flashing = NO IMU or Dual, ON = GPS fix with IMU)
#define GPSGREEN_LED 10           //Green (Flashing = Dual bad, ON = Dual good)
#define AUTOSTEER_STANDBY_LED 11  //Red
#define AUTOSTEER_ACTIVE_LED 12   //Green
uint32_t gpsReadyTime = 0;        //Used for GGA timeout

//for v2.2
// #define Power_on_LED 22
// #define Ethernet_Active_LED 23
// #define GPSRED_LED 20
// #define GPSGREEN_LED 21
// #define AUTOSTEER_STANDBY_LED 38
// #define AUTOSTEER_ACTIVE_LED 39

/*****************************************************************/

// Ethernet Options (Teensy 4.1 Only)
#ifdef ARDUINO_TEENSY41
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

struct ConfigIP {
  uint8_t ipOne = 192;
  uint8_t ipTwo = 168;
  uint8_t ipThree = 5;
};
ConfigIP networkAddress;  //3 bytes

// IP & MAC address of this module of this module
byte Eth_myip[4] = { 0, 0, 0, 0 };  //This is now set via AgIO
byte mac[] = { 0x00, 0x00, 0x56, 0x00, 0x00, 0x78 };

unsigned int portMy = 5120;            // port of this module
unsigned int AOGNtripPort = 2233;      // port NTRIP data from AOG comes in
unsigned int AOGAutoSteerPort = 8888;  // port Autosteer data from AOG comes in
unsigned int portDestination = 9999;   // Port of AOG that listens
char Eth_NTRIP_packetBuffer[512];      // buffer for receiving ntrip data

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Eth_udpPAOGI;      //Out port 5544
EthernetUDP Eth_udpNtrip;      //In port 2233
EthernetUDP Eth_udpAutoSteer;  //In & Out Port 8888

IPAddress Eth_ipDestination;
#endif  // ARDUINO_TEENSY41

byte CK_A = 0;
byte CK_B = 0;
int relposnedByteCount = 0;

//Speed pulse output
elapsedMillis speedPulseUpdateTimer = 0;
byte velocityPWM_Pin = 36;  // Velocity (MPH speed) PWM pin

#include "zNMEAParser.h"
#include <Wire.h>
#include "BNO08x_AOG.h"

//Used to set CPU speed
extern "C" uint32_t set_arm_clock(uint32_t frequency);  // required prototype

bool useDual = false;
bool dualReadyGGA = false;
bool dualReadyRelPos = false;

// booleans to see if we are using CMPS or BNO08x
bool useCMPS = false;
bool useBNO08x = false;

//CMPS always x60
#define CMPS14_ADDRESS 0x60

// BNO08x address variables to check where it is
const uint8_t bno08xAddresses[] = { 0x4A, 0x4B };
const int16_t nrBNO08xAdresses = sizeof(bno08xAddresses) / sizeof(bno08xAddresses[0]);
uint8_t bno08xAddress;
BNO080 bno08x;

//Dual
double headingcorr = 900;  //90deg heading correction (90deg*10)
// Heading correction 180 degrees, because normally the heading antenna is in front, but we have it at the back
//double headingcorr = 1800;  // 180deg heading correction (180deg*10)

double baseline = 0;
double rollDual = 0;
double relPosD = 0;
double heading = 0;


byte ackPacket[72] = { 0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

constexpr int serial_buffer_size = 512;
uint8_t GPSrxbuffer[serial_buffer_size];   //Extra serial rx buffer
uint8_t GPStxbuffer[serial_buffer_size];   //Extra serial tx buffer
uint8_t GPS2rxbuffer[serial_buffer_size];  //Extra serial rx buffer
uint8_t GPS2txbuffer[serial_buffer_size];  //Extra serial tx buffer
uint8_t RTKrxbuffer[serial_buffer_size];   //Extra serial rx buffer

/* A parser is declared with 3 handlers at most */
NMEAParser<2> parser;

bool isTriggered = false;
bool blink = false;

bool Autosteer_running = true;  //Auto set off in autosteer setup
bool Ethernet_running = false;  //Auto set on in ethernet setup
bool GGA_Available = false;     //Do we have GGA on correct port?
uint32_t PortSwapTime = 0;

float yaw = 0;

//Fusing BNO with Dual
double rollDelta;
double rollDeltaSmooth;
double correctionHeading;
double gyroDelta;
double imuGPS_Offset;
double gpsHeading;
double imuCorrected;
#define twoPI 6.28318530717958647692
#define PIBy2 1.57079632679489661923

// Buffer to read chars from Serial, to check if "!AOG" is found
uint8_t aogSerialCmd[4] = { '!', 'A', 'O', 'G' };
uint8_t aogSerialCmdBuffer[6];
uint8_t aogSerialCmdCounter = 0;

// Booleans to indictate to passthrough GPS or GPS2
bool passThroughGPS = false;
bool passThroughGPS2 = false;

//-=-=-=-=- UBX binary specific variables
struct ubxPacket {
  uint8_t cls;
  uint8_t id;
  uint16_t len;           //Length of the payload. Does not include cls, id, or checksum bytes
  uint16_t counter;       //Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
  uint16_t startingSpot;  //The counter value needed to go past before we begin recording into payload array
  uint8_t* payload;       // We will allocate RAM for the payload if/when needed.
  uint8_t checksumA;      //Given to us from module. Checked against the rolling calculated A/B checksums.
  uint8_t checksumB;

  ////sfe_ublox_packet_validity_e valid;			 //Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
  ////sfe_ublox_packet_validity_e classAndIDmatch; // Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID
};


// VS Start###################################################################################################################################################

#include <FlexCAN_T4.h>
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can;

#define ImuWire Wire
#include <stdio.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <BasicLinearAlgebra.h>

char vtgHeading[12] = {};

#define RAD_TO_DEG 57.295779513082320876798154814105 // Radians to degrees

Adafruit_BNO08x bno08xNew;

float speedkmhf;
float vtgHeadingf;
float latitudef;
float longitudef;
float latitudeNMEA;
float longitudeNMEA;
float fixQualityf;


char speedkmh[10] = {};
char latitude[15];
char longitude[15];
char fixQuality[2];

// State vector variables

float phi = 0;      //Course Angle - VTG Heading
float phi_IMU = 0;  //Course Angle - from IMU
float psi = 0;      //Vehicle Heading - Dual RTK
float v = 0;        //velocity to the actual moving direction - VTG // (need the V from IMU also? for low speeds.)
float psi_dot_i = 0;  // Actual rotation rate of vehicle body - slightly differnt to yaw rate from gyroscope


// Variable for roll and pitch angle
float gyro_x = 0;
float gyro_y = 0;
float gyro_z = 0;
float accel_x = 0;
float accel_y = 0;
float accel_z = 0;
float roll = 0;
float pitch = 0;
//float yaw = 0;
float roll_q = 0;
float pitch_q = 0;
float yaw_q = 0;

float gcorrectionHeading = 0.0;
bool headingCorrected = false;
float yaw_game = 0;

float IMUCalibrationstatus=0;

float L = 2.5;  // Wheelbase of the vehicle (adjust to tractor's wheelbase)

unsigned long previousMillisk = 0;    // Stores the last time the code was executed
const unsigned long intervalk = 100;  // Interval at which to run the code (100ms for 10Hz)  //  print update frequency

unsigned long previousMillisi = 0;
const unsigned long intervali = 10;  //  IMU sensor reading frequency

unsigned long previousMillisg = 0;
const unsigned long intervalg = 50;  //  GNSS measurement update rate // 5hz - 200ms update rate from GNSS module

unsigned long qualityFixTimestamp = 0; 


const unsigned long IMU_LAG_MS = 200;  // 200ms lag for IMU data
const size_t IMU_BUFFER_SIZE = 100;   // Buffer size for IMU lag handling


// Buffers for IMU data with lag
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
float Adjusted_Orientation=0;

// Variables for Kalman filtering
float RV = 0.01;  // Measurement noise covariance
float QV = 1;     // Process noise covariance
float PV = 1;     // Initial estimation error covariance
float yaw_modified_phi_VTG_filtered;

// Variables for slip angle filtering
float slip_angle_raw = 0; // Placeholder for slip angle data
float slip_angle_filtered = 0; // Store filtered slip angles
float P = 1; // Initial estimation error covariance
float R = 1; // Measurement noise covariance
float Q = 0.001; // Process noise covariance


// Variables for velocity filtering
float Velocity_filtered = 0.0;           
float P_v = 1.0;                 
float Q_v = 0.1;                  
float R_v = 0.5;    



// Helper functions
float mod360(float angle) {
  return fmod((angle + 360), 360);
}

static float prev_modified_phi_VTG = 0.0; // Previous modified_phi_VTG
float modified_phi_VTG = 0.0;
static unsigned long prevTime = 0;        // Previous timestamp


// VS End ###################################################################################################################################################

// Setup procedure ------------------------
void setup() {
  {
    delay(500); 

    pinMode(GGAReceivedLED, OUTPUT);
    pinMode(Power_on_LED, OUTPUT);
    pinMode(Ethernet_Active_LED, OUTPUT);
    pinMode(GPSRED_LED, OUTPUT);
    pinMode(GPSGREEN_LED, OUTPUT);
    pinMode(AUTOSTEER_STANDBY_LED, OUTPUT);
    pinMode(AUTOSTEER_ACTIVE_LED, OUTPUT);

    // the dash means wildcard
    parser.setErrorHandler(errorHandler);
    parser.addHandler("G-GGA", GGA_Handler);
    parser.addHandler("G-VTG", VTG_Handler);

    delay(10);
    Serial.begin(baudAOG);
    delay(10);
    Serial.println("Start setup");

    SerialGPS->begin(baudGPS);
    //SerialGPS->addMemoryForRead(GPSrxbuffer, serial_buffer_size);
    //SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);

    delay(10);
    SerialRTK.begin(baudRTK);
    SerialRTK.addMemoryForRead(RTKrxbuffer, serial_buffer_size);

    delay(10);
    SerialGPS2->begin(baudGPS);
    //SerialGPS2->addMemoryForRead(GPS2rxbuffer, serial_buffer_size);
    //SerialGPS2->addMemoryForWrite(GPS2txbuffer, serial_buffer_size);

    Serial.println("SerialAOG, SerialRTK, SerialGPS and SerialGPS2 initialized");

    Serial.println("\r\nStarting AutoSteer...");
    autosteerSetup();

    Serial.println("\r\nStarting Ethernet...");
    EthernetStart();

    Serial.println("\r\nStarting IMU...");
    //test if CMPS working
    uint8_t error;

    ImuWire.begin();

    ImuWire.beginTransmission(CMPS14_ADDRESS);
    error = ImuWire.endTransmission();

    if (error == 0) {
      //Serial.println("Error = 0");
      Serial.print("CMPS14 ADDRESs: 0x");
      Serial.println(CMPS14_ADDRESS, HEX);
      Serial.println("CMPS14 Ok.");
      useCMPS = true;
    } else {
      //Serial.println("Error = 4");
      Serial.println("CMPS not Connected or Found");
    }

    if (!useCMPS) {
      for (int16_t i = 0; i < nrBNO08xAdresses; i++) {
        bno08xAddress = bno08xAddresses[i];

        //Serial.print("\r\nChecking for BNO08X on ");
        //Serial.println(bno08xAddress, HEX);
        ImuWire.beginTransmission(bno08xAddress);
        error = ImuWire.endTransmission();

        if (error == 0) {
          //Serial.println("Error = 0");
          Serial.print("0x");
          Serial.print(bno08xAddress, HEX);
          Serial.println(" BNO08X Ok.");

          // Initialize BNO080 lib
          if (bno08x.begin(bno08xAddress, ImuWire))  //??? Passing NULL to non pointer argument, remove maybe ???
          {
            //Increase I2C data rate to 400kHz
            ImuWire.setClock(400000);

            delay(300);

            // Use gameRotationVector and set REPORT_INTERVAL
            bno08x.enableGameRotationVector(REPORT_INTERVAL);
            useBNO08x = true;
          } else {
            Serial.println("BNO080 not detected at given I2C address.");
          }
        } else {
          //Serial.println("Error = 4");
          Serial.print("0x");
          Serial.print(bno08xAddress, HEX);
          Serial.println(" BNO08X not Connected or Found");
        }
        if (useBNO08x) break;
      }
    }

    delay(100);
    Serial.print("\r\nuseCMPS = ");
    Serial.println(useCMPS);
    Serial.print("useBNO08x = ");
    Serial.println(useBNO08x);
  }

  // VS Setup Start###################################################################################################################################################


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
  //bno08xNew.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED);
  //bno08xNew.enableReport(SH2_RAW_MAGNETOMETER);

  Can.begin();
  Can.setBaudRate(500000);   // kbps
  Serial.println("CAN Initialized");

  // VS Setup End###################################################################################################################################################


  Serial.println("\r\nEnd setup, waiting for GPS...\r\n");


}

void loop() {
  {
    
    if (GGA_Available == false && !passThroughGPS && !passThroughGPS2) {
      if (systick_millis_count - PortSwapTime >= 10000) {
        //Serial.println("Swapping GPS ports...\r\n");
        SerialGPSTmp = SerialGPS;
        SerialGPS = SerialGPS2;
        SerialGPS2 = SerialGPSTmp;
        PortSwapTime = systick_millis_count;
      }
    }


    // Pass NTRIP etc to GPS
    if (SerialAOG.available())

    {
      uint8_t incoming_char = SerialAOG.read();

      // Check incoming char against the aogSerialCmd array
      // The configuration utility will send !AOGR1, !AOGR2 or !AOGED (close/end)
      if (aogSerialCmdCounter < 4 && aogSerialCmd[aogSerialCmdCounter] == incoming_char) {
        aogSerialCmdBuffer[aogSerialCmdCounter] = incoming_char;
        aogSerialCmdCounter++;
      }
      // Whole command prefix is in, handle it
      else if (aogSerialCmdCounter == 4) {
        aogSerialCmdBuffer[aogSerialCmdCounter] = incoming_char;
        aogSerialCmdBuffer[aogSerialCmdCounter + 1] = SerialAOG.read();

        if (aogSerialCmdBuffer[aogSerialCmdCounter] == 'R') {
          HardwareSerial* autoBaudSerial = NULL;

          // Reset SerialGPS and SerialGPS2
          SerialGPS = &Serial7;
          SerialGPS2 = &Serial2;

          if (aogSerialCmdBuffer[aogSerialCmdCounter + 1] == '1') {
            passThroughGPS = true;
            passThroughGPS2 = false;
            autoBaudSerial = SerialGPS;
          } else if (aogSerialCmdBuffer[aogSerialCmdCounter + 1] == '2') {
            passThroughGPS = false;
            passThroughGPS2 = true;
            autoBaudSerial = SerialGPS2;
          }

          const uint8_t UBX_SYNCH_1 = 0xB5;
          const uint8_t UBX_SYNCH_2 = 0x62;
          const uint8_t UBX_CLASS_ACK = 0x05;
          const uint8_t UBX_CLASS_CFG = 0x06;
          const uint8_t UBX_CFG_RATE = 0x08;

          ubxPacket packetCfg{};

          packetCfg.cls = UBX_CLASS_CFG;
          packetCfg.id = UBX_CFG_RATE;
          packetCfg.len = 0;
          packetCfg.startingSpot = 0;

          calcChecksum(&packetCfg);

          byte mon_rate[] = { 0xB5, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
          mon_rate[2] = packetCfg.cls;
          mon_rate[3] = packetCfg.id;
          mon_rate[4] = packetCfg.len & 0xFF;
          mon_rate[5] = packetCfg.len >> 8;
          mon_rate[6] = packetCfg.checksumA;
          mon_rate[7] = packetCfg.checksumB;

          // Check baudrate
          bool communicationSuccessfull = false;
          uint32_t baudrate = 0;

          for (uint32_t i = 0; i < nrBaudrates; i++) {
            baudrate = baudrates[i];

            Serial.print(F("Checking baudrate: "));
            Serial.println(baudrate);

            autoBaudSerial->begin(baudrate);
            delay(100);

            // first send dumb data to make sure its on
            autoBaudSerial->write(0xFF);

            // Clear
            while (autoBaudSerial->available() > 0) {
              autoBaudSerial->read();
            }

            // Send request
            autoBaudSerial->write(mon_rate, 8);

            uint32_t millis_read = systick_millis_count;
            constexpr uint32_t UART_TIMEOUT = 1000;
            int ubxFrameCounter = 0;
            bool isUbx = false;
            uint8_t incoming = 0;

            uint8_t requestedClass = packetCfg.cls;
            uint8_t requestedID = packetCfg.id;

            uint8_t packetBufCls = 0;
            uint8_t packetBufId = 0;

            do {
              while (autoBaudSerial->available() > 0) {
                incoming = autoBaudSerial->read();

                if (!isUbx && incoming == UBX_SYNCH_1)  // UBX binary frames start with 0xB5, aka μ
                {
                  ubxFrameCounter = 0;
                  isUbx = true;
                }

                if (isUbx) {
                  // Decide what type of response this is
                  if ((ubxFrameCounter == 0) && (incoming != UBX_SYNCH_1))  // ISO 'μ'
                  {
                    isUbx = false;                                                 // Something went wrong. Reset.
                  } else if ((ubxFrameCounter == 1) && (incoming != UBX_SYNCH_2))  // ASCII 'b'
                  {
                    isUbx = false;  // Something went wrong. Reset.
                  } else if (ubxFrameCounter == 1 && incoming == UBX_SYNCH_2) {
                    // Serial.println("UBX_SYNCH_2");
                    // isUbx should be still true
                  } else if (ubxFrameCounter == 2)  // Class
                  {
                    // Record the class in packetBuf until we know what to do with it
                    packetBufCls = incoming;        // (Duplication)
                  } else if (ubxFrameCounter == 3)  // ID
                  {
                    // Record the ID in packetBuf until we know what to do with it
                    packetBufId = incoming;  // (Duplication)

                    // We can now identify the type of response
                    // If the packet we are receiving is not an ACK then check for a class and ID match
                    if (packetBufCls != UBX_CLASS_ACK) {
                      // This is not an ACK so check for a class and ID match
                      if ((packetBufCls == requestedClass) && (packetBufId == requestedID)) {
                        // This is not an ACK and we have a class and ID match
                        communicationSuccessfull = true;
                      } else {
                        // This is not an ACK and we do not have a class and ID match
                        // so we should keep diverting data into packetBuf and ignore the payload
                        isUbx = false;
                      }
                    }
                  }
                }

                // Finally, increment the frame counter
                ubxFrameCounter++;
              }
            } while (systick_millis_count - millis_read < UART_TIMEOUT);

            if (communicationSuccessfull) {
              break;
            }
          }

          if (communicationSuccessfull) {
            SerialAOG.write(aogSerialCmdBuffer, 6);
            SerialAOG.print(F("Found reciever at baudrate: "));
            SerialAOG.println(baudrate);

            // Let the configuring program know it can proceed
            SerialAOG.println("!AOGOK");
          } else {
            SerialAOG.println(F("u-blox GNSS not detected. Please check wiring."));
          }

          aogSerialCmdCounter = 0;
        }
        // END command. maybe think of a different abbreviation
        else if (aogSerialCmdBuffer[aogSerialCmdCounter] == 'E' && aogSerialCmdBuffer[aogSerialCmdCounter + 1] == 'D') {
          passThroughGPS = false;
          passThroughGPS2 = false;
          aogSerialCmdCounter = 0;
        }
      } else {
        aogSerialCmdCounter = 0;
      }

      if (passThroughGPS) {
        SerialGPS->write(incoming_char);

      } else if (passThroughGPS2) {
        SerialGPS2->write(incoming_char);

      } else {
        SerialGPS->write(incoming_char);
      }
    }

    // Read incoming nmea from GPS
    if (SerialGPS->available()) {

      if (passThroughGPS) {
        SerialAOG.write(SerialGPS->read());

      } else {
        parser << SerialGPS->read();
      }
    }

    udpNtrip();

    // Check for RTK Radio
    if (SerialRTK.available()) {
      SerialGPS->write(SerialRTK.read());
    }

    // If both dual messages are ready, send to AgOpen
    if (dualReadyGGA == true && dualReadyRelPos == true) {
      BuildNmea();
      dualReadyGGA = false;
      dualReadyRelPos = false;
    }

    // If anything comes in SerialGPS2 RelPos data
    if (SerialGPS2->available()) {
      uint8_t incoming_char = SerialGPS2->read();  //Read RELPOSNED from F9P

      if (passThroughGPS2) {
        SerialAOG.write(incoming_char);

      } else {
        // Just increase the byte counter for the first 3 bytes
        if (relposnedByteCount < 4 && incoming_char == ackPacket[relposnedByteCount]) {
          relposnedByteCount++;
        } else if (relposnedByteCount > 3) {
          // Real data, put the received bytes in the buffer
          ackPacket[relposnedByteCount] = incoming_char;
          relposnedByteCount++;
        } else {
          // Reset the counter, becaues the start sequence was broken
          relposnedByteCount = 0;
        }
      }
    }

    // Check the message when the buffer is full
    if (relposnedByteCount > 71) {
      if (calcChecksum()) {
        //if(deBug) Serial.println("RelPos Message Recived");
        digitalWrite(GPSRED_LED, LOW);  //Turn red GPS LED OFF (we are now in dual mode so green LED)
        useDual = true;
        relPosDecode();
      }
      /*  else {
            if(deBug) Serial.println("ACK Checksum Failure: ");
            }
          */
      relposnedByteCount = 0;
    }

    //GGA timeout, turn off GPS LED's etc
    if ((systick_millis_count - gpsReadyTime) > 10000)  //GGA age over 10sec
    {
      digitalWrite(GPSRED_LED, LOW);
      digitalWrite(GPSGREEN_LED, LOW);
      useDual = false;
    }

    //Read BNO
    if ((systick_millis_count - READ_BNO_TIME) > REPORT_INTERVAL && useBNO08x) {
      READ_BNO_TIME = systick_millis_count;
      readBNO();
    }

    if (Autosteer_running) autosteerLoop();
    else ReceiveUdp();

    if (Ethernet.linkStatus() == LinkOFF) {
      digitalWrite(Power_on_LED, 1);
      digitalWrite(Ethernet_Active_LED, 0);
    }
    if (Ethernet.linkStatus() == LinkON) {
      digitalWrite(Power_on_LED, 0);
      digitalWrite(Ethernet_Active_LED, 1);
    }
  }


   // VS LoopStart###################################################################################################################################################
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


    if (fixQualityf >= 2){
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
      //get fix time
      phi = vtgHeadingf;    // degrees - VTG
      psi = heading;        // degrees - RTK - Orientation
      v = speedkmhf / 3.6;  // m/s
    }

    //IMU
    unsigned long currentMillisi = millis();  //IMU Update rate
    if (currentMillisi - previousMillisi >= intervali) {
          previousMillisi = currentMillisi;
           
          //sh2_SensorValue_t sensorValue;
          if (bno08xNew.getSensorEvent(&sensorValue)) {
                 IMUCalibrationstatus=sensorValue.status;
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

                if (sensorValue.sensorId ==SH2_ARVR_STABILIZED_RV) {
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
                  psi_dot_i = ((sin(roll_q) / cos(pitch_q)) * gyro_y + (cos(roll_q) / cos(pitch_q)) * gyro_z)* 180.0 / PI;
     
                }
          }
    }

////////////////////////////////////////////////////////////////////////////////////////Real time
   
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

            // Store IMU data with lag
          imuBuffer[imuBufferIndex] = {IMUCalibrationstatus, yaw_game, psi_dot_i, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, yaw_q, pitch_q, roll_q, Timestamp};
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
              yaw_modified_phi_VTG_filtered -= laggedIMU.gyro_z * deltaT*100; 
            } else {
              yaw_modified_phi_VTG_filtered = modified_phi_VTG;
            }

            prev_modified_phi_VTG = modified_phi_VTG; // Update for next iteration

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
          } else if ((yaw_modified_phi_VTG_filtered- Adjusted_Orientation) < -50) {
            slip_angle_raw = yaw_modified_phi_VTG_filtered - Adjusted_Orientation + 360;
          } else {
            slip_angle_raw = yaw_modified_phi_VTG_filtered - Adjusted_Orientation;
          }

          // Kalman filter for slip angle

          // Prediction step
          float x_pred = slip_angle_raw; // Predicted state 
          float P_pred = P + Q; // Predicted estimation error covariance

          // Update step
          float K = P_pred / (P_pred + R); // Kalman gain
          float x_est = x_pred + K * (slip_angle_raw- x_pred); // Updated state estimate
          P = (1 - K) * P_pred; // Updated estimation error covariance
          slip_angle_filtered = x_est; // Store filtered value
          
          float v_pred = Velocity_filtered;      
          float P_pred_v = P_v + Q_v;     


          float K_v = P_pred_v / (P_pred_v + R_v);                    
          Velocity_filtered = v_pred + K_v * (v - v_pred);              
          P_v = (1 - K_v) * P_pred_v; 

      
          Serial.print(Timestamp);
          Serial.print(",");
          Serial.print(IMUCalibrationstatus);  //  0,1,2,3
          Serial.print(",");
          Serial.print(fixQualityf); //0,1,2,3,4
          Serial.print(",");
          Serial.print(headingCorrected); //0,1
          Serial.print(",");
          Serial.print(longitudef, 8);
          Serial.print(",");
          Serial.print(latitudef, 8);  
          Serial.print(",");
          Serial.print(psi, 4);  // RTK degrees
          Serial.print(",");
          Serial.print(yaw_game, 4); // degrees
          Serial.print(",");
          Serial.print(phi, 4);  //VTG degrees
          Serial.print(",");
          Serial.print(v, 4);  //VTG velocity m/s   10
          Serial.print(",");  
          Serial.print(psi_dot_i, 4); //from pitch and roll // degrees 
          Serial.print(",");
          Serial.print(accel_x, 4); // m/s2
          Serial.print(",");
          Serial.print(accel_y, 4); // m/s2
          Serial.print(",");
          Serial.print(accel_z, 4); // m/s2
          Serial.print(",");
          Serial.print(gyro_x * RAD_TO_DEG, 4); // degrees/s
          Serial.print(",");
          Serial.print(gyro_y * RAD_TO_DEG, 4); // degrees/s
          Serial.print(",");
          Serial.print(gyro_z * RAD_TO_DEG, 4); // degrees/s 17
          Serial.print(",");
          Serial.print(yaw_q * RAD_TO_DEG, 4); // degrees
          Serial.print(",");
          Serial.print(pitch_q * RAD_TO_DEG, 4); // degrees
          Serial.print(",");
          Serial.print(roll_q * RAD_TO_DEG, 4); // degrees   20
          Serial.print(",");
          Serial.print(Adjusted_Orientation, 4); //Adjusted_RTK
          Serial.print(",");
          Serial.print(yaw_modified_phi_VTG_filtered, 4); //yaw_modified_phi_VTG
          Serial.print(",");
          Serial.print(slip_angle_raw, 4); //slip_angle_raw
          Serial.print(",");
          Serial.println(slip_angle_filtered, 4); //Slip Angle





          //scaling and data type conversion for CAN messages
          int phi_int=(phi)*10000;
          int gyro_z_deg_int = (gyro_z * RAD_TO_DEG)*10000;
          int v_int=(v)*10000;
          int psi_int=(psi)*10000;
          int yaw_game_int=(yaw_game)*10000;
          int accel_x_int=(accel_x)*1000;
          int psi_dot_int=(psi_dot_i)*10000;
          int accel_y_int=(accel_y)*1000;
          int accel_z_int=(accel_z)*1000;
          int gyro_x_deg_int = (gyro_x * RAD_TO_DEG)*10000;
          int gyro_y_deg_int = (gyro_y * RAD_TO_DEG)*10000;
          int yaw_q_deg_int = (yaw_q * RAD_TO_DEG)*10000;
          int longitude_int=(longitudef)*10000000;
          int latitude_int=(latitudef)*10000000;
          int slip_angle_filtered_int=(slip_angle_filtered)*10000;
          int slip_angle_raw_int=(slip_angle_raw)*10000;
          int yaw_modified_phi_VTG_filtered_int=(yaw_modified_phi_VTG_filtered)*10000;
          int Adjusted_Orientation_int=(Adjusted_Orientation)*10000;
          int Velocity_filtered_int=(Velocity_filtered)*10000;


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


          //Serial.print(yaw_game);
          //Serial.print(" ");
          //Serial.println(yaw_game_int);

          //Serial.print(Msg3.buf[4]);
          //Serial.print(" ");
          //Serial.print(Msg3.buf[5]);
          //Serial.print(" ");
          //Serial.print(Msg3.buf[6]);
          //Serial.print(" ");
          //Serial.println(Msg3.buf[7]);





    }



  // VS Loop End###################################################################################################################################################
}

//End Loop
//**************************************************************************


bool calcChecksum() {
  CK_A = 0;
  CK_B = 0;

  for (int i = 2; i < 70; i++) {
    CK_A = CK_A + ackPacket[i];
    CK_B = CK_B + CK_A;
  }

  return (CK_A == ackPacket[70] && CK_B == ackPacket[71]);
}

//Given a message, calc and store the two byte "8-Bit Fletcher" checksum over the entirety of the message
//This is called before we send a command message
void calcChecksum(ubxPacket* msg) {
  msg->checksumA = 0;
  msg->checksumB = 0;

  msg->checksumA += msg->cls;
  msg->checksumB += msg->checksumA;

  msg->checksumA += msg->id;
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len & 0xFF);
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len >> 8);
  msg->checksumB += msg->checksumA;

  for (uint16_t i = 0; i < msg->len; i++) {
    msg->checksumA += msg->payload[i];
    msg->checksumB += msg->checksumA;
  }
}

// VS Function Start###################################################################################################################################################



float convertToDecimalDegrees(float degMin) {
  int degrees = int(degMin / 100);                    // Extract the degrees part
  float minutes = degMin - (degrees * 100);           // Extract the minutes part
  float decimalDegrees = degrees + (minutes / 60.0);  // Convert minutes to decimal
  return decimalDegrees;
}





// VS Function End###################################################################################################################################################
