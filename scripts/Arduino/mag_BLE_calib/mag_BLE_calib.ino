/*

  created 24 Jul 2023
  by Yahya Tawil
*/


#include <ArduinoBLE.h>
#include "BMI270_AUX_BMM150.h"
#include "math.h"
#include "Wire.h"

// pins from data ready interrupt (IMU and Mag)
#define IMU_INT_PIN 25
#define MAG_INT_PIN 26
/**********************/



// Macros define program behaviour (disagned to enable one at once)
// print the IMU data in the format needed for calibration software tool
#define MOTION_CAL_SOFT 0



/**********************/

// Mag. calibration
// Hard ironing
//B
#define B1 -3.48
#define B2 0.42
#define B3 -5.78

//Soft ironing
//H column1
#define H11 1.036
#define H21 -0.023
#define H31 0.009
//H column2
#define H12 -0.023
#define H22 1.010
#define H32 -0.008
//H column3
#define H13 0.009
#define H23 -0.008
#define H33 0.956
/**********************/

// Global variables

// form the calibration arrayes
float B[3] = {B1, B2, B3}; // soft
float H[3][3] = {{H11, H12, H13}, {H21, H22, H23}, {H31, H32, H33}}; // hard

bool ImuRdy;
bool MagRdy;

float mag_x;
float mag_y;
float mag_z;

/* Create an instance of sensor data structure */
struct bmi2_sens_data sensor_data = { { 0 } };
struct bmm150_mag_data mag_data;

const char* deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* deviceServiceMovCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";
const char* deviceServiceMagCharacteristicUuid = "19b10002-e8f2-537e-4f6c-d104768a1214";
const char* deviceServiceCharacteristicUuid = "19b10003-e8f2-537e-4f6c-d104768a1214";


BLEService DataService(deviceServiceUuid);
BLEStringCharacteristic SettingsCharacteristic(deviceServiceCharacteristicUuid, BLERead | BLEWrite, 50);
BLECharacteristic MovCharacteristic(deviceServiceMovCharacteristicUuid, BLERead | BLENotify , 12 );
BLECharacteristic MagCharacteristic(deviceServiceMagCharacteristicUuid, BLERead | BLENotify , 6 );

void setupBLE() {
  if (!BLE.begin()) {

    Serial.println("- Starting Bluetooth® Low Energy module failed!");

    while (1);

  }
  BLE.setLocalName("Mag Cal");

  BLE.setAdvertisedService(deviceServiceUuid);
  DataService.addCharacteristic(SettingsCharacteristic);
  DataService.addCharacteristic(MovCharacteristic);
  DataService.addCharacteristic(MagCharacteristic);
  BLE.addService(DataService);

  BLE.advertise();
  Serial.println("Starting Bluetooth® Low Energy module");
  Serial.println("Mag Cal");
}

void SendBLEData() {

  //Serial.println("- Discovering central device...");

  //  delay(500);
  BLEDevice central = BLE.central();

  if (central) {

    //    Serial.println("* Connected to central device!");

    //    Serial.print("* Device MAC address: ");

    //Serial.println(central.address());

    //Serial.println(" ");

    uint8_t data[12];

    if (central.connected()) {

      memcpy(data, (int8_t *) &sensor_data.acc, sizeof(data) / 2 );
      memcpy(data + 6, (int8_t *) &sensor_data.gyr, sizeof(data) / 2 );
      // 2 ax 2 ay 2 az, 2 gx, 2 gy, 2 gz ,
      MovCharacteristic.writeValue(data, sizeof(data));

      struct bmm150_mag_data ble_mag_data;
      ble_mag_data.x = mag_data.x * 10;
      ble_mag_data.y = mag_data.y * 10;
      ble_mag_data.z = mag_data.z * 10;

      memcpy(data, (int8_t *) &ble_mag_data, sizeof(data) / 2);
      MagCharacteristic.writeValue(data, sizeof(data) / 2);

if (SettingsCharacteristic.written()) {
        String settings = SettingsCharacteristic.value();
        Serial.println(settings);
       // (HardI:H11,H21,H31) .i.e:(HardI:47.63,15.11,-6.80)
        if (settings.indexOf("HardI:") != -1)
        {
          int comma_position = -1;
          int old_comma_position = 5;
          float stored_HardIron [3];
          for (int comma_i = 0; comma_i < 3; comma_i++)
          {
            comma_position = settings.indexOf(',', comma_position + 1);
            String HardIron ;
            if (comma_i == 2)
            {
              HardIron = settings.substring(old_comma_position + 1, settings.length());
            }
            else
            {
              HardIron = settings.substring(old_comma_position + 1, comma_position);
            }

            old_comma_position = comma_position;
            stored_HardIron[comma_i] = HardIron.toFloat();
            B[comma_i] = HardIron.toFloat();
            Serial.println(HardIron.toFloat());
          }
        }
        // (SoftI:S11,S12,S13,S21,S22,S23,S31,S32,S33) .i.e:(SoftI:0.971,0.051,0.005,0.051,0.005,0.021,1.043)
        else if (settings.indexOf("SoftI:") != -1)
        {
          int comma_position = -1;
          int old_comma_position = 5;
          float stored_SoftIron [3][3];
          String SoftIron ;
          for (int comma_j = 0; comma_j < 3; comma_j++)
          {
            for (int comma_i = 0; comma_i < 3; comma_i++)
            {
              comma_position = settings.indexOf(',', comma_position + 1);

              if (comma_i == 2 && comma_j == 2) {
                SoftIron = settings.substring(old_comma_position + 1, settings.length());
              }
              else
              {
                SoftIron = settings.substring(old_comma_position + 1, comma_position );
              }
              H[comma_j][comma_i] = SoftIron.toFloat();
              Serial.println(SoftIron);
              old_comma_position = comma_position;
              stored_SoftIron[comma_j][comma_i] = SoftIron.toFloat();
              Serial.println(SoftIron.toFloat());
            }
          }
        }

      }
      
    }
  }
}


void setup() {

  Serial.begin(115200);
  while (!Serial);

  setupBLE();

  Serial.println("Started");

  IMU.debug(Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  /* To initialize the hal function */
  Wire.begin();

  pinMode(MAG_INT_PIN, INPUT);
  pinMode(IMU_INT_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), _onImuDrdy, RISING);
  attachInterrupt(digitalPinToInterrupt(MAG_INT_PIN), _onAuxDrdy, RISING);



}

void _onImuDrdy() {
  ImuRdy = true;
}

void _onAuxDrdy() {
  MagRdy = true;
}




void print_log()
{
  Serial.print("Raw:");
  Serial.print(sensor_data.acc.x);
  Serial.print(',');
  Serial.print(sensor_data.acc.y);
  Serial.print(',');
  Serial.print(sensor_data.acc.z);
  Serial.print(',');

  Serial.print(sensor_data.gyr.x);
  Serial.print(',');
  Serial.print(sensor_data.gyr.y);
  Serial.print(',');
  Serial.print(sensor_data.gyr.z);
  Serial.print(',');

  Serial.print(mag_data.x * 10);
  Serial.print(',');
  Serial.print(mag_data.y * 10);
  Serial.print(',');
  Serial.println(mag_data.z * 10);

}

void loop() {

  if (ImuRdy == true) // IMU dataready interupt flag
  {
    ImuRdy = false;
    IMU.readGyroAccel(sensor_data, true);
  }

  if (MagRdy == true) // mag. dataready interupt flag
  {
    // this make sure it is synced
    
    MagRdy = false;
    IMU.readAuxMag(mag_data);
    SendBLEData();
    print_log();

  }
  //delay(100);

}
