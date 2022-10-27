#include <ArduinoBLE.h>
#include <Arduino_LSM6DS3.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

#define BLE_UUID_ACCELEROMETER_SERVICE "1101"
#define BLE_UUID_ACCELEROMETER_X "2101"
#define BLE_UUID_ACCELEROMETER_Y "2102"
#define BLE_UUID_ACCELEROMETER_Z "2103"

BLEService accelerometerService(BLE_UUID_ACCELEROMETER_SERVICE);

BLEFloatCharacteristic accelerometerCharacteristicX(BLE_UUID_ACCELEROMETER_X, BLERead | BLENotify);
BLEFloatCharacteristic accelerometerCharacteristicY(BLE_UUID_ACCELEROMETER_Y, BLERead | BLENotify);
BLEFloatCharacteristic accelerometerCharacteristicZ(BLE_UUID_ACCELEROMETER_Z, BLERead | BLENotify);

unsigned long last_micros = 0;
float movement;

// variables for finding average gyro offset
float g_xoff, g_yoff, g_zoff;
int g_av_count = 0;

const float sample_rate = 1.0f / 104.0f;
const float g_cutoff = 0.0001;
const float deg_to_rad = PI / 180.0f;
const float movement_decel = 0.975f;
const float movement_k = 5.0f;
const float max_radians_per_cycle = 6.0f * sample_rate;

Matrix<3, 3> ROT_MAT_X;
Matrix<3, 3> ROT_MAT_Y;
Matrix<3, 3> ROT_MAT_Z;
Matrix<3, 3> CURRENT_ORIENTATION = {1, 0, 0, 0, 1, 0, 0, 0, 1};
Matrix<3> DOWN = {0, 0, 1};

Matrix<3> REAL_ACCEL;
Matrix<3> VELOCITY = {0, 0, 0};
Matrix<3> POSITION = {0, 0, 0};

void putCrossProductMatrix(Matrix<3, 3>& out, const Matrix<3>& vec) {
  out = {
    0, -vec(2), vec(1),
    vec(2), 0, -vec(0),
    -vec(1), vec(0), 0
  };
}

float dot(const Matrix<3>& a, const Matrix<3>& b) {
  return a(0)*b(0) + a(1)*b(1) + a(2)*b(2);
}

void putRotateAroundV(Matrix<3, 3>& out, const Matrix<3>& v, float radians) {
  putRotateAroundVCosSin(out, v, cos(radians), sin(radians));
}

// Rodrigues' rotation formula
// returns a rotation matrix that rotates around v by radians
// v must be a unit vector
void putRotateAroundVCosSin(Matrix<3, 3>& out, const Matrix<3>& v, float cos_ang, float sin_ang) {
  // K will hold the cross product matrix of v
  Matrix<3, 3> K;
  putCrossProductMatrix(K, v);

  // clear output
  out *= 0;

  // second term of equation, sin(theta) * K
  out += K;
  out *= sin_ang;

  // third term of equation, (1 - cos(theta)) * K^2
  K *= K;
  K *= 1 - cos_ang;
  out += K;

  // first term of equation, I
  out(0, 0) += 1;
  out(1, 1) += 1;
  out(2, 2) += 1;
}

// returns a rotation matrix that rotates A towards B. If amount = 1, it rotates A on to B
// a and b must be unit vectors
// 0 <= amount <= 1
void putRotateATowardsB(Matrix<3, 3>& out, const Matrix<3>& a, const Matrix<3>& b, float max_amount) {
  float cos_ang = dot(a, b);
  cos_ang = constrain(cos_ang, -1.0f, 1.0f);
  float ang = acos(cos_ang);

  ang = min(ang, max_amount);
  cos_ang = cos(ang);
  float sin_ang = sin(ang);

  // If parallel, do nothing
  if (sin_ang < 0.001f && cos_ang >= 0) {
    out = {
      1, 0, 0,
      0, 1, 0,
      0, 0, 1
    };
    return;
  }

  // If anti-parallel, invert
  if (sin_ang < 0.001f && cos_ang < 0) {
    Serial.println(F("\tFLIPPING TIME!"));
    out = {
      -1, 0, 0,
      0, -1, 0,
      0, 0, -1
    };
    return;
  }

  // u will hold vector normal to a and b
  putCrossProductMatrix(out, a);
  Matrix<3> u = out * b;
  u /= Norm(u);  

  putRotateAroundVCosSin(out, u, cos_ang, sin_ang);
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // initialize IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }

  Serial.print(F("Accelerometer sample rate = "));
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(F("Hz"));
  Serial.print(F("Gyroscope sample rate = "));
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(F("Hz"));

  // initialize BLE
  if (!BLE.begin()) {
    Serial.println(F("starting Bluetooth® Low Energy failed!"));

    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("Nano 33 IoT");
  BLE.setAdvertisedService(accelerometerService);

  // add the characteristic to the service
  accelerometerService.addCharacteristic(accelerometerCharacteristicX);
  accelerometerService.addCharacteristic(accelerometerCharacteristicY);
  accelerometerService.addCharacteristic(accelerometerCharacteristicZ);

  // add service
  BLE.addService(accelerometerService);

  // set the initial value for the characteristic:
  accelerometerCharacteristicX.writeValue(0);
  accelerometerCharacteristicY.writeValue(0);
  accelerometerCharacteristicZ.writeValue(0);

  // start advertising
  BLE.advertise();

  Serial.println(F("BLE LED Peripheral"));
}

void loop() {
  BLEDevice central = BLE.central();
  float ax, ay, az;
  float gx, gy, gz;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    float dt;
    if (last_micros == 0) {
      dt = sample_rate;
    } else {
      unsigned long delta_micros = micros() - last_micros;
      dt = ((float) delta_micros) / 1000000.0f;
    }
    last_micros = micros();

    gx *= dt * deg_to_rad;
    gy *= dt * deg_to_rad;
    gz *= dt * deg_to_rad;

    if (g_av_count < 500) {

      float r = ((float) g_av_count) / ((float) (g_av_count + 1));
      g_av_count++;

      g_xoff = g_xoff * r + gx / ((float) g_av_count);
      g_yoff = g_yoff * r + gy / ((float) g_av_count);
      g_zoff = g_zoff * r + gz / ((float) g_av_count);
      Serial.println("Averaging...");
      Serial.print(g_xoff * 1000);
      Serial.print(F("\t"));
      Serial.print(g_yoff * 1000);
      Serial.print(F("\t"));
      Serial.println(g_zoff * 1000);

      last_micros = micros();

    } else {

      Matrix<3> A = {ax, ay, az};

      gx -= g_xoff;
      gx = (abs(gx) < g_cutoff) ? 0 : gx;
      gy -= g_yoff;
      gy = (abs(gy) < g_cutoff) ? 0 : gy;
      gz -= g_zoff;
      gz = (abs(gz) < g_cutoff) ? 0 : gz;

      // Rotate orientation around current x (forward) axis
      putRotateAroundV(ROT_MAT_X, CURRENT_ORIENTATION.Submatrix<3, 1>(0, 0), gy);

      // Rotate orientation around current y (sideways) axis
      putRotateAroundV(ROT_MAT_Y, CURRENT_ORIENTATION.Submatrix<3, 1>(0, 1), gz);

      // Rotate orientation around current z (vertical) axis
      putRotateAroundV(ROT_MAT_Z, CURRENT_ORIENTATION.Submatrix<3, 1>(0, 2), gz);

      // not sure if order matters here
      // CURRENT_ORIENTATION = ROT_MAT_X * CURRENT_ORIENTATION;
      CURRENT_ORIENTATION = ROT_MAT_Z * (ROT_MAT_Y * (ROT_MAT_X * CURRENT_ORIENTATION));

      // stillness = e ^ (-k * movement)
      float mag_A = sqrt(dot(A, A));
      float current_movement = abs(mag_A - 1);
      movement = max(current_movement, movement * movement_decel);
      float max_amount = exp(-movement_k * movement) * max_radians_per_cycle;

      Matrix<3> imu_down = (~CURRENT_ORIENTATION) * DOWN;
      Serial << CURRENT_ORIENTATION * 100 << '\n';
      // Serial << imu_down * 100 << '\t' << A * 100 << '\n';

      // putRotateATowardsB(ROT_MAT, CURRENT_ORIENTATION.Submatrix<3, 1>(0, 2), A / mag_A, max_amount);
      // CURRENT_ORIENTATION = ROT_MAT * CURRENT_ORIENTATION;

      // subtract off the existing 1 G downwards from gravity
      REAL_ACCEL(0) = A(0) - CURRENT_ORIENTATION(0, 2);
      REAL_ACCEL(1) = A(1) - CURRENT_ORIENTATION(1, 2);
      REAL_ACCEL(2) = A(2) - CURRENT_ORIENTATION(2, 2);

      // change from IMU coordinate space to world space
      Matrix<3> world_accel = CURRENT_ORIENTATION * REAL_ACCEL;

      if (central) {
        accelerometerCharacteristicX.writeValue(world_accel(0));
        accelerometerCharacteristicY.writeValue(world_accel(1));
        accelerometerCharacteristicZ.writeValue(world_accel(2));
      }
    }
  }
}