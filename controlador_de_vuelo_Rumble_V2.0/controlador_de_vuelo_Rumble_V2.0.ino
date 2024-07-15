
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_I2CDevice.h>
#include <esp_now.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "freertos/timers.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define configCHECK_FOR_STACK_OVERFLOW 1

#define I2C_FREQ 400000
#define SDA_1 21
#define SCL_1 22

TwoWire I2C_1 = TwoWire(0);

#define NUM_SAMPLES 20 // número de muestras para la media móvil

float samples[NUM_SAMPLES]; // array circular para almacenar las muestras
int sampleIndex = 0;        // índice actual en el array circular
float runningTotal = 0;     // suma actual de las muestras
float input;
float movingAverage;

float sAccx[NUM_SAMPLES]; // array circular para almacenar las muestras
int sIAccx = 0;           // índice actual en el array circular
float rTAccx = 0;         // suma actual de las muestras
float iAccx;
float mAAccx;

float sAccy[NUM_SAMPLES]; // array circular para almacenar las muestras
int sIAccy = 0;           // índice actual en el array circular
float rTAccy = 0;         // suma actual de las muestras
float iAccy;
float mAAccy;

float sAccz[NUM_SAMPLES]; // array circular para almacenar las muestras
int sIAccz = 0;           // índice actual en el array circular
float rTAccz = 0;         // suma actual de las muestras
float iAccz;
float mAAccz;

unsigned char countx, county;

#define MPU 0x68
#define A_R 4096.0 // 32768/8
#define G_R 65.5   // 32768/500

#define MS5611_address 0x77

// Conversion de radianes a grados 180/PI
#define RAD_A_DEG = 57.295779

// MPU-6050 da los valores en enteros de 16 bits
// Valores RAW
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
int32_t acc_total_vector;
 float vel = 0.0f;

// Angulos

float Acc[3];
float Gy[3], GyI[3];
float Angle[3];
float ErrAngle[3];
float Accx[2], Accy[2];
float Velx[2], Vely[2];
float Posx[2], Posy[2];
float gravity[4];
float linear_acceleration[4];
float  lastBaroAlt;
double baroVel;

// Variables usadas para calcular intervalo de tiempo para PID
long tiempo_prev;
double dt;

// pines de PWM para control de motores
#define LF_PIN 25
#define LB_PIN 27
#define RF_PIN 33
#define RB_PIN 26

// datos recibidos del control para PID
int input_YAW;
int input_PITCH;
int input_ROLL;
int input_THROTTLE;

// Variables giroscopio
int gyro_error = 0; // variable usada para calcular el error del giroscopio una vez

float Gyro_raw_error_x = 0, Gyro_raw_error_y = 0, Gyro_raw_error_z = 0; // se guarda el error inicial para despues restarlo

// Variables del acelerometro
int acc_error = 0;
int alt_error = 0;                    // variable usada para calcular el error del acelerometro una vez
float rad_to_deg = 180 / 3.141592654; // radianes a grados
float deg_to_rad = 3.141592654 / 180; // grads a radianes

int16_t throttle;
float Acc_angle_error_x = 0, Acc_angle_error_y = 0, Acc_angle_error_z = 0; // Here we store the initial Acc data error
float Baro_error, Baro_measure;


float Total_angle_x, Total_angle_y, Total_angle_z;

///////////////////////////////CONSTANTES PID ROLL////////////////////
float roll_PID, pwm_L_F, pwm_L_B, pwm_R_F, pwm_R_B, roll_error, roll_previous_error;
float roll_pid_p = 0;
float roll_pid_i = 0;
float roll_pid_d = 0;

double roll_kp = 2.0;   // 3.5
double roll_ki = 0.003; // 0.003
double roll_kd = 2.5;   // 350.0
float roll_desired_angle = 0;
float Auto_roll;
//////////////////////////////CONSTANTES PID PITCH//////////////////////////
float pitch_PID, pitch_error, pitch_previous_error;
float pitch_pid_p = 0;
float pitch_pid_i = 0;
float pitch_pid_d = 0;

double pitch_kp = 2.0;   // 8.0
double pitch_ki = 0.003; // 0.035
double pitch_kd = 2.5;   // 2000.0
float pitch_desired_angle = 0;
float Auto_pitch;

///////////////////////////////CONSTANTES PID YAW///////////////////
float yaw_PID, yaw_error, yaw_previous_error;
float yaw_pid_p = 0;
float yaw_pid_i = 0;
float yaw_pid_d = 0;

double yaw_kp = 8.5;   // 8.5
double yaw_ki = 0.003; // 0.017
double yaw_kd = 0.0;   // 0.0
float yaw_desired_angle = 0;
float Auto_yaw;

float pid_error_temp;
float pid_p_gain_altitude = 2.0;           //Gain setting for the altitude P-controller (default = 1.4).
float pid_i_gain_altitude = 0.2;           //Gain setting for the altitude I-controller (default = 0.2).
float pid_d_gain_altitude = 2.5;          //Gain setting for the altitude D-controller (default = 0.75).
int pid_max_altitude = 300;   

uint8_t start;

uint8_t flight_mode;
uint8_t takeoff_detected, manual_altitude_change;
int16_t manual_throttle;
// Pressure variables.
float pid_error_gain_altitude, pid_throttle_gain_altitude;
uint16_t C[7];
uint8_t barometer_counter, temperature_counter, tempIndex;
int64_t OFF, OFF_C2, SENS, SENS_C1, P;
uint32_t raw_pressure, raw_temperature, temp[6], raw_temp;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altutude_hold_pressure, return_to_home_decrease;
int32_t dT, dT_C5;
// Altitude PID variables
float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude, pid_last_altitude_d_error;
uint8_t parachute_rotating_mem_location;
int32_t parachute_buffer[35], parachute_throttle;
float pressure_parachute_previous;
int32_t press_mem[50], press_med;
uint8_t pressIndex;
float pressure_rotating_mem_actual;

uint32_t tiempoAnterior;
const long intervalo = 4000; // Intervalo en milisegundos (1 segundo en este ejemplo)

uint8_t broadcastAddress[] = {0xC8, 0xF0, 0x9E, 0x79, 0x43, 0x7C}; // se debe obtener la direccion MAC del dispositivo receptor
// Adafruit_BMP085 bmp;
// 0x7C, 0x9E, 0xBD, 0x61, 0x4E, 0x88

// Extructura del dato a recibir
// esta estructura debe coincidir con el mensaje que se recibe
typedef struct struct_message
{
  float a;
  float b;
  float c;
  float d;
  float e;
} struct_message;

// BMP180 corresponde al mensaje que se enviar al control, myData corresponde al T,R,P,Y que se reciben del control.

struct_message BMP180;
struct_message myData;

esp_now_peer_info_t peerInfo;

// funcion de cuando se recibe un dato
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&myData, incomingData, sizeof(myData));

  // input_THROTTLE = myData.a;
  // roll_desired_angle = myData.b;
  // pitch_desired_angle = myData.c;
  // yaw_desired_angle = myData.d;
}
// funcion de cuando envia un dato
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
}

// configuracion para canales PWM
ledc_timer_config_t ledc_timer;
ledc_channel_config_t ledc_channel[4];

// dividir tareas, un nucleo balancea, y el otro mide
//  void mainProcess(void *params);
//  void measureData(void *params);

unsigned long lastTime;

void setup()
{

  Serial.begin(115200);

  // inicializar I2C
  I2C_1.begin(SDA_1, SCL_1, I2C_FREQ);
  // bool status= bmp.begin(BMP085_ULTRAHIGHRES,&I2C_1);  // inicializa sensor BMP Barometrico
  // if (!status) {
  //   Serial.println("Could not find a valid BME280 sensor, check wiring!");
  //  while (1);
  // }

  delay(1000); // tiempo mientras se inicializa

  // establecer el dispositivo como estacion WIFI
  WiFi.mode(WIFI_STA);

  // Inicializar ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  // Registrar peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Añadir peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  // configuracion de canales PWM del ESP32
  ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
  ledc_timer.duty_resolution = LEDC_TIMER_10_BIT;
  ledc_timer.bit_num = LEDC_TIMER_10_BIT;
  ledc_timer.timer_num = LEDC_TIMER_1;
  ledc_timer.freq_hz = 10000;
  ledc_timer.clk_cfg = LEDC_AUTO_CLK;

  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  ledc_channel[0].channel = LEDC_CHANNEL_0;
  ledc_channel[0].gpio_num = LF_PIN;

  ledc_channel[1].channel = LEDC_CHANNEL_1;
  ledc_channel[1].gpio_num = LB_PIN;

  ledc_channel[2].channel = LEDC_CHANNEL_2;
  ledc_channel[2].gpio_num = RF_PIN;

  ledc_channel[3].channel = LEDC_CHANNEL_3;
  ledc_channel[3].gpio_num = RB_PIN;
  for (int i = 0; i < 4; i++)
  {
    ledc_channel[i].speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_channel[i].timer_sel = LEDC_TIMER_1;
    ledc_channel[i].intr_type = LEDC_INTR_DISABLE;
    ledc_channel[i].duty = 0;
    ledc_channel[i].hpoint = 0;

    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel[i]));
  }

  // configurar e inicializar MPU-6050
  I2C_1.begin(); // D2(GPIO4)=SDA / D1(GPIO5)=SCL
  I2C_1.beginTransmission(MPU);
  I2C_1.write(0x6B);
  I2C_1.write(0);
  I2C_1.endTransmission(true);

  I2C_1.beginTransmission(0x68); // begin, Send the slave adress (in this case 68)
  I2C_1.write(0x1B);             // We want to write to the GYRO_CONFIG register (1B hex)
  I2C_1.write(0x08);             // Set the register bits as 00010000 (100dps full scale)
  I2C_1.endTransmission(true);   // End the transmission with the gyro  0x08

  I2C_1.beginTransmission(0x68); // Start communication with the address found during search.
  I2C_1.write(0x1C);             // We want to write to the ACCEL_CONFIG register (1A hex)
  I2C_1.write(0x10);             // Set the register bits as 00010000 (+/- 8g full scale range)   0x10
  I2C_1.endTransmission(true);

  I2C_1.beginTransmission(0x68);
  I2C_1.write(0x1A); // We want to write to the CONFIG register (1A hex)
  I2C_1.write(0x03); // Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  I2C_1.endTransmission(true);

  // for (start = 1; start <= 6; start++)
  // {
  //   I2C_1.beginTransmission(MS5611_address); // Start communication with the MPU-6050.
  //   I2C_1.write(0xA0 + start * 2);           // Send the address that we want to read.
  //   I2C_1.endTransmission();                 // End the transmission.

  //   I2C_1.requestFrom(MS5611_address, 2);        // Request 2 bytes from the MS5611.
  //   C[start] = I2C_1.read() << 8 | I2C_1.read(); // Add the low and high byte to the C[x] calibration variable.
  // }

  // OFF_C2 = C[2] * pow(2, 16);  // This value is pre-calculated to offload the main program loop.
  // SENS_C1 = C[1] * pow(2, 15); // This value is pre-calculated to offload the main program loop.

  // // The MS5611 needs a few readings to stabilize.
  // for (start = 0; start < 100; start++)
  // {                   // This loop runs 100 times.
  //   read_barometer(); // Read and calculate the barometer data.
  //   delay(2);         // The main program loop also runs 500Hz (2ms per loop).
  // }
  // actual_pressure = 0;

  ///////////////////////////////////////////////////////////
  if (gyro_error == 0)
  {
    for (int i = 0; i < 2000; i++)
    {
      I2C_1.beginTransmission(MPU); // begin, Send the slave adress (in this case 68)
      I2C_1.write(0x43);            // First adress of the Gyro data
      I2C_1.endTransmission(false);
      I2C_1.requestFrom(MPU, 6, true);        // We ask for just 4 registers
      GyX = I2C_1.read() << 8 | I2C_1.read(); // Cada valor ocupa 2 registros
      GyY = I2C_1.read() << 8 | I2C_1.read();
      GyZ = I2C_1.read() << 8 | I2C_1.read();

      /*---X---*/

      Gyro_raw_error_x = Gyro_raw_error_x + GyX;
      /*---Y---*/
      Gyro_raw_error_y = Gyro_raw_error_y + GyY;

      Gyro_raw_error_z = Gyro_raw_error_z + GyZ;
      if (i == 1999)
      {
        Gyro_raw_error_x = Gyro_raw_error_x / 2000;
        Gyro_raw_error_y = Gyro_raw_error_y / 2000;
        Gyro_raw_error_z = Gyro_raw_error_z / 2000;
        gyro_error = 1;
      }
    }
  } // end of gyro error calculation

  if (acc_error == 0)
  {
    for (int a = 0; a < 2000; a++)
    {
      I2C_1.beginTransmission(MPU);
      I2C_1.write(0x3B); // Ask for the 0x3B register- correspond to AcX
      I2C_1.endTransmission(false);
      I2C_1.requestFrom(MPU, 6, true);
      AcX = I2C_1.read() << 8 | I2C_1.read(); // Cada valor ocupa 2 registros
      AcY = I2C_1.read() << 8 | I2C_1.read();
      AcZ = I2C_1.read() << 8 | I2C_1.read();

      Acc_angle_error_x = Acc_angle_error_x + atan(AcY / sqrt(AcX * AcX + AcZ * AcZ)) * rad_to_deg;

      Acc_angle_error_y = Acc_angle_error_y + atan(-1 * AcX / sqrt(AcY * AcY + AcZ * AcZ)) * rad_to_deg;

      if (a == 1999)
      {
        Acc_angle_error_x = Acc_angle_error_x / 2000;
        Acc_angle_error_y = Acc_angle_error_y / 2000;
        acc_error = 1;
      }
    }
  } // end of acc error calculation

  // se declara las tareas a realizar por nucleo, indicando prioridad mayor al balanceo del dron y de segundo el medir datos

  //     xTaskCreatePinnedToCore(
  //   mainProcess
  //  ,  "MainProcess"   // A name just for humans
  //   ,  10000  // This stack size can be checked & adjusted by reading the Stack Highwater
  //  ,  NULL
  //  ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
  //   ,  NULL
  //   ,  0);

  //  xTaskCreatePinnedToCore(
  //   measureData
  //   ,  "MeasureData Process"   // A name just for humans
  //   ,  4000  // This stack size can be checked & adjusted by reading the Stack Highwater
  //   ,  NULL
  //   ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
  //   ,  NULL
  //   ,  1);

  tiempoAnterior = micros();
}
void loop()
{

  I2C_1.beginTransmission(MPU);
  I2C_1.write(0x3B); // Pedir el registro 0x3B - corresponde al AcX
  I2C_1.endTransmission(false);
  I2C_1.requestFrom(MPU, 6, true);        // A partir del 0x3B, se piden 6 registros
  AcX = I2C_1.read() << 8 | I2C_1.read(); // Cada valor ocupa 2 registros
  AcY = I2C_1.read() << 8 | I2C_1.read();
  AcZ = I2C_1.read() << 8 | I2C_1.read();
  //pitch
  Acc[0] = atan(AcY / sqrt(AcX * AcX + AcZ * AcZ)) * rad_to_deg - Acc_angle_error_x;
  //roll
  Acc[1] = atan(-1 * AcX / sqrt(AcY * AcY + AcZ * AcZ)) * rad_to_deg - Acc_angle_error_y;


  linear_acceleration[0] = AcX - gravity[0];
  linear_acceleration[1] = AcY - gravity[1];
  linear_acceleration[2] = AcZ - gravity[2];

  gravity[0] = gravity[0] + (0.004) * linear_acceleration[0];
  gravity[1] = gravity[1] + (0.004) * linear_acceleration[1];
  gravity[2] = gravity[2] + (0.004) * linear_acceleration[2];

  I2C_1.beginTransmission(MPU);
  I2C_1.write(0x43);
  I2C_1.endTransmission(false);
  I2C_1.requestFrom(MPU, 6, true);        // A partir del 0x43, se piden 6 registros
  GyX = I2C_1.read() << 8 | I2C_1.read(); // Cada valor ocupa 2 registros
  GyY = I2C_1.read() << 8 | I2C_1.read();
  GyZ = I2C_1.read() << 8 | I2C_1.read();
 
  Gy[0] = ((GyX - Gyro_raw_error_x) / G_R);
  Gy[1] = ((GyY - Gyro_raw_error_y) / G_R);
  Gy[2] = ((GyZ - Gyro_raw_error_z) / G_R);

  GyI[0] = (GyI[0] * 0.7) + (Gy[0] * 0.3);
  GyI[1] = (GyI[1] * 0.7) + (Gy[1] * 0.3);
  GyI[2] = (GyI[2] * 0.7) + (Gy[2] * 0.3);
  dt = (double)(micros() - tiempo_prev) / 1000000.0;
  tiempo_prev = micros();
  
  Angle[0] = Angle[0] + Gy[0] * dt;
  Angle[1] = Angle[1] + Gy[1] * dt;

  Angle[0] -= Angle[1] * sin((float)Gy[2] * dt * deg_to_rad);
  Angle[1] += Angle[0] * sin((float)Gy[2] * dt * deg_to_rad);

  ErrAngle[0] = Acc[0] - Angle[0];
  ErrAngle[1] = Acc[1] - Angle[1];

  Angle[0] = Angle[0] + 0.0004 * ErrAngle[0];
  Angle[1] = Angle[1] + 0.0004 * ErrAngle[1];
  Angle[2] = Gy[2];

  // iAccx = linear_acceleration[0];

  // rTAccx -= sAccx[sIAccx];
  // sAccx[sIAccx] = iAccx;
  // rTAccx += iAccx;
  // sIAccx = (sIAccx + 1) % NUM_SAMPLES;
  // mAAccx = (float)rTAccx / 20.0;

  // iAccy = linear_acceleration[1];

  // rTAccy -= sAccy[sIAccy];
  // sAccy[sIAccy] = iAccy;
  // rTAccy += iAccy;
  // sIAccy = (sIAccy + 1) % NUM_SAMPLES;
  // mAAccy = (float)rTAccy / 20.0;

  // iAccz = linear_acceleration[2];

  // rTAccz -= sAccz[sIAccz];
  // sAccz[sIAccz] = iAccz;
  // rTAccz += iAccz;
  // sIAccz = (sIAccz + 1) % NUM_SAMPLES;
  // mAAccz = (float)rTAccz / 20.0;

  // acc_total_vector = -sin(Angle[0]*deg_to_rad)*AcX + cos(Angle[0]*deg_to_rad)*sin(Angle[1]*deg_to_rad)*AcY + cos(Angle[1]*deg_to_rad)*cos(Angle[0]*deg_to_rad)*AcZ;
  // linear_acceleration[3] = acc_total_vector - gravity[3];
  // gravity[3] = gravity[3] + (0.004) * linear_acceleration[3];
  // vel += linear_acceleration[3] * 9.81 * 100 * dt;
  // vel = vel * 0.985f + baroVel * 0.015f;
 

  // Accx[1]= mAAccx * cos(Acc[1]*deg_to_rad) + mAAccz * sin(Acc[1]*deg_to_rad);
  // Accy[1]= - mAAccy * sin(Acc[0]*deg_to_rad) + mAAccz * cos(Acc[0]*deg_to_rad);

  // Serial.println (String(mAAccx * cos(Acc[0]*deg_to_rad))+" "+String(mAAccz * sin(Acc[0]*deg_to_rad)));

  // if ((Accx[1] <=265)&&(Accx[1] >= -265))
  //  {Accx[1] = 0;}

  //  if ((Accy[1] <=265)&&(Accy[1] >= -265))
  //  {Accy[1] = 0;}

  // Velx[1]= Velx[0]+ Accx[0]+ ((Accx[1] - Accx[0])/2);

  // Posx[1]= Posx[0] + Velx[0] + ((Velx[1] - Velx[0])/2);

  // Vely[1] = Vely[0] + Accy[0] + ((Accy[1] - Accy[0])/2);

  // Posy[1] = Posy[0] + Vely[0] + ((Vely[1] - Vely[0])/2);

  // Serial.println (String(Velx[1])+" "+String(Vely[1]));

  // Accx[0] = Accx[1];
  // Accy[0] = Accy[1];
  // Velx[0] = Velx[1];
  // Vely[0] = Vely[1];
  // movement_end_check();
  // Posx[0] = Posx[1];
  // Posy[0] = Posy[1];

  roll_desired_angle = 0;
  if (myData.b > 1508)
    roll_desired_angle = myData.b - 1508;
  else if (myData.b < 1492)
    roll_desired_angle = myData.b - 1492;
  roll_desired_angle -= Angle[1] * 16.4;
  roll_desired_angle /= 3.0;

  pitch_desired_angle = 0;
  if (myData.c > 1508)
    pitch_desired_angle = myData.c - 1508;
  else if (myData.c < 1492)
    pitch_desired_angle = myData.c - 1492;
  pitch_desired_angle -= Angle[0] * 16.4;
  pitch_desired_angle /= 3.0;

  yaw_desired_angle = 0;
  if (myData.d > 1508)
    yaw_desired_angle = (myData.d - 1508) / 3.0;
  else if (myData.d < 1492)
    yaw_desired_angle = (myData.d - 1492) / 3.0;

  Total_angle_x = GyI[0];
  Total_angle_y = GyI[1];
  Total_angle_z = GyI[2];

 //  Serial.println (String(roll_desired_angle)+" "+String(Total_angle_y)+" "+String( yaw_desired_angle));

  roll_error = roll_desired_angle - Total_angle_y;
  roll_pid_p = roll_kp * roll_error;
  roll_pid_i = roll_pid_i + (roll_ki * roll_error);
  if (roll_pid_i >= 300)
    roll_pid_i = 300;
  else if (roll_pid_i <= -300)
    roll_pid_i = -300;
  roll_pid_d = roll_kd * (Total_angle_y - roll_previous_error);
  roll_PID = roll_pid_p + roll_pid_i - roll_pid_d;
  if (roll_PID < -300)
    roll_PID = -300;
  if (roll_PID > 300)
    roll_PID = 300;
  roll_previous_error = Total_angle_y;

  pitch_error = pitch_desired_angle - Total_angle_x;
  pitch_pid_p = pitch_kp * pitch_error;
  pitch_pid_i = pitch_pid_i + (pitch_ki * pitch_error);
  if (pitch_pid_i >= 300)
    pitch_pid_i = 300;
  else if (pitch_pid_i <= -300)
    pitch_pid_i = -300;
  pitch_pid_d = pitch_kd * (Total_angle_x - pitch_previous_error);
  pitch_PID = pitch_pid_p + pitch_pid_i - pitch_pid_d;
  if (pitch_PID < -300)
    pitch_PID = -300;
  if (pitch_PID > 300)
    pitch_PID = 300;
  pitch_previous_error = Total_angle_x;

  yaw_error = yaw_desired_angle - Total_angle_z;
  yaw_pid_p = yaw_kp * yaw_error;
  yaw_pid_i = yaw_pid_i + (yaw_ki * yaw_error);
  if (yaw_pid_i >= 300)
    yaw_pid_i = 300;
  else if (yaw_pid_i <= -300)
    yaw_pid_i = -300;
  yaw_pid_d = yaw_kd * (Total_angle_z - yaw_previous_error);
  yaw_PID = yaw_pid_p + yaw_pid_i - yaw_pid_d;
  if (yaw_PID < -300)
    yaw_PID = -300;
  if (yaw_PID > 300)
    yaw_PID = 300;
  yaw_previous_error = Total_angle_z;

  // read_barometer();
  flight_mode = myData.e;
  //Serial.println(myData.e);
  throttle =myData.a ;
  if (flight_mode >= 2) {                                                          //If altitude mode is active.
  throttle =  1500 +pid_output_altitude + manual_throttle;    //The base throttle is the receiver throttle channel + the detected take-off throttle + the PID controller output.
  }

  pwm_R_F = +throttle + roll_PID - pitch_PID + yaw_PID;
  pwm_R_B = +throttle + roll_PID + pitch_PID - yaw_PID;
  pwm_L_B = +throttle - roll_PID + pitch_PID + yaw_PID;
  pwm_L_F = +throttle - roll_PID - pitch_PID - yaw_PID;

 // Serial.println (String(throttle)+" "+String( pid_output_altitude)+" "+String( manual_throttle));

  // Right front
  if (pwm_R_F < 1100)
    pwm_R_F = 1100;
  if (pwm_R_F > 2000)
    pwm_R_F = 2000;
  uint32_t RF_duty = map(pwm_R_F, 1100, 2000, 0, 1024);

  if (pwm_L_F < 1100)
    pwm_L_F = 1100;
  if (pwm_L_F > 2000)
    pwm_L_F = 2000;
  uint32_t LF_duty = map(pwm_L_F, 1100, 2000, 0, 1024);

  if (pwm_R_B < 1100)
    pwm_R_B = 1100;
  if (pwm_R_B > 2000)
    pwm_R_B = 2000;
  uint32_t RB_duty = map(pwm_R_B, 1100, 2000, 0, 1024);

  if (pwm_L_B < 1100)
    pwm_L_B = 1100;
  if (pwm_L_B > 2000)
    pwm_L_B = 2000;
  uint32_t LB_duty = map(pwm_L_B, 1100, 2000, 0, 1024);

  if (myData.a < 1050)
  {
    LB_duty = 0;
    LF_duty = 0;
    RF_duty = 0;
    RB_duty = 0;
    reset_pid();
  }

  // una vez calculados los PWM de cada motor,se actualizan dichos valores en cada motor.

  ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, LF_duty));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0));
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, LB_duty));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1));
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, RF_duty));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2));
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3, RB_duty));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3));


  BMP180.a = actual_pressure;
  BMP180.b = 0;
  BMP180.c = 0;
  BMP180.d = 0;
  BMP180.e = 0;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&BMP180, sizeof(BMP180));

  while (micros() - tiempoAnterior < 2000)
    ;
  tiempoAnterior = micros();
}



// void mainProcess(void *params) {
//   for(;;){

//   }
// }

// void measureData(void *params) {
//   for (;;) {

//   }

// }


void read_barometer(void)
{
  barometer_counter++;

  if (barometer_counter == 2)
   {  
    //  Serial.println(1);
  //     Serial.println(micros());
    if (temperature_counter == 0)
    {
      
      // Get temperature data from MS-5611
      I2C_1.beginTransmission(MS5611_address);
      I2C_1.write(0x00);
      I2C_1.endTransmission();
      I2C_1.requestFrom(MS5611_address, 3);

      raw_temp -= temp[tempIndex];
      temp[tempIndex] = I2C_1.read() << 16 | I2C_1.read() << 8 | I2C_1.read();
      raw_temp += temp[tempIndex];
      tempIndex = (tempIndex + 1) % 5;
      raw_temperature = raw_temp / 5;
    }
    else
    {
      // Get pressure data from MS-5611
      I2C_1.beginTransmission(MS5611_address);
      I2C_1.write(0x00);
      I2C_1.endTransmission();
      I2C_1.requestFrom(MS5611_address, 3);
      raw_pressure = I2C_1.read() << 16 | I2C_1.read() << 8 | I2C_1.read();
    }

    temperature_counter++;
    if (temperature_counter == 20)
    {
      temperature_counter = 0;
      // Request temperature data
      I2C_1.beginTransmission(MS5611_address);
      I2C_1.write(0x58);
      I2C_1.endTransmission();
    }
    else
    {
      // Request pressure data
      I2C_1.beginTransmission(MS5611_address);
      I2C_1.write(0x48);
      I2C_1.endTransmission();
    }
  }
  if (barometer_counter == 4)
  {
  
    dT = C[5];
    dT <<= 8;
    dT *= -1;
    dT += raw_temperature;
    OFF = OFF_C2 + ((int64_t)dT * (int64_t)C[4]) / pow(2, 7);
    SENS = SENS_C1 + ((int64_t)dT * (int64_t)C[3]) / pow(2, 8);
    P = ((raw_pressure * SENS) / pow(2, 21) - OFF) / pow(2, 15);
    press_med -= press_mem[pressIndex];
    press_mem[pressIndex] = P;
    press_med += press_mem[pressIndex];
    pressIndex = (pressIndex + 1) % 20;
    actual_pressure_fast = (float)press_med / 20.0;
    actual_pressure_slow = actual_pressure_slow * (float)0.985 + actual_pressure_fast * (float)0.015;
    actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;
    if (actual_pressure_diff > 8)
      actual_pressure_diff = 8;
    if (actual_pressure_diff < -8)
      actual_pressure_diff = -8;

    if (actual_pressure_diff > 1 || actual_pressure_diff < -1)
      actual_pressure_slow -= actual_pressure_diff / 6.0;
    actual_pressure = actual_pressure_slow;
 
  }
  if (barometer_counter == 6)
  { 

    barometer_counter = 0;                                                                                    //Set the barometer counter to 0 for the next measurements.
    //In the following part a rotating buffer is used to calculate the long term change between the various pressure measurements.
    //This total value can be used to detect the direction (up/down) and speed of the quadcopter and functions as the D-controller of the total PID-controller.
    if (manual_altitude_change == 1)pressure_parachute_previous = actual_pressure * 10;                       //During manual altitude change the up/down detection is disabled.
    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];                                  //Subtract the current memory position to make room for the new value.
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;   //Calculate the new change between the actual pressure and the previous measurement.
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];                                  //Add the new value to the long term avarage value.
    pressure_parachute_previous = actual_pressure * 10;                                                       //Store the current measurement for the next loop.
    parachute_rotating_mem_location++;                                                                        //Increase the rotating memory location.
    if (parachute_rotating_mem_location == 30)parachute_rotating_mem_location = 0;                            //Start at 0 when the memory location 20 is reached.

    if (flight_mode >= 2 ) {                                                          //If the quadcopter is in altitude mode and flying.
      if (pid_altitude_setpoint == 0)pid_altitude_setpoint = actual_pressure;                                 //If not yet set, set the PID altitude setpoint.
      //When the throttle stick position is increased or decreased the altitude hold function is partially disabled. The manual_altitude_change variable
      //will indicate if the altitude of the quadcopter is changed by the pilot.
      manual_altitude_change = 0;                                                    //Preset the manual_altitude_change variable to 0.
      manual_throttle = 0;                                                           //Set the manual_throttle variable to 0.
      if (myData.a > 1600) {                                                        //If the throtttle is increased above 1600us (60%).
        manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
        pid_altitude_setpoint = actual_pressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manual_throttle = (myData.a - 1600)/2 ;                                   // /(3) To prevent very fast changes in hight limit the function of the throttle.
      }
      if (myData.a < 1400) {                                                        //If the throtttle is lowered below 1400us (40%).
        manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
        pid_altitude_setpoint = actual_pressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manual_throttle = (myData.a - 1400)/3;                                    // /(5) To prevent very fast changes in hight limit the function of the throttle.
      }

      //Calculate the PID output of the altitude hold.
      pid_altitude_input = actual_pressure;                                          //Set the setpoint (pid_altitude_input) of the PID-controller.
      pid_error_temp = pid_altitude_input - pid_altitude_setpoint;                   //Calculate the error between the setpoint and the actual pressure value.

      //To get better results the P-gain is increased when the error between the setpoint and the actual pressure value increases.
      //The variable pid_error_gain_altitude will be used to adjust the P-gain of the PID-controller.
      pid_error_gain_altitude = 0;                                                   //Set the pid_error_gain_altitude to 0.
      if (pid_error_temp > 10 || pid_error_temp < -10) {                             //If the error between the setpoint and the actual pressure is larger than 10 or smaller then -10.
        pid_error_gain_altitude = (abs(pid_error_temp) - 10) / 20.0;                 //The positive pid_error_gain_altitude variable is calculated based based on the error.
        if (pid_error_gain_altitude > 3)pid_error_gain_altitude = 3;                 //To prevent extreme P-gains it must be limited to 3.
      }

      //In the following section the I-output is calculated. It's an accumulation of errors over time.
      //The time factor is removed as the program loop runs at 250Hz.
      pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp;
      if (pid_i_mem_altitude > pid_max_altitude)pid_i_mem_altitude = pid_max_altitude;
      else if (pid_i_mem_altitude < pid_max_altitude * -1)pid_i_mem_altitude = pid_max_altitude * -1;
      //In the following line the PID-output is calculated.
      //P = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp.
      //I = pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp (see above).
      //D = pid_d_gain_altitude * parachute_throttle.
      pid_output_altitude = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp + pid_i_mem_altitude + pid_d_gain_altitude * parachute_throttle;
      //To prevent extreme PID-output the output must be limited.
      if (pid_output_altitude > pid_max_altitude)pid_output_altitude = pid_max_altitude;
      else if (pid_output_altitude < pid_max_altitude * -1)pid_output_altitude = pid_max_altitude * -1;
    }

    //If the altitude hold function is disabled some variables need to be reset to ensure a bumpless start when the altitude hold function is activated again.
    else if (flight_mode < 2 && pid_altitude_setpoint != 0) {                        //If the altitude hold mode is not set and the PID altitude setpoint is still set.
      pid_altitude_setpoint = 0;                                                     //Reset the PID altitude setpoint.
      pid_output_altitude = 0;                                                       //Reset the output of the PID controller.
      pid_i_mem_altitude = 0;                                                        //Reset the I-controller.
      manual_throttle = 0;                                                           //Set the manual_throttle variable to 0 .
      manual_altitude_change = 1;                                                    //Set the manual_altitude_change to 1.
    }

  }
}

void reset_pid(void)
{
  roll_previous_error = 0;
  pitch_previous_error = 0;
  yaw_previous_error = 0;
  roll_pid_i = 0;
  pitch_pid_i = 0;
  yaw_pid_i = 0;
  roll_error = 0;
  pitch_error = 0;
  yaw_error = 0;
  Auto_roll = 0;
  Auto_pitch = 0;
  Auto_yaw = 0;
}

void movement_end_check(void)
{
  if (Accx[1] == 0) // we count the number of acceleration samples that equals cero
  {
    countx++;
  }
  else
  {
    countx = 0;
  }

  if (countx >= 25) // if this number exceeds 25, we can assume that velocity is cero
  {
    Velx[1] = 0;
    Velx[0] = 0;
  }

  if (Accy[1] == 0) // we do the same for the Y axis
  {
    county++;
  }
  else
  {
    county = 0;
  }

  if (county >= 25)
  {
    Vely[1] = 0;
    Vely[0] = 0;
  }
}
