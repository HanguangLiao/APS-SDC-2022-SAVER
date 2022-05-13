#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <math.h>
#include <MAX30105.h>
#include <spo2_algorithm.h>
// Mutex and Semaphore Definitions
SemaphoreHandle_t mutex_iic;
//--------------UART--Server--Task--Begin----------------------------------
QueueHandle_t queue_serial;
const int queue_serial_buffer_size = 128;
static void task_serial_server(void * param)
{
  char receive_buffer[queue_serial_buffer_size];

  Serial.begin(115200);
  while(1)
  {
    // If receives queued request to print on serial
    if(xQueueReceive(queue_serial,receive_buffer,128) == pdTRUE)
    {
      Serial.printf("%s\n", receive_buffer);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
//--------------UART--Server--Task--End-----------------------------------

//--------------BLE--Server--Task--Begin-----------------------------------
QueueHandle_t queue_ble_temperature,
 queue_ble_accel, 
 queue_ble_heartbeat, 
 queue_ble_spo2;
#define SERVICE_UUID "12a59900-17cc-11ec-9621-0242ac130002"
#define CHARACTERISTIC_UUID_SPO2 "12a5a148-17cc-11ec-9621-0242ac130002"
#define CHARACTERISTIC_UUID_TEMP "a93769a2-ace1-11ec-b909-0242ac120002"
#define CHARACTERISTIC_UUID_TXHB "d084d792-ace1-11ec-b909-0242ac120002"
#define CHARACTERISTIC_UUID_TXAcce "e6097a32-ace1-11ec-b909-0242ac120002"
class BLEServerCallback : public BLEServerCallbacks
{
  public :
    int is_device_connected = 0;
    int is_device_connected_last_time = 0;
    void onConnect(BLEServer *pServer)
      {
          is_device_connected = 1;
      };

      void onDisconnect(BLEServer *pServer)
      {
          is_device_connected = 0;
      }
};
static void task_ble_server(void * param)
{
  char serial_buffer[queue_serial_buffer_size];
  sprintf(serial_buffer,"BLE: Server Created");
  xQueueSend(queue_serial, serial_buffer, 128);
  BLEServerCallback ble_server_callback;
  BLEDevice::init("ESP32_BLE");
  // setup a BLE service
  BLEServer * ble_server = BLEDevice::createServer();
  ble_server->setCallbacks(&ble_server_callback);
  BLEService * ble_service = ble_server->createService(SERVICE_UUID);

  // setup BLE characteristics
  BLECharacteristic * ble_charcteristic_spo2 = ble_service->createCharacteristic(CHARACTERISTIC_UUID_SPO2, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ);
  BLECharacteristic * ble_charcteristic_temp = ble_service->createCharacteristic(CHARACTERISTIC_UUID_TEMP, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ);
  BLECharacteristic * ble_charcteristic_heartbeat = ble_service->createCharacteristic(CHARACTERISTIC_UUID_TXHB, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ);
  BLECharacteristic * ble_charcteristic_accel = ble_service->createCharacteristic(CHARACTERISTIC_UUID_TXAcce, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ);

  // Run BLE Service
  ble_service->start();              
  ble_server->getAdvertising()->start();
  sprintf(serial_buffer,"BLE: Waiting for Connection");
  xQueueSend(queue_serial, serial_buffer, 128);
  while(1)
  {
    if(!ble_server_callback.is_device_connected && ble_server_callback.is_device_connected_last_time)
    {
      // If device is disconnecting
      ble_server->startAdvertising();
      sprintf(serial_buffer,"BLE: Disconnected and Restart Broadcasting");
      xQueueSend(queue_serial, serial_buffer, 128);
      ble_server_callback.is_device_connected_last_time = ble_server_callback.is_device_connected;
    }
    else if (ble_server_callback.is_device_connected && !ble_server_callback.is_device_connected_last_time)
    {
      // If device is connected for the first time
      ble_server_callback.is_device_connected_last_time = 1;
      sprintf(serial_buffer,"BLE: Connected");
      xQueueSend(queue_serial, serial_buffer, 128);
    }

    if(ble_server_callback.is_device_connected)
    {
      float read_buffer;
      if(xQueueReceive(queue_ble_accel, &read_buffer, 0) == pdTRUE)
        ble_charcteristic_accel->setValue(read_buffer);

      if(xQueueReceive(queue_ble_spo2, &read_buffer, 0) == pdTRUE)
        ble_charcteristic_spo2->setValue(read_buffer);

      if(xQueueReceive(queue_ble_temperature, &read_buffer, 0) == pdTRUE)
        ble_charcteristic_temp->setValue(read_buffer);

      if(xQueueReceive(queue_ble_heartbeat, &read_buffer, 0) == pdTRUE)
        ble_charcteristic_heartbeat->setValue(read_buffer);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
//--------------BLE--Server--Task--End-------------------------------------

//--------------MAX30102-Sensor-Read-Task--Begin--------------------------------
QueueHandle_t queue_max30102_ir, queue_max30102_red;
static void task_max30102_read(void * param)
{
  MAX30105 max30105_sensor;
  
  char serial_buffer[queue_serial_buffer_size];
  
  float temperature = 0.0f;
  int temperature_wait_tick = 0;
  
  // Check if device is connected
  while(xSemaphoreTake(mutex_iic, 100) != pdTRUE);
  if(!max30105_sensor.begin(Wire, I2C_SPEED_FAST))
  {
    sprintf(serial_buffer, "MAX30105: Sensor was not found. Please check wiring/power");
    xQueueSend(queue_serial, serial_buffer, 0);
    // Quit this task if device is not found
    xSemaphoreGive(mutex_iic);
    vTaskDelete(NULL);
  }
  xSemaphoreGive(mutex_iic);

  // Initiate device
  int sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200, Sampling Rate is also limited by LED Pulse Width （Refer to Table 11)
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  while(xSemaphoreTake(mutex_iic, 100) != pdTRUE);
  max30105_sensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  xSemaphoreGive(mutex_iic);

  while(1)
  {
    // Read Temperature at a speed 101 times slower than red and ir sensor
    temperature_wait_tick += 1;
    if(temperature_wait_tick > 101)
    {
      temperature_wait_tick = 0;
      while(xSemaphoreTake(mutex_iic, 0) != pdTRUE);
      temperature = max30105_sensor.readTemperature();
      //Reset SPO2 Measurement
      max30105_sensor.clearFIFO();
      xSemaphoreGive(mutex_iic);
      xQueueSend(queue_ble_temperature, &temperature, 0);
      sprintf(serial_buffer, "MAX30102: Temperature = %.2f", temperature);
      xQueueSend(queue_serial, serial_buffer, 0);
    }

    // Read IR Value
    while(xSemaphoreTake(mutex_iic, 0) != pdTRUE) vTaskDelay(1);
    uint32_t IR = max30105_sensor.getIR();
    xSemaphoreGive(mutex_iic);
    while(IR == 0)
    {
      while(xSemaphoreTake(mutex_iic, 0) != pdTRUE) vTaskDelay(1);
      IR = max30105_sensor.getIR();
      xSemaphoreGive(mutex_iic);
    }
    xQueueSend(queue_max30102_ir, &IR, 0);

    // Read Red Value
    while(xSemaphoreTake(mutex_iic, 0) != pdTRUE) vTaskDelay(1);
    uint32_t Red =  max30105_sensor.getRed();
    xSemaphoreGive(mutex_iic);
    while(Red == 0)
    {
      while(xSemaphoreTake(mutex_iic, 0) != pdTRUE) vTaskDelay(1);
      Red =  max30105_sensor.getRed();
      xSemaphoreGive(mutex_iic);
    }
    xQueueSend(queue_max30102_red, &Red, 0);

    // Sampling Rate: 25Hz (4 averaging of 200Hz / 2 Channels)
  }
}
//--------------MAX30102-Sensor-Read-Task--End--------------------------------

//--------------Heartbeat-SpO2-Algorithm-Begin--------------------------------
static void task_heartbeat_spo2_algorithm(void * param)
{
  // The bufferLength must be matched with BUFFER_SIZE in spo2_algorithm.h
  const int bufferLength = 100; //Sample Rate is 25Hz, Buffer takes 1000ms / 25 * 100 = 4 seconds to be filled
  uint32_t * irBuffer = (uint32_t * )pvPortMalloc(bufferLength * sizeof(uint32_t)); //infrared LED sensor data
  uint32_t * redBuffer = (uint32_t * )pvPortMalloc(bufferLength * sizeof(uint32_t));  //red LED sensor data
  int32_t spo2; //SPO2 value
  int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
  int32_t heartRate; //heart rate value
  int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

  char serial_buffer[queue_serial_buffer_size];

  float spo2_stored = 100.0f;
  float spo2_float = 0.0f;
  const float spo2_limit = 10.0f;
  float heartRate_stored = 80.0f;
  float heartRate_float = 0.0f;
  const float heartRate_limit = 10.0f;

  if(irBuffer == NULL || redBuffer == NULL)
  {
    sprintf(serial_buffer, "Algorithm: Error, Cannot Create Heap for Sensor Data");
    xQueueSend(queue_serial, serial_buffer, 0);
    vTaskDelete(NULL);
  }

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 2 second
  while(1)
  {
    TickType_t start_tick = xTaskGetTickCount();
    // Reading takes 1000ms / 25 * 100 = 4 seconds
    for (int i = 0; i < bufferLength; i++)
    {
      while(xQueueReceive(queue_max30102_ir, &irBuffer[i], 0) != pdTRUE) vTaskDelay(1 / portTICK_PERIOD_MS);
      while(xQueueReceive(queue_max30102_red, &redBuffer[i], 0) != pdTRUE) vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    TickType_t count_tick = xTaskGetTickCount() - start_tick;
    sprintf(serial_buffer, "Algorithm: reading takes %d ticks", count_tick);
    xQueueSend(queue_serial, serial_buffer, 0);

    // Calculate Heart Rate and SpO2 Every 4 seconds
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    // Software Filtering (Maximum Slew Rate) to Reduce the Effect of Abrupt Change in Calculated HeartRate
    spo2_float = (float)spo2 * (float)validSPO2;
    if(spo2_float > spo2_stored + spo2_limit)
    {
      spo2_stored += spo2_limit;
    }
    else if(spo2_float < spo2_stored - spo2_limit)
    {
      spo2_stored -= spo2_limit;
    }
    else
    {
      spo2_stored = spo2_float;
    }
    
    heartRate_float = (float)heartRate * (float)validHeartRate;
    if(heartRate_float > heartRate_stored + heartRate_limit)
    {
      heartRate_stored += heartRate_limit;
    }
    else if(heartRate_float < heartRate_stored - heartRate_limit)
    {
      heartRate_stored -= heartRate_limit;
    }
    else
    {
      heartRate_stored = heartRate_float;
    }

    //Update Value to BLE and Serial Monitor
    float ble_heart_rate_buffer, ble_spo2_buffer;
    ble_heart_rate_buffer = heartRate_stored;
    xQueueSend(queue_ble_heartbeat, &ble_heart_rate_buffer, 0);
    ble_spo2_buffer = spo2_stored;
    xQueueSend(queue_ble_spo2, &ble_spo2_buffer, 0);

    sprintf(serial_buffer, "Algorithm: SpO2 = %.2f, HeartRate = %.2f", ble_spo2_buffer, ble_heart_rate_buffer);
    xQueueSend(queue_serial, serial_buffer, 0);

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  vPortFree(irBuffer);
  vPortFree(redBuffer);
}
//--------------Heartbeat-SpO2-Algorithm-End----------------------------------

//--------------MPU-Sensor-Read-Task--Begin--------------------------------
static void task_mpu_read(void * param)
{
  char serial_buffer[queue_serial_buffer_size];

  const u_char MPU6050_Addr = 0x68;
  const u_char ACCEL_CONFIG = 0x1C;
  const u_char PWR_MGMT_1 = 0x6B;
  const u_char ACCEL_XOUT = 0x3B;
  u_char XH,XL,YH,YL,ZH,ZL;
  int16_t ACCEL_X, ACCEL_Y, ACCEL_Z;

  // Turn on the MPU6050 Sensor
  while(xSemaphoreTake(mutex_iic, 10) != pdTRUE);
  Wire.begin();
	Wire.beginTransmission(MPU6050_Addr);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission(true);
  xSemaphoreGive(mutex_iic);

  // Set MPU6050 Accel Range
  while(xSemaphoreTake(mutex_iic, 10) != pdTRUE);
  Wire.beginTransmission(MPU6050_Addr);
  // AFS_SEL = 0b11, accel scale = 16g, Sensitivity = 2048LSB per g
  Wire.write(ACCEL_CONFIG);
  Wire.write((uint8_t)(0x00 | 0x03<<3));
  Wire.endTransmission(true);
  xSemaphoreGive(mutex_iic);

  sprintf(serial_buffer,"MPU6050: Sensor Initialized");
  xQueueSend(queue_serial, serial_buffer, 128);

  while(1)
  {
    while(xSemaphoreTake(mutex_iic, 10) != pdTRUE) vTaskDelay(pdMS_TO_TICKS(1));
    Wire.beginTransmission(MPU6050_Addr);
    Wire.write(ACCEL_XOUT);
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t)MPU6050_Addr, (uint8_t)6, (bool)true);
    // Reads the data from the register
    XH = Wire.read(); 
    XL = Wire.read(); 
    YH = Wire.read();  
    YL = Wire.read();
    ZH = Wire.read();
    ZL = Wire.read();
    xSemaphoreGive(mutex_iic);

    ACCEL_X = (((int16_t)XH) << 8) | (int16_t)XL;
    ACCEL_Y = (((int16_t)YH) << 8) | (int16_t)YL;
    ACCEL_Z = (((int16_t)ZH) << 8) | (int16_t)ZL;

    float accel_x = (float)ACCEL_X / 2048.0f;
    float accel_y = (float)ACCEL_Y / 2048.0f;
    float accel_z = (float)ACCEL_Z / 2048.0f;

    float accel = sqrtf(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
    xQueueSend(queue_ble_accel, &accel, 0);

    // sprintf(serial_buffer,"MPU6050: AccelX = %.2f\tAccelY = %.2f\tAccelZ = %.2f", accel_x, accel_y, accel_z);
    // xQueueSend(queue_serial, serial_buffer, 0);

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

//--------------MPU-Sensor-Read-Task--End----------------------------------
void setup()
{
  queue_serial = xQueueCreate(32, queue_serial_buffer_size * sizeof(char));
  xTaskCreate(task_serial_server, "Serial Server", 4096, NULL, 1, NULL);
  
  queue_ble_temperature = xQueueCreate(32, sizeof(float));
  queue_ble_accel = xQueueCreate(32, sizeof(float));
  queue_ble_heartbeat = xQueueCreate(32, sizeof(float));
  queue_ble_spo2 = xQueueCreate(32, sizeof(float));
  xTaskCreatePinnedToCore(task_ble_server, "BLE Server", 4096, NULL, 4, NULL, 0);

  mutex_iic = xSemaphoreCreateMutex();
  xTaskCreate(task_mpu_read, "MPU6050 Reader", 4096, NULL, 3, NULL);

  queue_max30102_ir = xQueueCreate(1024, sizeof(uint32_t));
  queue_max30102_red = xQueueCreate(1024, sizeof(uint32_t));
  xTaskCreate(task_max30102_read, "MAX30102 Reader", 4096, NULL, 3, NULL);

  xTaskCreate(task_heartbeat_spo2_algorithm, "Algorithm", 40960, NULL, 2, NULL);
}

void loop()
{
  vTaskDelete(NULL);
}
