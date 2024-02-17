#include <Arduino.h>
#include "MAX30105.h"  //MAX3010x library
#include "heartRate.h" //Heart rate calculating algorithm
#include <Adafruit_MLX90614.h>
#include <SPI.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>

// BLUETOOTH LOW ENERGY
// =============================================================================================
#define BLE_SERVER_NAME "esp32-server"
// SERVEICE UUID
#define SERVEICE_UUID "f024821c-45fd-4c07-a27d-185f92699c4c"
// CHARACTERISTIC UUID
#define OKSIMETER_CHARACTERISTIC_UUID "0e1c52dc-6e6f-462d-89ed-9c30d3564ce3"
#define TERMOMETER_CHARACTERISTIC_UUID "7f7cdd10-883b-4611-aa6a-939078035f7d"
// Define variabels
BLECharacteristic *pOksimeterCharacteristic = NULL;
BLECharacteristic *pTermometerCharacteristic = NULL;
BLEServer *pServer = NULL;
// connection
bool deviceConnected = false;

class MyServerCallback : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    BLEDevice::startAdvertising();
  };
  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
  }
};
// =============================================================================================

// HARDWARE SENSOR
// =============================================================================================
MAX30105 particleSensor;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// Alamat I2C yang ditetapkan untuk sensor
#define MAX30102_I2C_ADDRESS 0x57
#define MLX90614_I2C_ADDRESS 0x5A

// Variabel untuk menghitung detak jantung
const byte RATE_SIZE = 4; // Jumlah rata-rata
byte rates[RATE_SIZE];    // Array detak jantung
byte rateSpot = 0;
long lastBeat = 0; // Waktu ketika detak terakhir terjadi
float beatsPerMinute;
int beatAvg;

// Variabel untuk menghitung SpO2
double avered = 0;
double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
// Variabel suhu tubuh celcius
double suhuTubuh = 0;

double SpO2 = 0;
double ESpO2 = 90.0; // Nilai awal
double FSpO2 = 0.7;  // Faktor penyaringan untuk SpO2 yang diestimasi
double frate = 0.95; // Penyaring rendah untuk nilai LED IR/merah untuk menghilangkan komponen AC
int i = 0;
int Num = 30;             // Ambil sampel 30 kali sebelum menghitung sekali
#define FINGER_ON 7000    // Jumlah minimum inframerah (untuk menentukan apakah jari ada atau tidak)
#define MINIMUM_SPO2 90.0 // Jumlah minimum SpO2
// =============================================================================================

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 1000;

void setup()
{
  Wire.begin(21, 22);
  // mlx.begin();
  Serial.begin(115200);
  while (!Serial)
    ;

  // SENSOR SETUP
  // =================================================================================
  Serial.println("Mulai Sistem");
  delay(1000);
  // if (!mlx.begin())
  // {
  //   Serial.println("tidak dapat menyambungkan MLX sensor. Check perkabelan.");
  // };
  mlx.begin();
  delay(3000);
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) // Gunakan port I2C default, kecepatan 100kHz
  {
    Serial.println("Tidak dapat menemukan MAX30102");
    while (1)
      ;
  }

  byte ledBrightness = 0x7F; // Kecerahan LED disarankan = 127, Opsi: 0=Matikan hingga 255=50mA
  byte sampleAverage = 4;    // Opsi: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;          // Opsi: 1 = Hanya merah (detak jantung), 2 = Merah + IR (SpO2)
  int sampleRate = 800;      // Opsi: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 215;      // Opsi: 69, 118, 215, 411
  int adcRange = 16384;      // Opsi: 2048, 4096, 8192, 16384
  // Atur parameter yang diinginkan
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); // Konfigurasi sensor max30102 dengan pengaturan ini
  particleSensor.enableDIETEMPRDY();
  particleSensor.setPulseAmplitudeRed(0);   // Nyalakan LED Merah ke rendah untuk menunjukkan sensor sedang berjalan
  particleSensor.setPulseAmplitudeGreen(0); // Matikan LED Hijau
  // =================================================================================

  // BLE SETUP
  // =================================================================================
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  BLEDevice::init(BLE_SERVER_NAME);
  // create ble server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallback());
  // create ble serveice
  BLEService *service = pServer->createService(SERVEICE_UUID);
  // Create a BLE Characteristic
  pOksimeterCharacteristic = service->createCharacteristic(
      OKSIMETER_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE);
  pOksimeterCharacteristic->addDescriptor(new BLE2902());
  pTermometerCharacteristic = service->createCharacteristic(
      TERMOMETER_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE);
  pTermometerCharacteristic->addDescriptor(new BLE2902());
  // start service
  service->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVEICE_UUID);
  // pAdvertising->setMinPreferred(0x0);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");

  BLEAddress myAddress = BLEDevice::getAddress();
  Serial.print("Mac address: ");
  Serial.println(myAddress.toString().c_str());
  Serial.println("");
  // =================================================================================
}

void loop()
{
  if (deviceConnected)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    particleSensor.setPulseAmplitudeRed(0x0A); // Nyalakan LED Merah ke rendah untuk menunjukkan sensor sedang berjalan
    // pengecekan denyut jantung dan saturasi oksigen darah
    long irValue = particleSensor.getIR(); // Membaca nilai IR akan memperbolehkan kita untuk mengetahui apakah ada
    // mendeteksi jari
    if (irValue > FINGER_ON)
    {
      if (checkForBeat(irValue) == true)
      {
        long delta = millis() - lastBeat; // Menghitung perbedaan detak jantung
        lastBeat = millis();
        beatsPerMinute = 60 / (delta / 1000.0); // Menghitung detak jantung rata-rata
        if (beatsPerMinute < 255 && beatsPerMinute > 20)
        {
          // Detak jantung harus di antara 20-255
          rates[rateSpot++] = (byte)beatsPerMinute; // Menyimpan nilai detak jantung dalam array
          rateSpot %= RATE_SIZE;
          beatAvg = 0; // Menghitung nilai rata-rata
          for (byte x = 0; x < RATE_SIZE; x++)
            beatAvg += rates[x];
          beatAvg /= RATE_SIZE;
        }
      }

      // Mengukur SpO2
      uint32_t ir, red;
      double fred, fir;
      particleSensor.check(); // Periksa sensor, baca hingga 3 sampel
      if (particleSensor.available())
      {
        i++;
        ir = particleSensor.getFIFOIR();   // Membaca nilai IR
        red = particleSensor.getFIFORed(); // Membaca nilai merah
        // Serial.println("red=" + String(red) + ",IR=" + String(ir) + ",i=" + String(i));
        fir = (double)ir;                                      // Konversi ke double
        fred = (double)red;                                    // Konversi ke double
        aveir = aveir * frate + (double)ir * (1.0 - frate);    // level IR rata-rata oleh penyaringan rendah
        avered = avered * frate + (double)red * (1.0 - frate); // level merah rata-rata oleh penyaringan rendah
        sumirrms += (fir - aveir) * (fir - aveir);             // jumlah kuadrat komponen bergantian dari level IR
        sumredrms += (fred - avered) * (fred - avered);        // jumlah kuadrat komponen bergantian dari level merah

        if ((i % Num) == 0)
        {
          double R = (sqrt(sumirrms) / aveir) / (sqrt(sumredrms) / avered);
          SpO2 = -23.3 * (R - 0.4) + 100;
          ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2; // penyaringan rendah
          // if (ESpO2 <= MINIMUM_SPO2)
          //   ESpO2 = MINIMUM_SPO2; // indikator untuk jari terlepas
          if (ESpO2 > 100)
            ESpO2 = 99.9;
          // Serial.print(",SPO2="); Serial.println(ESpO2);
          sumredrms = 0.0;
          sumirrms = 0.0;
          SpO2 = 0;
          i = 0;
        }
        particleSensor.nextSample(); // Selesai dengan sampel ini sehingga pindah ke sampel berikutnya
      }

      // Menampilkan data ke dalam serial monitor
      if ((millis() - lastTime) > timerDelay)
      {
        if (beatAvg > 30)
        {
          Serial.print("Detak Jantung:" + String(beatAvg));
          Serial.print(", SpO2:" + String(ESpO2) + " %");
          String dataOksimeter = "[" + String(ESpO2) + ", " + String(beatAvg) + "]";
          pOksimeterCharacteristic->setValue(dataOksimeter.c_str());
          pOksimeterCharacteristic->notify();
        }
        else
        {
          Serial.print("Detak Jantung: ... ");
          Serial.print(", SpO2: .... %");
        }
        Serial.println("");
        lastTime = millis();
      }
    }
    // Jika tidak mendeteksi jari, hapus semua data dan tampilkan "Jari Silahkan"
    else
    {
      // Menghapus data detak jantung
      for (byte rx = 0; rx < RATE_SIZE; rx++)
        rates[rx] = 0;
      beatAvg = 0;
      rateSpot = 0;
      lastBeat = 0;
      // Menghapus data SpO2
      avered = 0;
      aveir = 0;
      sumirrms = 0;
      sumredrms = 0;
      SpO2 = 0;
      ESpO2 = 90.0;
    }

    // Pengecekan Suhu
    if ((millis() - lastTime) > timerDelay)
    {
      suhuTubuh = mlx.readObjectTempC();
      Serial.println("Suhu Tubuh: " + String(suhuTubuh));
      String tempString = "[" + String(suhuTubuh) + "]";
      pTermometerCharacteristic->setValue(tempString.c_str());
      pTermometerCharacteristic->notify();
      lastTime = millis();
    }
  }
  else
  {
    delay(1000);
    // restar adversting
    pServer->startAdvertising();
    digitalWrite(LED_BUILTIN, LOW);
    particleSensor.setPulseAmplitudeRed(0); // Nyalakan LED Merah ke rendah untuk menunjukkan sensor sedang berjalan
    Serial.println("Bluetooth tidak tersambung...");
  }
}