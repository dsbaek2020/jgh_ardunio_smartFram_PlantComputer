
//#define NO_LCD    1

#include <ArduinoJson.h>

#ifndef NO_LCD  
  #include <Adafruit_GFX.h>    // Core graphics library
  #include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
  #include <SPI.h>
#endif

#include <Arduino_LPS22HB.h> // Pressure sensor library
#include <Arduino_HS300x.h>  // Humidity and temperature sensor library

#include <SensirionI2CScd4x.h>
#include <Wire.h>
#include "Adafruit_seesaw.h"

Adafruit_seesaw ss;
SensirionI2CScd4x scd4x;


#ifndef NO_LCD  
  // Define the pins for the display
  #define TFT_CS      10
  #define TFT_RST     9 
  #define TFT_DC      8
  #define SD_CS       7  // SD card select pin
#endif

#define PUMP_PIN      5  // pump pin
#define LAMP_PIN      3  // lamp pin
#define BUZZER_PIN    4  // buzzer pin

int T = 1000;

// Initialize the display
#ifndef NO_LCD  
  Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
#endif

typedef struct plant_sensor_data{
  float soil_tempC;
  uint16_t soil_capread;

  uint16_t air_co2 = 0;
  float air_temperature = 0.0f;
  float air_humidity = 0.0f;

  float light_lux = 0.0f;

  uint16_t pumpStatus;
  uint16_t ledStatus;

  float onboard_temperature;
  float onboard_humidity;
  float onboard_pressure;
}plant_sensor_data_t;


plant_sensor_data_t plant_data;


// Variables will change:
int pump_newCmd_arrived = 0;
int led_newCmd_arrived = 0;
int PumpState = LOW;  
int LedState = LOW;  
unsigned long previousMillis_pumpTimer = 0;  // will store last time LED was updated
const long MaxOn_TimeMs_pump = 1000*60;  // 60sec  
unsigned long previousMillis_ledTimer = 0;  // will store last time LED was updated
const long MaxOn_TimeMs_led = 1000*60*60;  // 60min(1hr) 


float myround(float num, int decimals) {
    float factor = pow(10, decimals); // 10의 'decimals' 제곱을 계산
    return round(num * factor) / factor; // num을 factor로 곱한 값에 대해 반올림 후 다시 factor로 나눔
}

void printUint16Hex(uint16_t value);
void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2);

void get_Rpi_Command(){
  StaticJsonDocument<100> Txdoc;
  String Txjson;
  // Check if data is available
  if (Serial1.available()) {
        Serial.println("serial1(Rpi uart received)~");
        // Read the incoming JSON string
        String Rxjson = Serial1.readString();

        // Deserialize the JSON string to object
        StaticJsonDocument<100> Rxdoc;
        DeserializationError error = deserializeJson(Rxdoc, Rxjson);

        if (!error) {
          // Access data
          String cmd = Rxdoc["cmd"];


          if (cmd != "reqYourData"){
            Serial.print("Received command from Rpi: ");
            Serial.println(cmd);

            if(cmd == "set_pump"){
              pump_newCmd_arrived = 1;
              String pumpCmd = Rxdoc["onOff"];
              if(pumpCmd == "on"){
                  PumpState = HIGH;
              }else{
                  PumpState = LOW;
              }

            }else if(cmd == "set_LED"){
              led_newCmd_arrived = 1;
              String ledCmd = Rxdoc["onOff"];
              if(ledCmd == "on"){
                  LedState = HIGH;
              }else{
                  LedState = LOW;
              }

            }else{
              Serial.print("not defined command from Rpi");
              Serial.println(cmd);
            }



            const char* onOff = Rxdoc["onOff"];
            int  time = Rxdoc["time"][0];
            const char* unit = Rxdoc["time"][1];
            // Print data to serial monitor
            Serial.print("cmd:");
            Serial.print(cmd);
            Serial.print("\t");
            Serial.print("onOff:");
            Serial.print(onOff);
            Serial.print("\t");
            Serial.print("time:");
            Serial.print(time);
            Serial.print("\t");
            Serial.print("unit:");
            Serial.println(unit);

          }else{
            String sensorType = Rxdoc["sensorType"];
            // Print data to serial monitor
            Serial.print("cmd:");
            Serial.print(cmd);
            Serial.print("\t");
            Serial.print("sensorType:");
            Serial.println(sensorType);

            if(sensorType =="all"){

              // Create a JSON object
              Txdoc["sensor"] = "all";
              Txdoc["time"] = 987654321;
              Txdoc["data"][0] = myround(plant_data.soil_tempC,3);
              Txdoc["data"][1] = myround(plant_data.soil_capread,3);
              Txdoc["data"][2] = myround(plant_data.light_lux,3);
              Txdoc["data"][3] = myround(plant_data.air_co2,3);
              Txdoc["data"][4] = myround(plant_data.air_temperature,3);
              Txdoc["data"][5] = myround(plant_data.air_humidity,3);
              Txdoc["data"][6] = PumpState;
              Txdoc["data"][7] = LedState;

            }
            else{
                  // Create a JSON object
                  Txdoc["sensor"] = "soil_humidity";
                  Txdoc["time"] = 1351824120;
                  Txdoc["data"][0] = 45.89;

            }      
            // Serialize JSON to string
            serializeJson(Txdoc, Txjson);
            Serial1.println(Txjson);


          }

        } else {}
  }
}
void setup_soil(){
    if (!ss.begin(0x36)) {
    Serial.println("ERROR! seesaw not found");
    while(1) delay(1);
  } else {
    Serial.print("seesaw started! version: ");
    Serial.println(ss.getVersion(), HEX);
  }
}
void setup_scd40(){
  Wire.begin();

    uint16_t error;
    char errorMessage[256];

    scd4x.begin(Wire);

    // stop potentially previously started measurement
    error = scd4x.stopPeriodicMeasurement();
    if (error) {
        Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    uint16_t serial0;
    uint16_t serial1;
    uint16_t serial2;
    error = scd4x.getSerialNumber(serial0, serial1, serial2);
    if (error) {
        Serial.print("Error trying to execute getSerialNumber(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        printSerialNumber(serial0, serial1, serial2);
    }

    // Start Measurement
    error = scd4x.startPeriodicMeasurement();
    if (error) {
        Serial.print("Error trying to execute startPeriodicMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    Serial.println("Waiting for first measurement... (5 sec)");
}


// 부팅 멜로디를 위한 음계의 주파수 정의
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523

int melody[] = {NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5};
int noteDurations[] = {4, 4, 4, 4, 4, 4, 4, 4}; // 4는 음표의 길이를 의미 (여기서는 4분음표)


void BOOT_music() {
  for (int thisNote = 0; thisNote < 8; thisNote++) {
    // 음표 재생
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(BUZZER_PIN, melody[thisNote], noteDuration);

    // 음표 사이의 간격을 둠
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    
    // 다음 음표 재생을 위해 버저 끄기
    noTone(BUZZER_PIN);
  }
}


void setup() {

  // Initialize serial communication for debug UART with PC.
  Serial.begin(2000000);
  // Initialize serial communication for Rpi-arduino Communication.
  Serial1.begin(115200);

    // set the digital pin as output:
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  
#ifndef NO_LCD 
  // Initialize the display
  tft.init(240, 320); // Initialize with width and height
  tft.setRotation(3); // Set display rotation
  tft.fillScreen(ST77XX_BLACK); // Clear the screen with black
#endif

  // Initialize the sensors
  if (!BARO.begin()) {
    Serial.println("Failed to initialize LPS22HB!");
    while (1);
  }
  if (!HS300x.begin()) {
    Serial.println("Failed to initialize HS300x!");
    while (1);
  }

  // Try to initialize!
  setup_scd40();
  tft.fillScreen(ST77XX_BLACK);
  
  // Set text color and size
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);

  // Display the pressure
  tft.setCursor(10, 30);
  tft.println("Jinhae");
  tft.println("Girls High School");
    BOOT_music();
  setup_soil();
  //setup_luxSensor() // not operate in ble 33 sense rev2 !!! more info (stackov)

}


void loop() {

  // Check if data is available
  if (Serial1.available()) {
    get_Rpi_Command();
  }

  // Read data from the On-Board sensors
  float temperature = HS300x.readTemperature();
  float humidity = HS300x.readHumidity();
  float pressure = BARO.readPressure();


#ifndef NO_LCD 
  // Clear the screen area where the data is displayed
  //tft.fillRect(0, 0, 240, 320, ST77XX_BLACK)

  // 
  tft.fillScreen(ST77XX_BLACK);
  
  // Set text color and size
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);

  // Display the pressure
  tft.setCursor(10, 30);
  tft.print("Pressure: ");
  tft.print(pressure);
  tft.println(" hPa");

#endif

//get environment data----------------
  get_scd40();
  get_soilSensor();
//------------------------------------

/*
  if (scd30.dataReady()){
      Serial.println("Data available!");

      if (!scd30.read()){ Serial.println("Error reading sensor data"); return; }

      Serial.print("Temperature: ");
      Serial.print(scd30.temperature);
      Serial.println(" degrees C");
      
      Serial.print("Relative Humidity: ");
      Serial.print(scd30.relative_humidity);
      Serial.println(" %");
      
      Serial.print("CO2: ");
      Serial.print(scd30.CO2, 3);
      Serial.println(" ppm");
      Serial.println("");

      // Display the pressure
      tft.setCursor(10, 150);
      tft.print("CO2: ");
      tft.print(scd30.CO2, 3);
      tft.println(" ppm");

  } else {
      //Serial.println("No data");
  }
*/
  unsigned long currentMillis = millis();

  //reset pump timer 
  if(pump_newCmd_arrived == 1){
    previousMillis_pumpTimer = currentMillis;
    pump_newCmd_arrived = 0;
  }
  if (currentMillis - previousMillis_pumpTimer >= MaxOn_TimeMs_pump) {
    // save the last time
    previousMillis_pumpTimer = currentMillis;
    if (PumpState == HIGH) {
      PumpState = LOW;
    }
  }

  //reset led timer 
  if(led_newCmd_arrived == 1){
    previousMillis_pumpTimer = currentMillis;
    led_newCmd_arrived = 0;
  }
  if (currentMillis - previousMillis_ledTimer >= MaxOn_TimeMs_led) {
    // save the last time
    previousMillis_ledTimer = currentMillis;
    if (LedState == HIGH) {
      LedState = LOW;
    }
  }

  digitalWrite(PUMP_PIN, PumpState);
  digitalWrite(LAMP_PIN, LedState);



  // Delay before next update
  delay(T);
}











void get_soilSensor(){
    float tempC = ss.getTemp();
    uint16_t capread = ss.touchRead(0);

    //Copy to global data 
    plant_data.soil_tempC = tempC;
    plant_data.soil_capread = capread;

      // Display the co2
      tft.setCursor(10, 190);
      tft.print("soil temp: ");
      tft.print(float(tempC), 0);
      tft.print(" , ");
      tft.print("spread: ");
      tft.print(float(capread), 0);
      tft.println("");

    Serial.print("Temperature: "); Serial.print(tempC); Serial.println("*C");
    Serial.print("Capacitive: "); Serial.println(capread);
}

void get_scd40() {
    uint16_t error;
    char errorMessage[256];

    // Read Measurement
    uint16_t co2 = 0;
    float temperature = 0.0f;
    float humidity = 0.0f;
    bool isDataReady = false;
    error = scd4x.getDataReadyFlag(isDataReady);
    if (error) {
        Serial.print("Error trying to execute getDataReadyFlag(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
        return;
    }
    if (!isDataReady) {
        return;
    }
    error = scd4x.readMeasurement(co2, temperature, humidity);
    if (error) {
        Serial.print("Error trying to execute readMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else if (co2 == 0) {
        Serial.println("Invalid sample detected, skipping.");
    } else {
        //Copy to global data 
        plant_data.air_co2 = co2;
        plant_data.air_temperature = temperature;
        plant_data.air_humidity = humidity;

        Serial.print("Co2:");
        Serial.print(co2);
        Serial.print("\t");
        Serial.print("Temperature:");
        Serial.print(temperature);
        Serial.print("\t");
        Serial.print("Humidity:");
        Serial.println(humidity);

#ifndef NO_LCD 
        // Display the temperature
        tft.setCursor(10, 70);
        tft.print("Temperature: ");
        tft.print(temperature);
        tft.println(" C");

        // Display the humidity
        tft.setCursor(10, 110);
        tft.print("Humidity: ");
        tft.print(humidity);
        tft.println(" %");

        // Display the co2
        tft.setCursor(10, 150);
        tft.print("CO2: ");
        tft.print(float(co2), 3);
        tft.println(" ppm");
#endif
    }
}

void printUint16Hex(uint16_t value) {
    Serial.print(value < 4096 ? "0" : "");
    Serial.print(value < 256 ? "0" : "");
    Serial.print(value < 16 ? "0" : "");
    Serial.print(value, HEX);
}

void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2) {
    Serial.print("Serial: 0x");
    printUint16Hex(serial0);
    printUint16Hex(serial1);
    printUint16Hex(serial2);
    Serial.println();
}
