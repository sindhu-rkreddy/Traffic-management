#include <Wire.h>  
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <MFRC522.h> // Include the MFRC522 library
#include <IRremote.h>

#define BUZZER_PIN 10
#define IR_SENSOR_1_PIN 6
#define IR_SENSOR_2_PIN 7
#define GPS_RX_PIN 7
#define GPS_TX_PIN 8

#define SS_PIN 2 // Set SS_PIN to the pin connected to the SDA (SS) pin of the RFID reader
#define RST_PIN 9 // Set RST_PIN to the pin connected to the RST pin of the RFID reader

MFRC522 mfrc522(SS_PIN, RST_PIN); // Create MFRC522 instance
LiquidCrystal_I2C lcd(0x27, 16, 2); // Specify the I2C address, number of columns, and number of rows
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);

IRrecv irrecv(IR_SENSOR_1_PIN);
IRrecv irrecv2(IR_SENSOR_2_PIN);

unsigned long previousMillis = 0;
const long interval = 1000; // Interval for distance checking in milliseconds
const int thresholdDistance = 100; // Threshold distance in meters

double latitude1 = 0.0, longitude1 = 0.0; // GPS coordinates of car 1
double latitude2 = 0.0, longitude2 = 0.0; // GPS coordinates of car 2

void setup() {
  Serial.begin(9600);
  lcd.init(); // Initialize the LCD
  lcd.backlight(); // Turn on the backlight
  irrecv.enableIRIn();
  irrecv2.enableIRIn();
  pinMode(BUZZER_PIN, OUTPUT);
  mfrc522.PCD_Init(); // Initialize MFRC522
  gpsSerial.begin(9600);
}

void loop() {
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) { // Check if a new RFID card is present
    lcd.clear();
    lcd.print("RFID Detected!");
    // Logic to read RFID tag data
    String rfidData = "";
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      rfidData += String(mfrc522.uid.uidByte[i], HEX);
    }
    lcd.setCursor(0, 1);
    lcd.print("RFID: " + rfidData);
    delay(2000);
  }
  
  // GPS data reading
  if (gpsSerial.available() > 0) {
    String gpsData = gpsSerial.readStringUntil('\n');
    parseGPSData(gpsData);
  }

  // IR sensor detection
  if (irrecv.decode()) {
    lcd.clear();
    lcd.print("IR Sensor 1 Detected!");
    delay(1000);
    irrecv.resume();
  }

  if (irrecv2.decode()) {
    lcd.clear();
    lcd.print("IR Sensor 2 Detected!");
    delay(1000);
    irrecv2.resume();
  }

  // Calculate distance between cars
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    double distance = calculateDistanceBetweenCars();
    if (distance < thresholdDistance) {
      digitalWrite(BUZZER_PIN, HIGH);
      lcd.clear();
      lcd.print("Cars too close!");
      lcd.setCursor(0, 1);
      lcd.print("Distance: ");
      lcd.print(distance);
      lcd.print("m");
      delay(2000);
      digitalWrite(BUZZER_PIN, LOW);
    }
  }
}

void parseGPSData(String data) {
  if (data.startsWith("$GPGGA")) {
    int index = 0;
    for (int i = 0; i < 13; i++) {
      index = data.indexOf(',', index) + 1;
    }
    String latitudeStr = data.substring(index, data.indexOf(',', index));
    index = data.indexOf(',', index) + 1;
    String longitudeStr = data.substring(index, data.indexOf(',', index));
    latitude1 = latitude2;
    longitude1 = longitude2;
    latitude2 = latitudeStr.toDouble();
    longitude2 = longitudeStr.toDouble();
  }
}

double calculateDistanceBetweenCars() {
  // Convert latitude and longitude from degrees to radians
  double lat1Rad = radians(latitude1);
  double lon1Rad = radians(longitude1);
  double lat2Rad = radians(latitude2);
  double lon2Rad = radians(longitude2);

  // Haversine formula for calculating distance between two points on Earth
  double dlon = lon2Rad - lon1Rad;
  double dlat = lat2Rad - lat1Rad;
  double a = pow(sin(dlat / 2), 2) + cos(lat1Rad) * cos(lat2Rad) * pow(sin(dlon / 2), 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double distance = 6371000 * c; // Radius of Earth in meters
  return distance;
}
