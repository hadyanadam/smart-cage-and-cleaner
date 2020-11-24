
#define BLYNK_PRINT Serial
#include <HX711.h>
#include <DHT.h>
#include "WiFi.h"
#include <ESP32Servo.h>
#include <BlynkSimpleEsp32.h>

const char* ssid = "kobex";
const char* pass =  "kobex1234";

const int motionSensor = 36;

float temp = 0;
float humidity = 0;

const int WATER_LEVEL = 34;
const int DAT_PIN_K = 14;
const int SCK_PIN_K = 12;
const int DAT_PIN_P = 33;
const int SCK_PIN_P = 32;
const int r1 = 21;
const int r2 = 19;
const int r3 = 25;
const int trig_m = 22;            // HC-SR04 trigger pin
const int echo_m = 23;            // HC-SR04 echo pin
const int trig_p = 27;            // HC-SR04 trigger pin
const int echo_p = 26;
#define DHTPIN 4
#define DHTTYPE DHT11
#define pinKipas V8
#define pinDehumid V1
#define pinPompa V3
const int servoPin = 18;
Servo servo;
DHT dht(DHTPIN, DHTTYPE);
// Motor A
int motor1Pin1 = 15;
int motor1Pin2 = 16;
int enable1Pin = 17;
long duration, distance;

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 195;

bool motionDetected = false;

HX711 hx_kotoran;
HX711 hx_pakan;

WidgetLED indicatorLed(V9);
WidgetLED kotoranLed(V9);
WidgetLED dehumidLed(V1);
WidgetLED kipasLed(V8);
WidgetLED pompaLed(V3);

void IRAM_ATTR detectsMovement() {
  Serial.println("MOTION DETECTED!!!");
  motionDetected = true;
  //   indicatorLed.setValue(255);
  // indicatorLed.on();
}

void calibrate() {
  hx_pakan.set_scale();
  hx_pakan.tare();
  long zero_factor1 = hx_pakan.read_average();
  hx_kotoran.set_scale();
  hx_kotoran.tare();
  long zero_factor2 = hx_kotoran.read_average();
}

void bersihkan_kotoran() {

  detachInterrupt(motionSensor);
  ledcWrite(pwmChannel, dutyCycle);

  // Move DC motor backwards at maximum speed
  Serial.println("Moving Backwards");
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  delay(2500);

  // Stop the DC motor
  Serial.println("Motor stopped");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  delay(2000);


  //   Move the DC motor forward at maximum speed
  Serial.println("Moving Forward");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  delay(2300);

  // Stop the DC motor
  Serial.println("Motor stopped");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  delay(1000);
  attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);
}

void buka_tutup_pakan() {

  servo.write(30);
  delay(1000);
  servo.write(100);
  delay(2000);
  servo.write(30);
}

int ping_us(int echoPin, int trigPin) {
  digitalWrite(echoPin, LOW);   // set the echo pin LOW
  digitalWrite(trigPin, LOW);   // set the trigger pin LOW
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);  // set the trigger pin HIGH for 10μs
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);  // measure the echo time (μs)
  distance = (duration / 2.0) * 0.0343; // convert echo time to distance (cm)
  return distance;
}

void dehumidifierHandler(bool val){
    if (val){
      dehumidLed.on();
    }else{
      dehumidLed.off();
    }
    digitalWrite(r3, !val);
    // Blynk.virtualWrite(pinDehumid,val);
    delay(1000);
}

void kipasHandler(bool val){
    if (val){
      kipasLed.on();
    }else{
      kipasLed.off();
    }
    digitalWrite(r1, !val);
    // Blynk.virtualWrite(pinKipas,val);
    delay(1000);
}

void pompaHandler(bool val){
    if (val){
      pompaLed.on();
    }else{
      pompaLed.off();
    }
    digitalWrite(r2, !val);
    // Blynk.virtualWrite(pinPompa,val);
    delay(1000);
}

void on_off_relay() {
  digitalWrite(r1, LOW);
  digitalWrite(r2, LOW);
  delay(1000);
  digitalWrite(r1, HIGH);
  digitalWrite(r2, HIGH);
  delay(1000);
}

void getDHT(){
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h) || !isnan(t)) {
      temp = t;
      humidity = h;
  }else{
      Serial.println("dht error");
  }
  Serial.print("temp: ");
  Serial.print(t); // Prints the distance making the unit explicit
  Serial.println("C");

  Serial.print("humid: ");
  Serial.print(h); // Prints the distance making the unit explicit
  Serial.println("%");
}

float getWaterLevel(){
  int value = analogRead(WATER_LEVEL);
  Serial.println(value);
  float data = map(value, 0, 4096, 0, 50);
  return data;
}