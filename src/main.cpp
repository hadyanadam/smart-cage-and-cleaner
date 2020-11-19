#include <Arduino.h>
#define BLYNK_PRINT Serial
#include <HX711.h>
#include <DHT.h>
#include "WiFi.h"
#include <ESP32Servo.h>
#include <BlynkSimpleEsp32.h>
#include <env.h>

Servo servo;  // create servo object to control a servo
const int servoPin = 18;

const int motionSensor = 25;

const int DAT_PIN_K = 14;
const int SCK_PIN_K = 12;
const int DAT_PIN_P = 33;
const int SCK_PIN_P = 32;
const int trig_m = 27;            // HC-SR04 trigger pin
const int echo_m = 26;            // HC-SR04 echo pin
const int trig_p = 22;            // HC-SR04 trigger pin
const int echo_p = 23;            // HC-SR04 echo pin

const int r1 = 21;
const int r2 = 19;
const int r3 = 5;

float duration, distance;
#define DHTPIN 13
#define DHTTYPE DHT11

bool motionDetected = false;

const char* ssid = "kobex";
const char* pass =  "kobex1234";

HX711 hx_kotoran;
HX711 hx_pakan;
DHT dht(DHTPIN, DHTTYPE);

const int motor1Pin1 = 15;
const int motor1Pin2 = 16;
const int enable1Pin = 17;

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
// int dutyCycle = 200;

int dutyCycle = 180;


void IRAM_ATTR detectsMovement() {
  //Serial.println("MOTION DETECTED!!!");
  motionDetected = true;
}

float get_weight(HX711 hx){
  float val = hx.get_units(10);
  Serial.println(hx.get_units(10), 1);

  hx.power_down();			        // put the ADC in sleep mode
  delay(250);
  hx.power_up();
  return val;
}
void calibrate(){
  Serial.println("Before setting up the scale:");
  Serial.print("read: \t\t");
  Serial.println(hx_pakan.read());			// print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(hx_pakan.read_average(20));  	// print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(hx_pakan.get_value(5));		// print the average of 5 readings from the ADC minus the tare weight (not set yet)

  Serial.print("get units: \t\t");
  Serial.println(hx_pakan.get_units(5), 1);	// print the average of 5 readings from the ADC minus tare weight (not set) divided
						// by the SCALE parameter (not set yet)

  hx_pakan.set_scale(-97);
  // hx_kotoran.set_scale(2280.f);
  hx_pakan.tare();
  // hx_kotoran.tare();
}

int ping_us(int echoPin, int trigPin){
  digitalWrite(echoPin, LOW);   // set the echo pin LOW
  digitalWrite(trigPin, LOW);   // set the trigger pin LOW
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);  // set the trigger pin HIGH for 10μs
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);  // measure the echo time (μs)
  distance = (duration/2.0)*0.0343;   // convert echo time to distance (cm)
  return distance;
}

void setPwm(int dutyCycle){
    ledcWrite(pwmChannel, dutyCycle);
}

void bersihkan_kotoran(){

  ledcWrite(pwmChannel, dutyCycle);
  // Move DC motor backwards at maximum speed
  Serial.println("Moving Backwards");
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  delay(1600);

 // Stop the DC motor
  Serial.println("Motor stopped");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  delay(1000);
//   Move the DC motor forward at maximum speed
  Serial.println("Moving Forward");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  delay(1400);
  // Stop the DC motor
  Serial.println("Motor stopped");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  delay(1000); 
}


void push_data(){
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // float berat_kotoran = get_weight(hx_kotoran);
  // float berat_pakan = get_weight(hx_pakan);
  int tinggi_pakan = 20 - ping_us(echo_p, trig_p);
  int tinggi_minum = 15 - ping_us(echo_m, trig_m);
  // int berat_kotoran = 100;
  // int berat_pakan = 200;
  // int tinggi_minum = 30;
  // int tinggi_pakan = 20;

  Serial.print('ultra minum :');
  Serial.println(tinggi_minum);
  Serial.print('ultra pakan :');
  Serial.println(tinggi_pakan);
  // Serial.print(t);
  // Serial.print(" ");
  // Serial.println(h);
  // Serial.println(berat_kotoran);
  // Serial.println(berat_pakan);
  // Blynk.virtualWrite(V4, berat_kotoran);
  // Blynk.virtualWrite(V6, berat_pakan);
  Blynk.virtualWrite(V2, tinggi_pakan);
  Blynk.virtualWrite(V0, tinggi_minum);
  // Blynk.virtualWrite(V5, t);
  // Blynk.virtualWrite(V7, h);
}


void buka_tutup_pakan(){
  servo.write(30);
  delay(1000);
  servo.write(100);
  delay(2000);
  servo.write(30);
}

void setup() {

  Serial.begin(115200);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);

  Serial.println("Connected to the WiFi network");
  pinMode(trig_m, OUTPUT);
  pinMode(echo_m, INPUT);
  pinMode(trig_p, OUTPUT);
  pinMode(echo_p, INPUT);

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);

  pinMode(r1, OUTPUT);
  pinMode(r2, OUTPUT);
  pinMode(r3, OUTPUT);

  ledcSetup(pwmChannel, freq, resolution);

  ledcAttachPin(enable1Pin, pwmChannel);
  hx_kotoran.begin(DAT_PIN_K, SCK_PIN_K);
  hx_pakan.begin(DAT_PIN_P, SCK_PIN_P);

	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	servo.setPeriodHertz(50);    // standard 50 hz servo
	servo.attach(servoPin, 500, 2400); // attaches the servo on pin 18 to the servo object
  dht.begin();
  Blynk.begin(auth, ssid, pass, server, port);
  // calibrate();
  // timer.setInterval(1000L, push_data);
}

void loop() {
  Blynk.run(); // Run blynk
  push_data();
  bersihkan_kotoran();
}
