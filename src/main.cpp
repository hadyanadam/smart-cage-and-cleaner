#include <Arduino.h>
#include <HX711.h>
#include <DHT.h>
#include "WiFi.h"
#include <ESP32Servo.h>

Servo myservo;  // create servo object to control a servo
const int servoPin = 13;

const int DAT_PIN_K = 12;
const int SCK_PIN_K = 14;
const int DAT_PIN_P = 33;
const int SCK_PIN_P = 32;
const int trig_m = 6;            // HC-SR04 trigger pin
const int echo_m = 7;            // HC-SR04 echo pin
const int trig_p = 22;            // HC-SR04 trigger pin
const int echo_p = 21;            // HC-SR04 echo pin
float duration, distance;
#define DHTPIN 8
#define DHTTYPE DHT11

const char* ssid = "Sundaya Office";
const char* password =  "Sundaya2019";

HX711 hx_kotoran;
HX711 hx_pakan;
DHT dht(DHTPIN, DHTTYPE);

const int motor1Pin1 = 36;
const int motor1Pin2 = 39;
const int enable1Pin = 14;

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;

void setup() {

  Serial.begin(9600);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  pinMode(trig_m, OUTPUT);
  pinMode(echo_m, INPUT);
  pinMode(trig_p, OUTPUT);
  pinMode(echo_p, INPUT);

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel);
  hx_kotoran.begin(DAT_PIN_K, SCK_PIN_K);
  hx_pakan.begin(DAT_PIN_P, SCK_PIN_P);
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo.setPeriodHertz(50);    // standard 50 hz servo
	myservo.attach(servoPin, 500, 2400); // attaches the servo on pin 18 to the servo object
  dht.begin();
}

void loop() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  Serial.print("one reading:\t");
  Serial.print(hx_kotoran.get_units(), 1);
  Serial.print("\t| average:\t");
  Serial.println(hx_kotoran.get_units(10), 1);

  hx_kotoran.power_down();			        // put the ADC in sleep mode
  delay(10);
  hx_kotoran.power_up();
  Serial.print("one reading:\t");
  Serial.print(hx_pakan.get_units(), 1);
  Serial.print("\t| average:\t");
  Serial.println(hx_pakan.get_units(10), 1);

  hx_pakan.power_down();			        // put the ADC in sleep mode
  delay(10);
  hx_pakan.power_up();
}


void calibrate(){
  hx_kotoran.set_scale();
  hx_kotoran.tare();
  hx_kotoran.get_units(10);

  hx_pakan.set_scale();
  hx_pakan.tare();
  hx_pakan.get_units(10);

  hx_pakan.set_scale(2280.f);
  hx_kotoran.set_scale(2280.f);
  hx_pakan.tare();
  hx_kotoran.tare();
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

void forward(){
  Serial.println("Moving Forward");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  delay(2000);
}

void backward(){
    Serial.println("Moving Backwards");
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  delay(2000);
}

void stop(){
  Serial.println("Motor stopped");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  delay(1000);
}

void open(){
		myservo.write(100);    // tell servo to go to position in variable 'pos'
		delay(1000);             // waits 15ms for the servo to reach the position
		myservo.write(60);    // tell servo to go to position in variable 'pos'
}

void close(){
		myservo.write(30);    // tell servo to go to position in variable 'pos'
		delay(1000);             // waits 15ms for the servo to reach the position
		myservo.write(60);    // tell servo to go to position in variable 'pos'
}