#include <functions.h>
#include <env.h>

void setup() {

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  hx_pakan.begin(DAT_PIN_K, SCK_PIN_K);
  hx_kotoran.begin(DAT_PIN_K, SCK_PIN_K);
  pinMode(motionSensor, INPUT_PULLUP);
  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);

  pinMode(r1, OUTPUT);
  pinMode(r2, OUTPUT);
  pinMode(r3, OUTPUT);
  digitalWrite(r1, HIGH);
  digitalWrite(r2, HIGH);
  digitalWrite(r3, HIGH);
  // configure LED PWM functi1onalitites
  ledcSetup(pwmChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel);

  //  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo.setPeriodHertz(50);    // standard 50 hz servo
  servo.attach(servoPin, 1000, 2000); // attaches the servo on pin 18 to the servo object

  Serial.begin(115200);
  dht.begin();
  pinMode(trig_m, OUTPUT);
  pinMode(echo_m, INPUT);
  pinMode(trig_p, OUTPUT);
  pinMode(echo_p, INPUT);

  // testing
  Blynk.begin(auth, ssid, pass, server, port);
  calibrate();
}

void loop() {
  Blynk.run();
  getDHT();
  if (temp > 27){
    detachInterrupt(motionSensor);
    delay(250);
    kipasHandler(true);
  }
  else if (temp < 24){
    detachInterrupt(motionSensor);
    delay(250);
    dehumidifierHandler(true);
  }
  attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);
  float berat_pakan = hx_pakan.get_units(10);
  float berat_kotoran = hx_pakan.get_units(10);
  Serial.print("M one reading:\t");
  Serial.println(berat_kotoran, 1);

  Serial.print("P one reading:\t");
  Serial.println(berat_pakan, 1);
  hx_kotoran.power_down();              // put the ADC in sleep mode

  hx_pakan.power_down();              // put the ADC in sleep mode

  int tinggi_pakan = 20 - ping_us(echo_p, trig_p);
  int tinggi_minum = 15 - ping_us(echo_m, trig_m);

  Serial.print("Sensor 01: ");
  Serial.print(tinggi_pakan); // Prints the distance on the default unit (centimeters)
  Serial.println("cm");

  Serial.print("Sensor 02: ");
  Serial.print(tinggi_minum); // Prints the distance making the unit explicit
  Serial.println("cm");

  Blynk.virtualWrite(V2, tinggi_pakan);
  Blynk.virtualWrite(V0, tinggi_minum);
  Blynk.virtualWrite(V5, temp);
  Blynk.virtualWrite(V7, humidity);
  Blynk.virtualWrite(V4, berat_kotoran);
  Blynk.virtualWrite(V6, berat_pakan);

  if (motionDetected){
     motionDetected = false;
     indicatorLed.off();
     delay(2500);
  }

  // bersihkan_kotoran();
  // buka_tutup_pakan();
  hx_kotoran.power_up();
  hx_pakan.power_up();

}
