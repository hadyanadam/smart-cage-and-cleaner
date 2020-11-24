#include <functions.h>
#include <env.h>

int pinState = 0;
bool pakanAda = true;
bool minumAda = true;

BLYNK_WRITE(pinDehumid) {  
  pinState = param.asInt();
  if(pinState == 1){
    Serial.println("dehumid on");
  }
  else if(pinState == 0){
    Serial.println("dehumid off");
  }   
}

BLYNK_WRITE(pinKipas) {  
  pinState = param.asInt();
  if(pinState == 1){
    Serial.println("kipas on");
  }
  else if(pinState == 0){
    Serial.println("kipas off");
  }   
}

BLYNK_WRITE(pinPompa) {  
  pinState = param.asInt();
  if(pinState == 1){
    Serial.println("pompa on");
  }
  else if(pinState == 0){
    Serial.println("pompa off");
  }   
}

void setup() {

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  hx_pakan.begin(DAT_PIN_P, SCK_PIN_P);
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

  Serial.begin(9600);
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
  hx_pakan.set_scale(-109525);
  hx_kotoran.set_scale(-179525);
  getDHT();
  if (temp > 27 && temp != 0){
    detachInterrupt(motionSensor);
    delay(250);
    dehumidifierHandler(false);
    kipasHandler(true);
  }
  else if (temp < 24 && temp != 0){
    detachInterrupt(motionSensor);
    delay(250);
    kipasHandler(false);
    dehumidifierHandler(true);
  }else{
    attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);
      kipasHandler(false);
    dehumidifierHandler(false);

  }
  float berat_pakan = hx_pakan.get_units(5) * 1000;
  float berat_kotoran = hx_kotoran.get_units(5) * 1000;
  hx_kotoran.power_down();              // put the ADC in sleep mode
  hx_pakan.power_down();              // put the ADC in sleep mode
  float value_water = getWaterLevel() / 10;
  if (value_water < 2.0){
    pompaHandler(true);
    delay(1000);
  }
  else if (value_water >= 2.0){
    pompaHandler(false);
    delay(1000);
  }
  Serial.print("water level :");
  Serial.println(value_water);
  Serial.print("berat kotoran:\t");
  Serial.println(berat_kotoran, 1);

  Serial.print("berat pakan:\t");
  Serial.println(berat_pakan, 1);


  int tinggi_pakan = 20 - ping_us(echo_p, trig_p);
  int tinggi_minum = 15 - ping_us(echo_m, trig_m);

  if (tinggi_pakan < 4 && pakanAda){
    Blynk.notify("Pakan Habis");
    pakanAda = false;
  }else if(tinggi_pakan >= 4){
    pakanAda = true;
  }
  if (tinggi_minum < 4 && minumAda){
    Blynk.notify("Minum Habis");
    minumAda = false;
  }else if (tinggi_minum >= 4){
    minumAda = true;
  }

  Serial.print("Sensor 01: ");
  Serial.print(tinggi_pakan); // Prints the distance on the default unit (centimeters)
  Serial.println("cm");

  Serial.print("Sensor 02: ");
  Serial.print(tinggi_minum); // Prints the distance making the unit explicit
  Serial.println("cm");

  Blynk.virtualWrite(V2, tinggi_pakan);
  Blynk.virtualWrite(V0, tinggi_minum);
  Blynk.virtualWrite(V5, temp);
  Blynk.virtualWrite(V10, value_water);
  Blynk.virtualWrite(V7, humidity);
  if(berat_kotoran < 0){
    Blynk.virtualWrite(V4, 0);
  }else{
    Blynk.virtualWrite(V4, berat_kotoran);
  }
  if(berat_pakan < 0){
  Blynk.virtualWrite(V6, 0);

  }else{
  Blynk.virtualWrite(V6, berat_pakan);

  }

  if (motionDetected){
     detachInterrupt(motionSensor);
     motionDetected = false;
     indicatorLed.on();
     delay(10000);
     indicatorLed.off();
     bersihkan_kotoran();
     attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);
    }

  // buka_tutup_pakan();
  hx_kotoran.power_up();
  hx_pakan.power_up();

}
