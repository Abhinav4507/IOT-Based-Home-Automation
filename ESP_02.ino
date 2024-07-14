
#define BLYNK_TEMPLATE_ID "TMPL3B9EtoeW5"
#define BLYNK_TEMPLATE_NAME "iot project"
#define BLYNK_AUTH_TOKEN "uONmtWM4qGyzLGDOJUuaxjYU_4fA0ysu"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
//#include <LiquidCrystal_I2C.h>

//LiquidCrystal_I2C lcd (0x27, 16, 2);

#define BLYNK_PRINT Serial

char auth[] = BLYNK_AUTH_TOKEN;
BlynkTimer timer;

char ssid[] = "Galaxy M311F42";
char pass[] = "qyvr7102";

#define RELAY_1 27
#define RELAY_2 26

#define BUZZER  23
#define TRIG    12
#define ECHO    13
#define DHT_PIN 19
#define FLAME   21

#define LDR 13
#define LED 4

int STATE_RELAY_1 = 0;
int STATE_RELAY_2 = 0;

#define VPIN_BUTTON_1     V0
#define VPIN_BUTTON_2     V1
#define VPIN_TEMPERATURE  V2
#define VPIN_HUMIDITY     V3
#define VPIN_ULTRASONIC   V4
#define VPIN_FLAME        V9

boolean flame_new   = LOW;  // current state of pin
boolean flame_old   = LOW;  // previous state of pin

float temp_new      = LOW;  // current state of pin
float temp_old      = LOW;  // previous state of pin

String fire_event = "FIRE";

boolean buzzer_state = false;
unsigned long buzzer_timer = 0;

int tank_height = 100; //in centimeter
int temp_limit  = 50;

#include <DHT.h>
#define DHTTYPE DHT11
DHT dht(DHT_PIN, DHTTYPE);

BLYNK_CONNECTED() {
  Blynk.syncVirtual(VPIN_BUTTON_1);
  Blynk.syncVirtual(VPIN_BUTTON_2);
}

BLYNK_WRITE(VPIN_BUTTON_1) {
  STATE_RELAY_1 = param.asInt();
  digitalWrite(RELAY_1, STATE_RELAY_1);
  Serial.print("Relay1 State = "); Serial.println(STATE_RELAY_1);
}

BLYNK_WRITE(VPIN_BUTTON_2) {
  STATE_RELAY_2 = param.asInt();
  digitalWrite(RELAY_2, STATE_RELAY_2);
  Serial.print("Relay2 State = "); Serial.println(STATE_RELAY_2);
}

void setup() {
  
  Serial.begin(115200);

  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);

  pinMode(FLAME, INPUT);
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  pinMode(LDR, INPUT);
  pinMode(LED, OUTPUT);

  dht.begin();
  Blynk.begin(auth, ssid, pass);

  timer.setInterval(100L, DHT11Sensor);
  timer.setInterval(100L, UltrasonicSensor);
  timer.setInterval(100L, FlameSensor);

  //During Starting all Relays should TURN OFF
  digitalWrite(RELAY_1, HIGH);
  digitalWrite(RELAY_2, HIGH);

  //lcd.init();
  //lcd.backlight();
  //lcd.clear();

}

void loop() {
  
  Blynk.run();
  timer.run();

  if (buzzer_state == true) {
    if (millis() - buzzer_timer > 5000) {
      digitalWrite(BUZZER, LOW);
      buzzer_state = false;
      buzzer_timer = 0;
    }
  }

  int LDRvalue = analogRead(LDR);

  if (LDRvalue <= 300){
    digitalWrite(LED, HIGH);
  }
  else{
    digitalWrite(LED, LOW);
  }


}

//RELAY CONTROL FUNCTION
void ControlRelay(int number, int relay_pin, int &status, int virtual_pin){
    delay(200);
    status = !status;
    digitalWrite(relay_pin, status);
    delay(50);
    Blynk.virtualWrite(virtual_pin, status); //update button state
    Serial.print("Relay"+String(number)+" State = "); 
    Serial.println(status);
}


//ULTRASONIC FUNCTION
void UltrasonicSensor() {
  //--------------------------------------------------
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  //--------------------------------------------------
  long t = pulseIn(ECHO, HIGH);
  long cm = (t * 0.034) / 2; //SOUND_SPEED = 0.034 i.e. 343m/s
  //--------------------------------------------------
  if(cm <= tank_height){
    Blynk.virtualWrite(VPIN_ULTRASONIC, tank_height-cm);
      digitalWrite(BUZZER, HIGH);
    }
  else{
    Blynk.virtualWrite(VPIN_ULTRASONIC, tank_height-cm);
    digitalWrite(BUZZER, LOW);
  }
}

//FLAME SENSOR FUNCTION
void FlameSensor() {
  //--------------------------------------------------------------
  flame_old = flame_new; // store old state
  flame_new = digitalRead(FLAME); //read new state
  //--------------------------------------------------------------
  if(flame_old == HIGH && flame_new == LOW) { //HIGH to LOW
    String text = "Fire is detected";
    Serial.println(text);
    Blynk.logEvent(fire_event, text);
    digitalWrite(BUZZER, HIGH);
    buzzer_state = true;
    buzzer_timer = millis();
  }
  //--------------------------------------------------------------
  Blynk.virtualWrite(VPIN_FLAME, !flame_new);
  //--------------------------------------------------------------
}

//DHT SENSOR FUNCTION
void DHT11Sensor() {

  float h = dht.readHumidity();

  temp_old = temp_new;
  temp_new = dht.readTemperature();
  
  Blynk.virtualWrite(VPIN_TEMPERATURE, temp_new);
  Blynk.virtualWrite(VPIN_HUMIDITY, h);
  
}
