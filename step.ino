//#define DEBUG

extern bool hasFallen;
extern uint32_t fallenTime;



#define RED_LED 11
#define BLUE_LED 12
#define PUSH_BTN 7
#define BUZZER 6

void setup() {
  #ifdef DEBUG
    Serial.begin(9600);
  #endif
  delay(3000);
  
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(BLUE_LED, HIGH);
  digitalWrite(RED_LED, HIGH);
  
  pinMode(PUSH_BTN, INPUT_PULLUP);

  simInit("1234");
  gpsInit();
  imuInit();

  if(checkGSM()) {
    digitalWrite(BLUE_LED, LOW);
  }
  else {
    digitalWrite(BLUE_LED, HIGH);
  }

  digitalWrite(RED_LED, LOW);
}


uint32_t lastBtnTime = 0;
bool btnState = false;
#define DEBOUNCE_TIME 300
void btnLoop() {
  if(digitalRead(PUSH_BTN) == LOW && millis() - lastBtnTime > DEBOUNCE_TIME) {
    lastBtnTime = millis();
    btnState = true;
  }

  if(hasFallen && btnState) {
    hasFallen = false;
  }
  else if(!hasFallen && btnState) {
    hasFallen = true;
    fallenTime = millis();
  }
  btnState = false;
}


uint32_t lastBuzzerTime;
int buzzerPeriod = -1;
bool buzzerState;
void buzzerLoop() {
  if(buzzerPeriod < 0) {
    noTone(BUZZER);     
  }
  else {
    if(millis() - lastBuzzerTime > buzzerPeriod) {
      lastBuzzerTime = millis();
      if(buzzerState) {
        noTone(BUZZER);
      }
      else {
        tone(BUZZER, 1000);
      }
      buzzerState = buzzerState?false:true;
    }
  }
}


#define CANCEL_TIMEOUT 5000

void loop() {
  
  buzzerLoop();
  gpsLoop();
  imuLoop();
  btnLoop();
  
  
  if(hasFallen) {
    digitalWrite(RED_LED, HIGH);
    buzzerPeriod = 500;
  }
  else {
    digitalWrite(RED_LED, LOW);
    buzzerPeriod = -1;
  }

  
  if(hasFallen && millis() - fallenTime > CANCEL_TIMEOUT) {
    hasFallen = 0;
    sendSMS("Fall detected", "+306975911841");
  }
}