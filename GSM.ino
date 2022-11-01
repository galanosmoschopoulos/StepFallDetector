#define SIM800L_RX 2
#define SIM800L_TX 3
#define SIM800L_BAUD 9600
#include <SoftwareSerial.h>

SoftwareSerial simSerial(SIM800L_RX, SIM800L_TX);

void simInit(String simPin) {
  simSerial.begin(SIM800L_BAUD);
  simSerial.println("AT"); //Once the handshake test is successful, it will back to OK
  delay(200);
  simSerial.println("AT+CMEE=1");
  delay(200);
  simSerial.println("AT+CPIN="+simPin);
  delay(200);
  simSerial.println("AT+CMGF=1");
  delay(200);
  simSerial.println("AT+CBAND=DCS_MODE"); 
  delay(200);
  simSerial.end();
}

extern SoftwareSerial gpsSerial;


bool checkGSM() {
  bool gsmGood = true;

  gpsSerial.end();
  simSerial.begin(SIM800L_BAUD);
  simSerial.println("AT+CCID");
  delay(200);
  String output = simSerial.readString();

  if(output.indexOf("ERROR") != -1) {
    gsmGood = false;
  }

  simSerial.end();
  gpsSerial.begin(GPS_BAUD);
  return gsmGood;
}

void sendSMS(String text, String emergNumber) {
  simSerial.begin(SIM800L_BAUD);
  simSerial.println("AT+CMGS=\"" + emergNumber + "\"");
  delay(500);
  simSerial.print(text);
  delay(500);
  simSerial.write(26);
  simSerial.end();
}
