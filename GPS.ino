#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define GPS_BAUD 9600
#define GPS_RX 4
#define GPS_TX 5

double gpsLongtitude;
double gpsLatitude;

bool coordsValid = false; // Holds if the chords in gpsLongtitude and gpsLatitude are valid
bool coordsUpToDate = true;

TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

String getCoords() {
  if (coordsValid && coordsUpToDate) {
    return String(gpsLongtitude) + " " + String(gpsLatitude);
  }
  else if (coordsValid) {
    return String(gpsLongtitude) + " " + String(gpsLatitude) + " (coordinates may be outdated)";
  }
  else {
    return "Location not available";
  }
}

void gpsInit() {
  gpsSerial.begin(GPS_BAUD);
}
#define MAX_GPS_HANG_TIME 3000
#define GPS_LOOP_PERIOD 200
unsigned long lastCoordsUpdate = 0;
unsigned long lastGpsLoopTime = 0;
void gpsLoop() {
  if (millis() - lastGpsLoopTime >= GPS_LOOP_PERIOD) {
    lastGpsLoopTime = millis();
  while (gpsSerial.available() > 0) {
    if(gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        gpsLongtitude = gps.location.lng();
        gpsLatitude = gps.location.lat();
        coordsValid = true;
        coordsUpToDate = true;
        lastCoordsUpdate = millis();
      }
    }
  }
  if(millis() - lastCoordsUpdate > MAX_GPS_HANG_TIME) {
    coordsUpToDate = false;
  }
  }
}
