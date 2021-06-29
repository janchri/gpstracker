/* Blink Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <sdkconfig.h>
#include <MPU9250.h>
#include <TinyGPS++.h>

#define RXD2 16
#define TXD2 17
HardwareSerial gps_serial(2);
TinyGPSPlus gps;
unsigned long last = 0UL;

SPIClass SPI0(VSPI);
MPU9250 IMU(SPI0, 5);
int status;

void setup()
{
    // serial to display data
    Serial.begin(115200);
    while (!Serial)
    {
    }

    // start communication with IMU
    /*status = IMU.begin();
    if (status < 0)
    {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
        while (1)
        {
        }
    }*/

    gps_serial.begin(9600, SERIAL_8N1, RXD2, TXD2);
}

void loop()
{
    while (gps_serial.available())
    {
        gps.encode(gps_serial.read());
    }
    if (gps.location.isUpdated())
    {
        Serial.print(F("LOCATION   Fix Age="));
        Serial.print(gps.location.age());
        Serial.print(F("ms Raw Lat="));
        Serial.print(gps.location.rawLat().negative ? "-" : "+");
        Serial.print(gps.location.rawLat().deg);
        Serial.print("[+");
        Serial.print(gps.location.rawLat().billionths);
        Serial.print(F(" billionths],  Raw Long="));
        Serial.print(gps.location.rawLng().negative ? "-" : "+");
        Serial.print(gps.location.rawLng().deg);
        Serial.print("[+");
        Serial.print(gps.location.rawLng().billionths);
        Serial.print(F(" billionths],  Lat="));
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(" Long="));
        Serial.println(gps.location.lng(), 6);
    }

    if (gps.date.isUpdated())
    {
        Serial.print(F("DATE       Fix Age="));
        Serial.print(gps.date.age());
        Serial.print(F("ms Raw="));
        Serial.print(gps.date.value());
        Serial.print(F(" Year="));
        Serial.print(gps.date.year());
        Serial.print(F(" Month="));
        Serial.print(gps.date.month());
        Serial.print(F(" Day="));
        Serial.println(gps.date.day());
    }

    if (gps.time.isUpdated())
    {
        Serial.print(F("TIME       Fix Age="));
        Serial.print(gps.time.age());
        Serial.print(F("ms Raw="));
        Serial.print(gps.time.value());
        Serial.print(F(" Hour="));
        Serial.print(gps.time.hour());
        Serial.print(F(" Minute="));
        Serial.print(gps.time.minute());
        Serial.print(F(" Second="));
        Serial.print(gps.time.second());
        Serial.print(F(" Hundredths="));
        Serial.println(gps.time.centisecond());
    }

    if (gps.speed.isUpdated())
    {
        Serial.print(F("SPEED      Fix Age="));
        Serial.print(gps.speed.age());
        Serial.print(F("ms Raw="));
        Serial.print(gps.speed.value());
        Serial.print(F(" Knots="));
        Serial.print(gps.speed.knots());
        Serial.print(F(" MPH="));
        Serial.print(gps.speed.mph());
        Serial.print(F(" m/s="));
        Serial.print(gps.speed.mps());
        Serial.print(F(" km/h="));
        Serial.println(gps.speed.kmph());
    }

    if (gps.course.isUpdated())
    {
        Serial.print(F("COURSE     Fix Age="));
        Serial.print(gps.course.age());
        Serial.print(F("ms Raw="));
        Serial.print(gps.course.value());
        Serial.print(F(" Deg="));
        Serial.println(gps.course.deg());
    }

    if (gps.altitude.isUpdated())
    {
        Serial.print(F("ALTITUDE   Fix Age="));
        Serial.print(gps.altitude.age());
        Serial.print(F("ms Raw="));
        Serial.print(gps.altitude.value());
        Serial.print(F(" Meters="));
        Serial.print(gps.altitude.meters());
        Serial.print(F(" Miles="));
        Serial.print(gps.altitude.miles());
        Serial.print(F(" KM="));
        Serial.print(gps.altitude.kilometers());
        Serial.print(F(" Feet="));
        Serial.println(gps.altitude.feet());
    }

    if (gps.satellites.isUpdated())
    {
        Serial.print(F("SATELLITES Fix Age="));
        Serial.print(gps.satellites.age());
        Serial.print(F("ms Value="));
        Serial.println(gps.satellites.value());
    }

    if (gps.hdop.isUpdated())
    {
        Serial.print(F("HDOP       Fix Age="));
        Serial.print(gps.hdop.age());
        Serial.print(F("ms raw="));
        Serial.print(gps.hdop.value());
        Serial.print(F(" hdop="));
        Serial.println(gps.hdop.hdop());
    }

    if (millis() - last > 5000)
    {
        Serial.println();
        if (gps.location.isValid())
        {
            static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
            double distanceToLondon =
                TinyGPSPlus::distanceBetween(
                    gps.location.lat(),
                    gps.location.lng(),
                    LONDON_LAT,
                    LONDON_LON);
            double courseToLondon =
                TinyGPSPlus::courseTo(
                    gps.location.lat(),
                    gps.location.lng(),
                    LONDON_LAT,
                    LONDON_LON);

            Serial.print(F("LONDON     Distance="));
            Serial.print(distanceToLondon / 1000, 6);
            Serial.print(F(" km Course-to="));
            Serial.print(courseToLondon, 6);
            Serial.print(F(" degrees ["));
            Serial.print(TinyGPSPlus::cardinal(courseToLondon));
            Serial.println(F("]"));
        }

        Serial.print(F("DIAGS      Chars="));
        Serial.print(gps.charsProcessed());
        Serial.print(F(" Sentences-with-Fix="));
        Serial.print(gps.sentencesWithFix());
        Serial.print(F(" Failed-checksum="));
        Serial.print(gps.failedChecksum());
        Serial.print(F(" Passed-checksum="));
        Serial.println(gps.passedChecksum());

        if (gps.charsProcessed() < 10)
            Serial.println(F("WARNING: No GPS data.  Check wiring."));

        last = millis();
        Serial.println();
    }
    delay(100);
    /*
    // read the sensor
    IMU.readSensor();
    // display the data
    Serial.print("Acc X: ");
    Serial.println(IMU.getAccelX_mss(), 6);
    Serial.print("Acc Y: ");
    Serial.println(IMU.getAccelY_mss(), 6);
    Serial.print("Acc Z: ");
    Serial.println(IMU.getAccelZ_mss(), 6);
    Serial.print("Gyro X: ");
    Serial.println(IMU.getGyroX_rads(), 6);
    Serial.print("Gyro Y: ");
    Serial.println(IMU.getGyroY_rads(), 6);
    Serial.print("Gyro Z: ");
    Serial.println(IMU.getGyroZ_rads(), 6);
    Serial.print("Mag X: ");
    Serial.println(IMU.getMagX_uT(), 6);
    Serial.print("Mag Y: ");
    Serial.println(IMU.getMagY_uT(), 6);
    Serial.print("Mag Z: ");
    Serial.println(IMU.getMagZ_uT(), 6);
    Serial.print("Temp: ");
    Serial.println(IMU.getTemperature_C(), 6);*/
}