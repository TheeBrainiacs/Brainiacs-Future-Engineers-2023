#include <Wire.h>
#include <Adafruit_TCS34725.h>

Adafruit_TCS34725 tcs;

// Color detection thresholds for orange
const int minRed = 20;
const int maxGreen = 150;
const int maxBlue = 90;
void setup() {
  Serial.begin(9600);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt the program
  }
}

void loop() {
  
  uint16_t clear, red, green, blue;

  tcs.setInterrupt(false); // disable color interrupt-
  tcs.getRawData(&red, &green, &blue, &clear);

  // Print the RGB values
  Serial.print("Red: "); Serial.print(red);
  Serial.print(" Green: "); Serial.print(green);
  Serial.print(" Blue: "); Serial.print(blue);
  Serial.print(" Clear: "); Serial.print(clear);

  // Calculate the color temperature
  float temperature = tcs.calculateColorTemperature(red, green, blue);
  Serial.print(" Color Temp: "); Serial.print(temperature, 2);
  
  // Calculate the lux
  float lux = tcs.calculateLux(red, green, blue);
  Serial.print(" Lux: "); Serial.println(lux, 2);

  // Check if the detected color is orange
  if (red > minRed && green < maxGreen && blue < maxBlue && red > blue && red > green) {
   
    digitalWrite(9, HIGH);
    digitalWrite(10, LOW);
    delay(10);
     Serial.println(digitalRead(9));
  } else if (blue > red && blue > green && clear < 5200 && blue >10 && red<25 ) {
    Serial.println("Detected color: Blue");
    digitalWrite(9, LOW);
    digitalWrite(10, HIGH);
    delay(10);
  } else {
    Serial.println("Detected color: Unknown");
    digitalWrite(9, LOW);
    digitalWrite(10, LOW);
  }
}
