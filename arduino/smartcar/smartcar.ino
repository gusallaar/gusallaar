//This is my iCar 

#include <vector>
#include <string>
#include <MQTT.h>
#include <WiFi.h>
#ifdef __SMCE__
#include <OV767X.h>
#endif

#include <Smartcar.h>

MQTTClient mqtt;
WiFiClient net;

char ssid[] = " ";
char pass[] = " ";
const int fSpeed   = 65;  // 70% of the full speed forward
const int bSpeed   = -50; // 70% of the full speed backward
const int lDegrees = -75; // degrees to turn left
const int rDegrees = 75;  // degrees to turn right

const int GYROSCOPE_OFFSET = 37;

ArduinoRuntime arduinoRuntime;
BrushedMotor leftMotor(arduinoRuntime, smartcarlib::pins::v2::leftMotorPins);
BrushedMotor rightMotor(arduinoRuntime, smartcarlib::pins::v2::rightMotorPins);
DifferentialControl control(leftMotor, rightMotor);
unsigned int pulsesPerMeter = 600;
 
DirectionlessOdometer leftOdometer{ arduinoRuntime,
                                    smartcarlib::pins::v2::leftOdometerPin,
                                    []() { leftOdometer.update(); },
                                    pulsesPerMeter };
DirectionlessOdometer rightOdometer{ arduinoRuntime,
                                     smartcarlib::pins::v2::rightOdometerPin,
                                     []() { rightOdometer.update(); },
                                     pulsesPerMeter };

// This is the initiallazation of the gyroscope
GY50 gyroscope(arduinoRuntime, GYROSCOPE_OFFSET);
SmartCar car(arduinoRuntime, control, gyroscope, leftOdometer, rightOdometer);


const auto oneSecond = 1000UL;
#ifdef __SMCE__
const auto triggerPin = 6;
const auto echoPin = 7;
const auto mqttBrokerUrl = "127.0.0.1";
#else
const auto triggerPin = 33;
const auto echoPin = 32;
const auto mqttBrokerUrl = "192.168.0.40";
#endif
const auto maxDistance = 400;

const int FRONT_IR_PIN = 0;
const int LEFT_IR_PIN  = 1;
const int RIGHT_IR_PIN = 2;
const int BACK_IR_PIN  = 3;

// These are the initiallazation of the accurare IR sensor
GP2Y0A21 frontIR(arduinoRuntime, FRONT_IR_PIN);
GP2Y0A21 leftIR(arduinoRuntime, LEFT_IR_PIN);
GP2Y0A21 rightIR(arduinoRuntime, RIGHT_IR_PIN);
GP2Y0A21 backIR(arduinoRuntime,BACK_IR_PIN);


SR04 front(arduinoRuntime, triggerPin, echoPin, maxDistance);

std::vector<char> frameBuffer;

void setSpeed(int newSpeed){
   car.setSpeed(newSpeed);
   mqtt.publish("/smartcar/info/speed", String(newSpeed));
}

void setup() {
  Serial.begin(9600);
#ifdef __SMCE__
  Camera.begin(QVGA, RGB888, 25);
  frameBuffer.resize(Camera.width() * Camera.height() * Camera.bytesPerPixel());
#endif

  
  
  WiFi.begin(ssid, pass);
  mqtt.begin(mqttBrokerUrl, 1883, net);

  Serial.println("Connecting to WiFi...");
  auto wifiStatus = WiFi.status();
  while (wifiStatus != WL_CONNECTED && wifiStatus != WL_NO_SHIELD) {
    Serial.println(wifiStatus);
    Serial.print(".");
    delay(1000);
    wifiStatus = WiFi.status();
  }

  //////////////////////////////////////////////////////////////////////////////////

  Serial.println("Connecting to MQTT broker");
  while (!mqtt.connect("arduino", "public", "public")) {
    Serial.print(".");
    delay(1000);
  }
  mqtt.subscribe("/smartcar/control/#", 1);
  mqtt.onMessage([](String topic, String message) {
    if (topic == "/smartcar/control/throttle") {
      car.setSpeed(message.toInt());
    } else if (topic == "/smartcar/control/steering") {
      car.setAngle(message.toInt());
    }  else {
      Serial.println(topic + " " + message);
    }
  });
}

/////////////////////////////////////////////////////////////////////////////////

void loop() {
 
  ObstacleAvoid ();
  MovementControl();
  handleInput();
  
  if (mqtt.connected()) {
    mqtt.loop();
    const auto currentTime = millis();
#ifdef __SMCE__
    static auto previousFrame = 0UL;
    if (currentTime - previousFrame >= 60) {
      previousFrame = currentTime;
      Camera.readFrame(frameBuffer.data());
      mqtt.publish("/smartcar/camera", frameBuffer.data(), frameBuffer.size(),
                   false, 0);
      const auto SpeedMeter = String(car.getSpeed());             
      const auto TravDist = String(car.getDistance());
      mqtt.publish("/smartcar/speedMeasure",  SpeedMeter);
      mqtt.publish("/smartcar/TravDist", TravDist); 

    }
#endif
    static auto previousTransmission = 0UL;
    if (currentTime - previousTransmission >= oneSecond) {
      previousTransmission = currentTime;
      const auto distance = String(front.getDistance());
      mqtt.publish("/smartcar/ultrasound/front", distance);
    }
  }
#ifdef __SMCE__
  // This delay is for avoiding the CPU time-over
  delay(1);
#endif
}


//////////////////////////////////////////////////////////////////////////////////

void ObstacleAvoid ()
{
  int distance = front.getDistance();
  if(distance > 0 && distance < 100)
  {
    Serial.println("An obstacle is detected. The car should stop moving... ");
     car.setSpeed(0);
     delay(400);
    Serial.println("Car is stopped. Rotating is in process.");
    car.setSpeed(bSpeed);
    delay(2000);
    car.setAngle (250);
    delay(1000);
    car.setSpeed(fSpeed);
    car.setAngle (0);
    SurroundingCheck();
  }
}

//////////////////////////////////////////////////////////////////////////////////

void SurroundingCheck()
{
  int leftdistance = leftIR.getDistance();
  int rightdistance = rightIR.getDistance();
  int backdistance = backIR.getDistance();

  if(leftdistance > 10 && leftdistance < 60)  //checking obstacle at leftside
{
  Serial.println("An Obstacle is detected on the leftside.");
  delay(400);
  turnright();
}
  else if(rightdistance > 10 && rightdistance < 60) // checking obstacle at rightside
{
  Serial.println("An Obstacle is detected on the leftside.");
   delay(400);
  turnleft();
}
  else if (backdistance > 10 && backdistance < 60) // checking obstacle at back
{
  Serial.println("Detected obstacle at back. ");
   delay(400);
  car.setSpeed(50); // Moving ahead
  
}
}
 
//Basic turning task to the right
void turnright()  
{
  car.setSpeed (30);
  car.setAngle (95);
  delay(2000);
  car.setAngle (0);
  car.setSpeed (50);
}

//Basic turning task to the left
void turnleft() 
{
  car.setSpeed (30);
  car.setAngle (-95);
  delay(2000);
  car.setAngle (0);
  car.setSpeed (50);
}

//////////////////////////////////////////////////////////////////////////////////

void handleInput()
{ // handle serial input if there is any
    if (Serial.available())
    {
        char input = Serial.read(); // read everything that has been received so far and log down
                                    // the last entry
        switch (input)
        {
        case 'a': // rotate counter-clockwise going forward
            car.setSpeed(fSpeed);
            car.setAngle(lDegrees);
            break;
        case 'd': // turn clock-wise
            car.setSpeed(fSpeed);
            car.setAngle(rDegrees);
            break;
        case 'w': // go ahead
            car.setSpeed(fSpeed);
            car.setAngle(0);
            break;
        case 's': // go back
            car.setSpeed(bSpeed);
            car.setAngle(0);
            break;
         case 'c': // checkside
           car.setSpeed(0);
           delay(400);
           SurroundingCheck();
            break;
        default: // if you receive something that you don't know, just stop
            car.setSpeed(0);
            car.setAngle(0);
        }
    }
}

////////////////////////////////////////////////////////////////////////////

void MovementControl()
{
  int leftside = leftIR.getDistance();
  int rightside = rightIR.getDistance();

  //checking left side so the car dont hit while moving.
  if(leftside > 15 && leftside < 30)  
{
  car.setAngle (20);
  delay(200);
  car.setAngle(0);
}
 // checking right side so the car dont hit while moving.
  else if(rightside > 15 && rightside < 30)
{
  car.setAngle (-20);
  delay(200);
  car.setAngle(0);
}

}
