#include <MotorDriver.h>
#include <Arduino.h>

MotorDriver motor;

const int motorL{1};
const int motorR{0};

const int lineL{2};
const int lineC{3};
const int lineR{4};

int lineLn{};
int lineCn{};
int lineRn{};

const int trigPin{5};
const int echoPin{6};

float distance{};
float duration{};

const void goForward()
{
  motor.speed(0,-75);
  motor.speed(1,-75);
}

const void goLeft()
{
  motor.stop(motorL);
  motor.speed(motorR,-75);
}

const void goRight()
{
  motor.speed(motorL,-75);
  motor.stop(motorR);
}

const void stop()
{
  motor.stop(motorL);
  motor.stop(motorR);
}

void setup()
{
  Serial.begin(9600);
  
  pinMode(lineL, INPUT);
  pinMode(lineC, INPUT);
  pinMode(lineR, INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  motor.begin();
}

void loop() 
{
  lineLn = digitalRead(lineL);
  lineCn = digitalRead(lineC);
  lineRn = digitalRead(lineR);

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  
  Serial.print("Distance: ");
  Serial.println(distance);
  
  Serial.print("lineL: "); Serial.println(lineLn);
  Serial.print("lineC: "); Serial.println(lineCn);
  Serial.print("lineR: "); Serial.println(lineRn);

  if (distance < 10)
  {
    stop();
    Serial.println("holy squid games batman!! theres an obstacle!!!!!!!");
    while(true); //I HATE VOID LOOP THEREFORE I WILL CAST INFINITE WHILE LOOP!!!!!
  }
  
  if (lineLn == 1 && lineCn == 0 && lineRn == 1)
  {
    Serial.println("Going forward");
    goForward();
  }
  else if ((lineLn == 0 && lineCn == 1 && lineRn == 1) || (lineLn == 0 && lineCn == 0 && lineRn == 1))
  {
    Serial.println("Turning left");
    goLeft();
  }
  else if ((lineLn == 1 && lineCn == 1 && lineRn == 0) || (lineLn == 1 && lineCn == 0 && lineRn == 0))
  {
    Serial.println("Turning right");
    goRight();
  }
  else if (lineLn == 1 && lineCn == 1 && lineRn == 1)
  {
    Serial.println("im like 90 percent sure we have finished the line"); //we will stop just incase because i dont want him to spin for no reason
    stop();
  }
  else{
    Serial.println("what the hell im just going to spin until i find a valid config");
    goLeft();
  }

  //delay(20); //can be removed if we need to but this is just incase 
}