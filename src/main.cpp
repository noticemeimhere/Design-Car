#include <MotorDriver.h>
#include <Arduino.h>

#define PID_GAMING

MotorDriver motor;

const int motorL{0};
const int motorR{1};

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

int lastTurned {0};

float Kp = 25.0;
float Ki = 0.0;
float Kd = 15.0;

float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;

int baseSpeed = 75; // base speed of motors

#ifndef PID_GAMING
const void goForward()
{
  motor.speed(motorL,0);
  motor.speed(motorR,0);
  motor.speed(0,-75);
  motor.speed(1,-75);
}

const void goLeft()
{
  motor.speed(motorL,0);
  motor.speed(motorR,0);
  motor.brake(motorL);
  motor.speed(motorR,-75);
  motor.speed(motorL,75);
  motor.stop(motorL);
}

const void goRight()
{
  motor.speed(motorL,0);
  motor.speed(motorR,0);
  motor.brake(motorR);
  motor.speed(motorR,75);
  motor.speed(motorL,-75);
  motor.stop(motorR);
}
#endif

const void stop1()
{
 motor.brake(motorR);
 motor.brake(motorL);
 motor.speed(motorL,0);
 motor.speed(motorR,0);
 motor.stop(motorL);
 motor.stop(motorR);
}


#ifdef PID_GAMING
int getLineError() {
  int left = digitalRead(lineL);
  int center = digitalRead(lineC);
  int right = digitalRead(lineR);

  if (left == 1 && center == 0 && right == 0) return -2;
  if (left == 1 && center == 1 && right == 0) return -1;
  if (left == 0 && center == 1 && right == 0) return 0;
  if (left == 0 && center == 1 && right == 1) return 1;
  if (left == 0 && center == 0 && right == 1) return 2;
  // Lost line
  return 0;
}
#endif


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
  delay(0.002);
  digitalWrite(trigPin, HIGH);
  delay(.010);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  if(distance > 300)
  {
    distance = 300;
  }
  
  #ifdef PID_GAMING
  Serial.print("Distance: ");
  Serial.println(distance);
  
  Serial.print("lineL: "); Serial.println(lineLn);
  Serial.print("lineC: "); Serial.println(lineCn);
  Serial.print("lineR: "); Serial.println(lineRn);

  
  error = getLineError();
  integral += error;
  derivative = error - previousError;
  previousError = error;

  float correction = Kp * error + Ki * integral + Kd * derivative;

  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  // Clamp speed values
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  motor.speed(motorL, -leftSpeed);  // negative since your current forward is -75
  motor.speed(motorR, -rightSpeed);
  #endif

  #ifndef PID_GAMING
  if (distance == 9.991)
  {
    stop1();
    Serial.println("holy squid games batman!! theres an obstacle!!!!!!!");
    while(true);
  }

  if (lineLn == 0 && lineCn == 1 && lineRn == 0)
  {
    Serial.println("Going forward");
    goForward();
    lastTurned = 0;
  }
  else if ((lineLn == 1 && lineCn == 1 && lineRn == 0) || (lineLn == 1 && lineCn == 0 && lineRn == 0))
  {
    Serial.println("Turning right");
    goRight();
    lastTurned = 1;
  }
  else if ((lineLn == 0 && lineCn == 1 && lineRn == 1) || (lineLn == 0 && lineCn == 0 && lineRn == 1))
  {
    Serial.println("Turning left");
    goLeft();
    lastTurned = 2;
  }
  else if (lineLn == 0 && lineCn == 0 && lineRn == 0)
  {
    Serial.println("im like 90 percent sure we have finished the line"); //we will stop just incase because i dont want him to spin for no reason
    if(lastTurned == 0)
    {
      #ifndef PID_GAMING
      stop1();
      #endif
    }
    else if(lastTurned==1)
    {
      #ifndef PID_GAMING
      goLeft();
      #endif
    }
    else if(lastTurned==2)
    {
      #ifndef PID_GAMING
      goRight();
      #endif
    }
  }
  else{
    Serial.println("what the hell im just going to spin until i find a valid config");
    goLeft();
  }
  #endif
  //delay(20); //can be removed if we need to but this is just incase 
}