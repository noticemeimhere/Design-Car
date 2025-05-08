#include <MotorDriver.h>
#include <Arduino.h>

//#define PID_GAMING

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

const int buzzer{7};

float distance{};
float duration{};

int lastTurned {0};


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


void setup()
{
  Serial.begin(9600);
  
  pinMode(lineL, INPUT);
  pinMode(lineC, INPUT);
  pinMode(lineR, INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(buzzer,OUTPUT);

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
  Serial.println(distance);
  tone(buzzer, (distance*10));

  if(distance > 300)
  {
    distance = 300;
  }

  if (distance < 10)
  {
    stop1();
    Serial.println("holy squid games batman!! theres an obstacle!!!!!!!");
    tone(buzzer, 1000);
    delay(100);
    noTone(buzzer);
    delay(100);
    tone(buzzer, 1000);
    delay(100);
    noTone(buzzer);
    delay(100);
    tone(buzzer, 1000);
    delay(100);
    noTone(buzzer);
    delay(100);
    tone(buzzer,3000);
    delay(1000);
    noTone(buzzer);
    for(int i{}; i < 700; i ++)
    {
      tone(buzzer,i);
      delay(10);
    }
    noTone(buzzer);
    while(true);
  }
  
  #ifndef PID_GAMING
  if ((lineLn == 0 && lineCn == 1 && lineRn == 0) || (lineLn == 1 && lineCn == 1 && lineRn == 1))
  {
    Serial.println("Going forward");
    goForward();
    lastTurned = 1;
  }
  else if ((lineLn == 1 && lineCn == 1 && lineRn == 0) || (lineLn == 1 && lineCn == 0 && lineRn == 0))
  {
    Serial.println("Turning right");
    goRight();
    lastTurned = 1;
  }
  else if ((lineLn == 0 && lineCn == 1 && lineRn == 1))
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