//ENCODER ITERATION
#include <avr/interrupt.h>
#include <Timer.h>

//Motor pins
#define IN1 12  // Right motor direction
#define IN2 11
#define IN3 7 // Left motor direction
#define IN4 6
#define ENA 10 // Right motor PWM
#define ENB 9  // Left motor PWM

//Encoder test
#define ENCA1 2 //right
#define ENCB1 4 //
#define ENCA2 3 //left
#define ENCB2 5 //
#define readA1 bitRead(PIND,2)//faster than digitalRead()
#define readB1 bitRead(PIND,3)

int r_pos = 0;
int l_pos = 0;


// Ultrasonic sensor pins
const int trigPin = 13;
const int echoPin = 8;
float duration, distance;


void setup() {
  Serial.begin(9600);
  //Motor initialize
  pinMode(IN1, OUTPUT); //right
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); //left
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  //Encoder
  pinMode(ENCA1,INPUT); //Right
  pinMode(ENCB1,INPUT);
  pinMode(ENCA2,INPUT); //Left
  pinMode(ENCB2,INPUT);

  //encoder interrupt
  attachInterrupt(digitalPinToInterrupt(ENCA1), rightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA2), leftEncoder, CHANGE);

  // Initialize ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  driveForward(1.0);
  Serial.print("Right: ");
  Serial.print(r_pos);
  Serial.print("      Left: ");
  Serial.print(l_pos);
  Serial.print("      Distance: ");
  Serial.println(measureDistance());  
  checkObstacle(measureDistance());
}

void rightEncoder() {
  if (digitalRead(ENCA1) == digitalRead(ENCB1)) {
    r_pos++; // Forward
  } else {
    r_pos--; // Reverse
  }
}

// Interrupt routine for the left encoder
void leftEncoder() {
  if (digitalRead(ENCA2) == digitalRead(ENCB2)) {
    l_pos++; // Forward
  } else {
    l_pos--; // Reverse
  }
}

void driveForward(float distance) {
  digitalWrite(IN1, HIGH); //forward right
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 148);

  digitalWrite(IN3, HIGH); //forward left
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 160);
}

void checkObstacle(float distanceCm) {
  Serial.print("checkObstacle function entered, waiting 2 seconds");
  delay(2000);
  if ((distanceCm < 20) && (distanceCm != 0)) {
    Serial.print("distance check determined we need to call stopMotors()");
    delay(2000);
    stopMotors();
  }
}

float measureDistance() {
  long duration;
  float distanceCm;

  // Clear the trigPin by setting it LOW
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Trigger the sensor by setting the trigPin HIGH for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH, 30000); // Timeout after 30ms (for max distance)

  // Calculate the distance in centimeters
  distanceCm = (duration / 2.0) * 0.0343;

  // Handle out-of-range values
  if (duration == 0) {
    distanceCm = 0; // Indicates out of range or no object detected
  }

  return distanceCm;
}

void stopMotors(){
  Serial.print("StopMotors() function entered.");
  delay(2000);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
