#include <CircularBuffer.h>
#include <Servo.h>

Servo doorServo;  

int pos = 0;    
const int servoPin = 9;
const int redLedPin = 8;
const int greenLedPin = 7;
const int buttonPin = 6;
const int remoteButtonPin = 5;

int redLedState = HIGH;
int greenLedState = LOW;
int buttonState;
int lastButtonState = LOW;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 150; 
unsigned long lastButtonLockTime = millis();
unsigned long buttonLockTime = 3000;

bool locked = false;

int servoState = LOW;
int servoMinSwing = 0;
int servoMaxSwing = 90;

typedef void (*Job)();

struct Task{
  String name;
  unsigned long dueDate;
  Job job;
  };

const int taskLimit = 20;
CircularBuffer <Task, taskLimit> tasks;

void setup() {
  doorServo.attach(servoPin);
  
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  
  Serial.begin(9600);
  
  resetServo();
}

void addTask(String name, unsigned long dueDate, void job ()) {
  struct Task task = {name, dueDate, job};
  tasks.push(task);
  }

void performTasks() {
  for (int i=0; i < tasks.size(); i++) {
      if (tasks[i].dueDate < millis()) {
        struct Task task = tasks.shift();
        task.job();
      } else {
        return;
      }   
    }
  }  

void toggleLed(String color) {
  if (color == "red") {
      redLedState = HIGH;
      greenLedState = LOW;
    }

  if (color == "green") {
      redLedState = LOW;
      greenLedState = HIGH;
    }  
  }

void checkButton() {
  int reading = digitalRead(buttonPin);
  int remote = digitalRead(remoteButtonPin);
  
  unsigned long now = millis();

  if ((now - lastButtonLockTime) < buttonLockTime) {
      return;
    }

//  if (reading != lastButtonState) {
//    lastDebounceTime = now;
//    }

  if ((now - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      lastButtonLockTime = now;
      if (buttonState == HIGH) {
        buttonPressed();
        }
      
      }
    }
  }

void resetServo() {
  doorServo.write(0);
  delay(2000);
  }

void toggleServo(int state) {
  if (state == servoState) {
    return;
    }

  if (state == HIGH) {
    for (int pos = 0; pos <= servoMaxSwing; pos += 1) {
      doorServo.write(pos);
      delay(10);
      }
  } else {
    for (int pos = servoMaxSwing; pos >= 0; pos -= 1) {
      doorServo.write(pos);
      delay(10);
      } 
    }
    
    servoState = state;
}

void buttonPressed() {
  unsigned long now = millis();
  addTask("ServoHigh", (now), []() {toggleServo(HIGH);});
  addTask("GreenOn", (now+1000), []() {toggleLed("green");});
  addTask("RedOn", (now+4000), []() {toggleLed("red");});
  addTask("ServoLow", (now+4500), []() {toggleServo(LOW);});
  }

void checkLeds() {
  digitalWrite(greenLedPin, greenLedState);
  digitalWrite(redLedPin, redLedState);
}

void loop() {
  checkButton();
  checkLeds();

  performTasks();
}
