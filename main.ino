#include <CircularBuffer.h>
#include <Servo.h>
#include <Wire.h>

# define I2C_SLAVE_ADDRESS 11

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

int piMessageSize = 2;
int piMessageOpenDoor = 100;
int piMessageCloseDoor = 1001;

bool piButtonPressed = false;

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
  Serial.begin(9600);
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  setupI2c();
  resetServo();
}

void resetServo() {
  doorServo.write(0);
  delay(2000);
  }

void setupI2c() {
  Wire.begin(I2C_SLAVE_ADDRESS);
  delay(1000);
//  Wire.onRequest(i2cRequest);
  Wire.onReceive(i2cReceive);
  }

void i2cReceive(int howMany) {
  if (howMany != piMessageSize) {
    return;
    }
  
  for (int i = 0; i < howMany; i++) {
    byte c = Wire.read();
    if (i == piMessageSize - 1) {
        handleI2cMessage(c);
      }
    }
  }

void handleI2cMessage(int message) {
  if (message==piMessageOpenDoor) {
      piButtonPressed = true;
    }
  } 

void i2cRequest() {
    
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
  int reading = piButtonPressed;
  
  if (!reading) {
    int reading = digitalRead(buttonPin);
    }
  
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
  piButtonPressed = false;
  
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
