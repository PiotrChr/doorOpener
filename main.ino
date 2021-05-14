#include <CircularBuffer.h>
#include <Servo.h>
#include <Wire.h>

# define I2C_SLAVE_ADDRESS 11

Servo doorServo;  

int pos = 0;    
const int servoPin = 9;
const int redLedPin = 8;
const int greenLedPin = 7;
const int buttonPin = 3;
const int remoteButtonPin = 5;

int redLedState = HIGH;
int greenLedState = LOW;
int buttonState;
int lastButtonState = LOW;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 150; 
unsigned long lastButtonLockTime = millis();
const unsigned long buttonLockTime = 3000;

bool locked = false;

int servoState = LOW;
const int servoStartPos = 0;
const int servoMinSwing = 0;
const int servoMaxSwing = 120;
const int stepDelay = 5;

const int piMessageSize = 2;
const int piMessageOpenDoor = 100;
const int piMessageCloseDoor = 1001;

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
//  Serial.begin(9600);
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  // pinMode(servoPin, OUTPUT)
  setupI2c();
  bootServo();
}

void stopServo() {
  doorServo.detach();
  }

void startServo() {
  if (doorServo.attached() == false) {
      doorServo.attach(servoPin);
    }
  }

void bootServo() {
  startServo();
  
  delay(1000);
  doorServo.write(servoStartPos);
  delay(2000);

  stopServo();
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
  if (reading == false) {
    reading = digitalRead(buttonPin);
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
    for (int pos = servoStartPos; pos <= servoMaxSwing + servoStartPos; pos += 1) {
      doorServo.write(pos);
      delay(stepDelay);
      }
  } else {
    for (int pos = servoMaxSwing+servoStartPos; pos >= servoStartPos; pos -= 1) {
      doorServo.write(pos);
      delay(stepDelay);
      } 
    }
    
    servoState = state;
}

void buttonPressed() {
  piButtonPressed = false;
  
  unsigned long now = millis();
  addTask("StartServo", (now), []() {startServo();});
  addTask("ServoHigh", (now+1000), []() {toggleServo(HIGH);});
  addTask("GreenOn", (now+2000), []() {toggleLed("green");});
  addTask("RedOn", (now+5000), []() {toggleLed("red");});
  addTask("ServoLow", (now+5500), []() {toggleServo(LOW);});
  addTask("StopServo", (now+7000), []() {stopServo();});
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
