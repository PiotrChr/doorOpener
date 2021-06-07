#include <CircularBuffer.h>
#include <Wire.h>
#include <AccelStepper.h>

# define I2C_SLAVE_ADDRESS 11

const int stepperEnPin = 2;
const int stepPin = 3;
const int dirPin = 4;
int stepperState = LOW;
int pos = 0;
const int stepperMinSwing = 0;
const int stepperMaxSwing = 120;
const int stepDelayMs = 500;
const int fullCyclePulses = 200;

float motorMaxSpeed = 1000.0;
float motorSpeed = 1000.0;   //pulse per second
float motorAccel = 1000.0; //87500; //steps/second/second to accelerate
AccelStepper stepper(1, stepPin, dirPin); 

const int redLedPin = 8;
const int greenLedPin = 7;
const int buttonPin = 6;
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

const int piMessageSize = 2;
const int piMessageOpenDoor = 100;
const int piMessageCloseDoor = 1001;

bool piButtonPressed = false;

typedef void (*Job)();

struct Task {
  String name;
  unsigned long dueDate;
  Job job;
};

const int taskLimit = 20;
CircularBuffer <Task, taskLimit> tasks;

void setup() {
    Serial.begin(9600);
  setupButton();
  setupLeds();
  setupStepper();
  setupI2c();
}

void setupButton() {
  pinMode(buttonPin, INPUT);
  }

void setupLeds() {
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  }

void setupStepper() {
//   pinMode(stepPin, OUTPUT);
   pinMode(stepperEnPin, OUTPUT);
//   pinMode(dirPin, OUTPUT);
    
    stepper.setMaxSpeed(motorMaxSpeed);
    stepper.setSpeed(motorSpeed);
    stepper.setAcceleration(motorAccel);

    stepper.setCurrentPosition(0);
    delay(2000);
  }

void stepperOn() {
  stepperState = HIGH;
  }

void stepperOff() {
  stepperState = LOW;
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
  if (message == piMessageOpenDoor) {
    piButtonPressed = true;
  }
}

void i2cRequest() {

}

void resetStepper() {
  stepper.setCurrentPosition(0);  
}

void addTask(String name, unsigned long dueDate, void job ()) {
  struct Task task = {name, dueDate, job};
  tasks.push(task);
}

void performTasks() {
  for (int i = 0; i < tasks.size(); i++) {
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

void stepperDir(String dir = "right") {
  if (dir == "right") {
    digitalWrite(dirPin,HIGH);
    } else {
      digitalWrite(dirPin,LOW);
      }
  }

void moveStepper(float rotation, String dir, int relSpeed = 1) {
//  stepperDir(dir);

  int pulses = (int)(rotation * fullCyclePulses) * ((dir == "right") ? 1 : -1);
  int delayMs = (int)(stepDelayMs / relSpeed);

  stepper.moveTo(pulses);
  stepper.runToPosition();
//  for(int x = 0; x < pulses; x++) {
//    digitalWrite(stepPin,HIGH); 
//    delayMicroseconds(stepDelayMs); 
//    digitalWrite(stepPin,LOW); 
//    delayMicroseconds(delayMs); 
//    }
  }

void buttonPressed() {
  piButtonPressed = false;

  unsigned long now = millis();
  addTask("StartStepper", (now), []() {
    resetStepper();
    stepperOn();
  });
  addTask("MoveLeft", (now + 1000), []() {
    moveStepper(0.9, "left", 1);
  });
  addTask("GreenOn", (now + 2000), []() {
    toggleLed("green");
  });
  addTask("RedOn", (now + 5000), []() {
    toggleLed("red");
  });
//  addTask("MoveRight", (now + 5500), []() {
//    moveStepper(0.5, "right", 1);
//  });
  addTask("StopStepper", (now + 6500), []() {
    stepperOff();
  });
}

void checkStepper() {
  digitalWrite(stepperEnPin, stepperState == LOW ? HIGH : LOW);
  }

void checkLeds() {
  digitalWrite(greenLedPin, greenLedState);
  digitalWrite(redLedPin, redLedState);
}

void loop() {
  checkButton();
  checkLeds();
  checkStepper();

  performTasks();
}
