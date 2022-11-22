#include <Arduino.h>

#include "Constants.h"
#include "SineStepper.h"
#include "SineStepperController.h"
#include "Queue.h"
#include "MoveBatch.h"
#include "RobotArmIK.h"
#include "Encoder.h"
#include "IK_Control.h"


enum Mode
{
    doingControlledMovements,
    error
};

Mode currentMode = doingControlledMovements;

Encoder Encoder1(ROTARY_ENC_1_A, ROTARY_ENC_1_B);

SineStepper sineStepper1(STEPPER1_STEP_PIN, STEPPER1_DIR_PIN, /*id:*/ 0);
SineStepper sineStepper2(STEPPER2_STEP_PIN, STEPPER2_DIR_PIN, /*id:*/ 1);
SineStepper sineStepper3(STEPPER3_STEP_PIN, STEPPER3_DIR_PIN, /*id:*/ 2);
SineStepper sineStepper4(STEPPER4_STEP_PIN, STEPPER4_DIR_PIN, /*id:*/ 3);
SineStepperController sineStepperController(/*endlessRepeat:*/ false);
RobotArmIK robotArmIK(43.0, 60.0, 70.0, 54.0);
IK_Control IKControl(55.3, 83.0, 83.0, 49.5);
MoveBatch mb;


int buttonCoolDownCounter = 0;
int StepsPerRevolution = 2048;
int controlCommand[4] = {1024, 0, 0, -256}; //position command for four motors
String inputStr;
String arrayStr[4] = {"","","",""}; // initialize a string array for splitting
bool addFlag = false;

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile SemaphoreHandle_t timerSemaphore;


void handleModeChange(Mode newMode)
{
    if (buttonCoolDownCounter < BUTTON_COOLDOWN_CYCLES)
    {
        buttonCoolDownCounter++;
    }
    if (digitalRead(BUTTON_PIN) && buttonCoolDownCounter >= BUTTON_COOLDOWN_CYCLES)
    {
        buttonCoolDownCounter = 0;
        currentMode = newMode;
    }
}

void IRAM_ATTR onTimer()
{
    digitalWrite(EXECUTING_ISR_CODE, HIGH);

    switch (currentMode)
    {
    case doingControlledMovements:
        portENTER_CRITICAL_ISR(&timerMux);
        sineStepperController.update();
        portEXIT_CRITICAL_ISR(&timerMux);
        break;
    default:
        break;
    }
    xSemaphoreGiveFromISR(timerSemaphore, NULL);
    digitalWrite(EXECUTING_ISR_CODE, LOW);
}

void drillingUpElbowUpAndDown(MoveBatch mb)
{
    mb = robotArmIK.runIK(50.0, 190.0, M_PI, mb);
    mb.addMove(/*id:*/ 3, /*pos:*/ 0);
    sineStepperController.addMoveBatch(mb);

    mb = robotArmIK.runIK(50.0, 170.0, M_PI, mb);
    mb.addMove(/*id:*/ 3, /*pos:*/ 1024);
    sineStepperController.addMoveBatch(mb);

    mb = robotArmIK.runIK(50.0, 190.0, M_PI, mb);
    mb.addMove(/*id:*/ 3, /*pos:*/ 0);
    sineStepperController.addMoveBatch(mb);

    mb = robotArmIK.runIK(50.0, 190.0, M_PI, mb, false);
    mb.addMove(/*id:*/ 3, /*pos:*/ 0);
    sineStepperController.addMoveBatch(mb);

    mb = robotArmIK.runIK(50.0, 170.0, M_PI, mb, false);
    mb.addMove(/*id:*/ 3, /*pos:*/ 1024);
    sineStepperController.addMoveBatch(mb);

    mb = robotArmIK.runIK(50.0, 190.0, M_PI, mb, false);
    mb.addMove(/*id:*/ 3, /*pos:*/ 0);
    sineStepperController.addMoveBatch(mb);
}

void lookingUpAndDown(MoveBatch mb)
{
    mb = robotArmIK.runIK(134.0, 60.0, M_PI_2 + 0.4, mb);
    sineStepperController.addMoveBatch(mb);

    mb = robotArmIK.runIK(144.0, 94.0, M_PI_2, mb);
    sineStepperController.addMoveBatch(mb);

    mb = robotArmIK.runIK(144.0, 94.0, M_PI_2 - 0.4, mb);
    sineStepperController.addMoveBatch(mb);

    mb = robotArmIK.runIK(144.0, 94.0, M_PI_2 + 0.4, mb);
    sineStepperController.addMoveBatch(mb);
}

void drillingLeft(MoveBatch mb)
{
    mb = robotArmIK.runIK(124.0, 74.0, M_PI_2, mb);
    mb.addMove(/*id:*/ 3, /*pos:*/ 0);
    sineStepperController.addMoveBatch(mb);

    mb = robotArmIK.runIK(144.0, 74.0, M_PI_2, mb);
    mb.moveDuration = 2.5;
    mb.addMove(/*id:*/ 3, /*pos:*/ 2048);
    sineStepperController.addMoveBatch(mb);

    mb = robotArmIK.runIK(124.0, 74.0, M_PI_2, mb);
    mb.moveDuration = 2.5;
    mb.addMove(/*id:*/ 3, /*pos:*/ 0);
    sineStepperController.addMoveBatch(mb);

    mb.moveDuration = 1.7;
}

void slidingAlongFloor(MoveBatch mb)
{
    mb = robotArmIK.runIK(94.0, 10.0, 0.0, mb);
    sineStepperController.addMoveBatch(mb);

    mb = robotArmIK.runIK(74.0, 10.0, 0.0, mb);
    sineStepperController.addMoveBatch(mb);

    mb = robotArmIK.runIK(94.0, 10.0, 0.0, mb);
    sineStepperController.addMoveBatch(mb);
}

void repeatabilityTest(MoveBatch mb)
{
    // get into ready position.
    mb = robotArmIK.runIK(120.0, 90.0, M_PI_2, mb);
    sineStepperController.addMoveBatch(mb);

    // down -> up -> down.
    mb = robotArmIK.runIK(140.0, 60.0, M_PI_2, mb);
    sineStepperController.addMoveBatch(mb);

    mb = robotArmIK.runIK(140.0, 90.0, M_PI_2, mb);
    sineStepperController.addMoveBatch(mb);

    mb = robotArmIK.runIK(140.0, 120.0, M_PI_2, mb);
    sineStepperController.addMoveBatch(mb);

    // stop for measurement
    mb = robotArmIK.runIK(140.0, 120.0, M_PI_2, mb);
    mb.moveDuration = 0.5;
    sineStepperController.addMoveBatch(mb);
    mb.moveDuration = 1.75;

    mb = robotArmIK.runIK(140.0, 90.0, M_PI_2, mb);
    sineStepperController.addMoveBatch(mb);

    mb = robotArmIK.runIK(140.0, 60.0, M_PI_2, mb);
    sineStepperController.addMoveBatch(mb);

    // get into second ready position
    mb = robotArmIK.runIK(85.0, 40.0, 0.0, mb);
    sineStepperController.addMoveBatch(mb);

    // near -> far -> near
    mb = robotArmIK.runIK(75.0, 20.0, 0.0, mb);
    sineStepperController.addMoveBatch(mb);

    mb = robotArmIK.runIK(95.0, 20.0, 0.0, mb);
    sineStepperController.addMoveBatch(mb);

    mb = robotArmIK.runIK(115.0, 20.0, 0.0, mb);
    sineStepperController.addMoveBatch(mb);

    mb = robotArmIK.runIK(95.0, 20.0, 0.0, mb);
    sineStepperController.addMoveBatch(mb);

    mb = robotArmIK.runIK(75.0, 20.0, 0.0, mb);
    sineStepperController.addMoveBatch(mb);

    // stop for measurement
    mb = robotArmIK.runIK(75.0, 20.0, 0.0, mb);
    mb.moveDuration = 0.5;
    sineStepperController.addMoveBatch(mb);
    mb.moveDuration = 1.75;

    // get into second ready position
    mb = robotArmIK.runIK(85.0, 40.0, 0.0, mb);
    sineStepperController.addMoveBatch(mb);
}

void jointTesting(MoveBatch mb){
    mb.addMove(/*id:*/ 0, /*pos:*/ 1024); //Motor 1:half-step & others:full-step
    mb.addMove(/*id:*/ 1, /*pos:*/ 128);
    mb.addMove(/*id:*/ 2, /*pos:*/ 256);
    mb.addMove(/*id:*/ 3, /*pos:*/ -256);
    mb.moveDuration = 5;
    sineStepperController.addMoveBatch(mb);

    mb.addMove(/*id:*/ 0, /*pos:*/ 0);
    mb.addMove(/*id:*/ 1, /*pos:*/ -128);
    mb.addMove(/*id:*/ 2, /*pos:*/ -256);
    mb.addMove(/*id:*/ 3, /*pos:*/ 256);
    mb.moveDuration = 5;
    sineStepperController.addMoveBatch(mb);
}

void serialControl(MoveBatch mb)
{   
    mb.addMove(/*id:*/ 0, /*pos:*/ controlCommand[0]);
    mb.addMove(/*id:*/ 1, /*pos:*/ controlCommand[1]);
    mb.addMove(/*id:*/ 2, /*pos:*/ controlCommand[2]);
    mb.addMove(/*id:*/ 3, /*pos:*/ controlCommand[3]);
    sineStepperController.addMoveBatch(mb);
}

void runIKTest(MoveBatch mb)
{
    mb = IKControl.runIKControl(120.0, -90.0, 60.0, mb);
    // mb.moveDuration = 3;
    Serial.println("Moving to the pose1...");
    sineStepperController.addMoveBatch(mb);
    
    mb = IKControl.runIKControl(120.0, -90.0, 40.0, mb);
    mb.moveDuration = 1;
    Serial.println("Moving to the pose2...");
    sineStepperController.addMoveBatch(mb);

    mb = IKControl.runIKControl(120.0, -90.0, 60.0, mb);
    // mb.moveDuration = 1;
    Serial.println("Moving to the pose3...");
    sineStepperController.addMoveBatch(mb);
    
}

void setup()
{
    Serial.begin(19200); //boundrate 115200

    pinMode(EXECUTING_ISR_CODE, OUTPUT);
    pinMode(BUTTON_PIN, INPUT);

    timerSemaphore = xSemaphoreCreateBinary();
    sineStepperController.attach(&sineStepper1);
    sineStepperController.attach(&sineStepper2);
    sineStepperController.attach(&sineStepper3);
    sineStepperController.attach(&sineStepper4);

    // initialize MoveBatches
    mb.moveDuration = 3;

    //repeatabilityTest(mb);
    //drillingUpElbowUpAndDown(mb);
    //lookingUpAndDown(mb);
    //drillingLeft(mb);
    //slidingAlongFloor(mb);
    // jointTesting(mb);
    // serialControl(mb);
    runIKTest(mb);


    // go back to gome position START
    //mb.moveDuration = 1.7;
    //mb.addMove(/*id:*/ 0, /*pos:*/ 0);
    //mb.addMove(/*id:*/ 1, /*pos:*/ 0);
    //mb.addMove(/*id:*/ 2, /*pos:*/ 0);
    //mb.addMove(/*id:*/ 3, /*pos:*/ 0);
    //sineStepperController.addMoveBatch(mb);
    //sineStepperController.addMoveBatch(mb);
    // go back to gome position END

    if (robotArmIK.nanErrorOccured)
    {
        currentMode = error;
    }

    // Set 80 divider for prescaler
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    // onTimer gets called every 100uS.
    timerAlarmWrite(timer, 100, true);
    timerAlarmEnable(timer);
}

// Split the info received from Serial into several spring
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for (int i=0; i<=maxIndex && found<=index; i++)
  {
    if (data.charAt(i)==separator || i==maxIndex)
    {
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void loop()
{
    if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE)
    {
        int32_t pos1 = 0;

        portENTER_CRITICAL(&timerMux);
        pos1 = sineStepper1.currentPos;
        if(addFlag)
        {
            sineStepperController.addMoveBatch(mb);
            addFlag = false;
            Serial.println("Received Control Update!");
        }
        portEXIT_CRITICAL(&timerMux);

        // Serial.print("pos1:");
        // Serial.println(pos1);

        // delay(10);
        if(Serial.available()) //if the uart is not NULL
        {
            inputStr = Serial.readString(); //update input string
            addFlag = true;
            for(int i = 0; i<=3 ; i++){
                arrayStr[i] = getValue(inputStr,',',i);
                controlCommand[i] = arrayStr[i].toInt();
                mb.addMove(/*id:*/ i, /*pos:*/ controlCommand[i]);
            }
        }        

        delay(10);
        
    }
}
