#include <Arduino_FreeRTOS.h>
#include <FreeRTOSVariant.h>
#include <task.h>
#include <semphr.h>
#include <timers.h>

const byte writerPin = 13;
const byte readerPin = 2;

uint32_t countBit = 0; // contador de bits transmitidos

TimerHandle_t xTimerPeriodic = NULL;
TimerHandle_t xTimerSignal = NULL;
TimerHandle_t xTimerReader = NULL;
BaseType_t xTimerSignalStarted = NULL;
BaseType_t xTimerPeriodicStarted = NULL;
BaseType_t xTimerReaderStarted = NULL;

void messageGenerator (TimerHandle_t xTimerPeriodic); // CallBack to writer the mensagem
void startMessage(TimerHandle_t xTimerSignal); //CallBack to initialize the transmission of the message

void setup() {
  Serial.begin(9600);

  //pin
  pinMode(writerPin,OUTPUT);
  digitalWrite(writerPin,HIGH);

  //timers
  xTimerPeriodic = xTimerCreate("Frame",1000/ portTICK_PERIOD_MS ,pdTRUE,0,messageGenerator); // Creates a periodic timer for sending de message
  xTimerSignal = xTimerCreate("Start",1000/portTICK_PERIOD_MS,pdFALSE,0,startMessage); // Creates the oneShot to initialize the transmission of the message
  xTimerReader = xTimerCreate("Received",1000/ portTICK_PERIOD_MS ,pdTRUE,0,readerMessage); // Creates a periodic timer to reader the message
  xTimerSignalStarted = xTimerStart(xTimerSignal,0); // Initializes the one_shot
 

  //interrupt
  attachInterrupt(digitalPinToInterrupt(readerPin), messageInterrupt, FALLING);
}

void loop() {

}

void messageGenerator (TimerHandle_t xTimerPeriodic){ //
  Serial.println("Message");
  if(digitalRead(writerPin) == HIGH) digitalWrite(writerPin,LOW);
  else digitalWrite(writerPin,HIGH);
  countBit++;
  if(countBit == 8){
    xTimerStop(xTimerPeriodic,0); // Checks if already transmited 8 bits then stop the timer
    digitalWrite(writerPin,HIGH); //Sets the port in HIGH after the transmission
  }
  Serial.println(countBit);
}

void startMessage(TimerHandle_t xTimerSignal){
  Serial.println("One Shot");
  digitalWrite(writerPin,LOW);
  xTimerPeriodicStarted = xTimerStart(xTimerPeriodic,0);// Initializes the timer
}

void readerMessage(TimerHandle_t xTimerReader){
  Serial.println("Reading");
  if(digitalRead(readerPin) == HIGH) Serial.println("Received HIGH");
  else Serial.println("Received LOW");
}

void messageInterrupt(void){
  detachInterrupt(readerPin);
  delay(1500);
  xTimerReaderStarted = xTimerStart(xTimerReader,0);
}
