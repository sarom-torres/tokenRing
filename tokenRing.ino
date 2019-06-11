#include <Arduino_FreeRTOS.h>
#include <FreeRTOSVariant.h>
#include <task.h>
#include <semphr.h>
#include <timers.h>

const byte writerPin = 13;
const byte readerPin = 2;

uint32_t countBit = 0; // contador de bits transmitidos
byte dataSend = B00000111;
byte dataReceived;

TimerHandle_t xTimerPeriodic = NULL;
TimerHandle_t xTimerSignal = NULL;
TimerHandle_t xTimerReader = NULL;
TimerHandle_t xTimerStartReading = NULL;

BaseType_t xTimerSignalStarted = NULL;
BaseType_t xTimerPeriodicStarted = NULL;
BaseType_t xTimerReaderStarted = NULL;
BaseType_t xTimerStartReadingStarted = NULL;

void messageGenerator (TimerHandle_t xTimerPeriodic); // CallBack to writer the mensagem
void startMessage(TimerHandle_t xTimerSignal); //CallBack to initialize the transmission of the message
void TaskCounter(void *pvParameters);

void setup() {
  Serial.begin(9600);

  //pin
  pinMode(writerPin,OUTPUT);
  digitalWrite(writerPin,HIGH);

  //timers create
  xTimerPeriodic = xTimerCreate("Frame",1000/ portTICK_PERIOD_MS ,pdTRUE,0,messageGenerator); // Creates a periodic timer for sending de message
  xTimerSignal = xTimerCreate("Start",1000/portTICK_PERIOD_MS,pdFALSE,0,startMessage); // Creates the oneShot to initialize the transmission of the message
  xTimerReader = xTimerCreate("Received",1000/ portTICK_PERIOD_MS ,pdTRUE,0,readerMessage); // Creates a periodic timer to reader the message
  xTimerStartReading= xTimerCreate("Start",500/portTICK_PERIOD_MS,pdFALSE,0,startReadingMessage); // Creates the oneShot to initialize the reading of the message

  xTaskCreate(TaskCounter, (const portCHAR*)"Counter", 256, NULL, 1, NULL);

  //timers start
  xTimerSignalStarted = xTimerStart(xTimerSignal,0); // Initializes the one_shot

  //interrupt
  attachInterrupt(digitalPinToInterrupt(readerPin), messageInterrupt, FALLING);
}

void loop() {

}

void startMessage(TimerHandle_t xTimerSignal){
  Serial.println("Init Transmission");
  digitalWrite(writerPin,LOW);
  xTimerPeriodicStarted = xTimerStart(xTimerPeriodic,0);// Initializes the timer
  
}

void messageGenerator (TimerHandle_t xTimerPeriodic){ //

  if(dataSend & B00000001){
    digitalWrite(writerPin,HIGH);
  }else{
    digitalWrite(writerPin,LOW);  
  }

  dataSend = dataSend >> 1; 
  countBit++;
  
  Serial.print("Send ");
  Serial.println(countBit);
  
  if(countBit == 8){
    xTimerStop(xTimerPeriodic,0); // Checks if already transmited 8 bits then stop the timer
    digitalWrite(writerPin,HIGH); //Sets the port in HIGH after the transmission
    countBit=0;
  }
}



void readerMessage(TimerHandle_t xTimerReader){
  if(digitalRead(readerPin) == HIGH) Serial.println("Received HIGH");
  else Serial.println("Received LOW");
}

void startReadingMessage(TimerHandle_t xTimerStartReading){
  if(digitalRead(readerPin) == LOW){
    xTimerReaderStarted = xTimerStart(xTimerReader,0); // Initializes the timer to reader the message
    Serial.println("Received StartBit");
  }else{
    attachInterrupt(digitalPinToInterrupt(readerPin), messageInterrupt, FALLING);
    Serial.println("Falso start");
  }
  
}

void messageInterrupt(void){
  detachInterrupt(digitalPinToInterrupt(readerPin));
  Serial.println("INT0");
  xTimerStartReadingStarted= xTimerStart(xTimerStartReading,0);
}

void TaskCounter(void *pvParameters) {

  (void) pvParameters;
  for (;;);
}
