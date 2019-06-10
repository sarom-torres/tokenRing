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
BaseType_t xTimerSignalStarted = NULL;
BaseType_t xTimerPeriodicStarted = NULL;


void setup() {
  Serial.begin(9600);
  pinMode(writerPin,OUTPUT);
  digitalWrite(writerPin,HIGH);
  xTimerPeriodic = xTimerCreate("Frame",1000/ portTICK_PERIOD_MS ,pdTRUE,0,messageGenerator); // Cria um timer periodico para envio da mensagem
  xTimerSignal = xTimerCreate("Start",1000/portTICK_PERIOD_MS,pdFALSE,0,startMessage); // Cria uma oneShot para iniciar a transmissão da mensagem
  xTimerSignalStarted = xTimerStart(xTimerSignal,0); // inicia o one_shot
}

void loop() {

}

void messageGenerator (TimerHandle_t xTimerPeriodic){ //
  Serial.println("Message");
  if(digitalRead(writerPin) == HIGH) digitalWrite(writerPin,LOW);
  else digitalWrite(writerPin,HIGH);
  countBit++;
  if(countBit == 8){
    xTimerStop(xTimerPeriodic,0); // Verifica se já foram transmitidos 8 bits
    digitalWrite(writerPin,HIGH);
  }
  Serial.println(countBit);
}

void startMessage(TimerHandle_t xTimerSignal){
  Serial.println("One Shot");
  
  xTimerPeriodicStarted = xTimerStart(xTimerPeriodic,0);// inicia o timer periódico
}
