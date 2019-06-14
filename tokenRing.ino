#include <Arduino_FreeRTOS.h>
#include <FreeRTOSVariant.h>
#include <task.h>
#include <semphr.h>
#include <timers.h>

const byte writerPin = 13;
const byte readerPin = 2;

uint32_t countBitSend = 0; // contador de bits transmitidos
uint32_t countBitReceived = 0;
//byte dataSend = B00000011;
//byte dataReceived = B00000000;


byte dataSend;
byte dataSend1 = 'A';
byte dataSend2 = 'L';
byte dataSend3 = 'O';
byte dataReceived = B00000000;

bool flagStopSend = false; // Flag for to stop the transmission

TimerHandle_t xTimerPeriodic = NULL;
TimerHandle_t xTimerSignal = NULL;
TimerHandle_t xTimerReader = NULL;
TimerHandle_t xTimerStartReading = NULL;

QueueHandle_t xQueueSend = NULL;
QueueHandle_t xQueueStorage = NULL;

SemaphoreHandle_t xSemaphoreTransmission = NULL;

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
  xTimerPeriodic = xTimerCreate("Frame",1000/ portTICK_PERIOD_MS ,pdTRUE,0,byteGenerator); // Creates a periodic timer for sending de message
  xTimerSignal = xTimerCreate("Start",1000/portTICK_PERIOD_MS,pdFALSE,0,startByte); // Creates the oneShot to initialize the transmission of the message
  xTimerReader = xTimerCreate("Received",1000/ portTICK_PERIOD_MS ,pdTRUE,0,readerByte); // Creates a periodic timer to reader the message
  xTimerStartReading= xTimerCreate("Start",500/portTICK_PERIOD_MS,pdFALSE,0,startReadingByte); // Creates the oneShot to initialize the reading of the message

  //Tasks
  xTaskCreate(TaskSender, (const portCHAR*)"Sender", 256, NULL, 1, NULL);

  //timers start
  //xTimerSignalStarted = xTimerStart(xTimerSignal,0); // Initializes the one_shot

  //Data Queues
  xQueueSend = xQueueCreate(5,sizeof(uint8_t));
  xQueueStorage = xQueueCreate(5,sizeof(uint8_t)); 

  //Semaphore
  xSemaphoreTransmission = xSemaphoreCreateBinary();

  //interrupt
  attachInterrupt(digitalPinToInterrupt(readerPin), messageInterrupt, FALLING);
}

void loop() {

}

/**** Envio das mensagens ****/

void startByte(TimerHandle_t xTimerSignal){
  Serial.println("startMessage: Init Transmission");
  digitalWrite(writerPin,LOW);
  xTimerPeriodicStarted = xTimerStart(xTimerPeriodic,0);// Initializes the timer
}

void byteGenerator (TimerHandle_t xTimerPeriodic){ //

  if (countBitSend == 0) { // For to test the dataSend
      Serial.print("messageGenerator: Dado a ser enviado:");
      Serial.println(dataSend);  
  }

  if(flagStopSend){
    xTimerStop(xTimerPeriodic,0);
    digitalWrite(writerPin,HIGH); //Sets the port in HIGH after the transmission
    countBitSend=0;
    flagStopSend = false;
    xSemaphoreGive(xSemaphoreTransmission);
  }else{
      if(dataSend & B00000001) digitalWrite(writerPin,HIGH); 
      else digitalWrite(writerPin,LOW); 

      dataSend = dataSend >> 1; 
      countBitSend++;
  }

  if(countBitSend == 8)flagStopSend = true;//Checks if already transmited 8 bits
  
}

void carry_Queue(QueueHandle_t xQueueCarry){ //carrega os dados a serem enviados na fila
  
  BaseType_t xStatus = xQueueSendToBack(xQueueCarry,&dataSend1,0);
  BaseType_t xStatus2 = xQueueSendToBack(xQueueCarry,&dataSend2,0);
  BaseType_t xStatus3 = xQueueSendToBack(xQueueCarry,&dataSend3,0);
  
}

void send_message(QueueHandle_t xQueueTemp){ //o envio dos dados que estão na fila de envio

  int sizeQueue = uxQueueMessagesWaitingFromISR(xQueueTemp);
  int cont = 0;

  while(cont < sizeQueue){ //laço para retirar o dado da fila e iniciar a transmissão
    Serial.println("Task Sender: Lendo da fila");
    xQueueReceive(xQueueTemp, &dataSend, portMAX_DELAY);
    xTimerSignalStarted = xTimerStart(xTimerSignal,0);
    xSemaphoreTake(xSemaphoreTransmission,portMAX_DELAY); // Wait the transmission to finish
  }
}

/**** Leitura das mensagens ****/

void readerByte(TimerHandle_t xTimerReader){

  if(digitalRead(readerPin) == HIGH) dataReceived = dataReceived | B10000000;
  else dataReceived = dataReceived | B00000000;

  countBitReceived++; 

  if(countBitReceived == 8){
    xTimerStop(xTimerReader,0); 
    attachInterrupt(digitalPinToInterrupt(readerPin), messageInterrupt, FALLING);
    Serial.print("readerMessage: Dado Recebido: ");
    Serial.println(dataReceived);
    xQueueSendToBack(xQueueStorage,&dataReceived,0);
    dataReceived = B00000000;
    countBitReceived = 0;
  }else{
    dataReceived = dataReceived >> 1;  
  }

  
}

void startReadingByte(TimerHandle_t xTimerStartReading){
  if(digitalRead(readerPin) == LOW){
    xTimerReaderStarted = xTimerStart(xTimerReader,0); // Initializes the timer to reader the message
    Serial.println("startReadingMessage: Received StartBit");
  }else{
    attachInterrupt(digitalPinToInterrupt(readerPin), messageInterrupt, FALLING);
  }
  
}

/*** Interrupções ***/

void messageInterrupt(void){
  detachInterrupt(digitalPinToInterrupt(readerPin));
  xTimerStartReadingStarted= xTimerStart(xTimerStartReading,0);
}

/*** Tarefas ***/

void TaskSender(void *pvParameters) {
  (void) pvParameters;

  carry_Queue(xQueueSend);
  send_message(xQueueSend);

  for(;;);
}

void TaskReceiver(void *pvParameters){
  (void) pvParameters;
  for (;;);
  
}
