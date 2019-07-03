#include <Arduino_FreeRTOS.h>
#include <FreeRTOSVariant.h>
#include <task.h>
#include <semphr.h>
#include <timers.h>

const byte writerPin = 13;
const byte readerPin = 2;

uint32_t countBitSend = 0; // contador de bits transmitidos
uint32_t countBitReceived = 0;
uint32_t countByteR = 0;

struct Frame{
  byte stx = B00000010; // valor binário tabela ASCII 3
  byte mac = B11011110; //1101->destino(Eq5) 1110->origem
  byte port = B11000000;
  byte data[10] = {245,246,247,248,249,250,251,252,253,254};
  byte bcc = 200;
  byte etx = B00000011; // tabela ASCII
};

struct Package{
  byte mac;
  byte port;
  byte data[10];
};

Frame frame;

byte dataSend;
byte dataReceived; 
byte frameReceived[15];

bool flagStopSend = false; // Flag for to stop the transmission
bool flagIsDestiny = false;

TimerHandle_t xTimerPeriodic = NULL;
TimerHandle_t xTimerSignal = NULL;
TimerHandle_t xTimerReader = NULL;
TimerHandle_t xTimerStartReading = NULL;

QueueHandle_t xQueueSend = NULL;
QueueHandle_t xQueueApp = NULL;

QueueHandle_t xQueueRetransmission = NULL;

SemaphoreHandle_t xSemaphoreTransmission = NULL;
SemaphoreHandle_t xSemaphoreRouting = NULL;
SemaphoreHandle_t xSemaphoreApp = NULL;

BaseType_t xTimerSignalStarted = NULL;
BaseType_t xTimerPeriodicStarted = NULL;
BaseType_t xTimerReaderStarted = NULL;
BaseType_t xTimerStartReadingStarted = NULL;

void messageGenerator (TimerHandle_t xTimerPeriodic); // CallBack to writer the mensagem
void startMessage(TimerHandle_t xTimerSignal); //CallBack to initialize the transmission of the message

void setup() {
  Serial.begin(9600);
  //Serial.println("Setup");

  //pin
  pinMode(writerPin,OUTPUT);
  digitalWrite(writerPin,HIGH);

  //timers create
  xTimerPeriodic = xTimerCreate("Frame",100/ portTICK_PERIOD_MS ,pdTRUE,0,byteGenerator); // Creates a periodic timer for sending de message
  xTimerSignal = xTimerCreate("Start",100/portTICK_PERIOD_MS,pdFALSE,0,startByte); // Creates the oneShot to initialize the transmission of the message
  xTimerReader = xTimerCreate("Received",100/ portTICK_PERIOD_MS ,pdTRUE,0,readerByte); // Creates a periodic timer to reader the message
  xTimerStartReading= xTimerCreate("Start",50/portTICK_PERIOD_MS,pdFALSE,0,startReadingByte); // Creates the oneShot to initialize the reading of the message

  //Tasks
  xTaskCreate(TaskSender, (const portCHAR*)"Sender", 256, NULL, 1, NULL);
  xTaskCreate(TaskReceiver, (const portCHAR*)"Receiver", 256, NULL, 1, NULL);
  xTaskCreate(TaskSenderApp, (const portCHAR*)"App", 256, NULL, 1, NULL);
  xTaskCreate(TaskReceiverApp, (const portCHAR*)"App", 256, NULL, 1, NULL);

  //Data Queues
  xQueueSend = xQueueCreate(15,sizeof(uint8_t));
  xQueueApp = xQueueCreate(2,(sizeof(Package))); // fila para o envio do dado recebido para a aplicação
  xQueueTransmissionApp = xQueueCreate(2,(sizeof(Package))); // fila para a receber dados da aplicação e enviar para outro equipamento

  //Semaphore
  xSemaphoreTransmission = xSemaphoreCreateBinary();
  xSemaphoreRouting = xSemaphoreCreateBinary();
  xSemaphoreApp = xSemaphoreCreateBinary();

  //interrupt
  attachInterrupt(digitalPinToInterrupt(readerPin), messageInterrupt, FALLING);
}

void loop() {

}

/**** Envio das mensagens ****/

void frameCreate(Frame newFrame, Package p1){

  newFrame.stx = B00000010; // valor binário tabela ASCII 3
  newFrame.mac = p1.mac;
  newFrame.port = p1.port;
  for(int i=0; i<10;i++) newFrame.data[i] = p1.data[i];
  newFrame.etx = B00000011; // tabela ASCII
  
}

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

void carry_Queue(QueueHandle_t xQueueCarry, Frame fr1){ //carrega os dados a serem enviados na fila
  
  xQueueSendToBack(xQueueCarry,&fr1.stx,0);
  xQueueSendToBack(xQueueCarry,&fr1.mac,0);
  xQueueSendToBack(xQueueCarry,&fr1.port,0);
  xQueueSendToBack(xQueueCarry,&fr1.data[0],0);
  xQueueSendToBack(xQueueCarry,&fr1.data[1],0);
  xQueueSendToBack(xQueueCarry,&fr1.data[2],0);
  xQueueSendToBack(xQueueCarry,&fr1.data[3],0);
  xQueueSendToBack(xQueueCarry,&fr1.data[4],0);
  xQueueSendToBack(xQueueCarry,&fr1.data[5],0);
  xQueueSendToBack(xQueueCarry,&fr1.data[6],0);
  xQueueSendToBack(xQueueCarry,&fr1.data[7],0);
  xQueueSendToBack(xQueueCarry,&fr1.data[8],0);
  xQueueSendToBack(xQueueCarry,&fr1.data[9],0);
  xQueueSendToBack(xQueueCarry,&fr1.bcc,0);
  xQueueSendToBack(xQueueCarry,&fr1.etx,0);
}

void send_message(QueueHandle_t xQueueTemp){ //o envio dos dados que estão na fila de envio

  int sizeQueue = uxQueueMessagesWaitingFromISR(xQueueTemp);
  int cont = 0;

  while(cont < sizeQueue){ //laço para retirar o dado da fila e iniciar a transmissão
    //Serial.println("Task Sender: Lendo da fila");
    xQueueReceive(xQueueTemp, &dataSend, portMAX_DELAY);
    xTimerSignalStarted = xTimerStart(xTimerSignal,0);
    cont++;
    xSemaphoreTake(xSemaphoreTransmission,portMAX_DELAY); // Wait the transmission to finish
  }
}

/**** Leitura das mensagens ****/

void startReadingByte(TimerHandle_t xTimerStartReading){
  if(digitalRead(readerPin) == LOW){
    xTimerReaderStarted = xTimerStart(xTimerReader,0); // Initializes the timer to reader the message
    countByteR++;
  }else{
    attachInterrupt(digitalPinToInterrupt(readerPin), messageInterrupt, FALLING);
  }
}

void readerByte(TimerHandle_t xTimerReader){

  if(digitalRead(readerPin) == HIGH) dataReceived = dataReceived | B10000000;
  else dataReceived = dataReceived | B00000000;

  countBitReceived++; 

  if(countBitReceived == 8){ //Verifica se os 8 bits foram recebidos
    xTimerStop(xTimerReader,0); 
    attachInterrupt(digitalPinToInterrupt(readerPin), messageInterrupt, FALLING);
    Serial.print("readerMessage: Dado Recebido: ");
    Serial.println(dataReceived);
    frameReceived[countByteR-1] = dataReceived;
    
    switch(countByteR){
      case 2: checkDestiny(dataReceived);
              break;
      case 15: 
              xSemaphoreGive(xSemaphoreRouting);
              countByteR = 0;
              break;
    }
    dataReceived = B00000000;
    countBitReceived = 0;
  }else{ // Caso os 8  bits não tenham sido recebidos apenas segue a serialização
    dataReceived = dataReceived >> 1;  
  }
}

bool checkStx(byte stx){ //Checa se o byte é o STX """MUDAR"""
  return stx == B00001101;
}

void checkDestiny(byte mac){ //Checa se o host é o destino
  mac = mac >> 4;
  if(mac == B00001101) flagIsDestiny = true;
}

void carryPackageApp(Package * package){ // Transfere os dados do frame recebido para ao pacote da aplicação
  Serial.println("Entrou aqui na Carry");
  package->mac = frameReceived [1];
  package->port = frameReceived[2];
  int i=3, j=0;
  while(i<13){
    package->data[j] = frameReceived[i]; 
    j++; 
    i++;
  }
}

void sendRetransmition(){ //Carrega o frame com os dados recebidos e envia o frame para a fila de transmissão.

  Frame frameR;
  frameR.stx = frameReceived[0];
  frameR.mac = frameReceived[1];
  frameR.port = frameReceived[2];
  int i=3, j=0;
  while(i<13){
    frameR.data[j] = frameReceived[i]; 
    j++; 
    i++;
  } 
  frameR.bcc = frameReceived[13];
  frameR.etx = frameReceived[14];
  Serial.println("sendRetransmition: Retransmitindo");
  carry_Queue(xQueueSend,frameR);
  
}

void routing(){
    if(flagIsDestiny){
      Serial.println("Entrou aqui na Rounting TRUE");
      Package p1;
      carryPackageApp(&p1);
      Serial.println(p1.mac);
      xQueueSendToBack(xQueueApp,(void *)&p1,0);
    }else{
      sendRetransmition();
    }    
}

/**** Camada Aplicação *****/


/*** Interrupções ***/

void messageInterrupt(void){
  detachInterrupt(digitalPinToInterrupt(readerPin));
  xTimerStartReadingStarted= xTimerStart(xTimerStartReading,0);
}

/*** Tarefas ***/

void TaskSender(void *pvParameters) {
  (void) pvParameters;
  carry_Queue(xQueueSend,frame);
  send_message(xQueueSend);

  for(;;);
}


void TaskReceiver(void *pvParameters){
  (void) pvParameters;
  
  for (;;){
    xSemaphoreTake(xSemaphoreRouting,portMAX_DELAY);
    Serial.println("Entrou aqui na Task");
    routing();
  }
  
}

void TaskSenderApp(void *pvParameters){
  (void) pvParameters;
  Package package;
  
  for(;;){
    package.mac = B11011110;
    package.port = B11000000;
    
    for(int i = 0; i < 10; i++){
      package.data[i]= random(255);
    }
//    Frame fr1; //não é aqui
//    frameCreate(fr1,package); // não é aqui
    
    delay(3000);
  }
}

void TaskReceiverApp(void *pvParameters){

  (void) pvParameters;
  for(;;){
    Package dataApp;
    xQueueReceive(xQueueApp, &dataApp, portMAX_DELAY);
    Serial.println("Aplicação: ");
    Serial.println(dataApp.port);
    
  }
}  
