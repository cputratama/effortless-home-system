// EL4121 Embedded System
// Institut Teknologi Bandung
// Date	        :20 December 2016
// Author 1 	  :Christoporus Deo Putratama
// Author 2 	  :Irham Mulkan Rodiana
// Nama File    :EffortlessHomeSystemFreeRTOS.ino
// Description  :Utilize FreeRTOS in arduino by making 
//               task scheduler. These tasks are used 
//               to operate input and output interfacing 
//               such as sensors and actuators.


// Initialization library

#include <Arduino_FreeRTOS.h>
#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).
#include <Servo.h>

// Initialization pin
#define LDR_SENSOR_PIN  A0
#define TRIG_PIN       8
#define ECHO_PIN       9
#define RELAY_1_PIN    2
#define RELAY_2_PIN    3
#define RELAY_3_PIN    4
#define TV_PIN   7

// Initialization Constant
#define INIT_POS       0
#define INIT_ITERATION 0

#define CMD_PINTU_INIT  99
#define CMD_PINTU_CLOSE 0
#define CMD_PINTU_OPEN  1

#define CMD_TIRAI_INIT  99
#define CMD_TIRAI_CLOSE 0
#define CMD_TIRAI_OPEN  1

#define CMD_TV_INIT    99
#define CMD_TV_OFF      0
#define CMD_TV_ON     1

#define STATE_PINTU_INIT  0
#define STATE_PINTU_CLOSE 0
#define STATE_PINTU_OPEN  1

#define STATE_TIRAI_INIT  0
#define STATE_TIRAI_CLOSE 0
#define STATE_TIRAI_OPEN  1

#define STATE_TV_INIT 1
#define STATE_TV_OFF  0
#define STATE_TV_ON   1

#define ULTRA_TRASH_INIT  0
#define LDR_SENSOR_INIT 0
  
#define SERIAL_BEGIN    9600 //9600 baud serial monitor

//Initialization servo
Servo myservo; 

//Initialization variable

int pos = INIT_POS; //0
int commandpintu = CMD_PINTU_INIT; //99
int commandtirai = CMD_TIRAI_INIT; //99
int commandtv    = CMD_TV_INIT; //99

// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only only one Task is accessing this resource at any time.
SemaphoreHandle_t xSerialSemaphore;

// define five Tasks for this Projext
void TaskPintu( void *pvParameters );
void TaskTV( void *pvParameters );
void TaskTirai( void *pvParameters );
void TaskUltra( void *pvParameters );
void TaskLDR( void *pvParameters );


// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(SERIAL_BEGIN);

  // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
  // because it is sharing a resource, such as the Serial port.
  // Semaphores should only be used whilst the scheduler is running, but we can set it up here.
  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }

  // Now set up five Tasks to run independently.
  xTaskCreate(TaskPintu, (const portCHAR *)"Pintu",  128,  NULL,  1,  NULL );
  xTaskCreate(TaskTirai, (const portCHAR *)"Tirai",  128,  NULL,  1,  NULL );
  xTaskCreate(TaskTV, (const portCHAR *)"TV",  128,  NULL,  1,  NULL );
  xTaskCreate(TaskUltra, (const portCHAR *)"Ultra",  128,  NULL,  1,  NULL );
  xTaskCreate(TaskLDR, (const portCHAR *)"LDR",  128,  NULL,  1,  NULL );
  
  // Now the Task scheduler, which takes over control of scheduling individual Tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskPintu( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  myservo.attach(6);
  int statepintu = STATE_PINTU_INIT; //0
  int i = INIT_ITERATION; //0
  myservo.write(10);
   
  for (;;) // A Task shall never return or exit.
  {
    if (commandpintu == CMD_PINTU_OPEN && statepintu == STATE_PINTU_CLOSE)
    {      
      if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
          Serial.println("EHS melakukan OPEN Pintu");
          xSemaphoreGive( xSerialSemaphore );
      }     
      for (pos = 10; pos <= 180; pos += 2) { // goes from 0 degrees to 180 degrees
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        vTaskDelay(1);                       // waits 15ms for the servo to reach the position
      }
      commandpintu = CMD_PINTU_INIT;
      statepintu = STATE_PINTU_OPEN;
    }
    else if (commandpintu == CMD_PINTU_CLOSE && statepintu == STATE_PINTU_OPEN)
    {
      if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
          Serial.println("EHS melakukan CLOSE Pintu");
          xSemaphoreGive( xSerialSemaphore );
      }      
      for (pos = 180; pos >= 10; pos -= 2) { // goes from 180 degrees to 0 degrees
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        vTaskDelay(1);                       // waits 15ms for the servo to reach the position
      }
      commandpintu = CMD_PINTU_INIT;
      statepintu = STATE_PINTU_CLOSE;
    }    
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskTirai( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  pinMode(RELAY_1_PIN,OUTPUT);//Relay 1
  pinMode(RELAY_2_PIN,OUTPUT);//Relay 2
  pinMode(RELAY_3_PIN,OUTPUT);//Relay 3
  int statetirai = STATE_TIRAI_INIT;
  pinMode(13,OUTPUT);
  for (;;)
  {
    if (commandtirai == CMD_TIRAI_OPEN && statetirai == STATE_TIRAI_CLOSE)
    {      
      if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
          Serial.println("EHS melakukan OPEN Tirai");
          xSemaphoreGive( xSerialSemaphore );          
      }    
      digitalWrite(RELAY_1_PIN,LOW);
      digitalWrite(RELAY_2_PIN,HIGH);
      digitalWrite(RELAY_3_PIN,HIGH);
      vTaskDelay(8);
      digitalWrite(RELAY_1_PIN,HIGH);
      digitalWrite(RELAY_2_PIN,LOW);
      digitalWrite(RELAY_3_PIN,LOW);
      commandtirai = CMD_TIRAI_INIT;
      statetirai = STATE_TIRAI_OPEN;
    }
    else if (commandtirai == CMD_TIRAI_CLOSE && statetirai == STATE_TIRAI_OPEN){
      if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
          //digitalWrite(13,HIGH);
          Serial.println("EHS melakukan CLOSE Tirai");
          xSemaphoreGive( xSerialSemaphore );
          //digitalWrite(13,LOW);
      }
      digitalWrite(RELAY_1_PIN,HIGH);      
      digitalWrite(RELAY_2_PIN,LOW);
      digitalWrite(RELAY_3_PIN,HIGH);
      vTaskDelay(8);      
      digitalWrite(RELAY_1_PIN,LOW);
      digitalWrite(RELAY_2_PIN,HIGH);
      digitalWrite(RELAY_3_PIN,LOW);
      commandtirai = CMD_TIRAI_INIT;
      statetirai = STATE_TIRAI_CLOSE;
    }    
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskTV( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  pinMode(TV_PIN,OUTPUT);//TV
  int statetv = STATE_TV_INIT;
  pinMode(13,OUTPUT);
  for (;;)
  {
    if (commandtv == CMD_TV_ON && statetv == STATE_TV_OFF){      
      if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
          //digitalWrite(13,HIGH);
          Serial.println("EHS melakukan Lampu Nyala");
          xSemaphoreGive( xSerialSemaphore );
          //digitalWrite(13,LOW);
      }     
      digitalWrite(TV_PIN,HIGH);      
      commandtv = CMD_TV_INIT;
      statetv = STATE_TV_ON;
    }
    else if (commandtv == CMD_TV_OFF && statetv == STATE_TV_ON){
      if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
      {
          Serial.println("EHS melakukan Lampu Mati");
          xSemaphoreGive( xSerialSemaphore );
      }     
      digitalWrite(TV_PIN,LOW);      
      commandtv = CMD_TV_INIT;
      statetv = STATE_TV_OFF;
    }
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskUltra( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  int temp = ULTRA_TRASH_INIT;
  long duration;
  long distance;  
  for (;;) // A Task shall never return or exit.
  { 
      digitalWrite(TRIG_PIN, LOW);  // Added this line
      delayMicroseconds(2); // Added this line
      digitalWrite(TRIG_PIN, HIGH);
      //  delayMicroseconds(1000); - Removed this line
      delayMicroseconds(10); // Added this line
      digitalWrite(TRIG_PIN, LOW);
      duration = pulseIn(ECHO_PIN, HIGH);
      distance = (duration/2) / 29.1;
      if (abs(distance-temp)<10 && distance<22){
        //Serial.println();
        //Serial.println(distance);
        commandpintu = CMD_PINTU_OPEN;
        vTaskDelay(200);
      }
      else {
        //Serial.print(distance);
        //Serial.println(" cm");
        commandpintu = CMD_PINTU_CLOSE;
      }    
    vTaskDelay(10);  // one tick delay (16ms) in between reads for stability
    temp = distance;
  }
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskLDR( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  int sensorValue = LDR_SENSOR_INIT; // variable to store the value coming from the sensor
  Serial.begin(9600); //sets serial port for communication
  //pinMode(13,OUTPUT);
  for (;;)
  {
    //digitalWrite(13,HIGH);
    sensorValue = analogRead(LDR_SENSOR_PIN); // read the value from the sensor
    if (sensorValue <= 150){
        //Serial.println();
        commandtv = CMD_TV_ON;
        commandtirai = CMD_TIRAI_CLOSE;
        vTaskDelay(200);
      }
      else {
        commandtv = CMD_TV_OFF;
        commandtirai = CMD_TIRAI_OPEN;
        //Serial.print(distance);
        //Serial.println(" cm");
      }
    //Serial.print("    "); Serial.println(sensorValue); //prints the values coming from the sensor on the screen
    //digitalWrite(13,LOW);
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}
