#include "Thread.h"
#include "ThreadController.h"
#include "FlexCAN.h"


const uint8_t buttonPin = 24;
String commandString;
static CAN_message_t temp_txmsg;

// Create a new Class
ThreadController can_thread_controller = ThreadController();

class CanThread: public Thread
{
public:
  uint32_t stop_after_count;
  uint32_t transmit_number;
  uint8_t channel = 0;
  CAN_message_t txmsg;
  
  
//  bool shouldRun(){
//    // Override enabled on thread when pin goes LOW.
//  //  enabled = ext_enable; //digitalRead(buttonPin);
//    
////    if (stop_after_count > 0){
////      if (transmit_number >= stop_after_count) enabled = false;
////    }
//    // Let default method check for it.
//    return Thread::shouldRun();
//  }
  
	void run(){
    if (channel == 0) Can0.write(txmsg);
    else if (channel = 1) Can1.write(txmsg);
    transmit_number++;
  	runned();
	}
};


//CanThread can_message1 = CanThread();
//CanThread can_message2 = CanThread();

CanThread* can_messages[MAX_THREADS] ={};

void setup(){
  pinMode(buttonPin,INPUT_PULLUP);
	Serial.begin(9600);
  Can0.begin(250000);
  Can1.begin(250000);
  

//
//  temp_txmsg.id=0x101;
//  
//	// Configures Thread analog1
//	can_message1.channel = 0;
//	can_message1.setInterval(100);
//  can_message1.txmsg = temp_txmsg;
//  can_message1.stop_after_count = 50;
//
//  temp_txmsg.id=0x102;
//  can_message2.channel = 0;
//  can_message2.setInterval(120);
//  can_message2.txmsg = temp_txmsg;
//  can_message2.stop_after_count = 50;
  
//  can_thread_controller.add(&can_message1);
//  can_thread_controller.add(&can_message2);
//  
	temp_txmsg.buf[0]=3;
  temp_txmsg.len=3;
  Serial.print("MAX_THREADS: ");
  Serial.println(MAX_THREADS);
}

String commandChars;
elapsedMillis displayTimer;
int i=0;

int period = 100;


void loop(){
 can_thread_controller.run();

/****************************************************************/
  /*            Begin Serial Command Processing                   */
  if (Serial.available() >= 2 && Serial.available() < 140) {
    commandChars = Serial.readStringUntil(',');
    if (Serial.available()) commandString = Serial.readStringUntil('\n');
    
    if (commandChars.startsWith("SM") || commandChars.startsWith("sm")) {
        temp_txmsg.id =  commandString.toInt();
       
        
        while (i < 25){
          temp_txmsg.id = i;
          temp_txmsg.buf[0] = 2*i;
          CanThread* can_message = new CanThread(); 
          can_message->channel = 0;
          can_message->setInterval(period);
          can_message->txmsg = temp_txmsg;
          can_message->stop_after_count = 50;
          if (can_thread_controller.add(can_message)) Serial.println("Added CAN TX message.");
          else Serial.println("ERROR failed to add CAN TX message.");
          can_messages[i] = can_message;
          int can_count = can_thread_controller.size(false);
          Serial.printf("can_count: %d\n",can_count);
          delay(1);
          can_thread_controller.run();
          i++;

        }
      }
    else if (commandChars.toLowerCase().startsWith("stop")){
      int index = commandString.toInt();
      Serial.printf("Stop,%d\n",index);
      can_messages[index]->enabled = false;
      can_messages[index]->txmsg.buf[1] = index;
      
      can_thread_controller.remove(index);
      //can_thread_controller.clear();
    }
    else if (commandChars.toLowerCase().startsWith("go")){
      int index = commandString.toInt();
      Serial.printf("Go,%d\n",index);
      can_messages[index]->enabled = true;
      //can_thread_controller.remove(index);
      //can_thread_controller.clear();
    }
    else if (commandChars.toLowerCase().startsWith("list")){
      for (int j = 0; j < can_thread_controller.size();j++){
         Serial.printf("ID: %X, enabled: %d \n", can_messages[j] -> txmsg.id,can_messages[j] -> enabled);
         can_thread_controller.run();
      }
    }
  }

//  if (displayTimer > 500){
//    displayTimer = 0;
//    
//    Serial.printf("Can_thread_1: %ul\n", &can_messages[0]->txmsg.id);
//  }
}

