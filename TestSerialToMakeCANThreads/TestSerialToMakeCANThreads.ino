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
protected:
  uint32_t main_period = 100;
  uint32_t sub_period = 100;
public:
  uint32_t stop_after_count;
  uint32_t transmit_number;
  uint8_t channel = 0;
  uint8_t num_messages = 1; 
  CAN_message_t txmsgs[256] = {};
  uint8_t message_index = 0;
  

  CanThread( uint8_t _channel, uint32_t _main_period,uint32_t _sub_period): Thread(){
    if (_main_period >= 10) main_period = _main_period;
    else main_period = 10; //don't flood the bus

    channel = _channel;
        
    sub_period = constrain(_sub_period,0,main_period);
    
    interval = sub_period;
    Serial.printf("Interval: %d\n",interval);
    enabled = true;
  };
  
  bool shouldRun(long time){
     if (stop_after_count > 0){
      if (transmit_number >= stop_after_count) enabled = false;
    }
    Serial.printf("enabled: %d\n", enabled);
    time = millis();
    // Let default method check for it.
    return Thread::shouldRun(time);
  }
  
  void run(){
    if (channel == 0) {
      Can0.write(txmsgs[message_index]);
      Serial.printf("CAN0 message_index: %lu\n", message_index);
    }
    else if (channel = 1){
      Can1.write(txmsgs[message_index]);
      Serial.printf("CAN1 message_index: %lu\n", message_index);
    }
    message_index++;
    
    transmit_number++;
    Serial.printf("transmit_number: %lu\n", transmit_number);
    //interval = sub_period;
    if (message_index = num_messages) {
      uint32_t majorDelay = main_period - num_messages * sub_period; 
      //interval = constrain(majorDelay,sub_period,main_period);
      message_index = 0;
      
    }
    //Thread::run();
    runned();
  }
  
};


CanThread* can_messages[MAX_THREADS] ={};

int setupPeriodicCANMessage(uint16_t index){
  CanThread can_message = CanThread(0,100,40); 
  can_message.txmsgs[index] = temp_txmsg;
  can_message.stop_after_count = 00;
  if (can_thread_controller.add(&can_message)) Serial.println("Added CAN TX message.");
  else Serial.println("ERROR failed to add CAN TX message.");
  can_message.run();
  can_messages[index] = &can_message;
  int can_count = can_thread_controller.size(false);
  Serial.printf("can_count: %d\n",can_count);
  return can_count;
}


void setup(){
  pinMode(buttonPin,INPUT_PULLUP);
	Serial.begin(9600);
  Can0.begin(250000);
  Can1.begin(250000);
  

	temp_txmsg.buf[0]=3;
  temp_txmsg.len=8;
  temp_txmsg.ext=1;
  temp_txmsg.id=0x18FEF1FA;
  
  Serial.print("MAX_THREADS: ");
  Serial.println(MAX_THREADS);

  for (int i=0;i<5;i++){
    temp_txmsg.buf[0] = 2*i;
    Serial.printf("CAN Count Return: %d\n", setupPeriodicCANMessage(i));
    delay(1);
    can_thread_controller.run();
    

  }
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
       
        
        while (i < 5){
          temp_txmsg.id = i;
          temp_txmsg.buf[0] = 2*i;
          setupPeriodicCANMessage;
          delay(1);
          can_thread_controller.run();
          i++;

        }
      }
    else if (commandChars.toLowerCase().startsWith("stop")){
      int index = commandString.toInt();
      Serial.printf("Stop,%d\n",index);
      can_messages[index]->enabled = false;
      can_messages[index]->txmsgs[0].buf[1] = index;
      
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
         Serial.printf("ID: %X, enabled: %d \n", can_messages[j] -> txmsgs[0].id,can_messages[j] -> enabled);
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

