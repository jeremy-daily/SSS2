#include "Thread.h"
#include "ThreadController.h"
#include "FlexCAN.h"

const uint8_t buttonPin = 24;

static CAN_message_t temp_txmsg;

// Create a new Class
ThreadController can_thread_controller = ThreadController();

class CanThread: public Thread
{
public:
  uint32_t stop_after_count = 0;
  uint32_t transmit_number;
  uint8_t channel = 0;
  CAN_message_t txmsg;
  
  bool shouldRun(){
    // Override enabled on thread when pin goes LOW.
    enabled = digitalRead(buttonPin);
    
    if (stop_after_count > 0){
      if (transmit_number >= stop_after_count) enabled = false;
    }
    // Let default method check for it.
    return Thread::shouldRun();
  }
  
	void run(){
    if (channel == 0) Can0.write(txmsg);
    else if (channel = 1) Can1.write(txmsg);
    transmit_number++;
  	runned();
	}
};


CanThread can_message1 = CanThread();
CanThread can_message2 = CanThread();


void setup(){
  pinMode(buttonPin,INPUT_PULLUP);
	Serial.begin(9600);
  Can0.begin(250000);
  Can1.begin(250000);
  


  temp_txmsg.id=0x101;
  
	// Configures Thread analog1
	can_message1.channel = 0;
	can_message1.setInterval(100);
  can_message1.txmsg = temp_txmsg;
  can_message1.stop_after_count = 50;

  temp_txmsg.id=0x102;
  can_message2.channel = 0;
  can_message2.setInterval(120);
  can_message2.txmsg = temp_txmsg;
  can_message2.stop_after_count = 50;
  
  can_thread_controller.add(&can_message1);
  can_thread_controller.add(&can_message2);
  
	
}

void loop(){
	// Do complex-crazy-timeconsuming-tasks here
	
	// Get the fresh readings
  Serial.printf("transmit_number: %lu, shouldRun: %d, key: %d, enabled: %d, id: %08X\n",
                        can_message1.transmit_number,
                        can_message1.shouldRun(),
                        digitalRead(buttonPin),
                        can_message1.enabled,
                        can_message1.txmsg.id);
	// Do more complex-crazy-timeconsuming-tasks here
 can_thread_controller.run();
 delay(50);

}

