	// --------------------------------------------------------------------------- //
	// --------------------------------------------------------------------------- //
  // 
  // Description : Code utilise pour la gestion du lidar
  // 
  // --------------------------------------------------------------------------- //
	// --------------------------------------------------------------------------- //


#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#include <stdio.h>
#include <unistd.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define PIN_MOTOR_1 22 
#define PIN_MOTOR_2 23

// set this to the hardware serial port you wish to use
#define HWSERIAL Serial1 //Lidar

rcl_publisher_t publisher;
rcl_subscription_t subscriber1;
rcl_subscription_t subscriber2;
std_msgs__msg__Float32 msg;

//Pin utilise pour le Lidar
unsigned long baud = 115200;
const int reset_pin = 4;
const int led_pin = 13;

//Initialisation du buffer pour le Lidar
long led_on_time=0;
byte buffer[80];

void setAngle(float angle, int pin)
{
  float value_ms = ((angle/270)*2)+0.5;
  float value_pwm = value_ms/20 * 32757;
  analogWrite(pin, ceil(value_pwm));
}


void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	if (timer != NULL) {
    //Creation d'une nouvelle variable msg pour publier la donnee
    std_msgs__msg__Float32 msg;
    std_msgs__msg__Float32__init(&msg);

    int rd, wr, n;
    // check if any data has arrived on the hardware serial port
    rd = HWSERIAL.available();
    if (rd >= 9) {
      // check if the USB virtual serial port is ready to transmit
      wr = Serial.availableForWrite();
      if (wr > 0) {
        // read data from the hardware serial port
        n = HWSERIAL.readBytes((char *)buffer, 9);

        if(buffer[0]==0x59 && buffer[1]==0x59){
          digitalWrite(led_pin, HIGH); //Recoit des donnees, donc flash la LED sur le Teensy
          msg.data = buffer[2]+buffer[3]*256; //Longueur du message trouve sur le lidar
        }
        digitalWrite(led_pin, LOW); //Fermeture de la LED sur le Teensy
      }
    }

    //Envoie de la valeur
		RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

    //Impression de ce qui a ete envoye
		printf("Donnees envoyee du Lidar: %f\n", msg.data);

    //Nettoyage de la variable msg
    std_msgs__msg__Float32__fini(&msg);
	}
}


void subscription_callback1(const void * msgin)
{
	const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

  //Afficher l'angle obtenu sur le terminal
	printf("Angle moteur 1 recu: %f\n", msg->data);

  //Appliquer l'angle sur le moteur
  setAngle(msg->data, PIN_MOTOR_1);
}


void subscription_callback2(const void * msgin)
{
	const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
	printf("Angle moteur 2 recu: %f\n", msg->data);

  //Appliquer l'angle sur le moteur
  setAngle(msg->data, PIN_MOTOR_2);
}


int main()
{	
	// init
	set_microros_transports();
	
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
	
  // create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "Teensy4_node", "", &support));


  //Definition des PINS du moteur
  pinMode(PIN_MOTOR_1, OUTPUT);  // sets the pin as output
  pinMode(PIN_MOTOR_2, OUTPUT);  // sets the pin as output
  analogWriteResolution(15);
  analogWriteFrequency(PIN_MOTOR_1, 50); 
  analogWriteFrequency(PIN_MOTOR_2, 50);

  //Definition des PINS du Lidar 
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, LOW);
  digitalWrite(reset_pin, HIGH);
  pinMode(reset_pin, OUTPUT);
  Serial.begin(baud);	// USB, communication to PC or Mac
  HWSERIAL.begin(baud);	// communication to hardware serial
  digitalWrite(led_pin, LOW);


	// --------------------------------------------------------------------------- //
	// --------------------------------------------------------------------------- //
	//	Publisher
	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"topic_lidar"));
		
	// --------------------------------------------------------------------------- //
	// --------------------------------------------------------------------------- //
	//	Subscriber 1
	// create subscriber 1
	RCCHECK(rclc_subscription_init_default(
		&subscriber1,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"topic_moteur1"));
	
	// --------------------------------------------------------------------------- //
	// --------------------------------------------------------------------------- //
	//	Subscriber 2
	// create subscriber 2
	RCCHECK(rclc_subscription_init_default(
		&subscriber2,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"topic_moteur2"));
	
	// --------------------------------------------------------------------------- //
	// --------------------------------------------------------------------------- //
	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 10;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// --------------------------------------------------------------------------- //
	// --------------------------------------------------------------------------- //
	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
	
	// subscription and timer
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber1, &msg, &subscription_callback1, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber2, &msg, &subscription_callback2, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	
	msg.data = 0;
	
	// spin
	rclc_executor_spin(&executor);
	
	//Nettoyage
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_subscription_fini(&subscriber1, &node));
	RCCHECK(rcl_subscription_fini(&subscriber2, &node));
	RCCHECK(rcl_node_fini(&node));
}

