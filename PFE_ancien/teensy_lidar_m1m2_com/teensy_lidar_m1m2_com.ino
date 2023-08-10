	// --------------------------------------------------------------------------- //
	// --------------------------------------------------------------------------- //
  // 
  // Description : Code utilise pour la gestion du lidar
  // 
  // --------------------------------------------------------------------------- //
	// --------------------------------------------------------------------------- //

#include <Arduino.h>
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

rcl_publisher_t publisher_lidar;
rcl_publisher_t publisher_m1;
rcl_publisher_t publisher_m2;
std_msgs__msg__Float32 msg_lidar;
std_msgs__msg__Float32 msg_m1;
std_msgs__msg__Float32 msg_m2;


//Pin utilise pour le Lidar
unsigned long baud = 115200;
const int reset_pin = 4;
const int led_pin = 13;

//Initialisation du buffer pour le Lidar
long led_on_time=0;
byte buffer[80];

//Initialisation des parametres des moteurs
float angle_moteur1_min = 70.0;
float angle_moteur1_max = 130.0;

float angle_moteur2_min = 70.0;
float angle_moteur2_max = 110.0;

float direction_positive = 1;
float direction_negative = 0;

float pas_moteur = 0.15;
//float time_publish_sec = 0.01;

//Initialisation des angles des moteurs
float angle_1 = angle_moteur1_min;
float angle_2 = angle_moteur2_min;

//Initialisation de la direction des moteurs
float direction_1 = 1;
float direction_2 = 1;

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
    std_msgs__msg__Float32__init(&msg_lidar);
    std_msgs__msg__Float32__init(&msg_m1);
    std_msgs__msg__Float32__init(&msg_m2);

    
	
    int rd, wr;
    // check if any data has arrived on the hardware serial port
    rd = HWSERIAL.available();
    if (rd >= 9) {
      // check if the USB virtual serial port is ready to transmit
      wr = Serial.availableForWrite();
      if (wr > 0) {
        // read data from the hardware serial port
        HWSERIAL.readBytes((char *)buffer, 9);

        if(buffer[0]==0x59 && buffer[1]==0x59){
          digitalWrite(led_pin, HIGH); //Recoit des donnees, donc flash la LED sur le Teensy
          msg_lidar.data = buffer[2]+buffer[3]*256; //Longueur du message trouve sur le lidar
        }
        digitalWrite(led_pin, LOW); //Fermeture de la LED sur le Teensy
      }
    }



        if (angle_1 <= angle_moteur1_min) { //Debute l'activation du moteur dans une direction a partir d'un angle
            direction_1 = direction_positive;
            
            if (direction_2 == direction_positive) {
                angle_2 += pas_moteur;
            }
            if (direction_2 == direction_negative) {
                angle_2 -= pas_moteur;
            }
        }
        if (angle_1 >= angle_moteur1_max) { //Changement de direction du moteur une fois l'angle maximale atteint
            direction_1 = direction_negative;

            if (direction_2 == direction_positive) {
                angle_2 += pas_moteur;
            }
            if (direction_2 == direction_negative) {
                angle_2 -= pas_moteur;
            }
        }

        if (direction_1 == direction_positive) { // #le moteur va dans une direction
            angle_1 += pas_moteur;
        }
        if (direction_1 == direction_negative) { //#Changement de direction du moteurs
            angle_1 -= pas_moteur;
        }
        if (angle_2 <= angle_moteur2_min) {
            direction_2 = direction_positive;
        }
        if (angle_2 >= angle_moteur2_max) {
            direction_2 = direction_negative;
        }

        msg_m1.data = angle_1;
        msg_m2.data = angle_2;


        setAngle(msg_m1.data, PIN_MOTOR_1);
        setAngle(msg_m2.data, PIN_MOTOR_2);

    //Envoie de la valeur
		RCSOFTCHECK(rcl_publish(&publisher_lidar, &msg_lidar, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_m1, &msg_m1, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_m2, &msg_m2, NULL));
    //Impression de ce qui a ete envoye
		//printf("Donnees envoyee du Lidar: %f\n", msg_lidar.data);

    //Nettoyage de la variable msg
    std_msgs__msg__Float32__fini(&msg_lidar);
    std_msgs__msg__Float32__fini(&msg_m1);
    std_msgs__msg__Float32__fini(&msg_m2);
	}
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
	// create publisher lidar
	RCCHECK(rclc_publisher_init_default(
		&publisher_lidar,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"topic_lidar"));

	// create publisher moteur 1
	RCCHECK(rclc_publisher_init_default(
		&publisher_m1,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"topic_moteur1"));

  // create publisher moteur 2
	RCCHECK(rclc_publisher_init_default(
		&publisher_m2,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"topic_moteur2"));

	// --------------------------------------------------------------------------- //
	// --------------------------------------------------------------------------- //
	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1; //Temps d'acquisition des donnees du lidar en ms
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
	
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	
	msg_lidar.data = 0;
  msg_m1.data = 0;
  msg_m2.data = 0;
	
	// spin
	rclc_executor_spin(&executor);
	
	//Nettoyage
	RCCHECK(rcl_publisher_fini(&publisher_lidar, &node));
	RCCHECK(rcl_publisher_fini(&publisher_m1, &node));
	RCCHECK(rcl_publisher_fini(&publisher_m2, &node));
	RCCHECK(rcl_node_fini(&node));
}

