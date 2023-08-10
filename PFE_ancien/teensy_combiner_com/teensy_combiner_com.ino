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
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/joint_state.h>

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
sensor_msgs__msg__JointState  msg_combine;

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

//Parametre qui influence la vitesse
float time_publish_ms = 10; //Temps entre chaque iteration du timer_callback
int nb_point_publish = 10; //Presentement du au capteur (1000Hz) le nombre de point ne peut pas etre plus grand
float pas_moteur = 0.15;

//Initialise le nombre d'execution du timer callback
int nb_ex = 0;

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
    float lidar_data = 0.0;
    float m1_data = 0.0;
    float m2_data = 0.0;
	
    //Lit la valeur du lidar et l'inscrit dans lidar_data
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
          lidar_data = buffer[2]+buffer[3]*256; //Longueur du message trouve sur le lidar
        }
        digitalWrite(led_pin, LOW); //Fermeture de la LED sur le Teensy
      }
    }

    //Calcule la valeur des moteurs
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

    m1_data = angle_1;
    m2_data = angle_2;

    setAngle(m1_data, PIN_MOTOR_1);
    setAngle(m2_data, PIN_MOTOR_2);

    
    msg_combine.position.data[nb_ex] =lidar_data;
    msg_combine.velocity.data[nb_ex] =m1_data;
    msg_combine.effort.data[nb_ex]   =m2_data;


    if (nb_ex == (nb_point_publish - 1)){
      nb_ex = 0;
      //Envoie de la valeur
		  RCSOFTCHECK(rcl_publish(&publisher_lidar, &msg_combine, NULL));
    }else{
      nb_ex += 1;
    }

	}
}

int main()
{	
	// init memoire MultiArray sensor_msgs__msg__JointState (Lidar, M1, M2)
  msg_combine.name.capacity = nb_point_publish;
  msg_combine.name.size = nb_point_publish;
  msg_combine.name.data =  (rosidl_runtime_c__String*) malloc(msg_combine.name.capacity*sizeof(std_msgs__msg__String));

  for(int i=0;i<nb_point_publish;i++) {
    msg_combine.name.data[i].data = (char*) malloc(12*sizeof(char));
    msg_combine.name.data[i].capacity = 5;
    sprintf(msg_combine.name.data[i].data,"j%d",i);
    msg_combine.name.data[i].size = strlen(msg_combine.name.data[i].data);
  }

  msg_combine.position.size=nb_point_publish;
  msg_combine.position.capacity=nb_point_publish;
  msg_combine.position.data = (double*) malloc(msg_combine.position.capacity*sizeof(double));
  msg_combine.velocity.size=nb_point_publish;
  msg_combine.velocity.capacity=nb_point_publish;
  msg_combine.velocity.data = (double*) malloc(msg_combine.velocity.capacity*sizeof(double));
  msg_combine.effort.size=nb_point_publish;
  msg_combine.effort.capacity=nb_point_publish;
  msg_combine.effort.data = (double*) malloc(msg_combine.effort.capacity*sizeof(double));

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
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
		"topic_lidar"));

	// --------------------------------------------------------------------------- //
	// --------------------------------------------------------------------------- //
	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = time_publish_ms/nb_point_publish; //Temps d'acquisition des donnees du lidar en ms
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
	
	// spin
	rclc_executor_spin(&executor);
	
	//Nettoyage
	RCCHECK(rcl_publisher_fini(&publisher_lidar, &node));
	RCCHECK(rcl_node_fini(&node));
}

