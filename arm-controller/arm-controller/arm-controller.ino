#include <AccelStepper.h>
#include <MultiStepper.h>
#include <ros.h>
#include <Servo.h> 
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <math.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>  
#include <std_msgs/Float64.h>

#define JOINT1_STEP_PIN        48 //CLK+
#define JOINT1_DIR_PIN         46 //CW+

#define JOINT2_STEP_PIN        44 //CLK+
#define JOINT2_DIR_PIN         42 //CW+

#define JOINT3_STEP_PIN        40 //CLK+
#define JOINT3_DIR_PIN         38 //CW+

AccelStepper joint1   (1, JOINT1_STEP_PIN, JOINT1_DIR_PIN);
AccelStepper joint2   (1, JOINT2_STEP_PIN, JOINT2_DIR_PIN);
AccelStepper joint3   (1, JOINT3_STEP_PIN, JOINT3_DIR_PIN);

Servo gripper; 
int pos_gripper = 0;
int algo = 0;
MultiStepper steppers; 

int joint_step[4];//[joint1,joint2,joint3,joint4,joint5,joint6,servo]
int joint_status = 0;
int pos = 0;
int eff0 = 0;//Efector final cerrado
int eff1 = 0;//Efector final abierto

float Sensibility = 0.185;

ros::NodeHandle nh; // Declaración del NodeHandle con la instancia nh
std_msgs::Int16 msg;
std_msgs::Float64 test;


void arm_cb(const sensor_msgs::JointState& arm_steps){
  joint_status = 1;
  joint_step[0] = arm_steps.position[0];
  joint_step[1] = arm_steps.position[1];
  joint_step[2] = arm_steps.position[2];
  joint_step[3] = arm_steps.position[3]; //Posición del Gripper <0-89>
}

void gripper_cb( const std_msgs::UInt16& cmd_msg){
  //gripper.write(msg_angulo.data);
   
  if(cmd_msg.data > 0)
  {
    for(pos = 0; pos < 90; pos += 1)   // Va de 0 a 89° En pasos de 1 grado
    {                                   
      gripper.write(pos);              // Indicarle al servo que adquiera la variable pos 
      delay(5);                        // Esperar 5ms pra que el servo llegue a la posición 
    }
  }
  
  if(cmd_msg.data == 0)
  {
    for(pos = 90; pos>=1; pos-=1)      // Va de 89 a 0° 
    {                                
      gripper.write(pos);              // Indicarle al servo que adquiera la variable pos
      delay(5);                        // Esperar 5ms pra que el servo llegue a la posición
    }
  }
}
/* //Current
float get_current(int n){
  float VoltSens;
  float current = 0;
  for(int i = 0; i < n; i++){
    VoltSens = analogRead(A0) * (5.0 / 1023.0);
    current = current+(VoltSens-2.5)/Sensibility;
  }
  current = current/n;
  return(current);
}
*/
/*------------------definición de los objetos subscriptores------------------*/
//la función arm_cb se ejecuta cuando hay un mensaje en el topic joint_steps
ros::Subscriber<sensor_msgs::JointState> arm_sub("joint_steps",arm_cb);

//la función arm_cb se ejecuta cuando hay un mensaje en el topic gripper_angle
ros::Subscriber<std_msgs::UInt16> gripper_sub("gripper_angle", gripper_cb); 

/* //Current
ros::Publisher p("current", &test);
*/

void setup() {
  //Serial.begin(57600);
  joint_status = 1;

  // inicializacion del nodo para el uso de la comunicación por puerto serial
  nh.initNode(); 
  
  //Inicializar subscriptores
  nh.subscribe(arm_sub); 
  nh.subscribe(gripper_sub);
  /* //Current
  nh.advertise(p);
  */

  //Asignación de valor de maxima velocidad para cada motor
  joint1.setMaxSpeed(1500);
  joint2.setMaxSpeed(200);
  joint3.setMaxSpeed(500);

  //Agregar motores a la libreria MultiStepper
  steppers.addStepper(joint1);
  steppers.addStepper(joint2);
  steppers.addStepper(joint3);

  //Asignar el puerto PWM 4 AL Gripper
  gripper.attach(4);

}

void loop() {
  /* //Current
  float I = get_current(200); 
  test.data = I;
  p.publish( &test );
  */
  if (joint_status == 1){ // Si arm_cb esta siendo llamado asigna el estado de joint_state a 1.
    
    long positions[7];

    positions[0] = joint_step[0];
    positions[1] = joint_step[1];
    positions[2] = joint_step[2];
    positions[3] = joint_step[3];  


    steppers.moveTo(positions);
    nh.spinOnce();
    steppers.runSpeedToPosition();
    
    if(joint_step[6] > 0){
      if(eff1 == 0){
        for(pos = 0; pos < 90; pos += 1){  // Va de 0 a 89° En pasos de 1 grado                                   
          gripper.write(pos);              // Indicarle al servo que adquiera la variable pos 
          delay(5);                        // Esperar 5ms para que el servo llegue a la posición 
        }        
      }
      eff1++;
      eff0 = 0;
    }

    if(joint_step[6] == 0){
      if(eff0 == 0){
        for(pos = 90; pos>=1; pos-=1){     // Va de 89 a 0°                               
          gripper.write(pos);              // Indicarle al servo que adquiera la variable pos
          delay(5);                        // Esperar 5ms para que el servo llegue a la posición
        }
      }
      eff0++;
      eff1 = 0;
    }    
  }
  joint_status = 0;

  nh.spinOnce();
  delay(1);
}
