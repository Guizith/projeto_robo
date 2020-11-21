/*
 * File:          my_controller.c
 * Date: 20/11/2020
 * Description: 
 * Author: William Gabriel Barbosa  RA:22.117.064-0
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/supervisor.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 256
#define QtddSensoresProx 8
#define QtddLeds 10

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  
  int i=0;
  // variavel de interação para saber qual led acender
  int j = 0;
  
  char texto[256];
  double LeituraSensorProx[QtddSensoresProx];
  double AceleradorDireito=1.0, AceleradorEsquerdo=1.0;
  //flag de colisao para saber se colidiu ou não
  int fgcolisao = 0;
  
  /* necessary to initialize webots stuff */
  
  for(i=0;i<257;i++) texto[i]='0';
  
  wb_robot_init();
  
  //configurei MOTORES
  WbDeviceTag MotorEsquerdo, MotorDireito;
  
  MotorEsquerdo = wb_robot_get_device("left wheel motor");
  MotorDireito = wb_robot_get_device("right wheel motor");
  
  wb_motor_set_position(MotorEsquerdo, INFINITY);
  wb_motor_set_position(MotorDireito, INFINITY);
  
  wb_motor_set_velocity(MotorEsquerdo,0);
  wb_motor_set_velocity(MotorDireito,0);
  
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("robo"); //captura o supervisor e o nome atribuido para o robo foi "robo"
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation"); //identifica o campo de posição
  
  const double *posicao; //variáel que vai receber a posição do robo
  
   //configurei Sensores de Proximidade
   WbDeviceTag SensorProx[QtddSensoresProx];
   
   SensorProx[0] = wb_robot_get_device("ps0");
   SensorProx[1] = wb_robot_get_device("ps1");
   SensorProx[2] = wb_robot_get_device("ps2");
   SensorProx[3] = wb_robot_get_device("ps3");
   SensorProx[4] = wb_robot_get_device("ps4");
   SensorProx[5] = wb_robot_get_device("ps5");
   SensorProx[6] = wb_robot_get_device("ps6");
   SensorProx[7] = wb_robot_get_device("ps7");
   
   wb_distance_sensor_enable(SensorProx[0],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[1],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[2],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[3],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[4],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[5],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[6],TIME_STEP);
   wb_distance_sensor_enable(SensorProx[7],TIME_STEP);

    //config leds
    WbDeviceTag Leds[QtddLeds];
    Leds[0] = wb_robot_get_device("led0");
    Leds[1] = wb_robot_get_device("led1");
    Leds[2] = wb_robot_get_device("led2");
    Leds[3] = wb_robot_get_device("led3");
    Leds[4] = wb_robot_get_device("led4");
    Leds[5] = wb_robot_get_device("led5");
    Leds[6] = wb_robot_get_device("led6");
    Leds[7] = wb_robot_get_device("led7");
    wb_led_set(Leds[0],-1);
    wb_led_set(Leds[1],-1);
    wb_led_set(Leds[2],-1);
    wb_led_set(Leds[3],-1);
    wb_led_set(Leds[4],-1);
    wb_led_set(Leds[5],-1);
    wb_led_set(Leds[6],-1);
    wb_led_set(Leds[7],-1);

  
  
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    for(i=0;i<256;i++) texto[i]=0;
    //memcpy(texto,0,10);
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    /* Process sensor data here */
    for(i=0;i<QtddSensoresProx;i++){
       LeituraSensorProx[i]= wb_distance_sensor_get_value(SensorProx[i])-60;
       sprintf(texto,"%s|%d: %5.2f  ",texto,i,LeituraSensorProx[i]);
    }
    printf("%s\n",texto);
    
    
    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
    posicao = wb_supervisor_field_get_sf_vec3f(trans_field);
    printf("Posição do robo | x= %.2f | y= %.2f | z= %.2f |\n", posicao[0], posicao[1], posicao[2]);
    
    /* encontrei a posicao da caixa e utilizei as coordenadas encontradas para que quando entrar nessa coordenada 
     * ele sabe que se bater na caixa, ela vai se mexer.
     */ 
       
    if(posicao[0] < -0.35 && posicao[0] > -0.62 
       && posicao[1] < 0.0 && posicao[1] > -1.0
       && posicao[2] < -0.35 && posicao[2] > -0.62){           
      if(LeituraSensorProx[0] > 300 || LeituraSensorProx[1] > 250 
      || LeituraSensorProx[2] > 250 || LeituraSensorProx[5] > 250 
      || LeituraSensorProx[6] > 250 || LeituraSensorProx[7] > 300){                                                                 
         //coloquei para sempre acelerar quando encontrar a caixa, para que seja empurrada mais longe 
         AceleradorDireito = 1;
         AceleradorEsquerdo = 1;
          
          printf("\n\n\tColidiu com a caixa!!!\n\n");
          
          //pisca os 3 vezes leds quando colide com a caixa
          int pisca = 0;
          for(pisca; pisca < 3; pisca++){
            wb_led_set(Leds[0], wb_led_get(Leds[0])*-1);
            
            wb_led_set(Leds[1], wb_led_get(Leds[1])*-1);
            
            wb_led_set(Leds[2], wb_led_get(Leds[2])*-1);
            
            wb_led_set(Leds[3], wb_led_get(Leds[3])*-1);
            
            wb_led_set(Leds[4], wb_led_get(Leds[4])*-1);
            
            wb_led_set(Leds[5], wb_led_get(Leds[5])*-1);
            
            wb_led_set(Leds[6], wb_led_get(Leds[6])*-1);
            
            wb_led_set(Leds[7], wb_led_get(Leds[7])*-1);
            
          }
                    
          //flag de colisao vira 1 para saber que já bateu uma vez               
          fgcolisao = 1;   
        }                       
    }
    // se já houve colisao, então ele vai acender os demais leds    
    if(fgcolisao == 1){
       // utilizei até 7 leds para serem acessos    
       if(j < 7){
         //jogo a flag para 0 quando entra no if 
         fgcolisao = 0;
         wb_led_set(Leds[j], wb_led_get(Leds[j])*-1);
         j++;
       }
       //se chegar no limite de leds, ele reseta apagando todos
       if(j == 7){
         j = 0; 
       }
       
    }
    
    // nos "else if's" utilizei 6 sensores apenas para o movimento do robo  
    else if(LeituraSensorProx[0]>100){
      AceleradorDireito = 1;
      AceleradorEsquerdo = -0.62;
      }
          
     else if(LeituraSensorProx[1]>100) {
      AceleradorDireito = 1;
      AceleradorEsquerdo = -0.64;
     }
     
     else if(LeituraSensorProx[2]>100) {
      AceleradorDireito = 1;
      AceleradorEsquerdo = -0.69;
     }
     
     else if(LeituraSensorProx[5]>100) {
      AceleradorDireito = -0.68;
      AceleradorEsquerdo = 1;
     }
     
     else if(LeituraSensorProx[6]>100) {
      AceleradorDireito = -0.62;
      AceleradorEsquerdo = 1;
     }
     
     else if(LeituraSensorProx[7]>100) {
      AceleradorDireito = -0.64;
      AceleradorEsquerdo = 1;
     }
            
     else {
      AceleradorDireito = 1;
      AceleradorEsquerdo = 1;
     }
      
    wb_motor_set_velocity(MotorEsquerdo,6.28*AceleradorEsquerdo);
    wb_motor_set_velocity(MotorDireito, 6.28*AceleradorDireito);

  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}