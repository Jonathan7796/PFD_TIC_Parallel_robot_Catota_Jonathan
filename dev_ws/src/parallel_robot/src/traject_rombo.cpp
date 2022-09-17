//INCLUSION DE LIBRERIAS
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "my_messages/msg/mensajes.hpp" 
#include "std_msgs/msg/float64_multi_array.hpp"

#include <iostream>     // std:cout
#include <fstream>      //std ifstream 
#include <sstream>     // std:istringstream
#include <iomanip>

using std::placeholders::_1;
using namespace std::chrono_literals;

//DECLARACION DE PARAMETROS GLOBALES
float kp,ki,kd,kp4,ki4,kd4;
int iter = 0;
float ts;
float tiempo = 0;
float max_f;
float qd[4];

float error_limit;
float error_pos[4];
float PID[4];
float current_speed[4];
float cumerror[4];
float lasterror[4];
float last_velocity[4];
float der_error_pos[4];
float der_velocity[4];
float joint[3],joint_v[3];
int sel=0;

 std::vector<double> sp, offset;
 std::ifstream infile;
 std::string line;

FILE* fichero0;
FILE* fichero1;
FILE* fichero2;
FILE* fichero3;

//CREACION DEL NODO DE CONTROL PARA TRAYECTORIA
class ControlAction : public rclcpp::Node
{

public:
  ControlAction()
  : Node("pid_control")
  {
    RCLCPP_INFO(this->get_logger(), "Inicio de Trayectoria tipo Rombo");
    //DECLARACION DE PARAMETROS
        this->declare_parameter<float>("ts_ms", 1.0);
    //Parametros PID para piernas externas
        this->declare_parameter<double>("kp", 2500);    //50 
        this->declare_parameter<double>("ki",0.1);    //5  //100
        this->declare_parameter<double>("kd", 20);  //10
    //Parametro PID para pierna central
        this->declare_parameter<double>("kp4", 2500);    //50 
        this->declare_parameter<double>("ki4",1);    //5  //100
        this->declare_parameter<double>("kd4", 20);  //10
        this->declare_parameter<float>("max_f", 1000);
    //Ingreso de parametros hacia variables globales  
        this->get_parameter("kp", kp);
        this->get_parameter("ki", ki);
        this->get_parameter("kd", kd);
        this->get_parameter("kp", kp4);
        this->get_parameter("ki", ki4);
        this->get_parameter("kd", kd4);
        this->get_parameter("ts_ms", ts);
        this->get_parameter("max_f", max_f);
    //Reseteo del error para el ocntrol PID    
           for( int i = 0; i < 4; i++)
          {
          lasterror[i]=0;
          cumerror[i]=0;
           }

    //INICIO DE LA COMUNICACION
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&ControlAction::topic_callback, this, _1)); 
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controllers/commands",10);
   
    //CREACION DE ARCHIVOS DE DATOS PARA PODER OBTENER GRAFICAS DE RESULTADOS
    fichero0 = fopen("/home/jonathan/dev_ws/src/parallel_robot/results/Posiciones.txt", "wt");
    fichero1 = fopen("/home/jonathan/dev_ws/src/parallel_robot/results/time.txt", "wt");
    fichero2 = fopen("/home/jonathan/dev_ws/src/parallel_robot/results/referencias.txt", "wt");
    fichero3 = fopen("/home/jonathan/dev_ws/src/parallel_robot/results/Fuerzas.txt", "wt");
    //LECTURA DE TRAYECTORIA
    sp.resize(4,0);
    infile.open("/home/jonathan/dev_ws/src/parallel_robot/results/try_rombo.txt");
  }

private:
 
  void topic_callback(const sensor_msgs::msg::JointState   & msg) const//const //const  // CHANGE
  {
    //OBTENCION DE POSICIONES ARTICULARES PARA EJECUCION DE TRAYECTORIA
    std::getline(infile, line);
    std::istringstream iss(line);
      if(!(iss>>sp[0]>>sp[1]>>sp[2]>>sp[3]))
          {
          }
          else
          {
            
            //If axis length are less than min length of actuators don't send any value
            if( (sp[0] >= 0) && (sp[1] >= 0) && (sp[2] >= 0) && (sp[3] >= 0) &&
              (sp[0] <= 300) && (sp[1] <= 300) && (sp[2] <= 300) && (sp[3] <= 300) )
            {
              for( int i = 0; i < 4; i++)
              {
                qd[i] = sp[i] ;
              }
            }
          }
     std_msgs::msg::Float64MultiArray commands;

    //ADQUISICION DE POSICIONES ARTICULARES MEDIANTE CONTROLADOR JOINT_STATE_BROADCASTER
      joint[0]=msg.position[0];
      joint[1]=msg.position[1];
      joint[2]=msg.position[2];
      joint[3]=msg.position[3];

      joint_v[0] = msg.velocity[0];
      joint_v[1] = msg.velocity[1];
      joint_v[2] = msg.velocity[2];
      joint_v[3] = msg.velocity[3];

      tiempo=iter*ts/1000;
    //LAZO DE CONTROL CONJUNTO DE LAS ARTICULACIONES
      for (int i = 0; i < 4; i++)
           {
             error_pos[i]= qd[i]-joint[i];
             der_error_pos[i] = (error_pos[i]-lasterror[i])/0.0001;  //0.5;
             cumerror[i] = cumerror[i] + (error_pos[i]*0.0001) ;   //* 0.1);           
             lasterror[i]=error_pos[i];    //erroranterior

            //Impresion de datos hacia archivos txt.          
            fprintf(fichero0,"%f \t",joint[i]);
            fprintf(fichero1,"%f \t",tiempo);
            fprintf(fichero2,"%f \t",qd[i]);
            fprintf(fichero3,"%f \t",PID[i]);    
            
            }
    //EXPORTACION DE RESULTADOS HACIA txt.
       fprintf(fichero0,"\n");
       fprintf(fichero1,"\n");
       fprintf(fichero2,"\n");
       fprintf(fichero3,"\n");


//CONTROL INDIVIDUAL POR CADA ARTICULAICON

        
       PID[0] = kp * error_pos[0]+ ki * cumerror[0] + kd * der_error_pos[0]; //
       PID[1] = kp * error_pos[1]+ ki * cumerror[1] + kd * der_error_pos[1];
       PID[2] = kp * error_pos[2]+ ki * cumerror[2] + kd * der_error_pos[2];
       PID[3] = kp4 * error_pos[3]+ ki4 * cumerror[3] + kd4 * der_error_pos[3];

       for (int i = 0; i < 4; i++)
           {
             if(PID[i] > max_f) PID[i] = max_f;
             if(PID[i] < -max_f) PID[i] = -max_f;
            
            }
//ENVIO DE ACCIONES DE CONTROL
    
       commands.data.push_back(0);
       commands.data.push_back(1);
       commands.data.push_back(2);
       commands.data.push_back(3);
    
       publisher_->publish(commands);
       
     
       commands.data[0] = PID[0]; 
       commands.data[1] = PID[1]; 
       commands.data[2] = PID[2]; 
       commands.data[3] = PID[3]; 
       publisher_->publish(commands);
       
//IMPRESION DE ACCIONES DE CONTROL HACIA CADA ARTICULACION
    //   RCLCPP_INFO(this->get_logger(), "PID positions '%f'", commands.data[0]);
    //   RCLCPP_INFO(this->get_logger(), "PID positions '%f'", commands.data[1]);
    //   RCLCPP_INFO(this->get_logger(), "PID positions '%f'", commands.data[2]);
    //   RCLCPP_INFO(this->get_logger(), "PID positions '%f'", commands.data[3]);

     
      iter++;    
  }
   rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_; 
   rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subscription_2;  
   rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlAction>());
  rclcpp::shutdown();
   
    
  return 0;
}


