
#INPORTACION DE LIBRERIAS Y ELEMENTOS REQUERIDOS PARA LE EJECUCION DE LA SIMULACION
import os
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro
#DESCRIPCION DEL ARCHIVO DE LANZAMIENTO
def generate_launch_description():
    #CONFIGURACION DEL ENTORNO GAZEBO
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )
    #OBTENCION DE LA DESCRIPCION DEL MODELO ROBOTICO A PARTIR DEL ARCHIVO XACRO
    xacro_file = "/home/jonathan/dev_ws/src/parallel_robot/urdf/parallel_robot.xacro" 
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    #INICIO DEL NODO PARA PUBLICAR EL ESTADO DE LAS PARTES DEL ROBOT
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    #NODO PARA EJECUCION DEL MODELO DENTRO DEL ENTORNO GAZEBO
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'parallel_robot'],
                        output='screen')
    #CARGANDO EL CONTROLADOR PARA PUBLICAR EL ESTADO DE LAS ARTICULACIONES                    
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )
  
    #CARGANDO EL CONTROLADOR DE INGRESO DE FUERZA HACIA LAS ARTICULACIONES
    load_effort_controller = ExecuteProcess(
         cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
              'effort_controllers'],
         output='screen'
     )
     
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
         RegisterEventHandler( 
             event_handler=OnProcessExit( 
                target_action=load_joint_state_controller, 
                 on_exit=[load_effort_controller], 
             ) 
         ), 
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
       
    ])

