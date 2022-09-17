 
import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, ThisLaunchFileDir, LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node, PushRosNamespace 
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch.event_handlers import OnProcessExit
 
import xacro
import launch_ros
import launch 
def generate_launch_description():
 
  # Obtener el paquete de origen
  pkg_share = FindPackageShare(package='parallel_robot').find('parallel_robot')
  
  # Obtener la descripcion URDF
  xacro_file = "/home/jonathan/dev_ws/src/parallel_robot/urdf/parallel_robot.xacro" 
  doc = xacro.parse(open(xacro_file))
  xacro.process_doc(doc)
  params = {'robot_description': doc.toxml()}
  default_urdf_model_path = os.path.join(pkg_share, 'urdf/parallel_robot.xacro')
    
  # Configuracion de parametros para simulacion
  gui = LaunchConfiguration('True')
  urdf_model = LaunchConfiguration('urdf_model')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  use_robot_state_pub = LaunchConfiguration('True')
  use_rviz = LaunchConfiguration('True')
  use_sim_time = LaunchConfiguration('True')
 
 
  # Especificar acciones
  declare_urdf_model_path_cmd = DeclareLaunchArgument(
    name='urdf_model', 
    default_value=default_urdf_model_path, 
    description='Absolute path to robot urdf file')
  
 
  # Publicacion de los valores de las articulaciones "joint_states" desde URDF.
  joint_state_publisher_cmd = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    output='screen',
    name='joint_state_publisher',
    parameters=[params]
    )
 
  # GUI para la manipulacion de los valores articulares
  joint_state_publisher_gui = Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    output='screen',
    name='joint_state_publisher_gui',
    parameters=[params]
    )
 
  # Suscripcion hacia los valores de joint_states y publcacion del modelo en 3D.
  robot_state_publisher_cmd = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[params],
    arguments=[default_urdf_model_path]
    )

  # Inicio de RViz2
  rviz_cmd = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', os.path.join(pkg_share, 'rviz/rviz_config_file.rviz')]
    )
    
  return LaunchDescription([

   declare_urdf_model_path_cmd, 
   joint_state_publisher_cmd,
   joint_state_publisher_gui,
   robot_state_publisher_cmd,
   rviz_cmd,
 ])
 
 
  