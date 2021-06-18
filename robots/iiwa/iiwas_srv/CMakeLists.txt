cmake_minimum_required(VERSION 3.0.2)
project(iiwas_srv)

find_package(catkin REQUIRED COMPONENTS
             std_msgs
             message_generation)

add_service_files(
   FILES
   CancelMotion.srv
   StartHandguiding.srv
   StartPositionControl.srv
   PTP.srv
   SetBlueLight.srv
   SetESMState.srv
   SetImpedanceParam.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
