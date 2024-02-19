cmake_minimum_required(VERSION 3.0.2)
project(eddi_messages)


# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation

)

add_message_files(
  DIRECTORY msg
  FILES
  NodeOutput.msg
  BayesianNetworkOutput.msg
  SafeMLOutput.msg
  Guarantee.msg
  OutcomeOutput.msg
  ConSertOutput.msg

)

generate_messages(
  DEPENDENCIES

)

catkin_package(
  CATKIN_DEPENDS
  message_runtime

)
