cmake_minimum_required(VERSION 2.8.3)
project(nao_interaction_msgs)

#List to make rest of code more readable
set( MESSAGE_DEPENDENCIES
    std_msgs
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    trajectory_msgs)

#Declare build dependencies
find_package(catkin REQUIRED 
    COMPONENTS
    message_generation
    ${MESSAGE_DEPENDENCIES} )

#Add message files
add_message_files(DIRECTORY msg
    FILES
    #~ nao_vision
    FaceDetected.msg 
    MovementDetected.msg 
    LandmarkDetected.msg 
    #~ nao_audio
    AudioSourceLocalization.msg
)

#Add service files
add_service_files(DIRECTORY srv
    FILES
    #~ nao_vision
    VisionMotionSensitivity.srv
    LearnFace.srv
    #~ nao_audio
    AudioRecorder.srv
    AudioMasterVolume.srv
    AudioPlayback.srv
)

#And now generate the messages
generate_messages(DEPENDENCIES ${MESSAGE_DEPENDENCIES})

#Declare package run-time dependencies
catkin_package( CATKIN_DEPENDS message_runtime ${MESSAGE_DEPENDENCIES})

