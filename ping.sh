rosservice call /gazebo/unpause_physics
rostopic pub -1 /tailsitter/command/motor_speed mav_msgs/Actuators "{header: auto, normalized: [1,-1,0.7,0.7]}" && rosservice call /gazebo/reset_simulation && rosservice call /gazebo/reset_simulation
# does not works
# rosservice call /gazebo/apply_body_wrench '{body_name: "tailsitter::tailsitter/base_link",reference_point: { x: 0, y: 0, z: 0 },  wrench: {force: {x: 5.0, y: 5.0, z: 5.0}}, start_time: 0, duration: -1}'
