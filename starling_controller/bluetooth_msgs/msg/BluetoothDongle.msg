# Standard message header which includes a timestamp 
std_msgs/Header header

# 0 == no failed node // other id == failed_id
uint8 failed_id 

# A custom id from zero onwards specifing the unique vehicle_id
uint8 node_id

# state for future use
uint8 state

#timestamp
float64 test_num

# Drone postion x
float64 position_x

# Drone postion Y
float64 position_y