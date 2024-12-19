# Ewellix Lift State

int32[]     actual_positions # actual positions of actuators
int32[]     remote_positions # commanded positions of actuators
uint16[]    speeds           # percentage speed of actuators
uint16[]    currents         # currents of actuators in 0.1 A
uint8[]     status           # status of actuators
uint32[]    errors           # error history, last 5 errors
