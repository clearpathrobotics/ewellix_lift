# Ewellix Lift State

# Actual positions of all six possible actuators
int32[]     actual_positions

# Commanded positions of all six possible actuators
int32[]     remote_positions

# Percentage speed of all six possible actuators
uint16[]    speeds

# Currents of all six possible actuators in 0.1 A
uint16[]    currents

# Status codes of all six possible actuators
uint8[]     status

# Error history, last 5 error codes
uint32[]    errors
