# ArduinoStatus.msg
# Published by arduino_driver_node. Reflects the current state of the
# Arduino connection and any status the sketch reports.

# True when the serial port is open and the board has replied.
bool connected

# Battery voltage reported by the sketch (0.0 if not implemented).
float32 battery

# True when an E-stop condition is active on the Arduino side.
bool estop

# Human-readable state string, e.g. "IDLE", "DRIVING", "ERROR".
string state

# Last raw command string successfully sent, e.g. "F40".
string last_cmd
