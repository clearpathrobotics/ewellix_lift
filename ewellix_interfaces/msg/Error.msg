# Ewellix Lift Latest Error

int32 code

# CRC error with ROM test. Faulty ROM. Motions are stopped and the control unit carries out a reset.
bool fault_ram

# Error with RAM test. Faulty RAM. Motions are stopped and the control unit carries out a reset.
bool fault_rom

# Error with CPU test. Faulty CPU. Motions are stopped and the control unit carries out a reset.
bool fault_cpu

# STACK overrun detected. Motions are stopped (fast stop) and the control unit carries out a reset.
bool stack_overrun

# Program sequence error. Watchdog reset. Motions are stopped (fast stop) and the control unit carries out a reset.
bool sequence_error

# Error with hand switch test. Short detected in hand switch. Only occurs if hand switch is parameterized as 'safe'. Motions are stopped (fast stop).
bool hand_switch_short

# Error with binary inputs. Short detected between binary inputs. Only occurs if binary inputs are parameterized as safe and no analogue input is parameterized. Motions are stopped (fast stop).
bool binary_inputs_short

# Error with relay and FET tests. Faulty relay or FET. Test performed at start of motion. Motion not executed.
bool faulty_relay

# Error with communication with MoveEnable controller. No reply form MoveEnable controller. Motions stopped (fast stop).
bool move_enable_comms

# Error with MoveEnable output test. The MoveEnable controller output is incorrect. Motions stopped (fast stop).
bool move_enable_incorrect

# Over-temperature detected at rectifier or FET. Motions stopped (fast stop).
bool over_temperature

# Switching off due to excessive discharge of battery. Motions stopped (fast stop). Control unit switches itself off.
bool battery_discharge

# Total current is exceeded. Occurs if motion in process. Motions stopped (fast stop). Bit reset in the next motion.
bool over_current

# Error with drive. Occurs when peak current reached, short circuit current, sensor monitor, over current or timeout. Drive stopped (fast stop). Bit reset on next motion.
bool drive_1_error
bool drive_2_error
bool drive_3_error
bool drive_4_error
bool drive_5_error
bool drive_6_error

# Position between drives too great. Only if synchronized parallel run is parameterized. Motion not started. If motion ins progress the motion is stopped (fast stop). Bit reset on next motion.
bool position_difference

# Remote communication timeout. Depends on SafetyMode set at activation.
bool remote_timeout

# Locking box I2C communication error. Only if locking box parameterized as 'safe'. Motion not performed or stopped
bool lockbox_comm_error

# RAM copy of EEPROM configuration data indicates incorrect CRC. Motion not performed or stopped.
bool ram_config_data_crc

# RAM copy of EEPROM user data indicates incorrect CRC. Motion not performed or stopped.
bool ram_user_data_crc

# EEPROM locking box data indicates incorrect CRC. Motion not performed or stopped.
bool ram_lockbox_data_crc

# RAM copy of EEPROM dynamic data indicate incorrect CRC. Motion not performed or stopped.
bool ram_dynamic_data_crc

# RAM copy of EEPROM calibration data indicate incorrect CRC. Motion not performed or stopped.
bool ram_calib_data_crc

# RAM copy of EEPROM HW settings indicate incorrect CRC. Motion not performed or stopped.
bool ram_hw_settings_crc

# IO test performed if no motion is active. Motion not performed.
bool io_test

# IDF operating system error. Motion not performed or stopped.
bool idf_opsys_error
