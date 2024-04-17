from pymavlink import mavutil
import time

# Create a MAVLink connection
master = mavutil.mavlink_connection('COM4', baud=57600)

# Wait for the heartbeat message to find the system ID and component ID
print("Waiting for heartbeat")
master.wait_heartbeat()
print(f"Heartbeat from system (system ID {master.target_system}, component ID {master.target_component})")

# message = master.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
# if message:
#     print(message)

# servo_output = master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=5)
# if servo_output:
#     print(servo_output)

def set_arming_check(value):
    """
    Set the ARMING_CHECK parameter.
    :param value: The value to set for ARMING_CHECK.
    """
    print(f"Setting ARMING_CHECK to {value}")
    # Ensure the parameter name is a byte string
    param_name = b'ARMING_CHECK'
    # Specify the parameter type; MAV_PARAM_TYPE_INT32 for an integer
    param_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_name,
        value,
        param_type
    )
    # Wait a bit for the parameter to be set
    time.sleep(2)

def check_vehicle_status():
    state = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if state:
        armed = "ARMED" if state.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED else "DISARMED"
        mode = mavutil.mode_string_v10(state)
        print(f"Vehicle is {armed}. Current Mode: {mode}")
    else:
        print("Failed to receive vehicle state.")

def set_manual():

    # Fetch the mode mapping
    mode_mapping = master.mode_mapping()

    # Manual mode (example might need adjustment based on vehicle type)
    manual_mode = mode_mapping['MANUAL'] if 'MANUAL' in mode_mapping else None
    if manual_mode is None:
        print("Manual mode not found in mode mapping.")
        return

    try:
        # Request to change the mode
        print("Setting mode to MANUAL...")
        master.set_mode_manual()
        print("Mode set command sent. Please check the vehicle status.")

    except Exception as e:
        print(f"Failed to set MANUAL mode: {e}")


def check_pre_arm_conditions():
    """
    Checks various conditions that are typically necessary before arming the drone.
    Returns True if all checks pass, False otherwise.
    """
    # GPS Status Check
    gps_status = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=3)
    if not gps_status or gps_status.fix_type < 3:
        print("Pre-arm check failed: GPS fix not sufficient.")
        return False

    # Battery Level Check
    battery_status = master.recv_match(type='BATTERY_STATUS', blocking=True, timeout=3)
    if not battery_status or battery_status.battery_remaining > 10:  # Check if battery level is below 20%
        print("Pre-arm check failed: Battery level is too low.")
        return False
    
    # RC (Remote Control) Signal Check
    rc_status = master.recv_match(type='RC_CHANNELS_RAW', blocking=True, timeout=3)
    if not rc_status or rc_status.rssi < 30:  # Assuming RSSI is a measure of signal strength
        print("Pre-arm check failed: RC signal strength is too low.")
        return False

    print("All pre-arm checks passed.")
    return True


def arm_drone():
    print("Attempting to arm the drone...")

    # Check pre-arm conditions manually if needed
    # if not check_pre_arm_conditions():
    #     print("Pre-arm checks failed. Drone not armed.")
    #     return

    state = master.recv_match(type='HEARTBEAT', blocking=True)
    if state and not (state.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        try:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,  # Confirmation
                1,  # 1 to arm
                0, 0, 0, 0, 0, 0  # Unused parameters
            )
            print("Arming command sent. Waiting for confirmation...")

            # Listen for the next heartbeat to confirm arming
            while True:
                state = master.recv_match(type='HEARTBEAT', blocking=True, timeout=30)
                if state:
                    if state.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                        print("Drone is now armed.")
                        break
                    else:
                        # Check for statustext if arming didn't succeed
                        status_message = master.recv_match(type='STATUSTEXT', blocking=True, timeout=5)
                        print(f"Failed to arm the drone: {status_message}")

                        break
                else:
                    print("Failed to confirm arming status.")
                    break
        except Exception as e:
            print(f"An error occurred while attempting to arm the drone: {e}")
    else:
        if state:
            print("Drone is already armed.")
        else:
            print("Failed to fetch current state.")

def disarm_drone():
    print("Attempting to disarm the drone...")
    try:
        # Disarm the drone using the arducopter_disarm function
        master.arducopter_disarm()
        # Alternatively, you can use the command_long_send method directly:
        # master.mav.command_long_send(
        #     master.target_system,
        #     master.target_component,
        #     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        #     0,  # Confirmation
        #     0,  # 0 to disarm
        #     0, 0, 0, 0, 0, 0  # Unused parameters
        # )
        print("Disarm command sent.")
    except Exception as e:
        print(f"An error occurred while attempting to disarm the drone: {e}")

# Function to inject fake GPS data
def inject_fake_gps(lat, lon, alt):
    """
    Injects fake GPS data for testing purposes.
    :param lat: Latitude in decimal degrees
    :param lon: Longitude in decimal degrees
    :param alt: Altitude in meters
    """
    master.mav.gps_input_send(
        0,  # Timestamp (ignored)
        0,  # GPS ID
        0,  # Ignore flags
        0,  # Time week (ignored)
        0,  # GPS fix type (3D fix)
        234,  # Latitude (degrees * 1e7)
        23,  # Longitude (degrees * 1e7)
        123,  # Altitude (MSL)4
        1,  # HDOP
        1,  # VDOP
        0,  # Velocity (ignored)
        0,  # Course over ground (ignored)
        10,  # Number of satellites visible
        0,  # Altitude (ellipsoid)
        0, 0, 0, 0  # State, pDop, reserved
    )
    print(f"Injected fake GPS data: Lat {lat}, Lon {lon}, Alt {alt}")


def trigger_esc_beep(channel, pwm_value=1100):
    print(f"Attempting to trigger beep on channel {channel} with PWM value {pwm_value}...")
    result = master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,
        pwm_value,
        0, 0, 0, 0, 0
    )
    print(f"Command sent to set PWM {pwm_value} on channel {channel}. Result: {result}")
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)
    if ack:
        print(f"Received ACK: {ack}")
    else:
        print("No ACK received.")



#inject_fake_gps(37.7749, -122.4194, 10)  # Example coordinates
#check_pre_arm_conditions()

#set_arming_check(0)

def spin_motors(duration=5):
    start_time = time.time()  # Record the start time
    print("Spinning motors at minimal throttle...")
    
    # Loop to continuously send override commands for the duration
    while (time.time() - start_time) < duration:
        master.mav.rc_channels_override_send(
            master.target_system, 
            master.target_component,
            0,  # Roll
            0,  # Pitch
            1300,  # Throttle (minimal value to spin motors)
            0,  # Yaw
            0, 0, 0, 0  # Other channels
        )
        time.sleep(0.5)  # Short delay between commands to not overwhelm the communication link

    print(f"Motors have been spinning for {duration} seconds, stopping now...")
    stop_motors()

def stop_motors():
    print("Sending stop command...")
    for _ in range(5):  # Multiple attempts to ensure the stop command is received
        master.mav.rc_channels_override_send(
            master.target_system, 
            master.target_component,
            0, 0, 0, 0, 0, 0, 0, 0  # Clear the override to stop the motors
        )
        time.sleep(1)  # Short delay between stop commands


def takeoff(altitude):
    """
    Commands the vehicle to take off and climb to the specified altitude.
    :param altitude: The altitude to climb to in meters.
    """
    print(f"Commanding takeoff to {altitude} meters altitude.")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # Command ID for takeoff
        0,  # Confirmation: 0 to execute immediately, 1 to confirm
        0,  # Param1, pitch angle, set to 0 if not used
        0,  # Param2, empty
        0,  # Param3, empty
        0,  # Param4, Yaw angle, set to NaN to ignore
        0,  # Param5, Latitude, set to NaN if not used
        0,  # Param6, Longitude, set to NaN if not used
        altitude  # Param7, Altitude, the altitude to climb to
    )
    # Wait for the vehicle to confirm the command
    # Add any necessary logic to verify that the vehicle has received the command and is executing it

def land_drone():
    """
    Function to command the drone to land at the current location.
    """
    print("Commanding the drone to land...")
    # Send the land command
    master.mav.command_long_send(
        master.target_system,  # target system
        master.target_component,  # target component
        mavutil.mavlink.MAV_CMD_NAV_LAND,  # command
        0,  # confirmation
        0, 0, 0, 0,  # Parameters 1-4 are not used
        0, 0,  # Latitude (0 means use current location)
        0, 0   # Longitude (0 means use current location)
    )
    print("Landing command sent.")

def set_parameter(param_id, value, param_type):
    """
    Set a parameter on the drone.
    :param param_id: The ID of the parameter as a string or bytes
    :param value: The value to set the parameter to
    :param param_type: The type of the parameter
    """
    # If param_id is a string, encode it to bytes; if it's bytes, use as is
    param_id_bytes = param_id.encode() if isinstance(param_id, str) else param_id
    print(f"Setting {param_id} to {value}")
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_id_bytes,  # Use the bytes version of param_id
        value,
        param_type
    )
    # It's important to wait for the parameter to be confirmed before moving on
    # We will wait for the PARAM_VALUE message with the confirmation
    while True:
        message = master.recv_match(type='PARAM_VALUE', blocking=True)
        # param_id_bytes needs to be converted to string for comparison if message.param_id is a string
        param_id_str = param_id_bytes.decode('utf-8') if isinstance(message.param_id, str) else param_id_bytes
        if message and message.param_id == param_id_str:
            print(f"{param_id_str} confirmed at {message.param_value}")
            break

def check_parameter(param_id):
    """
    Request the value of a specific parameter from the drone and print it.
    :param param_id: The ID of the parameter as a string or bytes
    """
    # Ensure the parameter ID is in bytes
    param_id_bytes = param_id.encode() if isinstance(param_id, str) else param_id
    print(f"Requesting value for parameter {param_id_bytes.decode()}")

    # Request the parameter value
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        param_id_bytes,
        -1  # Use -1 to request the parameter by name
    )

    # Wait for the parameter value response
    message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=10   )
    print(message)
    if message and message.param_id == param_id_bytes.decode():
        print(f"Parameter {param_id_bytes.decode()}: Value = {message.param_value}")
    else:
        print(f"Failed to receive value for parameter {param_id_bytes.decode()}")

# Example usage:


# #Disable the pre-arm checks by setting the parameters
# set_parameter(b'EKF2_GPS_CHECK', 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
# set_parameter(b'COM_ARM_MAG_STR', 10000, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
# set_parameter(b'EKF2_MAG_CHECK', 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)

# check_parameter(b'EKF2_GPS_CHECK')
# check_parameter(b'COM_ARM_MAG_STR')
# check_parameter(b'EKF2_MAG_CHECK')
#check_parameter(b"GPS_TYPE")
# check_pre_arm_conditions()
# set_arming_check(0)
# check_parameter(b'ARMING_CHECK')

# arm_drone()
# check_vehicle_status()
#takeoff(10)

# time.sleep(5)

# disarm_drone()

#spin_motors(duration=10)  # Adjust duration as needed



#EKF2_GPS_CHECK - gps
#COM_ARM_MAG_STR and EKF2_MAG_CHECK - magnetic interference
#COM_RC_IN_MODE mode
#COM_ARM_MAG_ANG set to -1
#COM_ARM_WO_GPS
#EKF2_HGT_REF

# mav_cmd_do_set_mode
def set_px4_parameter(param_id, value):
    """
    Set a parameter on a PX4 drone.
    :param param_id: The ID of the parameter as a string
    :param value: The value to set the parameter to
    """
    print(f"Setting {param_id} to {value}")
    param_id_bytes = param_id.encode()  # Ensure the parameter name is encoded to bytes
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_id_bytes,  # Use the bytes version of param_id
        value,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    # Wait for confirmation
    while True:
        message = master.recv_match(type='PARAM_VALUE', blocking=True)
        if message and message.param_id.decode() == param_id:
            print(f"{param_id} confirmed at {message.param_value}")
            break

def arm_disarm(value):
    """
    Directly arm a PX4 drone.
    """
    print("Attempting to arm the drone...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # Confirmation
        value,  # Param1: 1 to arm
        0, 0, 0, 0, 0, 0  # Unused parameters
    )
    print("Arming command sent. Please check the vehicle status.")

# Disable arming checks
#set_px4_parameter('COM_ARM_CHK', 0)

# Give it a moment for the parameter change to take effect
#time.sleep(2)

# Attempt to arm the drone
#arm_px4_drone()

def arm_by_rc(value):
    """
    Function to arm the drone by simulating RC stick movement to the bottom right position.
    This typically involves setting the throttle to its minimum and yaw to its maximum.
    """
    print("Attempting to arm the drone by RC...")
    # Override RC channels: 
    # Assuming channel 3 is throttle and channel 4 is yaw (adjust these as per your setup)
    # Throttle to minimum (e.g., 1100), yaw to maximum (e.g., 1900)
    # Set other channels (roll, pitch) to neutral (1500) if needed
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1500,  # Roll (set to neutral)
        1500,  # Pitch (set to neutral)
        value,  # Throttle (set to minimum to arm)
        1900,  # Yaw (set to maximum to arm)
        0,     # Optional other channels
        0,
        0,
        0
    )
    print("RC override sent to arm the drone.")

# Call the arm function
#arm_by_rc()

#disarm_drone()
