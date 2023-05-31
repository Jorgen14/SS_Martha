from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('udpin:localhost:57600')

# Wait for the first heartbeat
# This sets the system and component ID of remote system for the link
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

# Get some information !
while True:
    try:
        altitude = master.messages['GLOBAL_POSITION_INT'].alt  # Note, you can access message fields as attributes!
        print('Altitude:', altitude)
        time.sleep(1)
    except:
        pass