import socket
import time
from dronekit import connect, VehicleMode

# Connect to the vehicle
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port where the server is listening
server_address = ('192.168.0.1', 55555)  # replace with your server's port
print('connecting to {} port {}'.format(*server_address))
sock.connect(server_address)

def get_NMEA_sentence(vehicle):
    # For simplicity, we'll just use a placeholder NMEA sentence
    # Normally, you'd generate a real NMEA sentence using vehicle's GPS data
    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon
    alt = vehicle.location.global_relative_frame.alt

    # Very simplified conversion to NMEA format, consider using a proper library for this
    nmea_lat = "{:02.0f}{:07.4f}".format(abs(lat), (abs(lat) % 1) * 60)
    nmea_lon = "{:03.0f}{:07.4f}".format(abs(lon), (abs(lon) % 1) * 60)

    # Just a placeholder, refer NMEA GGA sentence for proper format
    nmea_sentence = "$GPGGA,123519,{},N,{},E,1,08,0.9,{},M,0.0,M,,*47\r\n".format(nmea_lat, nmea_lon, alt)
    return nmea_sentence

try:
    while True:
        # Generate the NMEA sentence
        sentence = get_NMEA_sentence(vehicle)

        # Send data
        print('sending {!r}'.format(sentence))
        sock.sendall(sentence.encode('utf-8'))

        # Wait a bit before sending the next one
        time.sleep(1)

finally:
    print('closing socket')
    sock.close()
    vehicle.close()