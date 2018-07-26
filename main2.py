"""
Proof-of-concept RSI communicator

Set IP to 172.31.1.146
    $ sudo ifconfig enp3s0 172.31.1.146 netmask 255.255.255.0 up
"""

import socket
# xml parser
from xml.dom import minidom


SERVER_IP = '172.31.1.146'
SERVER_PORT = 49152

BUFFER_SIZE = 1024

HEADER = '<Sen Type=\"ImFree\"><EStr>KRCnexxt - RSI Object ST_ETHERNET</EStr>'
DEFAULT_INSTRUCTION = '<RKorr X=\"0.0000\" Y=\"0.0000\" Z=\"0.0000\" A=\"0.0000\" \
               B=\"0.0000\" C=\"0.0000\" />'
FOOTER = '<Tech T21=\"1.09\" T22=\"2.08\" T23=\"3.07\" T24=\"4.06\" T25=\"5.05\" T26=\"6.04\" T27=\"7.03\" ' \
           'T28=\"8.02\" T29=\"9.01\" T210=\"10.00\"/><DiO>125</DiO><IPOC>0000000000</IPOC></Sen>'


# TODO: re-write
# The function below is taken from kuka-rsi3-communicator
# by Eren Sezener and Osman Kaya (https://github.com/erensezener/kuka-rsi3-communicator)
# Big thank you to their project for helping me to understand the communication
# protocol.

#####################################################
# Updates the timestamp of the data to send based on the timestamp of the received data
def mirror_timestamp(received_data, data_to_send):
    ipoc_begin_index = received_data.index("<IPOC>")
    ipoc_end_index = received_data.index("</IPOC>")
    received_ipoc = received_data[ipoc_begin_index + 6: ipoc_end_index]

    old_ipoc_begin_index = data_to_send.index("<IPOC>")
    old_ipoc_end_index = data_to_send.index("</IPOC>")
    old_ipoc = data_to_send[old_ipoc_begin_index + 6: old_ipoc_end_index]

    return data_to_send.replace("<IPOC>" + old_ipoc + "</IPOC>", "<IPOC>" + received_ipoc + "</IPOC>")
#####################################################


def graceful_exit(socket):
    # socket.shutdown(socket.SHUT_RDWR)
    socket.close()
    print('\nsocket closed')
    print('exiting...')
    exit()


def display_angle_data(ang_data):
    print('Angle data:')
    print('- a1: %s' % ang_data['a1'])
    print('- a2: %s' % ang_data['a2'])
    print('- a3: %s' % ang_data['a3'])
    print('- a4: %s' % ang_data['a4'])
    print('- a5: %s' % ang_data['a5'])
    print('- a6: %s' % ang_data['a6'])


def display_position_data(pos_data):
    print('\nPosition data:')
    print('- x: %s' % pos_data['x'])
    print('- y: %s' % pos_data['y'])
    print('- z: %s' % pos_data['z'])
    print('- a: %s' % pos_data['a'])
    print('- b: %s' % pos_data['b'])
    print('- c: %s' % pos_data['c'])


def send_command(s, target_pos):
    received_data, socket_of_krc = s.recvfrom(BUFFER_SIZE)
    print('data received:')
    print(received_data)


    xml_response = minidom.parseString(received_data)

    ang_data_xml = xml_response.getElementsByTagName('AIPos')[0]
    pos_data_xml = xml_response.getElementsByTagName('RIst')[0]

    ang_data = {
        'a1': float(ang_data_xml.getAttribute('A1')),
        'a2': float(ang_data_xml.getAttribute('A2')),
        'a3': float(ang_data_xml.getAttribute('A3')),
        'a4': float(ang_data_xml.getAttribute('A4')),
        'a5': float(ang_data_xml.getAttribute('A5')),
        'a6': float(ang_data_xml.getAttribute('A6'))
    }

    pos_data = {
        'x': float(pos_data_xml.getAttribute('X')),
        'y': float(pos_data_xml.getAttribute('Y')),
        'z': float(pos_data_xml.getAttribute('Z')),
        'a': float(pos_data_xml.getAttribute('A')),
        'b': float(pos_data_xml.getAttribute('B')),
        'c': float(pos_data_xml.getAttribute('C'))
    }

    display_angle_data(ang_data)
    display_position_data(pos_data)

    print('')


    # For reference - default cartesian position:
    # x: 280.0
    # y:   0.0
    # z: 550.0
    # a: 180.0
    # b:   0.0
    # c: 180.0

    x = '0.0'
    y = '0.0'
    z = '0.0'
    a = '0.0'
    b = '0.0'
    c = '0.0'

    threshold = 0.5
    base_vel = 0.02
    max_vel = 0.1
    diff_divisor = 850.0

    diff_x = target_pos['x'] - pos_data['x']
    diff_y = target_pos['y'] - pos_data['y']
    diff_z = target_pos['z'] - pos_data['z']

    if (diff_x < threshold and diff_y < threshold and diff_z < threshold):
        print('Position reached!')
        print('Ending communication.')
        return True

    # cartesian co-ordinate corrections to make every ~12ms(?)
    # x = '-0.1' if (diff_x >= threshold) else '0.0'
    # y = '0.1'  if (diff_y >= threshold) else '0.0'
    # z = '-0.1' if (diff_z >= threshold) else '0.0'

    if (abs(pos_data['x'] - target_pos['x']) > threshold):
        if (pos_data['x'] < target_pos['x']):
            base_vel_x = base_vel
            x = str(min(max_vel, base_vel_x + (diff_x / diff_divisor)))
        else:
            base_vel_x = -(base_vel)
            x = str(max(-max_vel, base_vel_x + (diff_x / diff_divisor)))

    if (abs(pos_data['y'] - target_pos['y']) > threshold):
        if (pos_data['y'] < target_pos['y']):
            base_vel_y = base_vel
            y = str(min(max_vel, base_vel_y + (diff_y / diff_divisor)))
        else:
            base_vel_y = -(base_vel)
            y = str(max(-max_vel, base_vel_y + (diff_y / diff_divisor)))

    if (abs(pos_data['z'] - target_pos['z']) > threshold):
        if (pos_data['z'] < target_pos['z']):
            base_vel_z = base_vel
            z = str(min(max_vel, base_vel_z + (diff_z / diff_divisor)))
        else:
            base_vel_z = -(base_vel)
            z = str(max(-max_vel, base_vel_z + (diff_z / diff_divisor)))

    print('Making adjustments:')
    print('- x: %s' % x)
    print('- y: %s' % y)
    print('- z: %s' % z)
    print('- a: %s' % a)
    print('- b: %s' % b)
    print('- c: %s' % c)
    print('')

    """
    instruction_template = \
        '<AKorr A1=\"%s\" A2=\"%s\" A3=\"%s\" A4=\"%s\" A5=\"%s\" A6=\"%s\" />'

    instruction = instruction_template % (a1, a2, a3, a4, a5, a6)
    """

    instruction_template = \
        '<RKorr X=\"%s\" Y=\"%s\" Z=\"%s\" A=\"%s\" B=\"%s\" C=\"%s\" />'

    instruction = instruction_template % (x, y, z, a, b, c)

    command = HEADER + instruction + FOOTER
    data_to_send = mirror_timestamp(received_data, command)
    print('data to send:')
    print(data_to_send)

    s.sendto(data_to_send, socket_of_krc)
    print('\nsent!')

    print('\n---\n')

    return False


def main():
    # Create a UDP socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Re-use socket
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((SERVER_IP, SERVER_PORT))
    xml_file = open('DataTemplate.xml', 'r')
    default_command = xml_file.read()

    print('socket opened')
    print('waiting for robot connection...')

    try:
        target_pos = {
            'x': 360.0,
            'y': 350.0,
            'z': 30.0
        }
        while(True):
            is_reached = send_command(s, target_pos)
            if (is_reached):
                break
        graceful_exit(s)
    except KeyboardInterrupt:
        graceful_exit(s)


if (__name__ == '__main__'):
    main()
