#!/usr/bin/python3

import binascii
import logging
import os
import serial
import sys
import time

# set up some basic commands here
battery_query = bytes([0x80, # header
                       0x10, # destination = Subaru ECU
                       0xF0, # source = Diagnostic tool
                       0x05, # Data size...
                       0xA8, # command = Address read
                       0x00, # single response
                       0x0, 0x0, 0x1C]) # battery voltage (x*.08)

rpm_query = bytes([0x80, # header
                   0x10, # destination = Subaru ECU
                   0xF0, # source = Diagnostic tool
                   0x08, # Data size...
                   0xA8, # command = Address read
                   0x00, # single response
                   0x00, 0x00, 0x0E,  # Engine Speed - 2 bytes (x * 0.25)
                   0x00, 0x00, 0x0F]) # Engine Speed - 2 bytes (x * 0.25)

data_query = bytes([0x80, # header
                    0x10, # destination = Subaru ECU
                    0xF0, # source = Diagnostic tool
                    0x4A, # Data size...
                    0xA8, # command = Address read
                    0x00, # single response
                    0x00, 0x00, 0x09,  # AF Correction #1 ((x-128) * 0.78125)
                    0x00, 0x00, 0x0A,  # AF Learning #1 ((x-128) * 0.78125)
                    0x00, 0x00, 0x46,  # AF Sensor #1 (x * 0.11484375)
                    0x02, 0x00, 0xF4,  # Engine load - 2 bytes (x * 0.0001220703125)
                    0x02, 0x00, 0xF5,  # Engine load - 2 bytes (x * 0.0001220703125)
                    0x00, 0x00, 0x13,  # Mass Airflow - 2 bytes (x * 0.01)
                    0x00, 0x00, 0x14,  # Mass Airflow - 2 bytes (x * 0.01)
                    0x02, 0x17, 0xB6,  # Manifold Relative Pressure Direct - 2 bytes ((x-32768) * 0.01933677)
                    0x02, 0x17, 0xB7,  # Manifold Relative Pressure Direct - 2 bytes ((x-32768) * 0.01933677)
                    0x00, 0x00, 0x0E,  # Engine Speed - 2 bytes (x * 0.25)
                    0x00, 0x00, 0x0F,  # Engine Speed - 2 bytes (x * 0.25)
                    0x02, 0x0B, 0x59,  # Fine Learning Knock Correction - (x*0.3515625-45)
                    0x02, 0x0B, 0x54,  # Feedback Knock Correction - (x*0.3515625-45)
                    0x00, 0x00, 0xF9,  # IAM - (x*0.0625)
                    0x00, 0x00, 0x11,  # Ignition Total Timing - ((x-128)*0.5)
                    0x00, 0x00, 0x15,  # Throttle Opening Angle - (x*0.3921569)
                    0x02, 0x0A, 0x3E,  # Fueling Final Base - 2 bytes (30105.6/x)
                    0x02, 0x0A, 0x3F,  # Fueling Final Base - 2 bytes (30105.6/x)
                    0x02, 0x0D, 0xE2,  # CL/OL Fueling - (x)
                    0x00, 0x00, 0x12,  # Intake Air Temperature - (x-40)
                    0x00, 0x00, 0x20,  # IPW - (x*.256)
                    0x00, 0x00, 0x10,  # Vehicle Speed (x * 0.621371192)
                    0x00, 0x00, 0x64,  # Switches
                    0x02, 0x10, 0x74]) # Knock count

data_cont_query = bytes([0x80, # header
                         0x10, # destination = Subaru ECU
                         0xF0, # source = Diagnostic tool
                         0x4A, # Data size...
                         0xA8, # command = Address read
                         0x01, # continuous response
                         0x00, 0x00, 0x09,  # AF Correction #1 ((x-128) * 0.78125)
                         0x00, 0x00, 0x0A,  # AF Learning #1 ((x-128) * 0.78125)
                         0x00, 0x00, 0x46,  # AF Sensor #1 (x * 0.11484375)
                         0x02, 0x00, 0xF4,  # Engine load - 2 bytes (x * 0.0001220703125)
                         0x02, 0x00, 0xF5,  # Engine load - 2 bytes (x * 0.0001220703125)
                         0x00, 0x00, 0x13,  # Mass Airflow - 2 bytes (x * 0.01)
                         0x00, 0x00, 0x14,  # Mass Airflow - 2 bytes (x * 0.01)
                         0x02, 0x17, 0xB6,  # Manifold Relative Pressure Direct - 2 bytes ((x-32768) * 0.01933677)
                         0x02, 0x17, 0xB7,  # Manifold Relative Pressure Direct - 2 bytes ((x-32768) * 0.01933677)
                         0x00, 0x00, 0x0E,  # Engine Speed - 2 bytes (x * 0.25)
                         0x00, 0x00, 0x0F,  # Engine Speed - 2 bytes (x * 0.25)
                         0x02, 0x0B, 0x59,  # Fine Learning Knock Correction - (x*0.3515625-45)
                         0x02, 0x0B, 0x54,  # Feedback Knock Correction - (x*0.3515625-45)
                         0x00, 0x00, 0xF9,  # IAM - (x*0.0625)
                         0x00, 0x00, 0x11,  # Ignition Total Timing - ((x-128)*0.5)
                         0x00, 0x00, 0x15,  # Throttle Opening Angle - (x*0.3921569)
                         0x02, 0x0A, 0x3E,  # Fueling Final Base - 2 bytes (30105.6/x)
                         0x02, 0x0A, 0x3F,  # Fueling Final Base - 2 bytes (30105.6/x)
                         0x02, 0x0D, 0xE2,  # CL/OL Fueling - (x)
                         0x00, 0x00, 0x12,  # Intake Air Temperature - (x-40)
                         0x00, 0x00, 0x20,  # IPW - (x*.256)
                         0x00, 0x00, 0x10,  # Vehicle Speed (x * 0.621371192)
                         0x00, 0x00, 0x64,  # Switches
                         0x02, 0x10, 0x74]) # Knock count

ecu_init = bytes([0x80, # header
                  0x10, # destination = Subaru ECU
                  0xF0, # source = Diagnostic tool
                  0x01, # Data size...
                  0xBF]) # command = ECU Init

#baud_change = bytes([0x80, # header
#                     0x10, # destination = Subaru ECU
#                     0xF0, # source = Diagnostic tool
#                     0x05, # Data size..
#                     0xB8, # Write single address
#                     0x00, 0x01, 0x98, # Address for baud?
#                     0x5A]) # Value to write

def checksum(query):
    checksum = 0
    for i in query:
        checksum += i
    return bytes([checksum & 0xff])

def ecu_send(ser, query):
    bytes_to_send = query
    bytes_to_send += checksum(query)
    #print("query: {}".format(binascii.hexlify(bytes_to_send)))
    ser.write(bytes_to_send)

def ecu_receive(ser, length):
    response = ser.read(length)
    #print("response of len {}: {}".format(len(response), binascii.hexlify(response)))
    if len(response) != length and len(response) != 0:
        logging.error(f"Unexpected receive bytes on read: {len(response)}")
    return response

            
if __name__ == "__main__":

    ser =  serial.Serial('/dev/ttyUSB0',
                         baudrate=4800,
                         timeout=0.5,
                         parity=serial.PARITY_NONE,
                         stopbits=serial.STOPBITS_ONE,
                         bytesize=serial.EIGHTBITS)

    ecu_send(ser, ecu_init)
    response = ecu_receive(ser, 68)

    for r in response:
        print(r)

    ecu_send(ser, battery_query)
    len_response = (len(battery_query)+1          # Echoed response
                                                  # (including
                                                  # checksum byte,
                                                  # hence +1).
#                    + 1                           # Additional byte
                                                  # not yet understood.
                    + 5                           # Add 5
                                                  # bytes for
                                                  # response
                                                  # boilerplate.
                    + 1                           # Byte for returned
                                                  # read value.
                    +1 )                          # For total checksum

    # length=17
    print(len_response)
    response = ser.read(len_response)
    print(response[-2]*0.08)

    ecu_send(ser, rpm_query)
    response = ser.read(21)
    print(0.25*((response[-3] << 8) | response[-2]))
    #print( << , response[-2]*0.25)
