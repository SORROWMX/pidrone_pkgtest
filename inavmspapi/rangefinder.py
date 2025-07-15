#!/usr/bin/env python

"""rangefinder.py: Functions for sending rangefinder data to INAV flight controller."""

import struct
from inavmspapi import MultirotorControl

def create_rangefinder_packet(distance_mm):
    """
    Create an MSP V2 packet for sending rangefinder data.
    
    Args:
        distance_mm (int): Distance in millimeters
        
    Returns:
        bytearray: Complete MSP V2 packet ready to send
    """
    MSP2_SENSOR_RANGEFINDER = 0x1F01  # Code from msp_protocol_v2_sensor.h
    
    # Header
    packet = bytearray()
    packet.append(0x24)  # $
    packet.append(0x58)  # X
    packet.append(0x3C)  # < (direction: to flight controller)
    
    # Flag (0 for regular message)
    packet.append(0x00)
    
    # Message ID (2 bytes, little endian)
    packet.append(MSP2_SENSOR_RANGEFINDER & 0xFF)
    packet.append((MSP2_SENSOR_RANGEFINDER >> 8) & 0xFF)
    
    # Data length (2 bytes, little endian)
    packet.append(0x02)  # 2 bytes for distance
    packet.append(0x00)
    
    # Data (2 bytes for distance in mm, little endian)
    packet.append(distance_mm & 0xFF)
    packet.append((distance_mm >> 8) & 0xFF)
    
    # Calculate CRC
    crc = 0
    for i in range(3, len(packet)):
        crc = MultirotorControl._crc8_dvb_s2(crc, packet[i])
    
    packet.append(crc)
    
    return packet

def send_rangefinder_data(board, distance_mm):
    """
    Send rangefinder data to the flight controller.
    
    Args:
        board (MultirotorControl): The board connection object
        distance_mm (int): Distance in millimeters
        
    Returns:
        bool: True if data was sent successfully
    """
    # Pack the distance data
    data = struct.pack('<H', distance_mm)
    
    # Send using the MSP V2 protocol
    return board.send_RAW_msg(MultirotorControl.MSPCodes['MSP2_SENSOR_RANGEFINDER'], 
                             bytearray(data))

class RangefinderManager:
    """
    A class to manage sending rangefinder data to the flight controller.
    """
    def __init__(self, board):
        """
        Initialize the RangefinderManager.
        
        Args:
            board (MultirotorControl): The board connection object
        """
        self.board = board
        self.last_distance = 0
        
    def send_range(self, distance_meters):
        """
        Send rangefinder distance to the flight controller.
        
        Args:
            distance_meters (float): Distance in meters
            
        Returns:
            bool: True if data was sent successfully
        """
        # Convert meters to millimeters
        distance_mm = int(distance_meters * 1000)
        
        # Store the last distance
        self.last_distance = distance_mm
        
        # Send the data
        return send_rangefinder_data(self.board, distance_mm)
    
    def send_range_direct(self, distance_mm):
        """
        Send rangefinder distance directly in millimeters.
        
        Args:
            distance_mm (int): Distance in millimeters
            
        Returns:
            bool: True if data was sent successfully
        """
        # Store the last distance
        self.last_distance = distance_mm
        
        # Send the data
        return send_rangefinder_data(self.board, distance_mm) 