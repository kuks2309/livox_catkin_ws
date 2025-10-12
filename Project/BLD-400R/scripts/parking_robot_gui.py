#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import math
import os
import sys
import time
import threading
import signal  # signal 모듈 추가
import ntplib  # current time
import ntplib
import glob
import serial
import serial.tools.list_ports


# PyQt5 imports
from PyQt5.QtCore import *
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QTabWidget, QAction
from PyQt5.QtGui import *
from PyQt5.QtCore import QEvent
from PyQt5.QtCore import QTimer

import numpy as np
# UI 파일 로드
form_class = uic.loadUiType("parking_robot_gui.ui")[0]

class Robot_Control:
	def __init__(self):
		self.serial_port = None
		self.serial_command_type = 0
		self.found_motors = []  # 발견된 모터 ID 저장용
		self.is_scanning = False  # 스캔 상태 표시
		self.serial_reader = None  # 추가
		
	def connect_serial(self, port, baudrate=115200):
		"""시리얼 포트 연결"""
		try:
			self.serial_port = serial.Serial(
				port=port,
				baudrate=baudrate,
				bytesize=serial.EIGHTBITS,
				parity=serial.PARITY_NONE,
				stopbits=serial.STOPBITS_ONE,
				timeout=0.2,          # 타임아웃 설정
				write_timeout=0.2,    # 쓰기 타임아웃 설정
			)
			
			self.serial_port.reset_input_buffer()  # 입력 버퍼 초기화
			self.serial_port.reset_output_buffer() # 출력 버퍼 초기화
			return True
		except serial.SerialException as e:
			print(f"Serial connection error: {e}")
			return False
	
	def disconnect_serial(self):
		
		# 스레드 종료 이벤트 설정
		if self.serial_reader:
			self.serial_reader.exit_event.set()  
			self.serial_reader.running = False
			self.serial_reader.join()  # 스레드 종료 대기
			self.serial_reader = None
		
		"""시리얼 포트 연결 해제"""
		if self.serial_port and self.serial_port.is_open:
		   self.serial_port.close()
	
	def send_robot_run(self, speed):
		""" 로봇 속도 설정 - 11 byte 프로토콜"""
		if self.serial_port and self.serial_port.is_open:
			protocol = bytearray([0] * 11)  # 11 byte로 변경
			self.serial_command_type = 5

			# 헤더 및 명령어
			protocol[0] = ord('#')
			protocol[1] = ord('R')
			protocol[2] = ord('S')  # 'P' 추가
			protocol[3] = 0x00  # Motor ID 설정  
			
			# 속도 데이터
			protocol[4] = (speed >> 8) & 0xFF  # High byte
			protocol[5] = speed & 0xFF         # Low byte
			
			# 예비 영역 추가 (프로토콜 맞춤)
			protocol[6] = 0x00
			protocol[7] = 0x00
			
			# CRC 계산 및 설정 (8바이트에 대해 계산)
			crc = self.CRC16_MODBUS(protocol[:8], 8)
			protocol[8] = crc & 0xFF          # CRC Low byte
			protocol[9] = (crc >> 8) & 0xFF   # CRC High byte
			
			# 종료 문자
			protocol[10] = ord('*')
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
			
			self.serial_port.write(protocol)
			return True
		return False

	def send_robot_steering_turn(self,angle,mode):
		""" 로봇 회전 설정 - 11 byte 프로토콜"""
		if self.serial_port and self.serial_port.is_open:
			protocol = bytearray([0] * 11)  # 11 byte로 변경
			self.serial_command_type = 6

			# 헤더 및 명령어
			protocol[0] = ord('#')
			protocol[1] = ord('R')
			protocol[2] = ord('T')  # 'TP' 추가
			protocol[3] = 0x00  # Motor ID 설정
			
			# 속도 데이터
			protocol[4] = (angle >> 8) & 0xFF  # High byte
			protocol[5] = angle & 0xFF         # Low byte
			
			# 예비 영역 추가 (프로토콜 맞춤)
			protocol[6] = 0x00
			protocol[7] = mode
			print("steering mode ", mode)
			# CRC 계산 및 설정 (8바이트에 대해 계산)
			crc = self.CRC16_MODBUS(protocol[:8], 8)
			protocol[8] = crc & 0xFF          # CRC Low byte
			protocol[9] = (crc >> 8) & 0xFF   # CRC High byte
			
			# 종료 문자
			protocol[10] = ord('*')
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
			
			self.serial_port.write(protocol)
			return True
		return False


	def send_motor_speed(self, motor_id, speed):
		"""모터 속도 설정 - 11 byte 프로토콜"""
		if self.serial_port and self.serial_port.is_open:
			protocol = bytearray([0] * 11)  # 11 byte로 변경
			self.serial_command_type = 2
			
			# 헤더 및 명령어
			protocol[0] = ord('#')
			protocol[1] = ord('S')
			protocol[2] = ord('P')  # 'P' 추가
			protocol[3] = motor_id  # Motor ID 설정
			
			# 속도 데이터
			protocol[4] = (speed >> 8) & 0xFF  # High byte
			protocol[5] = speed & 0xFF         # Low byte
			
			# 예비 영역 추가 (프로토콜 맞춤)
			protocol[6] = 0x00
			protocol[7] = 0x00
			
			# CRC 계산 및 설정 (8바이트에 대해 계산)
			crc = self.CRC16_MODBUS(protocol[:8], 8)
			protocol[8] = crc & 0xFF          # CRC Low byte
			protocol[9] = (crc >> 8) & 0xFF   # CRC High byte
			
			# 종료 문자
			protocol[10] = ord('*')
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
			
			self.serial_port.write(protocol)
			return True
		return False
   
	def read_steering_angle(self):
		if self.serial_port and self.serial_port.is_open:
			protocol = bytearray([0] * 11)  # 11 byte로 변경
			self.serial_command_type = 8
			# 헤더 및 명령어
			protocol[0] = ord('#')
			protocol[1] = ord('R')
			protocol[2] = ord('A')  # 'A' 추가
			protocol[3] = 0x00  # Motor ID 설정
			
			# 속도 데이터
			protocol[4] = 0x00            # High byte
			protocol[5] = 0x00            # Low byte
			
			# 예비 영역 추가 (프로토콜 맞춤)
			protocol[6] = 0x00
			protocol[7] = 0x00
			
			# CRC 계산 및 설정 (8바이트에 대해 계산)
			crc = self.CRC16_MODBUS(protocol[:8], 8)
			protocol[8] = crc & 0xFF          # CRC Low byte
			protocol[9] = (crc >> 8) & 0xFF   # CRC High byte
			
			# 종료 문자
			protocol[10] = ord('*')
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
						
			self.serial_port.write(protocol)
			return True
		return False
		
	def read_encoder(self):
		
		if self.serial_port and self.serial_port.is_open:
			protocol = bytearray([0] * 11)  # 11 byte로 변경
			self.serial_command_type = 7
			# 헤더 및 명령어
			protocol[0] = ord('#')
			protocol[1] = ord('R')
			protocol[2] = ord('E')  # 'P' 추가
			protocol[3] = 0x00  # Motor ID 설정
			
			# 속도 데이터
			protocol[4] = 0x00            # High byte
			protocol[5] = 0x00            # Low byte
			
			# 예비 영역 추가 (프로토콜 맞춤)
			protocol[6] = 0x00
			protocol[7] = 0x00
			
			# CRC 계산 및 설정 (8바이트에 대해 계산)
			crc = self.CRC16_MODBUS(protocol[:8], 8)
			protocol[8] = crc & 0xFF          # CRC Low byte
			protocol[9] = (crc >> 8) & 0xFF   # CRC High byte
			
			# 종료 문자
			protocol[10] = ord('*')
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
			
			self.serial_port.write(protocol)
			return True
		return False
		
	def read_limit_switch(self):
		"""리미트 스위치 상태 읽기 - 11 byte 프로토콜"""
		if self.serial_port and self.serial_port.is_open:
			protocol = bytearray([0] * 11)  # 11 byte로 변경
			self.serial_command_type = 13
			
			# 헤더 및 명령어
			protocol[0] = ord('#')
			protocol[1] = ord('R')
			protocol[2] = ord('L')  # 'P' 추가
			protocol[3] = 0x00  # Motor ID 설정
			
			# 속도 데이터
			protocol[4] = 0x00            # High byte
			protocol[5] = 0x00            # Low byte
			
			# 예비 영역 추가 (프로토콜 맞춤)
			protocol[6] = 0x00
			protocol[7] = 0x00
			
			# CRC 계산 및 설정 (8바이트에 대해 계산)
			crc = self.CRC16_MODBUS(protocol[:8], 8)
			protocol[8] = crc & 0xFF          # CRC Low byte
			protocol[9] = (crc >> 8) & 0xFF   # CRC High byte
			
			# 종료 문자
			protocol[10] = ord('*')
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
			
			self.serial_port.write(protocol)
			return True
		return False


	def send_robot_mode(self, mode):
		""" 로봇 속도 설정 - 11 byte 프로토콜"""
		if self.serial_port and self.serial_port.is_open:
			protocol = bytearray([0] * 11)  # 11 byte로 변경
			self.serial_command_type = 9

			# 헤더 및 명령어
			protocol[0] = ord('#')
			protocol[1] = ord('R')
			protocol[2] = ord('M')  # 'P' 추가
			protocol[3] = mode  # Motor ID 설정  
			
			# 속도 데이터
			protocol[4] = 0x00    # High byte
			protocol[5] = 0x00    # Low byte
			
			# 예비 영역 추가 (프로토콜 맞춤)
			protocol[6] = 0x00
			protocol[7] = 0x00
			
			# CRC 계산 및 설정 (8바이트에 대해 계산)
			crc = self.CRC16_MODBUS(protocol[:8], 8)
			protocol[8] = crc & 0xFF          # CRC Low byte
			protocol[9] = (crc >> 8) & 0xFF   # CRC High byte
			
			# 종료 문자
			protocol[10] = ord('*')
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
			
			self.serial_port.write(protocol)
			return True
		return False	
		 
	def send_uart_test(self):
		"""모터 속도 설정"""
		if self.serial_port and self.serial_port.is_open:
			protocol = bytearray([0] * 1)
			
			protocol[0] = ord('#')
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
			
			self.serial_port.write(protocol)
			return True
		return False
		
			
	
	def CRC16_MODBUS(self, data, length):
		wCRCTable = [
			0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241, 0XC601, 0X06C0, 0X0780,
			0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440, 0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1,
			0XCE81, 0X0E40, 0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841, 0XD801,
			0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40, 0X1E00, 0XDEC1, 0XDF81, 0X1F40,
			0XDD01, 0X1DC0, 0X1C80, 0XDC41, 0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680,
			0XD641, 0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040, 0XF001, 0X30C0,
			0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240, 0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501,
			0X35C0, 0X3480, 0XF441, 0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
			0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840, 0X2800, 0XE8C1, 0XE981,
			0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41, 0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1,
			0XEC81, 0X2C40, 0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640, 0X2200,
			0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041, 0XA001, 0X60C0, 0X6180, 0XA141,
			0X6300, 0XA3C1, 0XA281, 0X6240, 0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480,
			0XA441, 0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41, 0XAA01, 0X6AC0,
			0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840, 0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01,
			0X7BC0, 0X7A80, 0XBA41, 0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
			0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640, 0X7200, 0XB2C1, 0XB381,
			0X7340, 0XB101, 0X71C0, 0X7080, 0XB041, 0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0,
			0X5280, 0X9241, 0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440, 0X9C01,
			0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40, 0X5A00, 0X9AC1, 0X9B81, 0X5B40,
			0X9901, 0X59C0, 0X5880, 0X9841, 0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81,
			0X4A40, 0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41, 0X4400, 0X84C1,
			0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641, 0X8201, 0X42C0, 0X4380, 0X8341, 0X4100,
			0X81C1, 0X8081, 0X4040
		]

		wCRCWord = 0xFFFF
		i = 0
		
		while length > 0:
			nTemp = data[i] ^ (wCRCWord & 0xFF)
			wCRCWord >>= 8
			wCRCWord ^= wCRCTable[nTemp]
			i += 1
			length -= 1
			
		return wCRCWord	
		

# test.ui를 사용하는 Dialog 클래스 추가
class TestDialog(QDialog):
	def __init__(self):
		super().__init__()
		uic.loadUi("test.ui", self)
		self.setWindowTitle("Test Dialog")

# SerialReaderThread의 첫 번째 정의만 유지하고 두 번째는 삭제
class SerialReaderThread(threading.Thread):
	def __init__(self, robot_control, window_class, exit_event=None):
		super().__init__()
		self.robot_control = robot_control
		self.window_class = window_class
		self.running = False
		self.read_buf = bytearray([0] * 11)
		self.exit_event = exit_event if exit_event else threading.Event()
		self.daemon = True  # 메인 스레드가 종료되면 자동으로 종료되도록 설정
		
		
	def clear_buffers(self):
		"""버퍼 초기화"""
		self.read_buf = bytearray([0] * 11)	

	def run(self):
		print("Serial reader thread started")
		self.running = True
		
		while self.running and not self.exit_event.is_set():
			try:
				if self.robot_control.serial_port and self.robot_control.serial_port.is_open:
					if self.robot_control.serial_port.in_waiting > 0:
						data = self.robot_control.serial_port.read()
						
						if not data:
							continue             
						data_length = 11
						
						for i in range(data_length - 1):
							self.read_buf[i] = self.read_buf[i + 1]
						self.read_buf[data_length - 1] = data[0]
						
						# 현재 버퍼의 전체 상태 출력
						for byte in self.read_buf:
							print(f"0x{byte:02X} ", end='')
						print()  # 줄바꿈
						
						self.handle_limit_switch_response(11)
						self.handle_read_encoder_response(11)
						self.handle_read_steering_angle_response(11)
						
				#time.sleep(0.01)  # CPU 사용량 감소를 위한 짧은 대기 시간 추가           
			except Exception as e:
				print(f"Serial reading error: {e}")
				time.sleep(0.1)
	
	
	def handle_read_steering_angle_response(self, data_length):
		if (self.read_buf[0] == 0x23 and  
			self.read_buf[1] == 0x52 and # Function code	
			self.read_buf[2] == 0x42):     # Byte count (4 bytes data)
			print("Raw data bytes: ", end='')
			
			for i in range(0, 11):  # 데이터 4바이트 출력
				print(f"0x{self.read_buf[i]:02X} ", end="")
			print()
						
			encoder_value = (self.read_buf[7] << 24) | (self.read_buf[6] << 16) | (self.read_buf[5] << 8) | self.read_buf[4]			
			signed_encoder_value = self.convert_to_signed_32(encoder_value)
			
			if(self.read_buf[3] == 0x01):				
				steering_front_angle =   (signed_encoder_value -13150)/16400 * -1.0;							
				self.window_class.label_front_wheel_angle.setText(f"{steering_front_angle:.1f}°")
				
				print(f"Steering1 Encoder Value: {signed_encoder_value}")
				print(f"Steering1 Angle Value: {steering_front_angle:.2f}")
			elif(self.read_buf[3] == 0x02):	
				steering_rear_angle =   (signed_encoder_value -15350)/16400;
				self.window_class.label_rear_wheel_angle.setText(f"{steering_rear_angle:.1f}°")
				
				print(f"Steering2 Encoder Value: {signed_encoder_value}")
				print(f"Steering2 Angle Value: {steering_rear_angle:.2f}")
				
		
	def handle_read_encoder_response(self, data_length):
		if (self.read_buf[0] == 0x23 and  
			self.read_buf[1] == 0x52 and # Function code	
			self.read_buf[2] == 0x45):     # Byte count (4 bytes data)
			print("Raw data bytes: ", end='')
			
			for i in range(0, 11):  # 데이터 4바이트 출력
				print(f"0x{self.read_buf[i]:02X} ", end="")
			print()
			
			encoder_value = (self.read_buf[7] << 24) | (self.read_buf[6] << 16) | (self.read_buf[5] << 8) | self.read_buf[4]
			
			signed_encoder_value = self.convert_to_signed_32(encoder_value)
                          
			if(self.read_buf[3] == 0x01):
				self.window_class.lineEdit_Bank_Motor_Encoder1.setText(str(signed_encoder_value))
				print(f"Motor 1 Encoder Value: {signed_encoder_value}")
			
			elif(self.read_buf[3] == 0x02):
				self.window_class.lineEdit_Bank_Motor_Encoder2.setText(str(signed_encoder_value))
				print(f"Motor 2 Encoder Value: {signed_encoder_value}")	
				
			elif(self.read_buf[3] == 0x03):
				self.window_class.lineEdit_Bank_Motor_Encoder3.setText(str(signed_encoder_value))
				print(f"Motor 3 Encoder Value: {signed_encoder_value}")	
				
			elif(self.read_buf[3] == 0x04):
				self.window_class.lineEdit_Bank_Motor_Encoder4.setText(str(signed_encoder_value))
				print(f"Motor 4 Encoder Value: {signed_encoder_value}")			
				
		
					
	def handle_limit_switch_response(self, data_length):
		if (self.read_buf[0] == 0x23 and  
			self.read_buf[1] == 0x52 and # Function code	
			self.read_buf[2] == 0x4C):     # Byte count (4 bytes data)
			print("Raw data bytes: ", end='')
			
			for i in range(0, 11):  # 데이터 4바이트 출력
				print(f"0x{self.read_buf[i]:02X} ", end="")
			print()
			
			 # CRC 체크
			received_crc   = (self.read_buf[9] << 8) | self.read_buf[8]
			calculated_crc = self.robot_control.CRC16_MODBUS(self.read_buf[:8], 8)
			#print("crc check :", received_crc,	calculated_crc)
			
			
			if received_crc == calculated_crc:
				# 스티어링 리미트 스위치 상태 (바이트 5)
				steering_status = self.read_buf[5]
				front_left   = bool(steering_status & 0x01)
				front_right  = bool(steering_status & 0x02)
				rear_left    = bool(steering_status & 0x04)
				rear_right   = bool(steering_status & 0x08)
				# 포크 리미트 스위치 상태 (바이트 6)
				fork_status = self.read_buf[6]
            
				#print("limit switch check :", front_left, front_right, rear_left, rear_right )
		 
				 # 상태 업데이트
				self.window_class.update_steering_limit_switch_status(
                front_left, front_right, rear_left, rear_right)
				
				self.window_class.update_fork_limit_switch_status(fork_status)
		 			
	def stop(self):
		"""스레드를 안전하게 종료하기 위한 메소드"""
		self.running = False
		self.exit_event.set()
		if self.is_alive():
			self.join(timeout=2.0)  # 최대 2초간 종료 대기
			
	def convert_to_signed_32(self,unsigned_value):
		if unsigned_value >= (1 << 31):  # 최상위 비트가 1이면 음수
			return unsigned_value - (1 << 32)
		return unsigned_value
		
class WindowClass(QDialog, form_class):
	
	# Robot driving modes
	MODE_CRAB = 1
	MODE_TANK = 2
	MODE_CAR  = 3
    
	def __init__(self):
		super().__init__()
		self.setupUi(self)
		self.setWindowTitle("Parking Robot Control")
		
		# 현재 driving mode를 저장하는 변수 추가
		self.current_drive_mode = self.MODE_CRAB
		
		# Robot Control 인스턴스 생성
		self.robot_control = Robot_Control()
		
		# 시리얼 리더 스레드
		self.serial_reader = None
        
		# 시리얼 포트 관련 변수
		self.serial_port = None
		self.is_connected = False
		self.checkBox_USB.setChecked(True)
		
		# Fork limit switch status
		self.fork_limit_switch = [False, True ,False ,False, False, False ,False ,False]

		# 라벨 스타일 설정
		self.setup_label_styles()
        
		# 초기 각도 값 설정
		self.front_angle = 0
		self.rear_angle  = 0 
		
		#초기 속도 값 설정
		self.front_left_speed = 10
		self.front_right_speed = 0
		self.rear_left_speed = 0
		self.rear_right_speed = 0
		
		# tab 설정
		self.tabWidget_robot.setTabText(0, "Robot Control")  # 두 번째 탭의 이름 변경
		self.tabWidget_robot.setTabText(1, "Motor Control")  # 첫 번째 탭의 이름 변경
		self.tabWidget_robot.setTabText(2, "UART Setup")  # 두 번째 탭의 이름 변경
		
		#  UI  초기 설정
		self.init_ui()
				
		# Dial 설정
		self.setup_dial()
		
		# LCD 디스플레이 추가
		#self.setup_lcd()
		
		self.update_front_left_speed(20)
		
		# pushButton_dialog1 클릭 이벤트 연결
		#self.pushButton_dialog1.clicked.connect(self.open_test_dialog)
		self.pushButton_Connect.clicked.connect(self.connect_serial)
		self.pushButton_Clear_Terminal.clicked.connect(self.clear_terminal) 
        
		self.checkBox_USB.stateChanged.connect(self.update_serial_ports)  # 체크박스 상태 변경 시그널 연결
				
		self.pushButton_Send_Motor_Speed1.clicked.connect(self.send_motor1_speed)  # pushButton_Send_Motor_Speed1 클릭 시그널 연결
		self.pushButton_Send_Motor_Speed2.clicked.connect(self.send_motor2_speed)  # pushButton_Send_Motor_Speed2 클릭 시그널 연결
		self.pushButton_Send_Motor_Speed3.clicked.connect(self.send_motor3_speed)  # pushButton_Send_Motor_Speed3 클릭 시그널 연결
		self.pushButton_Send_Motor_Speed4.clicked.connect(self.send_motor4_speed)  # pushButton_Send_Motor_Speed4 클릭 시그널 연결	

		self.pushButton_Stop_Motor1.clicked.connect(self.stop_motor1)  # pushButton_Stop_Motor1 클릭 시그널 연결
		self.pushButton_Stop_Motor2.clicked.connect(self.stop_motor2)  # pushButton_Stop_Motor2 클릭 시그널 연결
		self.pushButton_Stop_Motor3.clicked.connect(self.stop_motor3)  # pushButton_Stop_Motor3 클릭 시그널 연결
		self.pushButton_Stop_Motor4.clicked.connect(self.stop_motor4)  # pushButton_Stop_Motor4 클릭 시그널 연결

		self.pushButton_Read_Steering_Angle.clicked.connect(self.read_steering_angle)

		# Robot Control
		self.pushButton_Robot_Run.clicked.connect(self.robot_run)  # pushButton_Robot_Run 클릭 시그널 연결
		self.pushButton_Robot_Stop.clicked.connect(self.robot_stop)  # pushButton_Robot_Stop 클릭 시그널 연결
		self.pushButton_Robot_Steering.clicked.connect(self.robot_steering_turn)  # pushButton_Robot_Turn 클릭 시그널 연결
		self.pushButton_Robot_Turn_Straight.clicked.connect(self.robot_steering_turn_straight)  # pushButton_Robot_Turn_Straight 클릭 시그널 연결
		self.pushButton_Robot_Read_Limit_Switch.clicked.connect(self.read_limit_switch)  # pushButton_Robot_Read_Limit_Switch 클릭 시그널 연결
		self.pushButton_Robot_Read_Encoder.clicked.connect(self.read_robot_encoder)      # pushButton_Robot_Read_Encoder      클릭 시그널 연결    
		
		self.radioButton_Crab_Mode.toggled.connect(self.on_crab_mode_toggled)
		self.radioButton_Tank_Mode.toggled.connect(self.on_tank_mode_toggled)
		self.radioButton_Car_Mode.toggled.connect(self.on_car_mode_toggled)
		
	def setup_dial(self):
		"""Dial 위젯 설정"""
		self.dial_front_angle.setMinimum(0)
		self.dial_front_angle.setMaximum(360)
		self.dial_front_angle.setValue(180)
		self.dial_front_angle.setNotchesVisible(True)  # 눈금 표시
		self.dial_front_angle.setWrapping(True)        # 360도 회전 가능
		self.dial_front_angle.valueChanged.connect(self.update_angle_front)
				
		self.dial_rear_angle.setMinimum(0)
		self.dial_rear_angle.setMaximum(360)
		self.dial_rear_angle.setValue(180)
		self.dial_rear_angle.setNotchesVisible(True)  # 눈금 표시
		self.dial_rear_angle.setWrapping(True)        # 360도 회전 가능
		self.dial_rear_angle.valueChanged.connect(self.update_angle_rear)
		
	def init_ui(self):
		# 시리얼 포트 검색 및 콤보박스에 추가
		self.update_serial_ports()
  
		# 초기 속도 값 설정
		self.lineEdit_Target_Speed_Motor1.setText("100")
		self.lineEdit_Target_Speed_Motor2.setText("100")
		self.lineEdit_Target_Speed_Motor3.setText("100")
		self.lineEdit_Target_Speed_Motor4.setText("100")

		self.lineEdit_Robot_Speed.setText("100")
		self.lineEdit_Robot_Turn_Angle.setText("0")

		self.lineEdit_Bank_Motor_Encoder1.setText("0")
		self.lineEdit_Bank_Motor_Encoder2.setText("0")
		self.lineEdit_Bank_Motor_Encoder3.setText("0")
		self.lineEdit_Bank_Motor_Encoder4.setText("0")
		
		self.radioButton_Crab_Mode.setChecked(True)
		self.radioButton_Tank_Mode.setChecked(False)
		
		# 초기 driving mode 설정
		self.radioButton_Crab_Mode.setChecked(True)
		self.radioButton_Tank_Mode.setChecked(False)
		self.radioButton_Car_Mode.setChecked(False)
		self.current_drive_mode = self.MODE_CRAB  # 초기 모드를 CRAB으로 설정
    
		'''
		self.checkBox_Fork1_Open.setChecked(True)
		self.checkBox_Fork2_Open.setChecked(True)
		self.checkBox_Fork3_Open.setChecked(True)
		self.checkBox_Fork4_Open.setChecked(True)
		'''
		
	def update_fork_limit_switch_status(self, fork_status):
		"""포크 리미트 스위치 상태 업데이트
		fork_status: 8비트 값으로 각 비트가 하나의 리미트 스위치 상태를 나타냄
		"""
		# 각 비트를 개별 상태로 분리
		for i in range(8):
			self.fork_limit_switch[i] = bool(fork_status & (1 << i))	
			
		# Fork 1 상태 업데이트
		self.checkBox_Fork1_Open.setChecked(self.fork_limit_switch[0])
		self.checkBox_Fork1_Close.setChecked(self.fork_limit_switch[1])
		
		# Fork 2 상태 업데이트
		self.checkBox_Fork2_Open.setChecked(self.fork_limit_switch[2])
		self.checkBox_Fork2_Close.setChecked(self.fork_limit_switch[3])
		
		# Fork 3 상태 업데이트
		self.checkBox_Fork3_Open.setChecked(self.fork_limit_switch[4])
		self.checkBox_Fork3_Close.setChecked(self.fork_limit_switch[5])
		
		# Fork 4 상태 업데이트
		self.checkBox_Fork4_Open.setChecked(self.fork_limit_switch[6])
		self.checkBox_Fork4_Close.setChecked(self.fork_limit_switch[7])
		
		# 디버그 출력
		print(f"Fork status: {bin(fork_status)}")
		print(f"Fork states: {self.fork_limit_switch}")	

	def update_serial_ports(self):
		"""시리얼 포트 목록을 업데이트합니다."""
		self.comboBox_Serial_Port_No.clear()
		
		if self.checkBox_USB.isChecked():  # USB만 검색
			# ttyUSB 장치만 추가 (Linux)
			for usb in glob.glob('/dev/ttyUSB*'):
				self.comboBox_Serial_Port_No.addItem(usb)
		else:  # 모든 포트 검색
			# Linux와 Windows 모두 지원하도록 구현
			ports = list(serial.tools.list_ports.comports())
			
			for port in ports:
				self.comboBox_Serial_Port_No.addItem(port.device)
				
			# ttyUSB 장치 추가 (Linux)
			for usb in glob.glob('/dev/ttyUSB*'):
				if usb not in [port.device for port in ports]:
					self.comboBox_Serial_Port_No.addItem(usb)
					
			# ttyACM 장치 추가 (Linux)
			for acm in glob.glob('/dev/ttyACM*'):
				if acm not in [port.device for port in ports]:
					self.comboBox_Serial_Port_No.addItem(acm)		
				
	def connect_serial(self):
		if not self.is_connected:
			try:
					port = self.comboBox_Serial_Port_No.currentText()
					if self.robot_control.connect_serial(port):
						self.is_connected = True
						self.pushButton_Connect.setText("Disconnect")
						self.comboBox_Serial_Port_No.setEnabled(False)
						
						QMessageBox.information(self, "Connection", f"Successfully connected to {port}")
						
						# 이전 스레드가 있다면 정리
						if self.serial_reader:
							self.serial_reader.stop()
							self.serial_reader = None
						
						# 새 스레드 시작
						self.serial_reader = SerialReaderThread(self.robot_control,self)
						self.robot_control.serial_reader = self.serial_reader
						self.serial_reader.start()
						QMessageBox.information(self, "Thread Start", "Serial reader thread started")
					else:
						QMessageBox.critical(self, "Error", "Failed to connect")
			except Exception as e:
				QMessageBox.critical(self, "Error", f"Unexpected error: {str(e)}")
				self.is_connected = False
		else:
			try:
				self.robot_control.disconnect_serial()
				self.is_connected = False
				self.pushButton_Connect.setText("Connect")
				self.comboBox_Serial_Port_No.setEnabled(True)
				QMessageBox.information(self, "Disconnection", "Successfully disconnected")
			except Exception as e:
				QMessageBox.critical(self, "Error", f"Failed to disconnect: {str(e)}")
	
	def clear_terminal(self):
		os.system('clear')

	# TestDialog를 여는 메서드 추가
	def open_test_dialog(self):
		dialog = TestDialog()
		dialog.exec_()
		
	def setup_lcd(self):
		"""LCD 디스플레이 설정"""
		self.lcd = QLCDNumber(self)
		self.lcd.setGeometry(170, 180, 121, 50)  # Dial 아래에 위치
		self.lcd.setDigitCount(3)
		self.lcd.display(0)

	def setup_label_styles(self):
		"""라벨 스타일 설정"""
		style = """
			QLabel {
				font-family: Arial;
				font-size: 14px;
				font-weight: bold;
				color: black;
				background-color: white;
				border: 1px solid gray;
				padding: 2px;
				qproperty-alignment: AlignCenter;
			}
		"""
		
		# 모든 라벨에 스타일 적용
		labels = [
			self.label_front_left_motor_speed,
			self.label_fornt_right_motor_speed,
			self.label_front_wheel_angle,
			self.label_rear_left_motor_speed,
			self.label_rear_rightmotor_speed,
			self.label_rear_wheel_angle
		]
		
		for label in labels:
			label.setStyleSheet(style)
			label.setText("0")
			
	def update_angle_front(self, value):
		"""전방 방향 값 업데이트"""
		self.front_angle = 180 - value
		self.label_front_wheel_angle.setText(f"{self.front_angle}°")
		print(f"Current Front Angle: {self.front_angle}°")
		
	def update_angle_rear(self, value):
		"""후방 방향 값 업데이트"""
		self.rear_angle = 180 - value
		self.label_rear_wheel_angle.setText(f"{self.rear_angle}°")
		print(f"Current Rear Angle: {self.rear_angle}°")  
		
	def update_front_left_speed(self, value):
		"""전방 좌측 모터 속도 업데이트"""
		self.front_left_speed = value
		self.label_front_left_motor_speed.setText(f"{int(value)}")

	def update_front_right_speed(self, value):
		"""전방 우측 모터 속도 업데이트"""
		self.front_right_speed = value
		self.label_fornt_right_motor_speed.setText(f"{int(value)}")

	def update_rear_left_speed(self, value):
		"""후방 좌측 모터 속도 업데이트"""
		self.rear_left_speed = value
		self.label_rear_left_motor_speed.setText(f"{int(value)}")

	def update_rear_right_speed(self, value):
		"""후방 우측 모터 속도 업데이트"""
		self.rear_right_speed = value
		self.label_rear_rightmotor_speed.setText(f"{int(value)}")
		
		
	def update_steering_limit_switch_status(self, front_left, front_right, rear_left, rear_right):
		"""스티어링 리미트 스위치 상태 업데이트"""
		# 전방 스티어링 리미트 스위치 상태 저장
		self.front_steering_limit_switch_status = [front_left, front_right]
		
		# 후방 스티어링 리미트 스위치 상태 저장
		self.rear_steering_limit_switch_status = [rear_left, rear_right]	
		
		# 전방 좌측 스티어링 리미트 스위치 상태 업데이트
		self.checkBox_Front_Steering_Left_Open.setChecked(front_left)
		self.checkBox_Front_Steering_Left_Close.setChecked(not front_left)

		# 전방 우측 스티어링 리미트 스위치 상태 업데이트
		self.checkBox_Front_Steering_Right_Open.setChecked(front_right)
		self.checkBox_Front_Steering_Right_Close.setChecked(not front_right)

		# 후방 좌측 스티어링 리미트 스위치 상태 업데이트
		self.checkBox_Rear_Steering_Right_Open.setChecked(rear_left)
		self.checkBox_Rear_Steering_Left_Close.setChecked(not rear_left)

		# 후방 우측 스티어링 리미트 스위치 상태 업데이트
		self.checkBox_Rear_Steering_Right_Open.setChecked(rear_right)
		self.checkBox_Rear_Steering_Right_Close.setChecked(not rear_right)
	
		
	# Tab 1	
	# 로봇 제어

	def robot_run(self):
		if not self.is_connected:
			QMessageBox.critical(self, "Error", "Please connect to the robot first")
			return
			
		try:
			print("Robot Run")
			speed = int(self.lineEdit_Robot_Speed.text())
			self.robot_control.send_robot_run(speed)
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Failed to run robot: {str(e)}")

			
	def robot_stop(self):
		if not self.is_connected:
			QMessageBox.critical(self, "Error", "Please connect to the robot first")
			return
			
		try:
			print("Robot Stop")
			speed = 0
			self.robot_control.send_robot_run(speed)
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Failed to stop robot: {str(e)}")
				
	def robot_steering_turn(self):
		if not self.is_connected:
			QMessageBox.critical(self, "Error", "Please connect to the robot first")
			return
			
		try:
			print("Robot steering")
			angle = int(self.lineEdit_Robot_Turn_Angle.text())
			self.robot_control.send_robot_steering_turn(angle,self.current_drive_mode)
		except ValueError:
			QMessageBox.critical(self, "Error", "Please enter a valid number for angle")
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Failed to turn robot: {str(e)}")
	
	def robot_steering_turn_straight(self):
		if not self.is_connected:
			QMessageBox.critical(self, "Error", "Please connect to the robot first")
			return
			
		try:
			print("Robot steering straight")
			angle = int(0)
			self.robot_control.send_robot_steering_turn(angle, self.current_drive_mode)
		except ValueError:
			QMessageBox.critical(self, "Error", "Please enter a valid number for angle")
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Failed to turn robot: {str(e)}")
			
			
	def robot_fork_open(self):
		print("Robot Fork Open")
		# Add your fork open command here
	
	def robot_fork_close(self):
		print("Robot Fork Close")
		# Add your fork close command here		

	#0x23 0x53 0x50 0x01 0x00 0x64 0x00 0x00 0x11 0x4C 0x2A
	#2353500100640000114C2A
	def send_motor1_speed(self):
		speed = int(self.lineEdit_Target_Speed_Motor1.text())
		print(speed)
		self.robot_control.send_motor_speed(1,speed) 	

	def send_motor2_speed(self):
		speed = int(self.lineEdit_Target_Speed_Motor2.text())
		print(speed)
		self.robot_control.send_motor_speed(2,speed)

	def send_motor3_speed(self):
		speed = int(self.lineEdit_Target_Speed_Motor3.text())
		print(speed)
		self.robot_control.send_motor_speed(3,speed)

	def send_motor4_speed(self):
		speed = int(self.lineEdit_Target_Speed_Motor4.text())
		print(speed)
		self.robot_control.send_motor_speed(4,speed)
  
	def stop_motor1(self):
		speed = 0
		print(speed)
		self.robot_control.send_motor_speed(1,speed)

	def stop_motor2(self):
		speed = 0
		print(speed)
		self.robot_control.send_motor_speed(2,speed)
  
	def stop_motor3(self):
		speed = 0
		print(speed)
		self.robot_control.send_motor_speed(3,speed)
  
	def stop_motor4(self):
		speed = 0
		print(speed)
		self.robot_control.send_motor_speed(4,speed)

	def read_limit_switch(self):
		"""Read the status of limit switches when button is clicked"""
		if not self.is_connected:
			QMessageBox.critical(self, "Error", "Please connect to the robot first")
			return
			
		try:
			self.robot_control.read_limit_switch()
			
			
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Failed to read limit switches: {str(e)}")
	
	def read_robot_encoder(self):
		if not self.is_connected:
			QMessageBox.critical(self, "Error", "Please connect to the robot first")
			return
			
		try:
			self.robot_control.read_encoder()
			
			
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Failed to read encoder: {str(e)}")
			
	def read_steering_angle(self):
		if not self.is_connected:
			QMessageBox.critical(self, "Error", "Please connect to the robot first")
			return
			
		try:
			self.robot_control.read_steering_angle()
			
			
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Failed to read steering angle: {str(e)}")	
		
	def on_crab_mode_toggled(self, checked):
		"""Handle Crab Mode radio button toggle"""
		if checked:
			print("Crab mode")
			self.radioButton_Tank_Mode.setChecked(False)
			self.radioButton_Car_Mode.setChecked(False)
			self.current_drive_mode = self.MODE_CRAB
			
			if not self.is_connected:
				QMessageBox.critical(self, "Error", "Please connect to the robot first")
				return
			
			try:
				self.robot_control.send_robot_mode(self.MODE_CRAB)
				print("Crab mode activated")
			except Exception as e:
				QMessageBox.critical(self, "Error", f"Failed to set crab mode: {str(e)}")

	def on_tank_mode_toggled(self, checked):
		"""Handle Tank Mode radio button toggle"""
		if checked:
			print("Tank mode")
			self.radioButton_Crab_Mode.setChecked(False)
			self.radioButton_Car_Mode.setChecked(False)
			self.current_drive_mode = self.MODE_TANK
			
			if not self.is_connected:
				QMessageBox.critical(self, "Error", "Please connect to the robot first")
				return
				
			try:
				self.robot_control.send_robot_mode(self.MODE_TANK)
				print("Tank mode activated")
			except Exception as e:
				QMessageBox.critical(self, "Error", f"Failed to set tank mode: {str(e)}")

	def on_car_mode_toggled(self, checked):
		"""Handle Car Mode radio button toggle"""
		if checked:
			print("Car mode")
			self.radioButton_Crab_Mode.setChecked(False)
			self.radioButton_Tank_Mode.setChecked(False)
			self.current_drive_mode = self.MODE_CAR
			
			if not self.is_connected:
				QMessageBox.critical(self, "Error", "Please connect to the robot first")
				return
				
			try:
				self.robot_control.send_robot_mode(self.MODE_CAR)
				print("Car mode activated")
			except Exception as e:
				QMessageBox.critical(self, "Error", f"Failed to set car mode: {str(e)}")	
						
	# 종료 이벤트 추가
	def closeEvent(self, event):
		"""애플리케이션 종료 시 호출되는 이벤트 핸들러"""
		try:
			# 시리얼 리더 스레드 종료
			if self.serial_reader:
				self.serial_reader.stop()  # 새로운 stop 메소드 사용
				self.serial_reader = None
			
			# 로봇 컨트롤 종료
			if self.robot_control:
				self.robot_control.disconnect_serial()

			event.accept()
		except Exception as e:
			print(f"Error during application close: {e}")
			event.accept()  # 에러가 발생해도 종료는 진행
	
def main():
    app = QApplication(sys.argv)
    window = WindowClass()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
