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

ui_file_path = os.path.join(os.path.dirname(__file__), 'controller_id_set.ui')
form_class = uic.loadUiType(ui_file_path)[0]

# SerialReaderThread의 첫 번째 정의만 유지하고 두 번째는 삭제
class SerialReaderThread(threading.Thread):
	def __init__(self, motor_control, exit_event=None):
		super().__init__()
		self.motor_control = motor_control
		self.running = False
		self.read_buf = bytearray([0] * 57)  # 최대 57바이트 버퍼
		self.exit_event = exit_event if exit_event else threading.Event()

	def run(self):
		print("Serial reader thread started")
		self.running = True
		
		while self.running and not self.exit_event.is_set():
			try:
				if self.motor_control.serial_port and self.motor_control.serial_port.is_open:
					if self.motor_control.serial_port.in_waiting > 0:
						# 1바이트씩 읽기
						data = self.motor_control.serial_port.read()
						
						if not data:
							continue

						# 데이터 길이 결정
						if self.motor_control.serial_command_type   == 0:
							data_length = 57
						elif self.motor_control.serial_command_type == 1:
							data_length = 8  # output enable response 
						elif self.motor_control.serial_command_type == 3:
							data_length = 7 
														
						elif self.motor_control.serial_command_type == 4:
							data_length = 7 
							
						elif self.motor_control.serial_command_type == 5:
							data_length = 7 
								
						elif self.motor_control.serial_command_type == 6:
							data_length = 7 
								
						elif self.motor_control.serial_command_type == 7:
							data_length = 9 	# 응답 패킷 크기 (ID + FC + Byte Count + Data(4) + CRC(2))
							
						elif self.motor_control.serial_command_type == 9:  # 엔코더 쓰기
							data_length = 8     # Slave ID(1) + Function Code(1) + Address(2) + Number of Registers(2) + CRC(2)
							
						elif self.motor_control.serial_command_type == 10:  # 레지스터 읽기
							data_length = 7  # ID(1) + FC(1) + BC(1) + Data(2) + CRC(2)
       
						elif self.motor_control.serial_command_type == 11:  # 레지스터 쓰기
							data_length = 8  # ID(1) + FC(1) + BC(1) + Data(2) + CRC(2)
       
						elif self.motor_control.serial_command_type == 12:  # 멀티 모터 쓰기
							None # Response 없음
          
						elif self.motor_control.serial_command_type == 100:
							data_length = 7 	
							
						else:
							data_length = 8
							

						# 버퍼 시프트 및 새 데이터 추가
						for i in range(data_length - 1):
							self.read_buf[i] = self.read_buf[i + 1]
						self.read_buf[data_length - 1] = data[0]
											
						print("self.motor_control.serial_command_type : ",self.motor_control.serial_command_type) 
						# 명령 타입에 따른 응답 처리
						if self.motor_control.serial_command_type == 0:  # 모니터링 명령
							self.handle_monitor_response(data_length)
						elif self.motor_control.serial_command_type == 1:  # 드라이브 출력 활성화
							self.handle_drive_output_response(data_length)
						elif self.motor_control.serial_command_type == 2:  # 속도 설정
							print("Speed Set")
							self.handle_speed_set_response(data_length)
						elif self.motor_control.serial_command_type == 3:  # 속도 읽기
							print("Speed Read")
							self.handle_speed_read_response(data_length)
						elif self.motor_control.serial_command_type == 4:  # 전압 읽기
							self.handle_voltage_read_response(data_length)						
						elif self.motor_control.serial_command_type == 5:  # 모터 스캔	
							self.handle_scan_response(data_length)							
						elif self.motor_control.serial_command_type == 6:  # 모터 ID 변경	
							self.handle_scan_response(data_length)	
						elif self.motor_control.serial_command_type == 7:  # 엔코더 읽기
							self.handle_encoder_read_response(data_length)
						elif self.motor_control.serial_command_type == 8:  # 모터 ID 변경	
							self.handle_scan_response(data_length)	
						elif self.motor_control.serial_command_type == 9:  # 엔코더 쓰기
							self.handle_encoder_write_response(data_length)	
							
						elif self.motor_control.serial_command_type == 10:  # 레지스터 읽기`	
							self.handle_register_read_response(data_length)
       
						elif self.motor_control.serial_command_type == 11:  # 레지스터 쓰기
							self.handle_register_write_response(data_length)
															
						elif self.motor_control.serial_command_type == 14:  # 파라미터 저장
							self.handle_parameter_save_response(data_length)
							
						elif self.motor_control.serial_command_type == 15:  # Modbus Enable							
							print("Motor ID Read")
							self.handle_modbus_enable_response(data_length)
							
						elif self.motor_control.serial_command_type == 100:  # ID 읽기 	
							self.handle_read_id_response(data_length)
						else:
							print("can not resolved")	
							
							
			except Exception as e:
				print(f"Serial reading error: {e}")
				time.sleep(0.1)

	def handle_monitor_response(self, data_length):
		"""모니터링 명령 응답 처리"""
		crc = self.motor_control.CRC16_MODBUS(self.read_buf[:55], 55)
		if (self.read_buf[55] == (crc & 0xFF) and 
			self.read_buf[56] == ((crc >> 8) & 0xFF)):
			
			# 현재 속도 추출
			current_speed = (self.read_buf[4] << 8) | self.read_buf[5]
			print(f"Current target speed: {current_speed}")

			# 엔코더 값 추출
			encoder_val = ((self.read_buf[48] << 24) | 
						  (self.read_buf[47] << 16) |
						  (self.read_buf[50] << 8) | 
						  self.read_buf[49])
			print(f"Current encoder value: {encoder_val}")
			
			# 여기에 UI 업데이트 코드 추가
			try:
				window = QApplication.activeWindow()
				if window and hasattr(window, 'encoder_updated'):
					window.encoder_updated.emit(self.read_buf[0], encoder_val)  # self.read_buf[0]는 motor_id
			except Exception as e:
				print(f"Error updating encoder display: {e}")
							
			# 버퍼 클리어
			self.clear_buffers()

	def handle_encoder_read_response(self, data_length):
		"""엔코더 읽기 응답 처리"""
		print("Encoder read response received: ", end='')
		for i in range(data_length):
			print(f"0x{self.read_buf[i]:02X} ", end="")
		print()
		
		if (self.read_buf[1] == 0x03 and  # Function code
			self.read_buf[2] == 0x04):     # Byte count (4 bytes data)
			print("Raw data bytes: ", end='')
			for i in range(3, 7):  # 데이터 4바이트 출력
				print(f"0x{self.read_buf[i]:02X} ", end="")
			print()
			# CRC 체크 - 데이터 길이를 고려하여 수정
			crc = self.motor_control.CRC16_MODBUS(self.read_buf[:7], 7)
			if (self.read_buf[7] == (crc & 0xFF) and 
				self.read_buf[8] == ((crc >> 8) & 0xFF)):
				
				# 엔코더 값 추출 (4바이트, Little Endian 순서)
				encoder_val = ((self.read_buf[4] << 0) |   # LSB
                          (self.read_buf[3] << 8) |
                          (self.read_buf[6] << 16) |
                          (self.read_buf[5] << 24))    # MSB
				
				# 32비트 signed integer로 변환
				if encoder_val & 0x80000000:  # 최상위 비트가 1이면 음수
					encoder_val = -(~encoder_val & 0xFFFFFFFF) - 1
					   
				print(f"Encoder value: {encoder_val}")
				
				# UI 업데이트 시그널 발생
				try:
					motor_id = self.read_buf[0]  # 응답의 첫 바이트는 motor ID
					window = QApplication.activeWindow()
					if window and hasattr(window, 'encoder_updated'):
						window.encoder_updated.emit(motor_id, encoder_val)
				except Exception as e:
					print(f"Error updating encoder display: {e}")
					
				self.clear_buffers()
			
		
	def handle_drive_output_response(self, data_length):
		"""드라이브 출력 활성화 응답 처리"""
		if self.check_response_packet(data_length):
			status = self.read_buf[5]
			print(f"Drive output {'enabled' if status == 1 else 'disabled'}")
			self.clear_buffers()

	def handle_speed_set_response(self, data_length):
		"""속도 설정 응답 처리"""
		if self.check_response_packet(data_length):
			speed = (self.read_buf[4] << 8) | self.read_buf[5]
			print(f"Speed set response: {speed}")
			self.clear_buffers()

	def handle_speed_read_response(self, data_length):
		"""속도 읽기 응답 처리"""
		print("Speed read response received: ", end='')
		for i in range(data_length):
			print(f"0x{self.read_buf[i]:02X} ", end="")
		print()
		
		if (self.read_buf[1] == 0x03 and  # Function code
			self.read_buf[2] == 0x02):     # Byte count (2 bytes data)
			
			# CRC 체크
			crc = self.motor_control.CRC16_MODBUS(self.read_buf[:5], 5)
			if (self.read_buf[5] == (crc & 0xFF) and 
				self.read_buf[6] == ((crc >> 8) & 0xFF)):
				
				# 속도값 계산 - 부호있는 16비트 정수로 처리
				raw_speed = ((self.read_buf[3] << 8) | self.read_buf[4])
				
				# 16비트 signed integer로 변환
				if raw_speed & 0x8000:
					speed = -((0x10000 - raw_speed))
				else:
					speed = raw_speed
				
				# 실제 속도값으로 변환 (필요한 경우 스케일 팩터 적용)
				actual_speed = float(speed) / 10.0  # 명시적으로 float으로 변환
				
				print(f"Raw speed bytes: {self.read_buf[3]:02X} {self.read_buf[4]:02X}")
				print(f"Raw speed value: {raw_speed}")
				print(f"Calculated speed: {actual_speed} RPM")
				
				try:
					motor_id = self.read_buf[0]
					window = QApplication.activeWindow()
					if window and hasattr(window, 'speed_updated'):
						window.speed_updated.emit(int(motor_id), float(actual_speed))
						print(f"Emitting speed value {actual_speed} for motor {motor_id}")
				except Exception as e:
					print(f"Error updating speed display: {e}")
					
				self.clear_buffers()
				return actual_speed
		
		return None
	
	
	def handle_voltage_read_response(self, data_length):
		"""시스템 전압 읽기 응답 처리"""
		print("System voltage read response received: ", end='')
		for i in range(data_length):
			print(f"0x{self.read_buf[i]:02X} ", end="")
		print()
		
		if (self.read_buf[1] == 0x03 and  # Function code
			self.read_buf[2] == 0x02):     # Byte count (2 bytes data)
			
			# CRC 체크
			crc = self.motor_control.CRC16_MODBUS(self.read_buf[:5], 5)
			if (self.read_buf[5] == (crc & 0xFF) and 
				self.read_buf[6] == ((crc >> 8) & 0xFF)):
				
				# 전압값 계산 - 데이터 시트에 따라 x/327
				raw_voltage = (self.read_buf[3] << 8) | self.read_buf[4]
				voltage = raw_voltage / 327.0  # 데이터 시트 스케일링 적용
				
				print(f"Raw voltage bytes: {self.read_buf[3]:02X} {self.read_buf[4]:02X}")
				print(f"Raw voltage value: {raw_voltage}")
				print(f"Calculated voltage: {voltage:.2f}V")
				
				try:
					motor_id = self.read_buf[0]
					window = QApplication.activeWindow()
					if window and hasattr(window, 'voltage_updated'):
						window.voltage_updated.emit(motor_id, float(voltage))
						print(f"Emitting voltage value {voltage:.2f}V for motor {motor_id}")
				except Exception as e:
					print(f"Error updating voltage display: {e}")
					
				self.clear_buffers()
				return voltage
		
		return None

	def handle_parameter_save_response(self, data_length):
		"""파라미터 저장 응답 처리"""
		if self.check_response_packet(data_length):
			print("Parameters saved successfully")
			self.clear_buffers()
			
	def handle_modbus_enable_response(self, data_length):
		"""Modbus Enable 응답 처리"""
		if self.check_response_packet(data_length):
			# 응답이 0x01 0x06 0x00 0x00 0x00 0x01의 형태
			if (self.read_buf[0] == self.read_buf[0] and  # ID 체크
				self.read_buf[1] == 0x06 and
				self.read_buf[2] == 0x00 and
				self.read_buf[3] == 0x00 and
				self.read_buf[4] == 0x00 and
				self.read_buf[5] == 0x01):
				print("Received: ", end='')
				for i in range(data_length):
					print(f"0x{self.read_buf[i]:02X} ", end="")
				print()	
				
				print("Modbus Enable response received successfully")
			else:
				print("Unexpected Modbus Enable response")
			
			#self.motor_control.serial_command_type = 0  # 응답 처리 후 리셋
			self.clear_buffers()

		#self.motor_control.serial_command_type = 0  # 응답 처리 후 리셋
		#self.clear_buffers()
    
	def handle_drive_output_response(self, data_length):
		"""드라이브 출력 활성화 응답 처리"""
		print("Drive output response received: ", end='')
		for i in range(data_length):
			print(f"0x{self.read_buf[i]:02X} ", end="")
		print()

		# 응답 패킷 구조 검증
		if (self.read_buf[1] == 0x06 and  # Function code
			self.read_buf[2] == 0x00 and  # Register address high byte
			self.read_buf[3] == 0x01):    # Register address low byte
			
			# CRC 체크
			crc = self.motor_control.CRC16_MODBUS(self.read_buf[:6], 6)
			if (self.read_buf[6] == (crc & 0xFF) and 
				self.read_buf[7] == ((crc >> 8) & 0xFF)):
				
				# 활성화 상태 확인
				status = self.read_buf[5]  # Data low byte
				if status == 0x01:
					print(f"Motor {self.read_buf[0]} output enabled successfully")
				else:
					print(f"Motor {self.read_buf[0]} output disabled")
					
				self.clear_buffers()
				return True
			else:
				print("CRC check failed")
		else:
			print("Invalid response format")
			
		return False	
 
 
 	
	def handle_scan_response(self, data_length):
		"""모터 스캔 응답 처리"""
		if (self.read_buf[1] == 0x03 and  # Function code
			self.read_buf[2] == 0x02 and
			self.read_buf[3] == 0x00 			
			):
			
			print("Scan response received: ", end='')
			for i in range(data_length):
				print(f"0x{self.read_buf[i]:02X} ", end="")
			print()	
			
			# CRC 체크
			crc = self.motor_control.CRC16_MODBUS(self.read_buf[:5], 5)
			if (self.read_buf[5] == (crc & 0xFF) and 
				self.read_buf[6] == ((crc >> 8) & 0xFF)):
				
				motor_id = self.read_buf[0]
				print(f"Found motor with ID: {motor_id}")
				self.motor_control.found_motors.append(motor_id)
				self.motor_control.is_scanning = False  # 스캔 중단
			
				# UI 업데이트 시그널 발생
				try:
					window = QApplication.activeWindow()
					if window and hasattr(window, 'id_updated'):
						window.id_updated.emit(motor_id)
				except Exception as e:
					print(f"Error updating UI: {e}")
				self.clear_buffers()	
			else:
				print("CRC check failed")	
	
	def handle_read_id_response(self, data_length):
		"""ID 읽기 응답 처리"""
		if (self.read_buf[1] == 0x03 and  # Function code: Read
			self.read_buf[2] == 0x02):     # Byte count
			
			print("ID Read response received: ", end='')
			for i in range(data_length):
				print(f"0x{self.read_buf[i]:02X} ", end="")
			print()    
			
			motor_id = self.read_buf[4]  # 실제 ID 값
			print(f"Current Motor ID: {motor_id}")
			
			# UI 업데이트 시그널 발생
			try:
				window = QApplication.activeWindow()
				if window and hasattr(window, 'id_updated'):
					window.id_updated.emit(motor_id)
			except Exception as e:
				print(f"Error updating ID display: {e}")
				
			self.clear_buffers()
	
	def handle_encoder_write_response(self, data_length):
		"""엔코더 쓰기 응답 처리"""
		print("Encoder write response received: ", end='')
		for i in range(data_length):
			print(f"0x{self.read_buf[i]:02X} ", end="")
		print()

		# 응답 형식 검증 (Motor ID 확인 추가)
		if (self.read_buf[0] != 0x00 and  # Motor ID가 0이 아님
			self.read_buf[1] == 0x10 and  # Function code
			self.read_buf[2] == 0x00 and  # Starting address high byte
			self.read_buf[3] == 0x16):    # Starting address low byte
			
			# CRC 체크
			crc = self.motor_control.CRC16_MODBUS(self.read_buf[:6], 6)
			if (self.read_buf[6] == (crc & 0xFF) and 
				self.read_buf[7] == ((crc >> 8) & 0xFF)):
				print("Encoder value written successfully")
				# 버퍼 클리어 추가
				self.clear_buffers()
				return True
		
		print("Failed to write encoder value")
		return False
	
	def handle_register_read_response(self, data_length):
		"""레지스터 읽기 응답 처리"""
		print("Register read response received: ", end='')
		for i in range(data_length):
			print(f"0x{self.read_buf[i]:02X} ", end="")
		print()

		if (self.read_buf[1] == 0x03 and  # Function code
			self.read_buf[2] == 0x02):     # Byte count (2 bytes data)

			# CRC 체크
			crc = self.motor_control.CRC16_MODBUS(self.read_buf[:5], 5)
			if (self.read_buf[5] == (crc & 0xFF) and 
				self.read_buf[6] == ((crc >> 8) & 0xFF)):

				# 값 추출 (16비트)
				value = (self.read_buf[3] << 8) | self.read_buf[4]
				print(f"Register value: {value} (0x{value:04X})")
				
				self.clear_buffers()
				return value

		return None
	
	def handle_register_write_response(self, data_length):
		"""레지스터 쓰기 응답 처리"""
		print("Register write response received: ", end='')
		for i in range(data_length):
			print(f"0x{self.read_buf[i]:02X} ", end="")
		print()

		# 먼저 완전한 패킷이 수신되었는지 확인
		if self.read_buf[0] != self.motor_control.last_motor_id:  # 예상되는 device ID
			return False

		 # write_register()에서 사용한 값을 저장하고 비교해야 함
		expected_register_address = self.motor_control.last_written_register
		expected_value = self.motor_control.last_written_value

		# 응답 패킷 구조 검증
		received_register_address = (self.read_buf[2] << 8) | self.read_buf[3]
		received_value = (self.read_buf[4] << 8) | self.read_buf[5]
  

		# 응답 패킷 구조 검증
		if (self.read_buf[1] == 0x06 and  # Function code (Write Single Register)
        received_register_address == expected_register_address and
        received_value == expected_value):


			# CRC 체크
			crc = self.motor_control.CRC16_MODBUS(self.read_buf[:6], 6)
			if (self.read_buf[6] == (crc & 0xFF) and 
				self.read_buf[7] == ((crc >> 8) & 0xFF)):
				
				motor_id = self.read_buf[0]
				register_address = (self.read_buf[2] << 8) | self.read_buf[3]
				value = (self.read_buf[4] << 8) | self.read_buf[5]
				
				print(f"Successfully wrote value 0x{value:04X} to register 0x{register_address:02X} for motor {motor_id}")
				
				self.clear_buffers()
				return True
				
			print("CRC check failed")
			return False
	
		print("Invalid response format")
		return False
 
 			
	def check_response_packet(self, data_length):
		"""응답 패킷 확인"""
		# 응답 표시
		
		print("Check Received: ", end='')
		for i in range(data_length):
			print(f"0x{self.read_buf[i]:02X} ", end="")
		print()
		

		# CRC 체크
		crc = self.motor_control.CRC16_MODBUS(self.read_buf[:6], 6)
		return (self.read_buf[6] == (crc & 0xFF) and 
				self.read_buf[7] == ((crc >> 8) & 0xFF))

	def clear_buffers(self):
		"""버퍼 클리어"""
		self.read_buf = bytearray([0] * 57)
		if self.motor_control.serial_port:
			self.motor_control.serial_port.reset_input_buffer()

	def stop(self):
		self.running = False
		if self.exit_event:
			self.exit_event.set()
            
class MotorControl:
	def __init__(self):
		self.serial_port = None
		self.serial_command_type = 0
		self.found_motors = []             # 발견된 모터 ID 저장용
		self.is_scanning           = False # 스캔 상태 표시
		self.last_written_register = None  # 마지막으로 쓴 레지스터 저장
		self.last_written_value    = None  # 마지막으로 쓴 값 저장
		self.last_motor_id         = None  # 마지막으로 읽은 모터 ID 저장
		
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

	def connect_serial(self, port, baudrate=19200):
		"""시리얼 포트 연결"""
		try:
			self.serial_port = serial.Serial(
				port=port,
				baudrate=baudrate,
				bytesize=serial.EIGHTBITS,
				parity=serial.PARITY_NONE,
				stopbits=serial.STOPBITS_ONE,
				timeout=0.1,          # 타임아웃 설정
				write_timeout=0.1,    # 쓰기 타임아웃 설정
			)
			
			self.serial_port.reset_input_buffer()  # 입력 버퍼 초기화
			self.serial_port.reset_output_buffer() # 출력 버퍼 초기화
			return True
		except serial.SerialException as e:
			print(f"Serial connection error: {e}")
			return False

	def disconnect_serial(self):
		"""시리얼 포트 연결 해제"""
		if self.serial_port and self.serial_port.is_open:
			self.serial_port.close()

	def modbus_enable(self, motor_id):
		"""모드버스 활성화"""
		if self.serial_port and self.serial_port.is_open:
			
			protocol = bytearray([0] * 8)
			self.serial_command_type = 15  # Modbus Enable 명령 타입을 15로 설정
			
			protocol[0] = int(motor_id)
			protocol[1] = 0x06
			protocol[2] = 0x00
			protocol[3] = 0x00
			protocol[4] = 0x00
			protocol[5] = 0x01
			
			crc = self.CRC16_MODBUS(protocol[:6], 6)
			protocol[6] = crc & 0xFF
			protocol[7] = (crc >> 8) & 0xFF
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()			
			self.serial_port.write(protocol)
			print("Modbus Enable ")
			return True
		return False
		
	def save_parameters(self, motor_id):
		"""Save parameter number (내부적으로 이전에 설정된 파라미터 번호 저장)"""
		if self.serial_port and self.serial_port.is_open:
			self.serial_command_type = 14
			protocol = bytearray([0] * 8)
			
			protocol[0] = int(motor_id)
			protocol[1] = 0x06  # Function code (Write)
			protocol[2] = 0x00  # Parameter address high byte
			protocol[3] = 0x14  # Parameter address low byte
			protocol[4] = 0x00  # Data high byte
			protocol[5] = 0x01  # Data low byte
			
			crc = self.CRC16_MODBUS(protocol[:6], 6)
			protocol[6] = crc & 0xFF        # CRC low byte
			protocol[7] = (crc >> 8) & 0xFF # CRC high byte
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
			
			self.serial_port.write(protocol)
			return True
		return False	
		
	def send_motor_drive_output_enable(self, motor_id, set):
		if self.serial_port and self.serial_port.is_open:
			self.serial_command_type = 1
			protocol = bytearray([0] * 8)
		
			protocol[0] = motor_id  # 하드코딩된 0x01 대신 파라미터 사용
			protocol[1] = 0x06
			protocol[2] = 0x00
			protocol[3] = 0x01
			protocol[4] = 0x00    
			protocol[5] = 0x01 if set else 0x00
			
			if set is True:
				protocol[5] = 0x01
			else:	
				protocol[5] = 0x00
				
			crc = self.CRC16_MODBUS(protocol[:6], 6)
			protocol[6] = crc & 0xFF
			protocol[7] = (crc >> 8) & 0xFF
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
			print("Motor Output Enable")	
			self.serial_port.write(protocol)
			return True
		return False
		
		
	def send_motor_speed(self, motor_id, speed):
		"""모터 속도 설정"""
		if self.serial_port and self.serial_port.is_open:
			protocol = bytearray([0] * 8)
			self.serial_command_type = 2
			
			protocol[0] = motor_id  # Motor ID 설정
			protocol[1] = 0x06
			protocol[2] = 0x00
			protocol[3] = 0x02
			protocol[4] = (speed >> 8) & 0xFF  # High byte
			protocol[5] = speed & 0xFF         # Low byte
			
			crc = self.CRC16_MODBUS(protocol[:6], 6)
			protocol[6] = crc & 0xFF
			protocol[7] = (crc >> 8) & 0xFF
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
			
			self.serial_port.write(protocol)
			return True
		return False
	
	def send_motor_speed_bank2(self, motor_id1, motor_id2, speed1, speed2):
		"""두 모터의 속도를 동기식으로 설정"""
		if self.serial_port and self.serial_port.is_open:
			# 데이터 길이 계산: header(7) + data(16) + CRC(2) = 25 bytes
			protocol = bytearray([0] * 25)
			self.serial_command_type = 12

			# Header 구성
			protocol[0] = 0x00              # Device Address (브로드캐스트)
			protocol[1] = 0x10              # Function code (Write Multiple Registers)
			protocol[2] = 0x00              # Register address high
			protocol[3] = 0x16              # Register address low (절대 비트 주소)
			protocol[4] = 0x00              # Register count high
			protocol[5] = 0x08              # Register count low (8개 레지스터)
			protocol[6] = 0x10              # Data length (16 bytes)

			# 첫 번째 모터 데이터 (8 bytes)
			protocol[7] = 0x00              # Position highest byte
			protocol[8] = 0x00              # Position high byte
			protocol[9] = 0x00              # Position low byte
			protocol[10] = 0x00             # Position lowest byte
			protocol[11] = (abs(speed1) >> 8) & 0xFF   # Speed high
			protocol[12] = abs(speed1) & 0xFF          # Speed low
			protocol[13] = 0x00             # Acceleration high
			protocol[14] = 0x00             # Acceleration low

			# 두 번째 모터 데이터 (8 bytes)
			protocol[15] = 0x00             # Position highest byte
			protocol[16] = 0x00             # Position high byte
			protocol[17] = 0x00             # Position low byte
			protocol[18] = 0x00             # Position lowest byte
			protocol[19] = (abs(speed2) >> 8) & 0xFF  # Speed high
			protocol[20] = abs(speed2) & 0xFF         # Speed low
			protocol[21] = 0x00             # Acceleration high
			protocol[22] = 0x00             # Acceleration low

			# CRC 계산
			crc = self.CRC16_MODBUS(protocol[:23], 23)
			protocol[23] = crc & 0xFF       # CRC low
			protocol[24] = (crc >> 8) & 0xFF # CRC high

			# 데이터 출력 (디버깅용)
			print("Send: ", end="")
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()

			self.serial_port.write(protocol)
			print(f"Motor speeds set - Speed1: {speed1}, Speed2: {speed2}")
			return True

		return False
      
      
 
	def set_encoder(self, motor_id, encoder_value):
		"""엔코더 값 설정"""
		if self.serial_port and self.serial_port.is_open:
			# 전체 프로토콜 길이: header(7) + data(4) + CRC(2) = 13 bytes
			protocol = bytearray([0] * 13)
			self.serial_command_type = 9  # 엔코더 설정용 command type
			
			# 데이터를 하위 16비트와 상위 16비트로 분리
			low_word = encoder_value & 0xFFFF
			high_word = (encoder_value >> 16) & 0xFFFF
			
			protocol[0] = motor_id        # Motor ID
			protocol[1] = 0x10            # Function code (Multiple registers write)
			protocol[2] = 0x00            # Register address high byte
			protocol[3] = 0x16            # Register address low byte (absolute position)
			protocol[4] = 0x00            # Number of registers high byte
			protocol[5] = 0x02            # Number of registers low byte (2개의 레지스터)
			protocol[6] = 0x04            # Byte count (4 bytes of data)
			
			
			 # 0x16 레지스터에 하위 16비트
			protocol[7] = (low_word >> 8) & 0xFF    # 하위 워드의 상위 바이트
			protocol[8] = low_word & 0xFF           # 하위 워드의 하위 바이트
			
			# 0x17 레지스터에 상위 16비트
			protocol[9] = (high_word >> 8) & 0xFF   # 상위 워드의 상위 바이트
			protocol[10] = high_word & 0xFF         # 상위 워드의 하위 바이트

			# CRC 계산 
			crc = self.CRC16_MODBUS(protocol[:11], 11)
			protocol[11] = crc & 0xFF
			protocol[12] = (crc >> 8) & 0xFF
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
			
			self.serial_port.write(protocol)
			return True
		return False
		
	def read_register(self, motor_id, register_address):
		"""특정 레지스터 값 읽기"""
		if self.serial_port and self.serial_port.is_open:
			protocol = bytearray([0] * 8)
			self.serial_command_type = 10  # 읽기 명령용 command type

			protocol[0] = motor_id  # Device ID
			protocol[1] = 0x03      # Function code (Read Holding Registers)
			protocol[2] = 0x00      # Register address high byte
			protocol[3] = register_address  # Register address low byte
			protocol[4] = 0x00      # Number of registers high byte
			protocol[5] = 0x01      # Number of registers low byte (1개 레지스터)

			# CRC 계산
			crc = self.CRC16_MODBUS(protocol[:6], 6)
			protocol[6] = crc & 0xFF
			protocol[7] = (crc >> 8) & 0xFF

			# 데이터 출력
			print(f"Reading register 0x{register_address:02X}")
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
			print()

			self.serial_port.write(protocol)
			return True
		return False
		
	def write_register(self, motor_id, register_address, value):
		"""레지스터 값 쓰기"""
		if self.serial_port and self.serial_port.is_open:
			protocol = bytearray([0] * 8)
			self.serial_command_type = 11  # 레지스터 쓰기 command type
   			
			protocol[0] = motor_id           # Device ID
			protocol[1] = 0x06              # Function code (Write Single Register)
			protocol[2] = 0x00              # Register address high byte
			protocol[3] = register_address  # Register address low byte
			protocol[4] = (value >> 8) & 0xFF     # Value high byte
			protocol[5] = value & 0xFF            # Value low byte

			# CRC 계산
			crc = self.CRC16_MODBUS(protocol[:6], 6)
			protocol[6] = crc & 0xFF
			protocol[7] = (crc >> 8) & 0xFF

			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()

			self.serial_port.write(protocol)
   
			# 🛠️ 마지막으로 보낸 레지스터 정보 저장
			self.last_written_register = register_address
			self.last_written_value    = value
			self.last_motor_id         = motor_id
   
			# 송신 후 잠시 대기 추가
			time.sleep(0.05)
   
			return True
		return False	
					
	def set_encoder_0x16(self, motor_id, encoder_value):
		"""0x16 레지스터 값 설정 (단일 레지스터 쓰기)"""
		if self.serial_port and self.serial_port.is_open:
			protocol = bytearray([0] * 8)
			self.serial_command_type = 9

			protocol[0] = motor_id           # Device ID
			protocol[1] = 0x06               # Function code (단일 레지스터 쓰기)
			protocol[2] = 0x00               # Register address high byte
			protocol[3] = 0x16               # Register address low byte
			protocol[4] = (encoder_value >> 8) & 0xFF  # Data high byte
			protocol[5] = encoder_value & 0xFF         # Data low byte

			# CRC 계산
			crc = self.CRC16_MODBUS(protocol[:6], 6)
			protocol[6] = crc & 0xFF
			protocol[7] = (crc >> 8) & 0xFF

			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()

			self.serial_port.write(protocol)
			return True
		return False
	
	def move_to_target_position(self, motor_id, target_pos):
		"""목표 위치로 이동하는 명령어 전송"""
		#print("목표 위치로 이동하는 명령어 전송")
		if self.serial_port and self.serial_port.is_open:
			
			protocol = bytearray([0] * 8)
			self.serial_command_type = 7  # Position move command type
			
			protocol[0] = motor_id        # Motor ID
			protocol[1] = 0x7B            # Function code (Position move)
			protocol[2] = (target_pos >> 24) & 0xFF  # Position highest byte
			protocol[3] = (target_pos >> 16) & 0xFF  # Position high byte
			protocol[4] = (target_pos >> 8) & 0xFF   # Position low byte
			protocol[5] = target_pos & 0xFF          # Position lowest byte
			print("목표 위치로 이동하는 명령어 전송2")
			# CRC 계산
			crc = self.CRC16_MODBUS(protocol[:6], 6)
			protocol[6] = crc & 0xFF
			protocol[7] = (crc >> 8) & 0xFF
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
			
			self.serial_port.write(protocol)
			return True
		return False
		
		
	
	def read_encoder(self, motor_id):
		"""엔코더 값 읽기"""
		if self.serial_port and self.serial_port.is_open:
			protocol = bytearray([0] * 8)
			self.serial_command_type = 7  # 엔코더 읽기 전용 command type
			
			protocol[0] = motor_id  # Motor ID 설정
			protocol[1] = 0x03      # Function code (Read)
			protocol[2] = 0x00      # Register address high byte
			protocol[3] = 0x16      # Register address low byte (엔코더 레지스터 주소)
			protocol[4] = 0x00      # Number of registers high byte
			protocol[5] = 0x02      # Number of registers low byte (4바이트 데이터이므로 2개 레지스터)
			
			crc = self.CRC16_MODBUS(protocol[:6], 6)
			protocol[6] = crc & 0xFF
			protocol[7] = (crc >> 8) & 0xFF
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
			
			self.serial_port.write(protocol)
			return True
		
		return False
	
	
	def monitor_command(self):
		"""모니터 명령 전송"""
		if self.serial_port and self.serial_port.is_open:
			protocol = bytearray([0] * 8)
			self.serial_command_type = 0
			
			protocol[0] = 0x01
			protocol[1] = 0x03
			protocol[2] = 0x00
			protocol[3] = 0x00
			protocol[4] = 0x00
			protocol[5] = 0x1A
			
			crc = self.CRC16_MODBUS(protocol[:6], 6)
			protocol[6] = crc & 0xFF
			protocol[7] = (crc >> 8) & 0xFF
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
			
			self.serial_port.write(protocol)
			return True
		return False
	
	def read_system_voltage(self, motor_id):
		"""시스템 전압 읽기"""
		if self.serial_port and self.serial_port.is_open:
			protocol = bytearray([0] * 8)
			self.serial_command_type = 4
			
			protocol[0] = motor_id  # Device ID 사용
			protocol[1] = 0x03
			protocol[2] = 0x00
			protocol[3] = 0x11  # 전압 레지스터 주소
			protocol[4] = 0x00
			protocol[5] = 0x01  # 읽을 레지스터 수
			
			crc = self.CRC16_MODBUS(protocol[:6], 6)
			protocol[6] = crc & 0xFF
			protocol[7] = (crc >> 8) & 0xFF
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
			
			self.serial_port.write(protocol)
			return True
		return False
		
		
	def change_motor_id(self, current_id, new_id):
		if self.serial_port and self.serial_port.is_open:
			protocol = bytearray([0] * 8)
			
			protocol[0] = current_id  # 현재 모터 ID (0x01)
			protocol[1] = 0x06        # Function code
			protocol[2] = 0x00        # Register address high byte
			protocol[3] = 0x15        # Register address low byte
			protocol[4] = 0x00        # Data high byte
			protocol[5] = new_id      # Data low byte (새로운 ID)
			
			# CRC 계산
			crc = self.CRC16_MODBUS(protocol[:6], 6)
			protocol[6] = crc & 0xFF
			protocol[7] = (crc >> 8) & 0xFF
			print("Change ID")
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
			
			self.serial_port.write(protocol)
			return True
				
		return False
		
		
	def find_connected_motors(self):
		"""연결된 모터의 ID를 스캔"""
		
		found_motors = []
		self.is_scanning = True
		if self.serial_port and self.serial_port.is_open:
			for motor_id in range(1, 11):
				
				if not self.is_scanning:  # 스캔이 중단되었는지 확인
					break
                
				protocol = bytearray([0] * 8)
				self.serial_command_type = 5  # 스캔용 특별 명령 타입
				
				protocol[0] = motor_id
				protocol[1] = 0x03
				protocol[2] = 0x00
				protocol[3] = 0x15
				protocol[4] = 0x00
				protocol[5] = 0x01
				
				crc = self.CRC16_MODBUS(protocol[:6], 6)
				protocol[6] = crc & 0xFF
				protocol[7] = (crc >> 8) & 0xFF
				
				print("Send: ", end='')
				for byte in protocol:
					print(f"0x{byte:02X} ", end="")
				print()

				# 명령어 전송
				self.serial_port.write(protocol)
				
				# 응답을 SerialReaderThread에서 처리하도록 대기
				time.sleep(0.2)  # 응답 처리를 위한 대기
			self.is_scanning = False	
			
			# 찾은 모터 목록 반환
			return self.found_motors.copy()  # 리스트 복사본 반환
		
	
	def read_motor_speed(self, motor_id):
		"""모터 속도 읽기"""
		print("self.serial_command_type:", self.serial_command_type)
		if self.serial_port and self.serial_port.is_open:
			protocol = bytearray([0] * 8)
			self.serial_command_type = 3
			
			protocol[0] = motor_id  # Motor ID 설정
			protocol[1] = 0x03
			protocol[2] = 0x00
			protocol[3] = 0x10  # 속도 레지스터 주소
			protocol[4] = 0x00
			protocol[5] = 0x01  # 읽을 레지스터 수
			
			crc = self.CRC16_MODBUS(protocol[:6], 6)
			protocol[6] = crc & 0xFF
			protocol[7] = (crc >> 8) & 0xFF
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
			
			self.serial_port.write(protocol)
			return True
		return False
		
	def test(self, motor_id):
		
		if self.serial_port and self.serial_port.is_open:
			protocol = bytearray([0] * 8)
			self.serial_command_type = 100
			
			protocol[0] = motor_id  # Motor ID 설정
			protocol[1] = 0x03
			protocol[2] = 0x00
			protocol[3] = 0x15  # ID 레지스터 주소
			protocol[4] = 0x00
			protocol[5] = 0x01  # 읽을 레지스터 수
			
			crc = self.CRC16_MODBUS(protocol[:6], 6)
			protocol[6] = crc & 0xFF
			protocol[7] = (crc >> 8) & 0xFF
			
			# 데이터 출력
			print("Send: ", end='')
			for byte in protocol:
				print(f"0x{byte:02X} ", end="")
			print()
			
			self.serial_port.write(protocol)
			return True
		return False

class WindowClass(QDialog, form_class):
	# Qt 시그널 정의
	id_updated = pyqtSignal(int)  # 상단에 추가

	voltage_updated = pyqtSignal(int, float)  # (motor_id, voltage) 형태로 전달
	speed_updated = pyqtSignal(int, float)      # (motor_id, speed) 형태로 전달
	encoder_updated = pyqtSignal(int, int)    # (motor_id, encoder_value)
	
	def __init__(self):
		super(WindowClass, self).__init__()
		self.setupUi(self)
		self.setWindowTitle("Controller ID setup GUI")
		
		# 종료 이벤트 추가
		self.exit_event = threading.Event()
		
		# MotorControl 인스턴스 생성
		self.motor_control = MotorControl()
        
		# 시리얼 리더 스레드
		self.serial_reader = None
        
		# 시리얼 포트 관련 변수
		self.serial_port = None
		self.is_connected = False
		self.checkBox_USB.setChecked(True)
		
		# 커멘트 관련 변수
		self.motor_command_type = 0
		# UI 초기화
		self.init_ui()		
		
		# 시그널 연결
		self.id_updated.connect(self.update_current_id)


		# Motor ID 관련 변수
		
		self.textEdit_Motor_ID1.setText("1")
		self.textEdit_Motor_ID2.setText("2")
		self.textEdit_Motor_ID3.setText("3")
		self.textEdit_Motor_ID4.setText("4")
		
		
		self.textEdit_Motor_Speed1.setText("100")		
		self.textEdit_Motor_Speed2.setText("100")
		self.textEdit_Motor_Speed3.setText("100")
		self.textEdit_Motor_Speed4.setText("100")
  
		self.lineEdit_Bank_Motor_Speed1.setText("100")
		self.lineEdit_Bank_Motor_Speed2.setText("-100")
  
		self.textEdit_Motor_Enable_ID.setText("1")
		
		# Motor Rotation Angle 관련 변수
		self.textEdit_Scale_Encoder_Angle.setText("16444.444")
		self.textEdit_Set_Motor_Angle3.setText("0")
		
		
		# TabWidget의 탭 이름 변경
		self.tabWidget.setTabText(0, "motor control 1")  # 첫 번째 탭의 이름 변경
		self.tabWidget.setTabText(1, "motor control 2")  # 두 번째 탭의 이름 변경
		self.tabWidget.setTabText(2, "motor control 3")  # 세 번째 탭의 이름 변경
		self.tabWidget.setTabText(3, "motor control 4")  # 세 번째 탭의 이름 변경
  
		self.tabWidget.setTabText(4, "multi motor")  # 네 번째 탭의 이름 변경
  
		self.tabWidget.setTabText(5, "motor comm. setting")  # 다섯 번째 탭의 이름 변경
		
		# 시그널 연결
		self.pushButton_Connect.clicked.connect(self.connect_serial)
		self.checkBox_USB.stateChanged.connect(self.update_serial_ports)  # 체크박스 상태 변경 시그널 연결
		
		self.pushButton_Modbus_Enable.clicked.connect(self.modbus_enable_function)
		self.pushButton_Output_Enable.clicked.connect(self.motor_output_enable_function)
		
		self.pushButton_Set_Motor_Speed1.clicked.connect(self.set_motor_speed)		
		self.pushButton_Set_Motor_Speed2.clicked.connect(self.set_motor_speed)		
		self.pushButton_Set_Motor_Speed3.clicked.connect(self.set_motor_speed)		
		self.pushButton_Set_Motor_Speed4.clicked.connect(self.set_motor_speed)
				
		self.pushButton_Set_Motor_Stop1.clicked.connect(self.set_motor_stop)
		self.pushButton_Set_Motor_Stop2.clicked.connect(self.set_motor_stop)
		self.pushButton_Set_Motor_Stop3.clicked.connect(self.set_motor_stop)
		self.pushButton_Set_Motor_Stop4.clicked.connect(self.set_motor_stop)
		
		self.pushButton_Read_Motor_Speed1.clicked.connect(self.read_motor_speed)
		self.pushButton_Read_Motor_Speed2.clicked.connect(self.read_motor_speed)
		self.pushButton_Read_Motor_Speed3.clicked.connect(self.read_motor_speed)
		self.pushButton_Read_Motor_Speed4.clicked.connect(self.read_motor_speed)
		
		self.pushButton_Read_Motor_Voltage1.clicked.connect(self.read_motor_voltage)
		self.pushButton_Read_Motor_Voltage2.clicked.connect(self.read_motor_voltage)
		self.pushButton_Read_Motor_Voltage3.clicked.connect(self.read_motor_voltage)
		self.pushButton_Read_Motor_Voltage4.clicked.connect(self.read_motor_voltage)
		
		
		self.pushButton_Read_Motor_Encoder1.clicked.connect(self.read_motor_encoder)
		self.pushButton_Read_Motor_Encoder2.clicked.connect(self.read_motor_encoder)
		self.pushButton_Read_Motor_Encoder3.clicked.connect(self.read_motor_encoder)
		self.pushButton_Read_Motor_Encoder4.clicked.connect(self.read_motor_encoder)
		
  
		self.pushButton_motor_speed_bank2.clicked.connect(self.set_motor_speed_bank2)
  
		# WindowClass의 __init__ 메소드에 추가할 버튼 연결 코드
		self.pushButton_Set_Motor_Encoder2.clicked.connect(self.set_motor_encoder)
		
		
		self.pushButton_Set_Motor_Encoder3.clicked.connect(self.set_target_encoder)
		self.pushButton_Set_Motor_Angle3.clicked.connect(self.set_target_angle)
		
		# Change ID 버튼 연결
		self.pushButton_Change_ID.clicked.connect(self.change_motor_id)
		self.pushButton_Write_Register.clicked.connect(self.write_register_data)
		self.pushButton_Read_Register.clicked.connect(self.read_register_data)
  
  
		self.pushButton_Save_Parameter.clicked.connect(self.save_parameters)
		self.pushButton_Find_ID.clicked.connect(self.scan_motors)
		
		#pyqtslot 처리 부분 
		self.id_updated.connect(self.update_current_id)
		
		self.voltage_updated.connect(self.update_voltage_display)
		self.speed_updated.connect(self.update_speed_display)  # 추가
		self.encoder_updated.connect(self.update_encoder_display)
		
		# test button
		self.pushButton_test.clicked.connect(self.test_button)
		
		self.pushButton_Clear_Terminal.clicked.connect(self.clear_terminal) 
  
		# 탭 변경 시그널 연결
		self.tabWidget.currentChanged.connect(self.tab_changed)		
		self.buttonBox.accepted.connect(self.handle_accept)
		self.buttonBox.rejected.connect(self.handle_reject)
		
		
	def init_ui(self):
		# 시리얼 포트 검색 및 콤보박스에 추가
		self.update_serial_ports()	
	
	def clear_terminal(self):
		os.system('clear')
			
	def tab_changed(self, index):
		print(f"Current tab index: {index}")
		# 또는 현재 탭 이름 출력
		current_tab_name = self.tabWidget.tabText(index)
		print(f"Current tab name: {current_tab_name}")
		
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
		"""시리얼 포트 연결/해제를 처리합니다."""
		if not self.is_connected:
			try:
				port = self.comboBox_Serial_Port_No.currentText()
				if self.motor_control.connect_serial(port):
					self.is_connected = True
					self.pushButton_Connect.setText("Disconnect")
					self.comboBox_Serial_Port_No.setEnabled(False)
					
					# 시리얼 리더 스레드 시작 (exit_event 파라미터 전달)
					self.serial_reader = SerialReaderThread(self.motor_control, self.exit_event)
					self.serial_reader.start()
					
					QMessageBox.information(self, "Connection", f"Successfully connected to {port}")
					
					# 연결 후 모드버스 활성화
					try:
						self.motor_control.modbus_enable(1)
					except Exception as e:
						QMessageBox.warning(self, "Warning", f"Modbus enable failed: {str(e)}")
						
					# 연결 후 드라이버 출력 활성화
					try:    
						self.motor_control.send_motor_drive_output_enable(2,True)
					except Exception as e:
						QMessageBox.warning(self, "Warning", f"Drive Output enable failed: {str(e)}")        
							
				else:
					QMessageBox.critical(self, "Error", "Failed to connect")
					
			except serial.SerialException as e:
				QMessageBox.critical(self, "Error", f"Serial connection error: {str(e)}")
				self.is_connected = False
			except Exception as e:
				QMessageBox.critical(self, "Error", f"Unexpected error: {str(e)}")
				self.is_connected = False
		else:
			try:
				# 스레드 정지
				if self.serial_reader:
					self.serial_reader.stop()
					self.serial_reader.join()
					self.serial_reader = None
					
				self.motor_control.disconnect_serial()
				self.is_connected = False
				self.pushButton_Connect.setText("Connect")
				self.comboBox_Serial_Port_No.setEnabled(True)
				QMessageBox.information(self, "Disconnection", "Successfully disconnected")
				
			except Exception as e:
				QMessageBox.critical(self, "Error", f"Failed to disconnect: {str(e)}")

	def modbus_enable_function(self):
		try:
			# 현재 선택된 탭의 Motor ID 사용
			current_tab = self.tabWidget.currentIndex()
			
			if current_tab == 0:
				motor_id = int(self.textEdit_Motor_ID1.toPlainText())
			elif current_tab == 1:
				motor_id = int(self.textEdit_Motor_ID2.toPlainText())
			elif current_tab == 2:
				motor_id = int(self.textEdit_Motor_ID3.toPlainText())
			elif current_tab == 3:
				motor_id = int(self.textEdit_Motor_ID4.toPlainText())
			else:
				motor_id = 1  # 기본값
				motor_id = int(self.textEdit_Motor_Enable_ID.toPlainText())

			self.motor_control.modbus_enable(motor_id)
		except Exception as e:
			QMessageBox.warning(self, "Warning", f"Modbus enable failed: {str(e)}")
		
	def motor_output_enable_function(self):
		"""모터 출력 활성화"""
		try:
			if self.is_connected:
				# Get Motor ID from current tab
				current_tab = self.tabWidget.currentIndex()
				
				if current_tab == 0:
					motor_id = int(self.textEdit_Motor_ID1.toPlainText())
				elif current_tab == 1:
					motor_id = int(self.textEdit_Motor_ID2.toPlainText())
				elif current_tab == 2:
					motor_id = int(self.textEdit_Motor_ID3.toPlainText())
				elif current_tab == 3:
					motor_id = int(self.textEdit_Motor_ID4.toPlainText())
				else:
					motor_id = int(self.textEdit_Motor_Enable_ID.toPlainText())

				# Send driver output enable command (register 0x01)
				if self.motor_control.send_motor_drive_output_enable(motor_id,True):
					print(f"Motor {motor_id} output enabled")
					QMessageBox.information(self, "Success", f"Motor {motor_id} output enabled")
				else:
					QMessageBox.warning(self, "Warning", "Failed to enable motor output")
			else:
				QMessageBox.warning(self, "Warning", "Please connect to serial port first")
		except ValueError:
			QMessageBox.critical(self, "Error", "Please enter a valid Motor ID")
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Error enabling motor output: {str(e)}")
		
		
		
			
	def set_motor_speed(self):
		"""모터 속도 설정"""
		try:
			# 현재 선택된 탭 인덱스 가져오기
			current_tab = self.tabWidget.currentIndex()
			print("current_tab :",current_tab )
			# 탭 인덱스에 따라 적절한 ID와 speed textEdit 선택
			if current_tab == 0:
				motor_id = int(self.textEdit_Motor_ID1.toPlainText())
				speed = int(self.textEdit_Motor_Speed1.toPlainText())
			elif current_tab == 1:
				motor_id = int(self.textEdit_Motor_ID2.toPlainText())
				speed = int(self.textEdit_Motor_Speed2.toPlainText())
			elif current_tab == 2:
				motor_id = int(self.textEdit_Motor_ID3.toPlainText())
				speed = int(self.textEdit_Motor_Speed3.toPlainText())
			elif current_tab == 3:
				motor_id = int(self.textEdit_Motor_ID4.toPlainText())
				speed = int(self.textEdit_Motor_Speed4.toPlainText())	
				             
			else:
				QMessageBox.warning(self, "Warning", "Invalid tab selected")
				return
				
			if self.is_connected:
				if self.motor_control.send_motor_speed(motor_id, speed):
					print(f"Motor speed set to {speed}")
				else:
					QMessageBox.warning(self, "Warning", "Failed to set motor speed")
			else:
				QMessageBox.warning(self, "Warning", "Please connect to serial port first")
		except ValueError:
			QMessageBox.critical(self, "Error", "Please enter valid numbers for Motor ID and speed")
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Unexpected error: {str(e)}")
	
 
	def set_motor_speed_bank2(self):
		"""모터 속도 설정"""
		try:
			# 현재 선택된 탭 인덱스 가져오기
			current_tab = self.tabWidget.currentIndex()
			print("current_tab :",current_tab )
			# 탭 인덱스에 따라 적절한 ID와 speed textEdit 선택
			if current_tab == 4:
				motor_id1 = 1
				motor_id2 = 2
				raw_speed1 = self.lineEdit_Bank_Motor_Speed1.text().strip()
				raw_speed2 = self.lineEdit_Bank_Motor_Speed2.text().strip()

				print(f"Raw speed1 input: '{raw_speed1}', type: {type(raw_speed1)}")
				print(f"Raw speed2 input: '{raw_speed2}', type: {type(raw_speed2)}")

				# 변환 수행
				try:
					speed1 = int(raw_speed1)
					speed2 = int(raw_speed2)
				except ValueError as e:
					print(f"ValueError: {e}")
					speed1, speed2 = 0, 0  # 에러 발생 시 기본값 설정

					print(f"Converted speed1: {speed1}, type: {type(speed1)}")
					print(f"Converted speed2: {speed2}, type: {type(speed2)}")
				             
			else:
				QMessageBox.warning(self, "Warning", "Invalid tab selected")
				return
				
			if self.is_connected:
				if self.motor_control.send_motor_speed_bank2(motor_id1,motor_id2, speed1,speed2):
					print(f"Motor speeds set to {speed1} {speed2}")
				else:
					QMessageBox.warning(self, "Warning", "Failed to set motor speed")
			else:
				QMessageBox.warning(self, "Warning", "Please connect to serial port first")
		except ValueError:
			QMessageBox.critical(self, "Error", "Please enter valid numbers for Motor ID and speed")
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Unexpected error: {str(e)}")
     
     
     
	def set_motor_stop(self):
		try:
			# Get current tab index
			current_tab = self.tabWidget.currentIndex()
			speed = 0
			
			# Get the appropriate motor ID based on current tab
			if current_tab == 0:
				motor_id = int(self.textEdit_Motor_ID1.toPlainText())
			elif current_tab == 1:
				motor_id = int(self.textEdit_Motor_ID2.toPlainText())
			elif current_tab == 2:
				motor_id = int(self.textEdit_Motor_ID3.toPlainText())
			elif current_tab == 3:
				motor_id = int(self.textEdit_Motor_ID4.toPlainText())	
			else:
				QMessageBox.warning(self, "Warning", "Invalid tab selected")
				return
				
			if self.is_connected:
				if self.motor_control.send_motor_speed(motor_id, speed):
					print(f"Motor {motor_id} stopped (speed set to {speed})")
				else:
					QMessageBox.warning(self, "Warning", "Failed to stop motor")
			else:
				QMessageBox.warning(self, "Warning", "Please connect to serial port first")
		except ValueError:
			QMessageBox.critical(self, "Error", "Please enter a valid Motor ID")
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Error stopping motor: {str(e)}")	
	
	
	def set_motor_encoder(self):
		"""엔코더 값 설정"""
		try:
			if self.is_connected:
				# 현재 선택된 탭 인덱스 가져오기
				current_tab = self.tabWidget.currentIndex()
				print("current_tab", current_tab)
				
				# 먼저 motor_id와 encoder_value를 None으로 초기화
				motor_id = None
				encoder_value = None
				
				if current_tab == 1:
					try:
						# 텍스트 값 먼저 가져오기
						motor_id_text = self.textEdit_Motor_ID2.toPlainText()
						encoder_text = self.textEdit_Set_Motor_Encoder_2.toPlainText()
						
						# 값 변환 시도
						motor_id = int(motor_id_text)
						encoder_value = int(encoder_text)
						
						print("Converted motor id:", motor_id)
						print("Converted encoder value:", encoder_value)
						
						# 엔코더 값 설정
						if self.motor_control.set_encoder_0x16(motor_id, encoder_value):
							print(f"Setting encoder value to {encoder_value} for motor {motor_id}")
							QMessageBox.information(self, "Success", f"Encoder value set to {encoder_value}")
						else:
							QMessageBox.warning(self, "Warning", "Failed to set encoder value")
						
						
					except ValueError as ve:
						print(f"Conversion error: {ve}")
						print(f"Motor ID text: '{motor_id_text}'")
						print(f"Encoder text: '{encoder_text}'")
						raise  # 원래 예외를 다시 발생시킴
			else:
				QMessageBox.warning(self, "Warning", "Please connect to serial port first")
				
		except ValueError:
			QMessageBox.critical(self, "Error", "Please enter valid numbers for Motor ID and encoder value")
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Error setting encoder value: {str(e)}")
						
					
	def set_target_encoder(self):
		"""목표 위치로 이동"""
		try:
			if self.is_connected:
				# 현재 선택된 탭 인덱스 가져오기
				current_tab = self.tabWidget.currentIndex()
				print("current_tab", current_tab)
				# Motor ID와 목표 위치 가져오기
				if current_tab == 2:  # motor control 3 탭인 경우
					motor_id = int(self.textEdit_Motor_ID3.toPlainText())
					target_pos = int(self.textEdit_Set_Motor_Encoder.toPlainText())
					print("target_pos", target_pos)
				else:
					QMessageBox.warning(self, "Warning", "Please select motor control 3 tab")
					return
					
				
				if self.motor_control.move_to_target_position(motor_id, target_pos):
					print(f"Moving motor {motor_id} to position {target_pos}")
				else:
					QMessageBox.warning(self, "Warning", "Failed to send move command")
			else:
				QMessageBox.warning(self, "Warning", "Please connect to serial port first")
		except ValueError:
			QMessageBox.critical(self, "Error", "Please enter valid numbers for Motor ID and target position")
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Error moving to position: {str(e)}")
	
	
	def set_target_angle(self):
		"""각도 위치로 이동"""
		try:
			if self.is_connected:
				# 현재 선택된 탭 인덱스 가져오기
				current_tab = self.tabWidget.currentIndex()
				print("current_tab", current_tab)
				# Motor ID와 목표 위치 가져오기
				if current_tab == 2:  # motor control 3 탭인 경우
					motor_id = int(self.textEdit_Motor_ID3.toPlainText())
					scale_factor = float(self.textEdit_Scale_Encoder_Angle.toPlainText())
					target_angle = float(self.textEdit_Set_Motor_Angle3.toPlainText())
					
					print("target angle",target_angle)
					
					target_pos = int(target_angle*scale_factor)
					print("target_pos", target_pos)
				else:
					QMessageBox.warning(self, "Warning", "Please select motor control 3 tab")
					return
					
				
				if self.motor_control.move_to_target_position(motor_id, target_pos):
					print(f"Moving motor {motor_id} to position {target_pos}")
				else:
					QMessageBox.warning(self, "Warning", "Failed to send move command")
			else:
				QMessageBox.warning(self, "Warning", "Please connect to serial port first")
		except ValueError:
			QMessageBox.critical(self, "Error", "Please enter valid numbers for Motor ID and target position")
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Error moving to position: {str(e)}")
			
	def change_motor_id(self):
		"""모터 ID 변경"""
		self.motor_control.serial_command_type = 6
		try:
			# 현재 ID와 새로운 ID 가져오기
			current_id = int(self.textEdit_Current_ID.toPlainText())
			new_id = int(self.textEdit_New_ID.toPlainText())
			
			# 입력값 검증
			if not (1 <= new_id <= 10):  # ID 범위는 1-10으로 제한
				QMessageBox.warning(self, "Warning", "New ID must be between 1 and 10")
				return
				
			if not self.is_connected:
				QMessageBox.warning(self, "Warning", "Please connect to serial port first")
				return
				
			# ID 변경 명령 전송
			print("current id : ", current_id,"new id :", new_id)
			if self.motor_control.change_motor_id(current_id, new_id):
				# 파라미터 저장 명령도 함께 전송
				#self.motor_control.save_parameters()
				QMessageBox.information(self, "Success", f"Motor ID changed from {current_id} to {new_id}")
				
				# textEdit 업데이트
				self.textEdit_Current_ID.setText(str(new_id))
			else:
				QMessageBox.warning(self, "Warning", "Failed to change motor ID")
            
		except ValueError:
			QMessageBox.critical(self, "Error", "Please enter valid numbers for Current ID and New ID")
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Error changing motor ID: {str(e)}")	
			
	def save_parameters(self):
		"""파라미터 저장 버튼 핸들러"""
		try:
			if self.is_connected:
				
				current_id = int(self.textEdit_Current_ID.toPlainText())
				if self.motor_control.save_parameters(current_id):
					QMessageBox.information(self, "Success", "Parameters saved successfully")
				else:
					QMessageBox.warning(self, "Warning", "Failed to save parameters")
			else:
				QMessageBox.warning(self, "Warning", "Please connect to serial port first")
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Error saving parameters: {str(e)}")
			
	def scan_motors(self):
		"""GUI에서 모터 스캔 버튼 핸들러"""
		self.motor_control.serial_command_type = 5
		try:
			if self.is_connected:
				found_motors = self.motor_control.find_connected_motors()
				if len(found_motors) > 0:  # found_motors가 비어있지 않은 경우
					message = "Found motors with IDs: " + ", ".join(map(str, found_motors))
					QMessageBox.information(self, "Scan Result", message)
				else:
					QMessageBox.warning(self, "Scan Result", "No motors found")
			else:
				QMessageBox.warning(self, "Warning", "Please connect to serial port first")
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Error scanning motors: {str(e)}")


	def read_motor_speed(self):
		"""모터 속도 읽기"""
		self.motor_control.serial_command_type = 3
		try:
			if self.is_connected:
				# 현재 선택된 탭 인덱스 가져오기
				current_tab = self.tabWidget.currentIndex()
				
				# 탭 인덱스에 따라 적절한 Motor ID 선택
				if current_tab == 0:
					motor_id = int(self.textEdit_Motor_ID1.toPlainText())
				elif current_tab == 1:
					motor_id = int(self.textEdit_Motor_ID2.toPlainText())
				elif current_tab == 2:
					motor_id = int(self.textEdit_Motor_ID3.toPlainText())
				elif current_tab == 3:
					motor_id = int(self.textEdit_Motor_ID4.toPlainText())			
				else:
					QMessageBox.warning(self, "Warning", "Invalid tab selected")
					return
					
				if self.motor_control.read_motor_speed(motor_id):
					print(f"Reading motor speed for Motor ID {motor_id}...")
				else:
					QMessageBox.warning(self, "Warning", "Failed to read motor speed")
			else:
				QMessageBox.warning(self, "Warning", "Please connect to serial port first")
		except ValueError:
			QMessageBox.critical(self, "Error", "Please enter a valid Motor ID")
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Error reading motor speed: {str(e)}")
	
	def read_motor_voltage(self):
		"""모터 전압 읽기"""
		self.motor_control.serial_command_type = 4
		try:
			if self.is_connected:
				# 현재 선택된 탭 인덱스 가져오기
				current_tab = self.tabWidget.currentIndex()
				
				# 탭 인덱스에 따라 적절한 Motor ID 선택
				if current_tab == 0:
					motor_id = int(self.textEdit_Motor_ID1.toPlainText())
				elif current_tab == 1:
					motor_id = int(self.textEdit_Motor_ID2.toPlainText())
				elif current_tab == 2:
					motor_id = int(self.textEdit_Motor_ID3.toPlainText())
				elif current_tab == 3:
					motor_id = int(self.textEdit_Motor_ID4.toPlainText())
						
				else:
					QMessageBox.warning(self, "Warning", "Invalid tab selected")
					return
					
				if self.motor_control.read_system_voltage(motor_id):
					print(f"Reading motor voltage for Motor ID {motor_id}...")
				else:
					QMessageBox.warning(self, "Warning", "Failed to read motor voltage")
			else:
				QMessageBox.warning(self, "Warning", "Please connect to serial port first")
		except ValueError:
			QMessageBox.critical(self, "Error", "Please enter a valid Motor ID")
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Error reading motor voltage: {str(e)}")
	
	
	def read_motor_encoder(self):
		"""모터 엔코더 값 읽기"""
		self.motor_control.serial_command_type = 7
		try:
			if self.is_connected:
				# 현재 선택된 탭 인덱스 가져오기
				current_tab = self.tabWidget.currentIndex()
				
				# 탭 인덱스에 따라 적절한 Motor ID 선택
				if current_tab == 0:
					motor_id = int(self.textEdit_Motor_ID1.toPlainText())
				elif current_tab == 1:
					motor_id = int(self.textEdit_Motor_ID2.toPlainText())
				elif current_tab == 2:
					motor_id = int(self.textEdit_Motor_ID3.toPlainText())
				elif current_tab == 3:
					motor_id = int(self.textEdit_Motor_ID4.toPlainText())	
				else:
					QMessageBox.warning(self, "Warning", "Invalid tab selected")
					return
					
				if self.motor_control.read_encoder(motor_id):
					print(f"Reading encoder value for Motor ID {motor_id}...")
				else:
					QMessageBox.warning(self, "Warning", "Failed to read encoder value")
			else:
				QMessageBox.warning(self, "Warning", "Please connect to serial port first")
		except ValueError:
			QMessageBox.critical(self, "Error", "Please enter a valid Motor ID")
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Error reading encoder value: {str(e)}")
	
	
	def write_register_data(self):
		"""레지스터 값 쓰기"""
		try:
			if self.is_connected:
				# Register address parsing
				register_hex = self.textEdit_Write_Register_Address.toPlainText().strip()
				register_value = self.textEdit_Write_Register_Data.toPlainText().strip()
				
				# 16진수 형식 검사 및 변환 (0x 접두사 필수)
				if not register_hex.startswith('0x'):
					QMessageBox.warning(self, "Warning", "Register address must be in hexadecimal format starting with '0x' (e.g., '0x01')")
					return
					
				try:
					register_address = int(register_hex, 16)
				except ValueError:
					QMessageBox.warning(self, "Warning", "Please enter a valid hexadecimal register address (e.g., '0x01')")
					return
				
				# Convert register value from hex or decimal
				try:
					if register_value.startswith('0x'):
						value = int(register_value, 16)
					else:
						value = int(register_value)
				except ValueError:
					QMessageBox.warning(self, "Warning", "Please enter a valid register value")
					return
				
				# Get Motor ID
				motor_id = int(self.textEdit_Motor_Enable_ID.toPlainText())
				
				# Create and send write register command
				if self.motor_control.write_register(motor_id, register_address, value):
					print(f"Writing value 0x{value:04X} to register 0x{register_address:02X} of motor {motor_id}")
				else:
					QMessageBox.warning(self, "Warning", "Failed to write register")
			else:
				QMessageBox.warning(self, "Warning", "Please connect to serial port first")
				
		except ValueError:
			QMessageBox.critical(self, "Error", "Please enter valid numbers for Motor ID and register")
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Error writing register: {str(e)}")
		
	def read_register_data(self):
		"""레지스터 값 읽기"""
		try:
			if self.is_connected:
				# 레지스터 주소 읽기 및 공백 제거
				register_hex = self.textEdit_Read_Register_Address.toPlainText().strip()
				
				# 16진수 형식 검사 및 변환 (0x 접두사 필수)
				if not register_hex.startswith('0x'):
					QMessageBox.warning(self, "Warning", "Register address must be in hexadecimal format starting with '0x' (e.g., '0x01')")
					return
					
				try:
					register_address = int(register_hex, 16)
				except ValueError:
					QMessageBox.warning(self, "Warning", "Please enter a valid hexadecimal register address (e.g., '0x01')")
					return
					
				# Motor ID 가져오기 
				motor_id = int(self.textEdit_Motor_Enable_ID.toPlainText())
				
				# 레지스터 읽기 실행
				if self.motor_control.read_register(motor_id, register_address):
					print(f"Reading register 0x{register_address:02X} from motor {motor_id}")
				else:
					QMessageBox.warning(self, "Warning", "Failed to read register")
			else:
				QMessageBox.warning(self, "Warning", "Please connect to serial port first")
				
		except ValueError:
			QMessageBox.critical(self, "Error", "Please enter valid numbers for Motor ID and register")
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Error reading register: {str(e)}")
	def test_button(self):
		"""test button"""
		self.motor_control.serial_command_type = 100
		
		try:
			if self.is_connected:
				motor_id = int(self.textEdit_Current_ID.toPlainText())
				self.motor_control.test(motor_id)
		except ValueError:
			QMessageBox.critical(self, "Error", "Please enter a valid Motor ID")
		except Exception as e:
			QMessageBox.critical(self, "Error", f"Error reading motor ID: {str(e)}")
							
	@pyqtSlot(int)		
	def update_current_id(self, motor_id):
		"""현재 ID를 UI에 업데이트"""
		self.textEdit_Current_ID.setText(str(motor_id))		
	
	def cleanup(self):
		"""리소스 정리"""
		try:
			# 종료 이벤트 설정
			self.exit_event.set()
			
			# 시리얼 연결 해제
			if self.is_connected:
				if self.serial_reader:
					self.serial_reader.stop()
					self.serial_reader.join(timeout=1.0)
				self.motor_control.disconnect_serial()
				self.is_connected = False
				
		except Exception as e:
			print(f"Cleanup error: {e}")
    
	@pyqtSlot(int, float)
	def update_voltage_display(self, motor_id, voltage):
		"""전압값을 UI에 업데이트"""
		# 현재 선택된 탭 확인
		current_tab = self.tabWidget.currentIndex()
		
		# 탭과 motor_id에 따라 적절한 textEdit 선택
		if current_tab == 0:
			self.textEdit_Read_Motor_Voltage1.setText(f"{voltage:.2f}")
		elif current_tab == 1:
			self.textEdit_Read_Motor_Voltage2.setText(f"{voltage:.2f}")
		elif current_tab == 2:
			self.textEdit_Read_Motor_Voltage3.setText(f"{voltage:.2f}")
		elif current_tab == 3:
			self.textEdit_Read_Motor_Voltage4.setText(f"{voltage:.2f}")
	
	@pyqtSlot(int, float)
	def update_speed_display(self, motor_id, speed):
		"""속도값을 UI에 업데이트"""
		try:
			# motor_id에 따라 적절한 텍스트 에디트 선택
			speed_displays = {
				1: self.textEdit_Read_Motor_Speed1,
				2: self.textEdit_Read_Motor_Speed2,
				3: self.textEdit_Read_Motor_Speed3,
				4: self.textEdit_Read_Motor_Speed4
			}
			
			if motor_id in speed_displays:
				# 직접 받은 speed 값을 사용 (추가 변환 없이)
				formatted_speed = f"{speed:.1f}"
				speed_displays[motor_id].setText(formatted_speed)
				print(f"Updated speed display for motor {motor_id}: {formatted_speed} RPM")
			else:
				print(f"Invalid motor ID: {motor_id}")
				
		except Exception as e:
			print(f"Error updating speed display: {e}", 
				  f"motor_id: {motor_id}, speed: {speed}",
				  f"speed type: {type(speed)}")

	@pyqtSlot(int, int)
	def update_encoder_display(self, motor_id, encoder_value):
		"""엔코더 값을 UI에 업데이트"""
		# 현재 선택된 탭 확인
		current_tab = self.tabWidget.currentIndex()
		
		# 탭과 motor_id에 따라 적절한 textEdit 선택
		if current_tab == 0:
			self.textEdit_Read_Motor_Encoder1.setText(str(encoder_value))
		elif current_tab == 1:
			self.textEdit_Read_Motor_Encoder2.setText(str(encoder_value))
		elif current_tab == 2:
			self.textEdit_Read_Motor_Encoder3.setText(str(encoder_value))
		elif current_tab == 3:
			self.textEdit_Read_Motor_Encoder4.setText(str(encoder_value))		
								
	def closeEvent(self, event):
		"""프로그램 종료시 처리"""
		self.cleanup()
		event.accept()
		QApplication.quit()
		os._exit(0)
		
	def handle_accept(self):
		"""OK 버튼 클릭 시 처리"""
		self.cleanup()
		self.accept()
		QApplication.quit()
		os._exit(0)
		
	def handle_reject(self):
		"""Cancel 버튼 클릭 시 처리"""
		self.cleanup()
		self.reject()
		QApplication.quit()
		os._exit(0)
			
def main():
	app = QApplication(sys.argv)

	# 예외 처리 추가
	try:
		myWindow = WindowClass()
		myWindow.show()

		# SIGINT 핸들러 설정
		signal.signal(signal.SIGINT, lambda signum, frame: app.quit())
		
		# 이벤트 루프 시작
		exit_code = app.exec_()
		
		# 프로그램 종료
		sys.exit(exit_code)
		
	except Exception as e:
		print(f"Error in main: {str(e)}")
		sys.exit(1)

if __name__ == '__main__':
	try:
		main()
	except Exception as e:
		print(f"Error: {str(e)}")
		
		
