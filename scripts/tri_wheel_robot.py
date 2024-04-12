#!/usr/bin/env python3
#
# This is an example of how to use a Logitech F310 Game Controller
# to control the motors and servos on the HLRobotController nodule
# via WiFi.
#
# This has been tested on MacOS, but not other platforms.
#
# This requires a few python packages. It is recommended to install
# them in a virtual environment like this:
#
# python3 -m venv venv
# source venv/bin/activate
# pip install --upgrade pip
# pip install hid websocket-client
#
# then
#
#  ./gamepad_example.py [host]
#
# where [host] should be replaced with the HLController IP address.
#
#-----------------------------------------------------------------
# Controls:
#
# push in the right joystick to enable all channels.
# push in the left joystick to disable all channels.
#
# Left  joystick controls M0(vertical) and M1(horizontal)
# Right joystick controls M2(vertical) and M3(horizontal)
#
# DPad up/down controls S0
# DPad left/right controls S1
#
# Buttons Y,A control S4
# Buttons X,B control S5

import math
import websocket
import threading
import time
import sys
import os
import subprocess
import hid
from datetime import datetime

host = "192.168.1.52"
port = "81"

if len(sys.argv) > 1:
	host = sys.argv[1]
	print("Setting host to: %s" % host)


def on_message(ws, message):
	# print(f"Received '{message}'")
	pass

def on_error(ws, error):
	print(f"Error: {error}")

def on_close(ws, close_status_code, close_msg):
	print("### closed ###")

def on_open(ws):
	print("Connection established")
	# ws.send("Hello ESP8266")

# websocket.enableTrace(True)
def WSThread():
	global ws
	ws = websocket.WebSocketApp("ws://" + host + ":" + port + "/",
							  on_open=on_open,
							  on_message=on_message,
							  on_error=on_error,
							  on_close=on_close)
	print("Websockets thread started.")
	ws.run_forever()

threading.Thread(target=WSThread).start()
 
last_move_command_send_time = datetime.now()

#-------------------------------------
# LogitechReportToState
#-------------------------------------
def LogitechReportToState( report ):
	state = {}
	state['left_joy_H'] = max( (float(report[0])-128.0)/127.0, -1.0)
	state['left_joy_V'] = max( (float(report[1])-128.0)/127.0, -1.0)
	state['right_joy_H'] = max( (float(report[2])-128.0)/127.0, -1.0)
	state['right_joy_V'] = max( (float(report[3])-128.0)/127.0, -1.0)
	dpad = report[4] & 0b00001111
	state['dpad_up'   ] = (dpad==0) or (dpad==1) or (dpad==7)
	state['dpad_down' ] = (dpad==3) or (dpad==4) or (dpad==5)
	state['dpad_left' ] = (dpad==5) or (dpad==6) or (dpad==7)
	state['dpad_right'] = (dpad==1) or (dpad==2) or (dpad==3)
	state['button_X'] = (report[4] & 0b00010000) != 0
	state['button_A'] = (report[4] & 0b00100000) != 0
	state['button_B'] = (report[4] & 0b01000000) != 0
	state['button_Y'] = (report[4] & 0b10000000) != 0
	state['bumper_left'  ] = (report[5] & 0b00000001) != 0
	state['bumper_right' ] = (report[5] & 0b00000010) != 0
	state['trigger_left' ] = (report[5] & 0b00000100) != 0
	state['trigger_right'] = (report[5] & 0b00001000) != 0
	state['back'         ] = (report[5] & 0b00010000) != 0
	state['start'        ] = (report[5] & 0b00100000) != 0
	state['L3'           ] = (report[5] & 0b01000000) != 0
	state['R3'           ] = (report[5] & 0b10000000) != 0
	return state

#-------------------------------------
# SetEnableAll
#-------------------------------------
def SetEnableAll(val):
	for i in range(4): ws.send("enable M" + str(i) + " " + str(val))
	for i in range(8): ws.send("enable S" + str(i) + " " + str(val))
    

# Setup HID (game controller)
# Find Logitech device
vendor_id = 0x0
product_id = 0x0
gamepad = None
state = None
for d in hid.enumerate():
	if d['product_string'] == 'Logitech Dual Action':
		vendor_id  = int(d['vendor_id' ])
		product_id = int(d['product_id'])
		print('Found Logictech gamepad: vendor_id:0x%x product_id:0x%x' % (vendor_id, product_id))
		gamepad = hid.Device(vid=vendor_id, pid=product_id)
		gamepad.nonblocking = True
#		gamepad = hid.device()
#		gamepad.open(vendor_id, product_id)
#		gamepad.set_nonblocking(True)
if not gamepad:
	print('Unable to find gamepad!')
else:
	last_R3 = False
	last_L3 = False
	last_left_joy_V = -1
	last_left_joy_H = -1
	last_right_joy_V = -1
	last_right_joy_H = -1

	last_P1 = -1
	last_P2 = -1
	last_P3 = -1

	while True:
		report = gamepad.read(512)
		if report:
			state = LogitechReportToState(report)
		else:
			#state = None
			time.sleep(0.050)
			#print('Unable to get controller state')
		if state:
			message = ''
      
			if state['L3']: SetEnableAll(0)
			if state['R3']: SetEnableAll(1)

			if state['dpad_up'   ]: ws.send("incr S0  0.01")
			if state['dpad_down' ]: ws.send("incr S0 -0.01")
			if state['dpad_left' ]: ws.send("incr S1  0.01")
			if state['dpad_right']: ws.send("incr S1 -0.01")

			if state['button_Y'  ]: ws.send("incr S4  0.01")
			if state['button_A'  ]: ws.send("incr S4 -0.01")
			if state['button_X'  ]: ws.send("incr S5  0.01")
			if state['button_B'  ]: ws.send("incr S5 -0.01")


			# For the joysticks we implement a deadband around zero
			# as well as the last value sent. This significantly 
			# reduces the number of messages sent.
			left_joy_V = state['left_joy_V']
			left_joy_H = state['left_joy_H']
			right_joy_V = state['right_joy_V']
			right_joy_H = state['right_joy_H']

			if( abs(left_joy_V ) < 0.05 ) :  left_joy_V  = 0
			if( abs(left_joy_H ) < 0.05 ) :  left_joy_H  = 0
			if( abs(right_joy_V) < 0.05 ) :  right_joy_V = 0
			if( abs(right_joy_H) < 0.05 ) :  right_joy_H = 0

			P1 = (2.0/3.0)*left_joy_H+(1.0/3.0)*right_joy_H
			P2 = (-1.0/3.0)*left_joy_H+(1.0/math.sqrt(3.0))*left_joy_V+(1.0/3.0)*right_joy_H
			P3 = (-1.0/3.0)*left_joy_H-(1.0/math.sqrt(3.0))*left_joy_V+(1.0/3.0)*right_joy_H

			# # Motors are reversed
			P1 = -P1
			P2 = -P2
			P3 = -P3

			if( abs(P1 ) < 0.05 ) :  P1 = 0
			if( abs(P2 ) < 0.05 ) :  P2 = 0
			if( abs(P3 ) < 0.05 ) :  P3 = 0

			if( abs( P1 - last_P1 ) > 0.05 ):
				last_P1 = P1
				cmd = "set M0 " + str(P1)
				print(cmd)
				ws.send(cmd)
			if( abs( P2 - last_P2 ) > 0.05 ):
				last_P2 = P2
				cmd = "set M1 " + str(P2)
				print(cmd)
				ws.send(cmd)
			if( abs( P3 - last_P3 ) > 0.05 ):
				last_P3 = P3
				cmd = "set M3 " + str(P3)
				print(cmd)
				ws.send(cmd)
			if( abs( right_joy_H - last_right_joy_H ) > 0.05 ):
				last_right_joy_H = right_joy_H
				cmd = "set M3 " + str(last_right_joy_H)
				print(cmd)
				#ws.send(cmd)
   
			if message : print(message)
			message = None



