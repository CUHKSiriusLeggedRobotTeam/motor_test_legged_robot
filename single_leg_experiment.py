'''
Motor Module example program.
'''

import motormodule as mm
import time
import matplotlib.pyplot as plt
import numpy as np
class TestModule:
	def __init__(self,can_id,stand_time,COM):
		self.l1=0.3
		self.l2=0.3
		self.COM=COM
		self.can_id=can_id
		self.stand_time=stand_time
		self.mmc = mm.MotorModuleController(self.COM)		# Connect to the controller's serial port

		self.mmc.disable_motor(self.can_id)
		self.mmc.enable_motor(self.can_id)							# Enable motor with CAN ID 1
		# Send a command:  
		# (CAN ID, position setpoint, velocity setpoint, position gain, velocity gain, feed-forward torque)
		# Units are:  Radians, Rad/s, N-m/rad, N-m/rad/s, N-m
		self.mmc.send_command(self.can_id, 0, 0, 0, 0,  0)
		time.sleep(.1)
	def Enable_three_motor(self,):
		self.mmc.disable_motor(1)

		self.mmc.disable_motor(2)
		self.mmc.disable_motor(3)

		self.mmc.enable_motor(1)
		self.mmc.enable_motor(2)
		self.mmc.enable_motor(3)
		time.sleep(.1)
		self.mmc.send_command(1, 0, 0, 0, 0,  0)
		self.mmc.send_command(2, 0, 0, 0, 0,  0)
		self.mmc.send_command(3, 0, 0, 0, 0,  0)
		time.sleep(.1)
	def Read_Reply_Data(self,p_des,v_des,t_des):

		# This example reads the start position of the motor
		# Then sends position commands to ramp the motor position to zero 
		# With a 1st-order response
		# rx_values are ordered (CAN ID, Position, Velocity, Current)
		real_position = self.mmc.rx_values[1]
		real_Velocity = self.mmc.rx_values[2]
		real_Current = self.mmc.rx_values[3]
		print("real_position:[ "+str(real_position[0])+" ]real_Velocity[ "+str(real_Velocity[0])+" ]real_Current[ "+str(real_Current[0])+" ]")
		print("Pos_Error"+str(p_des-real_position[0])+"Ves_Error"+str(v_des-real_Velocity[0])+"Current_Error"+str(t_des-real_Current[0]))
		#p_des = start_position[0]
		error=[p_des-real_position[0],v_des-real_Velocity[0],t_des-real_Current[0]]
		return error,[real_position,real_Velocity,real_Current]
	def disable_motor(self): 
		self.mmc.disable_motor(self.can_id) # Disable motor with CAN ID 1
	def end_point(self,stand_h):
		theta0=0
		theta1=-np.arccos((self.l1*self.l1+stand_h*stand_h-self.l2*self.l2)/(2.0*self.l1*stand_h))
		theta2=np.pi-np.arccos((self.l1*self.l1+self.l2*self.l2-self.stand_h*stand_h)/(2.0*self.l1*self.l2))
		return [theta0,theta1,theta2] 
	def cubicBezier(self,y0,yf,x):
		#y0,yf,x is 3 list
		bezier=[0,0,0]
		ydiff=[0,0,0]
		if x[0]<0.0 and x[1]<0.0 and x[2]<0.0:
			x=[0,0,0]
		if x[1]>1.0 and x[0]>1.0 and x[2]>1.0:
			x=[1.0,1.0,1.0]
		ydiff[0]=yf[0]-y0[0]
		ydiff[1]=yf[1]-y0[1]
		ydiff[2]=yf[2]-y0[2]

		bezier[0]=x[0]*x[0]*x[0]+3.0*(x[0]*x[0]*(1.0-x[0]))
		bezier[1]=x[1]*x[1]*x[1]+3.0*(x[1]*x[1]*(1.0-x[1]))
		bezier[2]=x[2]*x[2]*x[2]+3.0*(x[2]*x[2]*(1.0-x[2]))
		res=[0,0,0]
		res[0]=y0[0]+bezier[0]*ydiff[0]
		res[1]=y0[1]+bezier[1]*ydiff[1]
		res[2]=y0[2]+bezier[2]*ydiff[2]
		return res
	def cubicBezierFirstDerivative(self,y0,yf,x):
		#y0,yf,x is 3 list
		bezier=[0,0,0]
		ydiff=[0,0,0]
		if x[0]<0.0 and x[1]<0.0 and x[2]<0.0:
			x=[0,0,0]
		if x[1]>1.0 and x[0]>1.0 and x[2]>1.0:
			x=[1.0,1.0,1.0]
		ydiff[0]=yf[0]-y0[0]
		ydiff[1]=yf[1]-y0[1]
		ydiff[2]=yf[2]-y0[2]

		bezier[0]=6.0*(x[0]*(1.0-x[0]))
		bezier[1]=6.0*(x[1]*(1.0-x[1]))
		bezier[2]=6.0*(x[2]*(1.0-x[2]))
		res=[0,0,0]
		res[0]=bezier[0]*ydiff[0]/self.stand_time
		res[1]=bezier[1]*ydiff[1]/self.stand_time
		res[2]=bezier[2]*ydiff[2]/self.stand_time
		return res

def main():
	can_id=1
	stand_time=1.5
	COM="COM12"
	tm=TestModule(can_id,stand_time,COM)
	t_des=-48
	kp=200
	kd=0.3
	t_p_e=0
	detat=0.0
	dt=0.01
	cnt=0
	start_time=time.time()
	p_start=[0,0,0]
	p_des=[0,-1.4,2.7] #folding
	try:
		while(1):
			pres=tm.cubicBezier(p_start,p_des,[detat,detat,detat])
			pdres=tm.cubicBezierFirstDerivative(p_start,p_des,[detat,detat,detat])
			print(pres,pdres)
			if 0 not in pres and cnt>3:
				tm.mmc.send_command(1, pres[0] , pdres[0], kp, kd, t_des)
				tm.mmc.send_command(2, pres[1] , pdres[1], kp, kd, t_des)
				tm.mmc.send_command(3, pres[2] , pdres[2], kp, kd, t_des)
			detat=cnt*dt
			cnt+=1
			print(detat)
			time.sleep(.02)

	except KeyboardInterrupt:
		tm.disable_motor()
	# tm.disable_motor()
	
if __name__=='__main__':
	main()

	
	