'''
Motor Module example program.
'''

from os import closerange

import time
import matplotlib.pyplot as plt
import numpy as np
import csv
import datetime

class TestModule:
	def __init__(self,can_id,stand_time,COM,sim):
		self.COM=COM
		self.can_id=can_id	
		self.stand_time=stand_time
		if sim!=0:
			import motormodule as mm
			self.mmc = mm.MotorModuleController(self.COM)		# Connect to the controller's serial port
			self.init_motor_com(1)
		self.l1 = 0.079#_abadLinkLength;
		self.l2 = 0.3#_hipLinkLength;
		self.l3 = 0.3#_kneeLinkLength;
		self.l4 = 0#_kneeLinkY_offset;
		
		self.mini_knee_offset=4.35
		self.mini_hip_offset=np.pi/2
	def init_motor_com(self,flag):
		if flag:
			self.mmc.disable_motor(self.can_id[0])
			self.mmc.enable_motor(self.can_id[0])	
			self.mmc.disable_motor(self.can_id[1])
			self.mmc.enable_motor(self.can_id[1])							# Enable motor with CAN ID 1
			self.mmc.disable_motor(self.can_id[2])
			self.mmc.enable_motor(self.can_id[2])							# Enable motor with CAN ID 2
			# Send a command:  
			# (CAN ID, position setpoint, velocity setpoint, position gain, velocity gain, feed-forward torque)
			# Units are:  Radians, Rad/s, N-m/rad, N-m/rad/s, N-m
			self.mmc.send_command(self.can_id[0], 0, 0, 0, 0,  0)
			time.sleep(.1)
			self.mmc.send_command(self.can_id[1], 0, 0, 0, 0,  0)
			time.sleep(.1)
			self.mmc.send_command(self.can_id[2], 0, 0, 0, 0,  0)
			time.sleep(.1)
		else:
			self.mmc.disable_motor(self.can_id[1])
			self.mmc.enable_motor(self.can_id[1])							# Enable motor with CAN ID 1
						# Enable motor with CAN ID 1
			# Send a command:  
			# (CAN ID, position setpoint, velocity setpoint, position gain, velocity gain, feed-forward torque)
			# Units are:  Radians, Rad/s, N-m/rad, N-m/rad/s, N-m
			self.mmc.send_command(self.can_id[1], 0, 0, 0, 0,  0)
			time.sleep(.1)

	
	def Read_Reply_Data(self,can_id,p_des,v_des,t_des):

		# This example reads the start position of the motor
		# Then sends position commands to ramp the motor position to zero 
		# With a 1st-order response
		# rx_values are ordered (CAN ID, Position, Velocity, Current)
		if can_id == 1:
			real_position = self.mmc.rx_values_1[1]
			real_Velocity = self.mmc.rx_values_1[2]
			real_Current = self.mmc.rx_values_1[3]
		elif can_id == 2:
			real_position = self.mmc.rx_values_2[1]
			real_Velocity = self.mmc.rx_values_2[2]
			real_Current = self.mmc.rx_values_2[3]
		elif can_id == 3:
				real_position = self.mmc.rx_values_3[1]
				real_Velocity = self.mmc.rx_values_3[2]
				real_Current = self.mmc.rx_values_3[3]
		else:
			pass
		# print(real_position)
		# print("real_position:[ "+str(real_position[0])+" ]real_Velocity[ "+str(real_Velocity[0])+" ]real_Current[ "+str(real_Current[0])+" ]")
		# print("Pos_Error"+str(p_des-real_position[0])+"Ves_Error"+str(abs(v_des)-abs(real_Velocity[0]))+"Current_Error"+str(abs(t_des)-abs(real_Current[0])))
		#p_des = start_position[0]
		error=[p_des-real_position[0],v_des-real_Velocity[0],t_des-real_Current[0]]
		return error,[real_position,real_Velocity,real_Current]
	def disable_motor(self,flag,sim):
		if sim!=0:
			if flag:
				self.mmc.disable_motor(self.can_id[0]) # Disable motor with CAN ID 1
				self.mmc.disable_motor(self.can_id[1]) # Disable motor with CAN ID 1
				self.mmc.disable_motor(self.can_id[2]) # Disable motor with CAN ID 1
			else:
				self.mmc.disable_motor(self.can_id[1]) # Disable motor with CAN ID 1
	def send_zero_to_motor(self,sim):
		if sim!=0:
			self.mmc.send_command(self.can_id[0], 0 ,  0, 0,0, 0)
			self.mmc.send_command(self.can_id[1], 0 ,  0, 0,0, 0)
			self.mmc.send_command(self.can_id[2], 0 ,  0, 0,0, 0)
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
	def calculate_joint_angular(self,height,x,y):
		# l1=0.21 #0.3
		# l2=0.2 #0.3
		l1 = 0.3
		l2 = 0.3
		theat0=0
		# theta1=-np.arccos((l1*l1+height*height-l2*l2)/(2*l1*height))+np.arctan(x/y)
		# theta2=-np.pi-np.arccos((l1*l1-height*height+l2*l2)/(2*l1*l2))
		theta1=np.arccos((l1*l1+height*height-l2*l2)/(2*l1*height))+np.arctan(y/x)
		add = (l1*l1-height*height+l2*l2)/(2*l1*l2)
		if add <= -1:
			add = -1
		elif add >= 1:
			add = 1
		theta2=-(np.pi-np.arccos(add))
		# theta2=-(np.pi-np.arccos((l1*l1-height*height+l2*l2)/(2*l1*l2)))
		return [theat0,theta1,theta2]
	# def calculate_circle_for_foot(self,circle_center_co=(0., -0.3),radius=0.05):
	def calculate_circle_for_foot(self,circle_center_co=(0.5, 0),radius=0.1):
		#-radius<x<radius
		# l1=0.21
		# l2=0.21
		r = radius
		a, b = circle_center_co
		theta = np.arange(0, 2*np.pi, 0.25)#
		x = a + r * np.cos(theta)
		y = b + r * np.sin(theta)
		height=[]
		anguar=[]
		for i in range(len(x)):
			anguar.append(self.calculate_joint_angular(np.sqrt(x[i]*x[i]+y[i]*y[i]),x[i],y[i]))

		return anguar
	def getSideSign(self,leg):
		sideSigns = [-1, 1, -1, 1]
		return sideSigns[leg]
	def Calculate_Jocobian_for_leg_robot(self,q,inv_xyz,leg,J_flag,P_flag,Inv_flag):

		l1 = self.l1
		l2 = self.l2
		l3 = self.l3
		l4 = self.l4
  
		sideSign = self.getSideSign(leg)

		s1 = np.sin(q[0])
		s2 = np.sin(q[1])
		s3 = np.sin(q[2])

		c1 = np.cos(q[0])
		c2 = np.cos(q[1])
		c3 = np.cos(q[2])

		c23 = c2 * c3 - s2 * s3
		s23 = s2 * c3 + c2 * s3
	
		J=np.zeros( (3,3) )
		#fk method 1
		# if J_flag:
		# 	J[0, 0] = 0
		# 	J[0, 1] = l3 * c23 + l2 * c2
		# 	J[0, 2] = l3 * c23
		# 	J[1, 0] = l3 * c1 * c23 + l2 * c1 * c2 - (l1+l4) * sideSign * s1
		# 	J[1, 1] = -l3 * s1 * s23 - l2 * s1 * s2
		# 	J[1, 2] = -l3 * s1 * s23
		# 	J[2, 0] = l3 * s1 * c23 + l2 * c2 * s1 + (l1+l4) * sideSign * c1
		# 	J[2, 1] = l3 * c1 * s23 + l2 * c1 * s2
		# 	J[2, 2] = l3 * c1 * s23
		# 	return J
		if J_flag:
			J[0, 0] = 0
			J[0, 1] = l3 * c23 + l2 * c2
			J[0, 2] = l3 * c23
			J[1, 0] = l3 * c1 * c23 + l2 * c1 * c2 + l1 * c1
			J[1, 1] = -l3 * s1 * s23 - l2 * s1 * s2
			J[1, 2] = -l3 * s1 * s23
			J[2, 0] = l3 * s1 * c23 + l2 * c2 * s1 + l1 * s1
			J[2, 1] = l3 * c1 * s23 + l2 * c1 * s2
			J[2, 2] = l3 * c1 * s23
			return J
		
		p=[0,0,0]
		if P_flag==1:
			# p[0] = l3 * s23 + l2 * s2
			# p[1] = (l1+l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1
			# p[2] = (l1+l4) * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2
			p[0] = l3 * s23 + l2 * s2
			p[1] = (l1+l4) * sideSign * s1 + l3 * (s1 * c23) + l2 * c2 * s1
			p[2] = -(l1+l4) * sideSign * c1 - l3 * (c1 * c23) - l2 * c1 * c2
			return p
		if Inv_flag==1:
			#assuming just have z aixes
			theta0=0
			h=inv_xyz[2]
			theta1 = -np.arccos((l2*l2+h*h-l3*l3)/(2*l2*h))
			theta2 = np.pi - np.arccos((l2*l2+l3*l3-h*h)/(2*l2*l3))
			return [0,theta1,theta2]
	def VMC_Control(self,p_foot_real,p_foot_desire,q_joint_dot_desire,q_joint_dot_real,kp_virtual_spring,kd_virtual_spring):
		"""
			Note:p_foot_desire is q_real
  		"""
		Fx=0
		Fy=0
		Fz=0
  		#q,inv_xyz,leg,J_flag,P_flag,Inv_flag
		q_desire=self.Calculate_Jocobian_for_leg_robot([0,0,0],p_foot_real,0,0,0,1)
  		J=self.Calculate_Jocobian_for_leg_robot(q_desire,[0,0,0],0,1,0,0)
		p_foot_dot_desire=np.dot(np.linalg.inv(J),q_joint_dot_desire)
		p_foot_dot_real=np.dot(np.linalg.inv(J),q_joint_dot_real)
		# print(p_foot_dot_desire,p_foot_dot_real)
		#PD control
		Fx=kp_virtual_spring*(p_foot_real[0]-p_foot_desire[0])+kd_virtual_spring*(p_foot_dot_real[0]-p_foot_dot_desire[0])
		Fy=kp_virtual_spring*(p_foot_real[1]-p_foot_desire[1])+kd_virtual_spring*(p_foot_dot_real[1]-p_foot_dot_desire[1])
		Fz=kp_virtual_spring*(p_foot_real[2]-p_foot_desire[2])+kd_virtual_spring*(p_foot_dot_real[2]-p_foot_dot_desire[2])
		F=np.array([Fx,Fy,Fz])
		print(F)
		
		Torque=np.dot(J.T,F.T)
		return Torque


def main():
	can_id=[1,2,3]
	stand_time=3
	COM="COM10"
	sim=0 #use for test code
	tm=TestModule(can_id,stand_time,COM,sim)
	v_des=0
	t_des=0
	#big dog
	kp=35#200	#450  #150 #5
	kd=2.4#5 #25  #0.005 	
	#mini dog
	# kp=35#200	#450  #150 #5
	# kd=1.4#5 #25  #0.005 	
	kp_virtual_spring=10
	kd_virtual_spring=0.01
   
	kp_joint=30
	kd_joint=1.3
	t_p_e=0
	error=[]
	dt=0.05
	cnt=0
	count=0
	detat=0.01
	#flag=1 vmc control for leg
	#flag=2 position control for leg
	#flag=3 postion and torque control for one motor
	flag=1

	xx=[]#t
	yy=[]#pos
	realp=[]
	realv=[]
	desirev=[]
	open_draw_flag=0
	xx_v=[]
	yy_v=[]
	zz_v=[]
	pdres=[0,0,0]
	open_draw_flag_v=0
	if sim!=0:
		f = open('m41_0.csv','w',encoding='utf-8')
		csv_writer = csv.writer(f)
		csv_writer.writerow(["Time","P","V","C"])
	t_des_es=0
	pdes_circle=tm.calculate_circle_for_foot()
	p_start=[0,0,0]
	insert_count=0
	t_des1=0.
	t_des2=0.
	t_des3=0.
	t_des_es0=0.
	t_des_es1=0.
	t_des_es2=0.
	p_foot_desire=[0,0,0.35]
	q_joint_desire=tm.Calculate_Jocobian_for_leg_robot([0,0,0],p_foot_desire,0,0,0,1)
	try:
		while(1):
			starttime=time.time()
			# use for vmc control
			if flag==1:
				if cnt>3:
					# position control
					pres=tm.cubicBezier(p_start,q_joint_desire,[detat,detat,detat])
					if sim!=0:
						tm.mmc.send_command(can_id[0], pres[0] ,  pdres[0], kp, kd, t_des_es0)
						time.sleep(0.001)
						tm.mmc.send_command(can_id[1], pres[1] ,  pdres[0], kp, kd, t_des_es1)
						time.sleep(0.001)
						tm.mmc.send_command(can_id[2], pres[2] ,  pdres[0], kp, kd, t_des_es2)
						time.sleep(0.001)
						error,realdata1=tm.Read_Reply_Data(can_id[0],pres[0] , pdres[0],t_des1)
						error,realdata2=tm.Read_Reply_Data(can_id[1],pres[1] , pdres[0],t_des2)
						error,realdata3=tm.Read_Reply_Data(can_id[2],pres[2] , pdres[0],t_des3)
					else:
						realdata1=[[0],[0],[0]]
						realdata2=[[0],[0],[0]]
						realdata3=[[0],[0],[0]]

					t_des_es0=t_des1+kp_joint*( pres[0]-realdata1[0][0])+kd_joint*(pdres[0]-realdata1[1][0])
					t_des_es1=t_des2+kp_joint*( pres[1]-realdata2[0][0])+kd_joint*(pdres[1]-realdata2[1][0])
					t_des_es2=t_des3+kp_joint*( pres[2]-realdata3[0][0])+kd_joint*(pdres[2]-realdata3[1][0])
					
					#VMC_Control(self,p_foot_real,p_foot_desire,q_dot_desire,q_dot_real,kp_virtual_spring,kd_virtual_spring)
					#q,inv_xyz,leg,J_flag,P_flag,Inv_flag
					#foot pose
					p_foot_real=tm.Calculate_Jocobian_for_leg_robot([realdata1[0][0],realdata2[0][0],realdata3[0][0]],[0,0,0],0,0,1,0)
     
					t_des_vmc=tm.VMC_Control(p_foot_real,p_foot_desire,[0,0,0],[realdata1[1][0],realdata2[1][0],realdata3[1][0]],kp_virtual_spring,kd_virtual_spring)
					t_des1=t_des_vmc[0]
					t_des2=t_des_vmc[1]
					t_des3=t_des_vmc[2]

					xx.append(detat)
					yy.append(pres)
					desirev.append(pdres)
					realp.append(realdata1[0])
					realv.append(realdata1[1])
					# detat=count/(len(pdes_circle))
					# t_p_e=abs(p_des[0]-realdata[0][0])
					# print("Desire--->",pres,"Joint2->",realdata1,"Joint3",realdata2,count)
					# if insert_count < 10:
					insert_count+=1
					# detat = cnt * dt
					# 	counting = 0.01
					# 	detat = insert_count * counting
					# detat=cnt*dt
					if count>=(len(pdes_circle)-1):
						count=0
					if insert_count>=10:
						count+=1
					# 	# insert_count=0
						# detat = 1
						# p_start=pdes_circle[count-1]
						# cnt=0
						# print("ok---- t_p_e",t_p_e)
						# open_draw_flag=1
						# break
					# if count>30:
					# 	p_start=pdes_circle[count-10]
					# if t_p_e<0.05:
					# 	print("ok---- t_p_e",t_p_e)
					# 	open_draw_flag=1
					# 	break
				else:
					tm.send_zero_to_motor(sim)
			elif flag==2:
				if cnt>3:
					# position control
					pres=tm.cubicBezier(p_start,pdes_circle[count],[detat,detat,detat])
					print("----->",pres,pdes_circle[count],detat)
					# if pres[2] == np.nan:
					# 	pres[2] == 0
					# else:
					# 	pass
					# print(pres[2])
					tm.mmc.send_command(can_id[0], pres[0] ,  pdres[0], kp, kd, t_des)
					time.sleep(0.001)
					tm.mmc.send_command(can_id[1], pres[1] ,  pdres[0], kp, kd, t_des)
					time.sleep(0.001)
					tm.mmc.send_command(can_id[2], pres[2] ,  pdres[0], kp, kd, t_des)
					time.sleep(0.001)
					error,realdata1=tm.Read_Reply_Data(can_id[0],pres[0] , pdres[0],t_des)
					error,realdata1=tm.Read_Reply_Data(can_id[1],pres[1] , pdres[0],t_des)
					error,realdata2=tm.Read_Reply_Data(can_id[2],pres[2] , pdres[0],t_des)
					xx.append(count/(len(pdes_circle)))
					yy.append(pres)
					desirev.append(pdres)
					realp.append(realdata1[0])
					realv.append(realdata1[1])
					# detat=count/(len(pdes_circle))
					# t_p_e=abs(p_des[0]-realdata[0][0])
					# print("Desire--->",pres,"Joint2->",realdata1,"Joint3",realdata2,count)
					# if insert_count < 10:
					insert_count+=1
					# detat = cnt * dt
					# 	counting = 0.01
					# 	detat = insert_count * counting
					detat=cnt*dt
					if count>=(len(pdes_circle)-1):
						count=0
					if insert_count>=10:
						count+=1
					# 	# insert_count=0
						detat = 1
						# p_start=pdes_circle[count-1]
						# cnt=0
						# print("ok---- t_p_e",t_p_e)
						# open_draw_flag=1
						# break
					# if count>30:
					# 	p_start=pdes_circle[count-10]
					# if t_p_e<0.05:
					# 	print("ok---- t_p_e",t_p_e)
					# 	open_draw_flag=1
					# 	break
				else:
					tm.mmc.send_command(can_id[0], 0 ,  0, 0,0, 0)
					tm.mmc.send_command(can_id[1], 0 ,  0, 0,0, 0)
					tm.mmc.send_command(can_id[2], 0 ,  0, 0,0, 0)
			elif flag==3:
				if cnt>3:
					# pres=tm.cubicBezier(p_start,pdes_circle[count],[detat,detat,detat])
					pres=tm.cubicBezier([1,1,1],[2,2,2],[detat,detat,detat])
					tm.mmc.send_command(can_id[1], pres[2] ,  pdres[0], kp, kd, t_des_es)
					time.sleep(0.001)
					# p_des_=1
					v_des_=0#200RPM/ 0.5-42	
					t_des=3
					error,realdata1=tm.Read_Reply_Data(can_id[1],pres[2] , pdres[0],t_des)
					# torque mode ok
					t_des_es=t_des+kp_joint*( pres[2]-realdata1[0][0])+kd_joint*(v_des_-realdata1[1][0])
					xx.append(detat*cnt)
					yy.append(pres)
					desirev.append(pdres)
					realp.append(realdata1[0])
					realv.append(realdata1[1])
					
					# t_p_e=abs(p_des[0]-realdata[0][0])
					print("Desire--->",pres,"Joint2->",realdata1,count,t_des_es)
					count+=1
					if count>=len(pdes_circle):
						count=0
						# print("ok---- t_p_e",t_p_e)
						# open_draw_flag=1
						# break
					# if count>30:
					# 	p_start=pdes_circle[count-10]
					# if t_p_e<0.05:
					# 	print("ok---- t_p_e",t_p_e)
					# 	open_draw_flag=1
					# 	break
				else:
					tm.mmc.send_command(can_id[1], 0 ,  0, 0,0, 0)
			# count += 1
			detat = cnt * dt
			cnt+=1
			# detat=cnt*detat
		if open_draw_flag_v==1:
			open_draw_flag_v=0
			plt.figure(figsize=(10,5))
			plt.plot(np.array(xx_v),np.array(yy_v),color="deeppink",linewidth=2,linestyle=':',label='Vd', marker='o')
			plt.plot(np.array(xx_v),np.array(zz_v),color="goldenrod",linewidth=2,linestyle=':',label='Vr', marker='o')
			plt.show()
			tm.disable_motor(1,sim)
		if open_draw_flag==1:
			open_draw_flag=0
			plt.figure(figsize=(10,5))
			plt.plot(np.array(xx),np.array(yy),color="deeppink",linewidth=2,linestyle=':',label='DPos', marker='o')
			plt.plot(np.array(xx),np.array(realp),color="darkblue",linewidth=1,linestyle='--',label='RPos', marker='+')
			plt.plot(np.array(xx),np.array(realv),color="goldenrod",linewidth=1.5,linestyle='-',label='RVelocity', marker='*')
			plt.plot(np.array(xx),np.array(desirev),color="yellow",linewidth=1.5,linestyle='-',label='DVelocity', marker='x')
			plt.legend(loc=2,labels=['DPos','RPos','RVelocity','DVelocity'])
			plt.show()
			tm.disable_motor(1,sim)
	except KeyboardInterrupt:
		tm.disable_motor(1,sim)
		if sim!=0:
			f.close()
	tm.disable_motor(1,sim)
	
if __name__=='__main__':
	main()

	
	