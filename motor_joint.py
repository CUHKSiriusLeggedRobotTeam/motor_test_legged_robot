'''
Motor Module example program.
'''

from os import closerange
import motormodule as mm
import time
import matplotlib.pyplot as plt
import numpy as np
import csv
import datetime

class TestModule:
	def __init__(self,can_id,stand_time,COM):
		self.COM=COM
		self.can_id=can_id	
		self.stand_time=stand_time
		self.mmc = mm.MotorModuleController(self.COM)		# Connect to the controller's serial port
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
	def disable_motor(self,flag):
		if flag:
			self.mmc.disable_motor(self.can_id[0]) # Disable motor with CAN ID 1
			self.mmc.disable_motor(self.can_id[1]) # Disable motor with CAN ID 1
			self.mmc.disable_motor(self.can_id[2]) # Disable motor with CAN ID 1
		else:
			self.mmc.disable_motor(self.can_id[1]) # Disable motor with CAN ID 1
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
			# height.append(np.sqrt(x[i]*x[i]+y[i]*y[i]))
			# if xx[i]!=0:
			# 	x=xx[i]
			# 	y=yy[i]
			# 	theat0=0
			# 	thetap=np.arctan(x/y)
			# 	theat1=np.arccos(np.cos(thetap)*(x*x+y*y+l1*l1-l2*l2)/(2*l1*x))+np.arctan(y/x)
			# 	theat2=np.arccos(np.cos(thetap)*(x*x+y*y-l1*l1+l2*l2)/(2*l2*x))+np.arctan(y/x)-theat1
			# 	anguar.append([theat0,theat1,theat2])
			anguar.append(self.calculate_joint_angular(np.sqrt(x[i]*x[i]+y[i]*y[i]),x[i],y[i]))

		return anguar


def main():
	can_id=[1,2,3]
	stand_time=3
	COM="COM10"
	tm=TestModule(can_id,stand_time,COM)
	tm.init_motor_com(1)
	v_des=0
	t_des=0
	#big dog
	kp=35#200	#450  #150 #5
	kd=2.4#5 #25  #0.005 	
	#mini dog
	# kp=35#200	#450  #150 #5
	# kd=1.4#5 #25  #0.005 	
	kp_joint=55
	kd_joint=1.3
	t_p_e=0
	error=[]
	dt=0.05
	cnt=0
	count=0
	detat=0.01
	
	flag=1
	#dt=0.01
	xx=[]#t
	yy=[]#pos
	realp=[]
	realv=[]
	desirev=[]
	open_draw_flag=0
	xx_v=[]
	yy_v=[]
	zz_v=[]
	pdres=[0]
	open_draw_flag_v=0
	f = open('m41_0.csv','w',encoding='utf-8',newline='')
	csv_writer = csv.writer(f)
	csv_writer.writerow(["Time","P","V","C"])
	t_des_es=0
	pdes_circle=tm.calculate_circle_for_foot()
	p_start=[0,0,0]
	insert_count=0
	try:
		while(1):
			starttime=time.time()
			if flag==1:
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
			elif flag==2:
				if cnt>3:
					# pres=tm.cubicBezier(p_start,pdes_circle[count],[detat,detat,detat])
					pres=tm.cubicBezier([1,1,1],[2,2,2],[detat,detat,detat])
					tm.mmc.send_command(can_id[1], pres[2] ,  pdres[0], kp, kd, t_des_es)
					time.sleep(0.001)
					# p_des_=1
					v_des_=0#200RPM/ 0.5-42	
					t_des=15
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
			# detat = cnt * dt
			cnt+=1
			# detat=cnt*detat
		if open_draw_flag_v==1:
			open_draw_flag_v=0
			plt.figure(figsize=(10,5))
			plt.plot(np.array(xx_v),np.array(yy_v),color="deeppink",linewidth=2,linestyle=':',label='Vd', marker='o')
			plt.plot(np.array(xx_v),np.array(zz_v),color="goldenrod",linewidth=2,linestyle=':',label='Vr', marker='o')
			plt.show()
			tm.disable_motor(1)
		if open_draw_flag==1:
			open_draw_flag=0
			plt.figure(figsize=(10,5))#figsize用来设置图形的大小，10为图形的宽，5为图形的高，单位为英寸（1英寸=2.5cm）
			plt.plot(np.array(xx),np.array(yy),color="deeppink",linewidth=2,linestyle=':',label='DPos', marker='o')
			plt.plot(np.array(xx),np.array(realp),color="darkblue",linewidth=1,linestyle='--',label='RPos', marker='+')
			plt.plot(np.array(xx),np.array(realv),color="goldenrod",linewidth=1.5,linestyle='-',label='RVelocity', marker='*')
			plt.plot(np.array(xx),np.array(desirev),color="yellow",linewidth=1.5,linestyle='-',label='DVelocity', marker='x')
			plt.legend(loc=2,labels=['DPos','RPos','RVelocity','DVelocity'])
			plt.show()
			tm.disable_motor(1)
	except KeyboardInterrupt:
		tm.disable_motor(1)
		f.close()
	tm.disable_motor(1)
	
if __name__=='__main__':
	main()

	
	