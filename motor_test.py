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
	def init_motor_com(self):
		self.mmc.disable_motor(self.can_id)
		self.mmc.enable_motor(self.can_id)							# Enable motor with CAN ID 1
		# Send a command:  
		# (CAN ID, position setpoint, velocity setpoint, position gain, velocity gain, feed-forward torque)
		# Units are:  Radians, Rad/s, N-m/rad, N-m/rad/s, N-m
		self.mmc.send_command(self.can_id, 0, 0, 0, 0,  0)
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
		print(real_position)
		print("real_position:[ "+str(real_position[0])+" ]real_Velocity[ "+str(real_Velocity[0])+" ]real_Current[ "+str(real_Current[0])+" ]")
		print("Pos_Error"+str(p_des-real_position[0])+"Ves_Error"+str(abs(v_des)-abs(real_Velocity[0]))+"Current_Error"+str(abs(t_des)-abs(real_Current[0])))
		#p_des = start_position[0]
		error=[p_des-real_position[0],v_des-real_Velocity[0],t_des-real_Current[0]]
	
		print("===== ",real)
		return error,[real_position,real_Velocity,real_Current]
	def disable_motor(self): 
		self.mmc.disable_motor(self.can_id) # Disable motor with CAN ID 1
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
		l1=0.3
		l2=0.3
		theat0=0
		theta1=-np.arccos((l1*l1+height*height-l2*l2)/(2*l1*height))+np.arctan(x/y)
		theta2=np.pi-np.arccos((l1*l1-height*height+l2*l2)/(2*l1*l2))
		return [theat0,theta1,theta2]
	def calculate_circle_for_foot(self,circle_center_co=(0., -0.4),radius=0.1):
		#-radius<x<radius
		r = radius
		a, b = circle_center_co
		theta = np.arange(0, 2*np.pi, 0.5) #not max 1.5
		x = a + r * np.cos(theta)
		y = b + r * np.sin(theta)
		height=[]
		anguar=[]
		for i in range(len(x)):
			height.append(np.sqrt(x[i]*x[i]+y[i]*y[i]))
			anguar.append(self.calculate_joint_angular(np.sqrt(x[i]*x[i]+y[i]*y[i]),x[i],y[i]))

		return anguar
	def cosine_interpolation(self,y1,y2,x):
		if x[0]<0.0 and x[1]<0.0 and x[2]<0.0:
			x=[0,0,0]
		if x[1]>1.0 and x[0]>1.0 and x[2]>1.0:
			x=[1.0,1.0,1.0]
		res = [0,0,0]
		mu2 = [0,0,0]
		mu2[0] = (1-np.cos(x[0] * np.pi))/2
		mu2[1] = (1-np.cos(x[1] * np.pi))/2
		mu2[2] = (1-np.cos(x[2] * np.pi))/2
		res[0] = y1[0] * (1 - mu2[0]) + y2[0] * mu2[0]
		res[1] = y1[1] * (1 - mu2[1]) + y2[1] * mu2[1]
		res[2] = y1[2] * (1 - mu2[2]) + y2[2] * mu2[2]
		return res




def main():
	can_id=2
	stand_time=3
	COM="COM10"
	tm=TestModule(can_id,stand_time,COM)
	tm.init_motor_com()
	flag_1=1
	if flag_1==1:
		p_start=[0,0,0]
		p_des=[3,3,3]
	elif flag_1==0:
		p_start=[3,3,3]
		p_des=[0,0,0]
	v_des=0
	t_des=0
	kp=100#200	#450  #150 #5
	kd=2.4#5 #25  #0.005 	
	# kp=5
	# kd=0.1
	t_p_e=0
	error=[]
	dt=0.01
	cnt=0
	# flag=1
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
	# pdes_circle=tm.calculate_circle_for_foot()
	# tm.mmc.send_command(can_id, 0, 0, 0,0, 0)
	# error,realdata=tm.Read_Reply_Data(can_id,0 , 0,0)
	# print("-------->",realdata)
	# if abs(realdata[0][0])>0.1:
	# 	open_go_zero_flag=1
	# 	realdata_initial=realdata[0][0]
	# if open_go_zero_flag:
	# 	while(1):
	# 			pres=tm.cubicBezier([realdata_initial,realdata_initial,realdata_initial],[0,0,0],[0.01,0.01,0.01])
	# 			tm.mmc.send_command(can_id, pres[2] ,  0, kp, kd, t_des)
	# 			error,realdata=tm.Read_Reply_Data(can_id,pres[2] , pdres[0],t_des)
	# 			print("Desire",pres,pdres,realdata[0][0],t_p_e,count)
	# 			if t_p_e<0.05:
	# 				print("go to zero ok")
	# 				break
	
	# print(pdes_circle)
	# count=0
	try:
		while(1):
			starttime=time.time()
			if flag==1:
				if cnt>3:
					
					# pres=tm.cubicBezier(p_start,pdes_circle[count],[detat,detat,detat])
					# pres=tm.cubicBezier(p_start,p_des,[detat,detat,detat])
					# detat = np.arange(0,1,0.01)
					pres=tm.cosine_interpolation(p_start,p_des,[detat,detat,detat])
					# pdres=tm.cubicBezierFirstDerivative(p_start,p_des,[detat,detat,detat])
					tm.mmc.send_command(can_id, pres[1] , 0, kp, kd, t_des)
					error,realdata=tm.Read_Reply_Data(can_id,pres[1] , 0,t_des)
					print("-----",realdata)
					xx.append(detat)
					yy.append(pres)
					desirev.append(pdres)
					realp.append(realdata[0])
					realv.append(realdata[1])
					
					t_p_e=abs(p_des[0]-realdata[0][0])
					print("Desire",pres,pdres,realdata[0],t_p_e,count)
					# count+=1
					# if count>=len(pdes_circle):
					# 	count=0
						# print("ok---- t_p_e",t_p_e)
						# open_draw_flag=1
						# break
					# if count>30:
					# 	p_start=pdes_circle[count-10]
					if t_p_e<0.05:
						print("ok---- t_p_e",t_p_e)
						open_draw_flag=1
						break
				else:
					tm.mmc.send_command(can_id, 0 ,  0, 0,0, 0)
			elif flag==3:
				pres=tm.cubicBezier(p_start,p_des,[detat,detat,detat])
				# pdres=tm.cubicBezierFirstDerivative(p_start,p_des,[detat,detat,detat])
				
				print(pres,pdres)
				if cnt>3:
					tm.mmc.send_command(can_id, pres[0] ,  pdres[0], kp, kd, t_des)
					error,realdata=tm.Read_Reply_Data(can_id,pres[0] , pdres[0],t_des)
					
					xx.append(detat)
					yy.append(pres)
					desirev.append(pdres)
					realp.append(realdata[0])
					realv.append(realdata[1])
					
					t_p_e=abs(p_des[0]-realdata[0][0])
					print("Desire",pres,pdres,realdata[0][0],t_p_e)
					if t_p_e<0.05:
						print("ok---- t_p_e",t_p_e)
						open_draw_flag=1
						break

			elif flag==2:
				p_des_=0
				v_des_=0#200RPM/ 0.5-42	
				t_des=5
				kp_joint=30
				kd_joint=0.3
				resdata=[0,0,0,0]
				
				tm.mmc.send_command(can_id, p_des_ , v_des_, kp, kd, t_des)
				error,realdata=tm.Read_Reply_Data(can_id,p_des_,v_des_,t_des)
				t_des_es=t_des+kp_joint*(p_des_-realdata[0][0])+kd_joint*(v_des_-realdata[1][0])
				resdata[0]=str(datetime.datetime.now().day)+str("-")+str(datetime.datetime.now().hour)+str("-")+str(datetime.datetime.now().minute)+str("-")+str(datetime.datetime.now().second)
				resdata[1]=realdata[0][0]
				resdata[2]=realdata[1][0]
				resdata[3]=realdata[2][0]
				csv_writer.writerow(resdata)
				xx_v.append(detat)
				yy_v.append(v_des_)
				zz_v.append(realdata[1])
				if cnt>100:
					open_draw_flag_v=1
					break
				t_p_e=abs(error[0])

			

				
			#endtime=time.time()
			#detat=endtime-start
			detat=cnt*dt
			cnt+=1
			print("----------",detat)
			time.sleep(.05)
			endtime=time.time()
			print("loop time",endtime-starttime)
		if open_draw_flag_v==1:
			open_draw_flag_v=0
			plt.figure(figsize=(10,5))
			plt.plot(np.array(xx_v),np.array(yy_v),color="deeppink",linewidth=2,linestyle=':',label='Vd', marker='o')
			plt.plot(np.array(xx_v),np.array(zz_v),color="goldenrod",linewidth=2,linestyle=':',label='Vr', marker='o')
			plt.show()
			tm.disable_motor()
		if open_draw_flag==1:
			open_draw_flag=0
			plt.figure(figsize=(10,5))#figsize用来设置图形的大小，10为图形的宽，5为图形的高，单位为英寸（1英寸=2.5cm）
			# plt.plot(np.array(xx),np.array(yy),color="deeppink",linewidth=2, marker='o')
			# plt.plot(np.array(xx),np.array(realp),color="darkblue",linewidth=1, marker='+')
			# plt.plot(np.array(xx),np.array(realv),color="goldenrod",linewidth=1.5, marker='*')
			# plt.plot(np.array(xx),np.array(desirev),color="yellow",linewidth=1.5,marker='x')
			plt.plot(np.array(xx),np.array(yy),linewidth=2,marker='o',label="desired_position")
			plt.plot(np.array(xx),np.array(realp),linewidth=1, marker='+',label = "real_position")
			plt.plot(np.array(xx),np.array(realv),linewidth=1.5, marker='*',label = "real_velocity")
			plt.plot(np.array(xx),np.array(desirev),linewidth=1.5,marker='x',label = "desired_velocity")
			plt.legend(loc = 'upper right')
			plt.xlabel("x axis --  detat")
			plt.ylabel("y axis -- value")
			plt.title("comparison")
			# plt.legend()
			plt.show()
			tm.disable_motor()
	except KeyboardInterrupt:
		tm.disable_motor()
		f.close()
	tm.disable_motor()
	
if __name__=='__main__':
	main()

	
	