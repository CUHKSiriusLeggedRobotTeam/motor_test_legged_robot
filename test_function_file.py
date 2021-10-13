import numpy as np
def getSideSign(leg):
    sideSigns = [-1, 1, -1, 1]
    return sideSigns[leg]
def Calculate_Jocobian_for_leg_robot(q,leg,J_flag,P_flag):

    l1 = 0.079
    l2 = 0.3
    l3 = 0.3
    l4 = 0

    sideSign = getSideSign(leg);

    s1 = np.sin(q[0]);
    s2 = np.sin(q[1]);
    s3 = np.sin(q[2]);

    c1 = np.cos(q[0]);
    c2 = np.cos(q[1]);
    c3 = np.cos(q[2]);

    c23 = c2 * c3 - s2 * s3;
    s23 = s2 * c3 + c2 * s3;
    
    J=np.zeros( (3,3) )
    if J_flag:
        J[0, 0] = 0;
        J[0, 1] = l3 * c23 + l2 * c2;
        J[0, 2] = l3 * c23;
        J[1, 0] = l3 * c1 * c23 + l2 * c1 * c2 - (l1+l4) * sideSign * s1;
        J[1, 1] = -l3 * s1 * s23 - l2 * s1 * s2;
        J[1, 2] = -l3 * s1 * s23;
        J[2, 0] = l3 * s1 * c23 + l2 * c2 * s1 + (l1+l4) * sideSign * c1;
        J[2, 1] = l3 * c1 * s23 + l2 * c1 * s2;
        J[2, 2] = l3 * c1 * s23;
        return J
    
    p=[0,0,0]
    if P_flag==1:
        p[0] = l3 * s23 + l2 * s2;
        p[1] = (l1+l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
        p[2] = (l1+l4) * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;
        return p
def VMC_Control(q_start,q_desire,qdot_desire,qdot_real,kp_virtual_spring,kd_virtual_spring):
    """
        Note:q_start is q_real
    """
    #PD control
    Fx=kp_virtual_spring*(q_desire[0]-q_start[0])+kd_virtual_spring*(qdot_desire[0]-qdot_real[0])
    Fy=kp_virtual_spring*(q_desire[1]-q_start[1])+kd_virtual_spring*(qdot_desire[1]-qdot_real[1])
    Fz=kp_virtual_spring*(q_desire[2]-q_start[2])+kd_virtual_spring*(qdot_desire[2]-qdot_real[2])
    F=np.array([Fx,Fy,Fz])
    print(F,F.T)
    J=Calculate_Jocobian_for_leg_robot(q_desire,0,1,0)
    print(J)
    Torque=np.dot(J.T,F.T)
    return Torque
def main():
    q_start=[0,0,0]
    q_desire=[0,1,1]
    qdot_desire=[0.1,0.1,0.1]
    qdot_real=[0.2,0.2,0.2]
    kp_virtual_spring=5.2
    kd_virtual_spring=0.1
    print(VMC_Control(q_start,q_desire,qdot_desire,qdot_real,kp_virtual_spring,kd_virtual_spring))

if __name__ == '__main__':
    main()
        