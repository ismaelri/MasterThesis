#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64MultiArray
from my_ur10_msgs.msg import *
import matplotlib.pyplot as plt
import sys
import numpy as np


def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + 'I heard '+ error)

    global error0
    error0.append(data.e[0])
    global error1
    error1.append(data.e[1])
    global error2
    error2.append(data.e[2])
    global error3
    error3.append(data.e[3])
    global error4
    error4.append(data.e[4])
    global error5
    error5.append(data.e[5])

    global position0
    position0.append(data.p[0])
    global position1
    position1.append(data.p[1])
    global position2
    position2.append(data.p[2])
    global position3
    position3.append(data.p[3])
    global position4
    position4.append(data.p[4])
    global position5
    position5.append(data.p[5])

    global velocity0
    velocity0.append(data.v[0])
    global velocity1
    velocity1.append(data.v[1])
    global velocity2
    velocity2.append(data.v[2])
    global velocity3
    velocity3.append(data.v[3])
    global velocity4
    velocity4.append(data.v[4])
    global velocity5
    velocity5.append(data.v[5])

    global velocity0_e
    velocity0_e.append(data.v_e[0])
    global velocity1_e
    velocity1_e.append(data.v_e[1])
    global velocity2_e
    velocity2_e.append(data.v_e[2])
    global velocity3_e
    velocity3_e.append(data.v_e[3])
    global velocity4_e
    velocity4_e.append(data.v_e[4])
    global velocity5_e
    velocity5_e.append(data.v_e[5])

    global joints0
    joints0.append(data.j[0])
    global joints1
    joints1.append(data.j[1])
    global joints2
    joints2.append(data.j[2])
    global joints3
    joints3.append(data.j[3])
    global joints4
    joints4.append(data.j[4])
    global joints5
    joints5.append(data.j[5])

    global time
    time.append(data.t)

    global name 
    global name_pdf
    global name_png

    global mov
    mov = np.append(mov, [[data.m[0]],[data.m[1]],[data.m[2]],[data.m[3]],[data.m[4]],[data.m[5]],[data.m[6]],[data.m[7]]], axis = 1)




    #rospy.loginfo( "The errors %f , %f , %f , %f , %f , %f", e[0], e[1], e[2], e[3], e[4], e[5])
    #plt.ion()
    if data.finish:
        
        plt.figure(1)
        plt.title("End-effector error")
        plt.plot(time, error0, marker=',', c='red', label='x_e')
        plt.plot(time, error1, marker=',', c='blue', label='y_e')
        plt.plot(time, error2, marker=',', c='green', label='z_e')
        plt.plot(time, error3, marker=',', c='silver', label='roll_e')
        plt.plot(time, error4, marker=',', c='lightgreen', label='pitch_e')
        plt.plot(time, error5, marker=',', c='fuchsia', label='yaw_e')
        
        plt.ylabel("End-effector error, m and rad")
        plt.xlabel("Time, s")
        plt.legend(loc="best")
        plt.savefig(name + "_error_end_effector.eps" , format='eps')
        plt.savefig(name_pdf + "_error_end_effector.pdf" , format='pdf')
        plt.savefig(name_png + "_error_end_effector.png" , format='png')
        plt.show()
        plt.pause(0.1)
        #plt.draw()

        plt.figure(2)
        plt.title("End-effector position")
        #plt.plot(time, position0, marker='_', c='yellow')
        plt.plot(time, position0, marker=',', c='red', label='x')
        plt.plot(time, position1, marker=',', c='blue', label='y')
        plt.plot(time, position2, marker=',', c='green', label='z')
        plt.plot(time, position3, marker=',', c='silver', label='roll')
        plt.plot(time, position4, marker=',', c='lightgreen', label='pitch')
        plt.plot(time, position5, marker=',', c='fuchsia', label='yaw')
        plt.ylabel("End-effector position, m and rad")
        plt.xlabel("Time, s")
        plt.legend(loc="best")
        plt.savefig(name + "_position_end_effector.eps" , format='eps')
        plt.savefig(name_pdf + "_position_end_effector.pdf" , format='pdf')
        plt.savefig(name_png + "_position_end_effector.png" , format='png')
        plt.show()
        plt.pause(0.1)
        #plt.draw()
        
        plt.figure(3)
        plt.title("Joint velocities")
        #plt.plot(time,velocity0, marker='_', c='yellow')
        plt.plot(time,velocity0, marker=',', c='red', label='v. j0')
        plt.plot(time,velocity1, marker=',', c='blue', label='v. j1')
        plt.plot(time,velocity2, marker=',', c='green', label='v. j2')
        plt.plot(time,velocity3, marker=',', c='silver', label='v. j3')
        plt.plot(time,velocity4, marker=',', c='lightgreen', label='v. j4')
        plt.plot(time,velocity5, marker=',', c='fuchsia', label='v. j5')
        plt.ylabel("Joint velocities, rad/s")
        plt.xlabel("Time, s")
        plt.legend(loc="best")
        plt.savefig(name + "_joint_velocities.eps" , format='eps')
        plt.savefig(name_pdf + "_joint_velocities.pdf" , format='pdf')
        plt.savefig(name_png + "_joint_velocities.png" , format='png')
        plt.show()
        plt.pause(0.1)
        #plt.draw()

        plt.figure(4)
        plt.title("Joint positions")
        #plt.plot(time,joints0, marker='_', c='yellow')
        plt.plot(time,joints0, marker=',', c='red', label='p. j0')
        plt.plot(time,joints1, marker=',', c='blue', label='p. j1')
        plt.plot(time,joints2, marker=',', c='green', label='p. j2')
        plt.plot(time,joints3, marker=',', c='silver', label='p. j3')
        plt.plot(time,joints4, marker=',', c='lightgreen', label='p. j4')
        plt.plot(time,joints5, marker=',', c='fuchsia', label='p. j5')
        plt.ylabel("Joint positions, rad")
        plt.xlabel("Time, s")
        plt.legend(loc="best")
        plt.savefig(name + "_joint_positions.eps" , format='eps')
        plt.savefig(name_pdf + "_joint_positions.pdf" , format='pdf')
        plt.savefig(name_png + "_joint_positions.png" , format='png')
        plt.show()
        plt.pause(0.1)
        #plt.draw()

        plt.figure(5)
        plt.title("End-effector velocities")
        #plt.plot(time,joints0, marker='_', c='yellow')
        plt.plot(time,velocity0_e, marker=',', c='red', label='v. x')
        plt.plot(time,velocity1_e, marker=',', c='blue', label='v. y')
        plt.plot(time,velocity2_e, marker=',', c='green', label='v. z')
        plt.plot(time,velocity3_e, marker=',', c='silver', label='v. roll')
        plt.plot(time,velocity4_e, marker=',', c='lightgreen', label='v. pitch')
        plt.plot(time,velocity5_e, marker=',', c='fuchsia', label='v. yaw')
        plt.ylabel("End-effector velocities, m/s and rad/s")
        plt.xlabel("Time, s")
        plt.legend(loc="best")
        plt.savefig(name + "_end_effector_velocities.eps" , format='eps')
        plt.savefig(name_pdf + "_end_effector_velocities.pdf" , format='pdf')
        plt.savefig(name_png + "_end_effector_velocities.png" , format='png')
        plt.show()
        plt.pause(0.1)
        #plt.draw(

        
        plt.figure(6)
        plt.title("Movement of the ArUco marker in the image")
        #plt.plot(time,joints0, marker='_', c='yellow')
        plt.plot(mov[0,1:],mov[1,1:], marker='.', c='red', label='c0')
        plt.plot(mov[0,1],mov[1,1],marker='^', c='red')
        plt.plot(mov[0,-1],mov[1,-1],marker='o', c='red')
        plt.plot(mov[2,1:],mov[3,1:], marker='.', c='blue', label='c1')
        plt.plot(mov[2,1],mov[3,1],marker='^', c='blue')
        plt.plot(mov[2,-1],mov[3,-1],marker='o', c='blue')
        plt.plot(mov[4,1:],mov[5,1:], marker='.', c='green', label='c2')
        plt.plot(mov[4,1],mov[5,1],marker='^', c='green')
        plt.plot(mov[4,-1],mov[5,-1],marker='o', c='green')
        plt.plot(mov[6,1:],mov[7,1:], marker='.', c='silver', label='c3')
        plt.plot(mov[6,1],mov[7,1],marker='^', c='silver')
        plt.plot(mov[6,-1],mov[7,-1],marker='o', c='silver')

        plt.ylim((data.i_dim[0],0))
        plt.xlim((0,data.i_dim[1]))

        plt.ylabel("v, pixels")
        plt.xlabel("u, pixels")
        plt.legend(loc="best")
        plt.savefig(name + "_movement.eps" , format='eps')
        plt.savefig(name_pdf + "_movement.pdf" , format='pdf')
        plt.savefig(name_png + "_movement.png" , format='png')
        plt.show()
        plt.pause(0.1)
        #plt.draw()
        

######################################################################################
######################################################################################

def callback_IBVC(data):
    # rospy.loginfo(rospy.get_caller_id() + 'I heard '+ error)
    
    global error0_
    global error1_
    global error2_
    global error3_
    global error4_
    global error5_
    global mov0
    global mov1
    global mov2
    global mov3
    global mov4
    global mov5
    
    if data.m_info[0].id != 0:
        error0_ = np.append(error0_, [[data.m_info[0].errors[0]], [data.t], [data.m_info[0].errors[1]], [data.t], [data.m_info[0].errors[2]], [data.t], [data.m_info[0].errors[3]], [data.t], [data.m_info[0].errors[4]], [data.t], [data.m_info[0].errors[5]], [data.t], [data.m_info[0].errors[6]], [data.t], [data.m_info[0].errors[7]], [data.t]], axis = 1)
        mov0 = np.append(mov0, [[data.m_info[0].point_movements[0]], [data.m_info[0].point_movements[1]], [data.m_info[0].point_movements[2]], [data.m_info[0].point_movements[3]], [data.m_info[0].point_movements[4]], [data.m_info[0].point_movements[5]], [data.m_info[0].point_movements[6]], [data.m_info[0].point_movements[7]]], axis = 1)

    if data.m_info[1].id != 0:
        error1_ = np.append(error1_, [[data.m_info[1].errors[0]], [data.t], [data.m_info[1].errors[1]], [data.t], [data.m_info[1].errors[2]], [data.t], [data.m_info[1].errors[3]], [data.t], [data.m_info[1].errors[4]], [data.t], [data.m_info[1].errors[5]], [data.t], [data.m_info[1].errors[6]], [data.t], [data.m_info[1].errors[7]], [data.t] ], axis = 1)
        mov1 = np.append(mov1, [[data.m_info[1].point_movements[0]], [data.m_info[1].point_movements[1]], [data.m_info[1].point_movements[2]], [data.m_info[1].point_movements[3]], [data.m_info[1].point_movements[4]], [data.m_info[1].point_movements[5]], [data.m_info[1].point_movements[6]], [data.m_info[1].point_movements[7]]], axis = 1)

    if data.m_info[2].id != 0:
        error2_ = np.append(error2_, [[data.m_info[2].errors[0]], [data.t], [data.m_info[2].errors[1]], [data.t], [data.m_info[2].errors[2]], [data.t], [data.m_info[2].errors[3]], [data.t], [data.m_info[2].errors[4]], [data.t], [data.m_info[2].errors[5]], [data.t], [data.m_info[2].errors[6]], [data.t], [data.m_info[2].errors[7]], [data.t] ], axis = 1)
        mov2 = np.append(mov2, [[data.m_info[2].point_movements[0]], [data.m_info[2].point_movements[1]], [data.m_info[2].point_movements[2]], [data.m_info[2].point_movements[3]], [data.m_info[2].point_movements[4]], [data.m_info[2].point_movements[5]], [data.m_info[2].point_movements[6]], [data.m_info[2].point_movements[7]]], axis = 1)

    if data.m_info[3].id != 0:
        error3_ = np.append(error3_, [[data.m_info[3].errors[0]], [data.t], [data.m_info[3].errors[1]], [data.t], [data.m_info[3].errors[2]], [data.t], [data.m_info[3].errors[3]], [data.t], [data.m_info[3].errors[4]], [data.t], [data.m_info[3].errors[5]], [data.t], [data.m_info[3].errors[6]], [data.t], [data.m_info[3].errors[7]], [data.t] ], axis = 1)
        mov3 = np.append(mov3, [[data.m_info[3].point_movements[0]], [data.m_info[3].point_movements[1]], [data.m_info[3].point_movements[2]], [data.m_info[3].point_movements[3]], [data.m_info[3].point_movements[4]], [data.m_info[3].point_movements[5]], [data.m_info[3].point_movements[6]], [data.m_info[3].point_movements[7]]], axis = 1)

    if data.m_info[4].id != 0:
        error4_ = np.append(error4_, [[data.m_info[4].errors[0]], [data.t], [data.m_info[4].errors[1]], [data.t], [data.m_info[4].errors[2]], [data.t], [data.m_info[4].errors[3]], [data.t], [data.m_info[4].errors[4]], [data.t], [data.m_info[4].errors[5]], [data.t], [data.m_info[4].errors[6]], [data.t], [data.m_info[4].errors[7]], [data.t] ], axis = 1)
        mov4 = np.append(mov4, [[data.m_info[4].point_movements[0]], [data.m_info[4].point_movements[1]], [data.m_info[4].point_movements[2]], [data.m_info[4].point_movements[3]], [data.m_info[4].point_movements[4]], [data.m_info[4].point_movements[5]], [data.m_info[4].point_movements[6]], [data.m_info[4].point_movements[7]]], axis = 1)

    if data.m_info[5].id != 0:
        error5_ = np.append(error5_, [[data.m_info[5].errors[0]], [data.t], [data.m_info[5].errors[1]], [data.t], [data.m_info[5].errors[2]], [data.t], [data.m_info[5].errors[3]], [data.t], [data.m_info[5].errors[4]], [data.t], [data.m_info[5].errors[5]], [data.t], [data.m_info[5].errors[6]], [data.t], [data.m_info[5].errors[7]], [data.t] ], axis = 1)
        mov5 = np.append(mov5, [[data.m_info[5].point_movements[0]], [data.m_info[5].point_movements[1]], [data.m_info[5].point_movements[2]], [data.m_info[5].point_movements[3]], [data.m_info[5].point_movements[4]], [data.m_info[5].point_movements[5]], [data.m_info[5].point_movements[6]], [data.m_info[5].point_movements[7]]], axis = 1)

    
    global position0
    position0.append(data.p[0])
    global position1
    position1.append(data.p[1])
    global position2
    position2.append(data.p[2])
    global position3
    position3.append(data.p[3])
    global position4
    position4.append(data.p[4])
    global position5
    position5.append(data.p[5])

    global velocity0
    velocity0.append(data.v[0])
    global velocity1
    velocity1.append(data.v[1])
    global velocity2
    velocity2.append(data.v[2])
    global velocity3
    velocity3.append(data.v[3])
    global velocity4
    velocity4.append(data.v[4])
    global velocity5
    velocity5.append(data.v[5])

    global velocity0_e
    velocity0_e.append(data.v_c[0])
    global velocity1_e
    velocity1_e.append(data.v_c[1])
    global velocity2_e
    velocity2_e.append(data.v_c[2])
    global velocity3_e
    velocity3_e.append(data.v_c[3])
    global velocity4_e
    velocity4_e.append(data.v_c[4])
    global velocity5_e
    velocity5_e.append(data.v_c[5])

    global joints0
    joints0.append(data.j[0])
    global joints1
    joints1.append(data.j[1])
    global joints2
    joints2.append(data.j[2])
    global joints3
    joints3.append(data.j[3])
    global joints4
    joints4.append(data.j[4])
    global joints5
    joints5.append(data.j[5])

    global time
    time.append(data.t)

    global name 
    global name_pdf
    global name_png

    #global mov
    #mov = np.append(mov, [[data.m[0]],[data.m[1]],[data.m[2]],[data.m[3]],[data.m[4]],[data.m[5]],[data.m[6]],[data.m[7]]], axis = 1)




    #rospy.loginfo( "The errors %f , %f , %f , %f , %f , %f", e[0], e[1], e[2], e[3], e[4], e[5])
    #plt.ion()
    if data.finish:
        
        plt.figure(1)
        plt.title("x,y error")

        plt.plot(error0_[1,1:], error0_[0,1:], marker=',', c='red', label='A 0')
        plt.plot(error0_[3,1:], error0_[2,1:], marker=',', c='red')
        plt.plot(error0_[5,1:], error0_[4,1:], marker=',', c='red')
        plt.plot(error0_[7,1:], error0_[6,1:], marker=',', c='red')
        plt.plot(error0_[9,1:], error0_[8,1:], marker=',', c='red')
        plt.plot(error0_[11,1:], error0_[10,1:], marker=',', c='red')
        plt.plot(error0_[13,1:], error0_[12,1:], marker=',', c='red')
        plt.plot(error0_[15,1:], error0_[14,1:], marker=',', c='red')

        plt.plot(error1_[1,1:], error1_[0,1:], marker=',', c='blue', label='A 1')
        plt.plot(error1_[3,1:], error1_[2,1:], marker=',', c='blue')
        plt.plot(error1_[5,1:], error1_[4,1:], marker=',', c='blue')
        plt.plot(error1_[7,1:], error1_[6,1:], marker=',', c='blue')
        plt.plot(error1_[9,1:], error1_[8,1:], marker=',', c='blue')
        plt.plot(error1_[11,1:], error1_[10,1:], marker=',', c='blue')
        plt.plot(error1_[13,1:], error1_[12,1:], marker=',', c='blue')
        plt.plot(error1_[15,1:], error1_[14,1:], marker=',', c='blue')

        plt.plot(error2_[1,1:], error2_[0,1:], marker=',', c='green', label='A 2')
        plt.plot(error2_[3,1:], error2_[2,1:], marker=',', c='green')
        plt.plot(error2_[5,1:], error2_[4,1:], marker=',', c='green')
        plt.plot(error2_[7,1:], error2_[6,1:], marker=',', c='green')
        plt.plot(error2_[9,1:], error2_[8,1:], marker=',', c='green')
        plt.plot(error2_[11,1:], error2_[10,1:], marker=',', c='green')
        plt.plot(error2_[13,1:], error2_[12,1:], marker=',', c='green')
        plt.plot(error2_[15,1:], error2_[14,1:], marker=',', c='green')

        plt.plot(error3_[1,1:], error3_[0,1:], marker=',', c='silver', label='A 3')
        plt.plot(error3_[3,1:], error3_[2,1:], marker=',', c='silver')
        plt.plot(error3_[5,1:], error3_[4,1:], marker=',', c='silver')
        plt.plot(error3_[7,1:], error3_[6,1:], marker=',', c='silver')
        plt.plot(error3_[9,1:], error3_[8,1:], marker=',', c='silver')
        plt.plot(error3_[11,1:], error3_[10,1:], marker=',', c='silver')
        plt.plot(error3_[13,1:], error3_[12,1:], marker=',', c='silver')
        plt.plot(error3_[15,1:], error3_[14,1:], marker=',', c='silver')

        plt.plot(error4_[1,1:], error4_[0,1:], marker=',', c='lightgreen', label='A 4')
        plt.plot(error4_[3,1:], error4_[2,1:], marker=',', c='lightgreen')
        plt.plot(error4_[5,1:], error4_[4,1:], marker=',', c='lightgreen')
        plt.plot(error4_[7,1:], error4_[6,1:], marker=',', c='lightgreen')
        plt.plot(error4_[9,1:], error4_[8,1:], marker=',', c='lightgreen')
        plt.plot(error4_[11,1:], error4_[10,1:], marker=',', c='lightgreen')
        plt.plot(error4_[13,1:], error4_[12,1:], marker=',', c='lightgreen')
        plt.plot(error4_[15,1:], error4_[14,1:], marker=',', c='lightgreen')

        plt.plot(error5_[1,1:], error5_[0,1:], marker=',', c='fuchsia', label='A 5')
        plt.plot(error5_[3,1:], error5_[2,1:], marker=',', c='fuchsia')
        plt.plot(error5_[5,1:], error5_[4,1:], marker=',', c='fuchsia')
        plt.plot(error5_[7,1:], error5_[6,1:], marker=',', c='fuchsia')
        plt.plot(error5_[9,1:], error5_[8,1:], marker=',', c='fuchsia')
        plt.plot(error5_[11,1:], error5_[10,1:], marker=',', c='fuchsia')
        plt.plot(error5_[13,1:], error5_[12,1:], marker=',', c='fuchsia')
        plt.plot(error5_[15,1:], error5_[14,1:], marker=',', c='fuchsia')
        
        plt.ylabel("x,y error, m")
        plt.xlabel("Time, s")
        plt.legend(loc="best")
        plt.savefig(name + "_error_x_y.eps" , format='eps')
        plt.savefig(name_pdf + "_error_x_y.pdf" , format='pdf')
        plt.savefig(name_png + "_error_x_y.png" , format='png')
        plt.show()
        plt.pause(0.1)
        #plt.draw()

        plt.figure(2)
        plt.title("End-effector position")
        #plt.plot(time, position0, marker='_', c='yellow')
        plt.plot(time, position0, marker=',', c='red', label='x')
        plt.plot(time, position1, marker=',', c='blue', label='y')
        plt.plot(time, position2, marker=',', c='green', label='z')
        plt.plot(time, position3, marker=',', c='silver', label='roll')
        plt.plot(time, position4, marker=',', c='lightgreen', label='pitch')
        plt.plot(time, position5, marker=',', c='fuchsia', label='yaw')
        plt.ylabel("End-effector position, m and rad")
        plt.xlabel("Time, s")
        plt.legend(loc="best")
        plt.savefig(name + "_position_end_effector.eps" , format='eps')
        plt.savefig(name_pdf + "_position_end_effector.pdf" , format='pdf')
        plt.savefig(name_png + "_position_end_effector.png" , format='png')
        plt.show()
        plt.pause(0.1)
        #plt.draw()
        
        plt.figure(3)
        plt.title("Joint velocities")
        #plt.plot(time,velocity0, marker='_', c='yellow')
        plt.plot(time,velocity0, marker=',', c='red', label='v. j0')
        plt.plot(time,velocity1, marker=',', c='blue', label='v. j1')
        plt.plot(time,velocity2, marker=',', c='green', label='v. j2')
        plt.plot(time,velocity3, marker=',', c='silver', label='v. j3')
        plt.plot(time,velocity4, marker=',', c='lightgreen', label='v. j4')
        plt.plot(time,velocity5, marker=',', c='fuchsia', label='v. j5')
        plt.ylabel("Joint velocities, rad/s")
        plt.xlabel("Time, s")
        plt.legend(loc="best")
        plt.savefig(name + "_joint_velocities.eps" , format='eps')
        plt.savefig(name_pdf + "_joint_velocities.pdf" , format='pdf')
        plt.savefig(name_png + "_joint_velocities.png" , format='png')
        plt.show()
        plt.pause(0.1)
        #plt.draw()

        plt.figure(4)
        plt.title("Joint positions")
        #plt.plot(time,joints0, marker='_', c='yellow')
        plt.plot(time,joints0, marker=',', c='red', label='p. j0')
        plt.plot(time,joints1, marker=',', c='blue', label='p. j1')
        plt.plot(time,joints2, marker=',', c='green', label='p. j2')
        plt.plot(time,joints3, marker=',', c='silver', label='p. j3')
        plt.plot(time,joints4, marker=',', c='lightgreen', label='p. j4')
        plt.plot(time,joints5, marker=',', c='fuchsia', label='p. j5')
        plt.ylabel("Joint positions, rad")
        plt.xlabel("Time, s")
        plt.legend(loc="best")
        plt.savefig(name + "_joint_positions.eps" , format='eps')
        plt.savefig(name_pdf + "_joint_positions.pdf" , format='pdf')
        plt.savefig(name_png + "_joint_positions.png" , format='png')
        plt.show()
        plt.pause(0.1)
        #plt.draw()

        plt.figure(5)
        plt.title("Camera velocities")
        #plt.plot(time,joints0, marker='_', c='yellow')
        plt.plot(time,velocity0_e, marker=',', c='red', label='v. x')
        plt.plot(time,velocity1_e, marker=',', c='blue', label='v. y')
        plt.plot(time,velocity2_e, marker=',', c='green', label='v. z')
        plt.plot(time,velocity3_e, marker=',', c='silver', label='v. roll')
        plt.plot(time,velocity4_e, marker=',', c='lightgreen', label='v. pitch')
        plt.plot(time,velocity5_e, marker=',', c='fuchsia', label='v. yaw')
        plt.ylabel("Camera velocities, m/s and rad/s")
        plt.xlabel("Time, s")
        plt.legend(loc="best")
        plt.savefig(name + "_end_effector_velocities.eps" , format='eps')
        plt.savefig(name_pdf + "_end_effector_velocities.pdf" , format='pdf')
        plt.savefig(name_png + "_end_effector_velocities.png" , format='png')
        plt.show()
        plt.pause(0.1)
        #plt.draw(

        
        plt.figure(6)
        plt.title("Movement of the ArUco markers in the image")
        #plt.plot(time,joints0, marker='_', c='yellow')
        plt.plot(mov0[0,1:],mov0[1,1:], marker='.', c='red', label='A 0')
        plt.plot(mov0[0,1],mov0[1,1],marker='^', c='red')
        plt.plot(mov0[0,-1],mov0[1,-1],marker='o', c='red')
        plt.plot(mov0[2,1:],mov0[3,1:], marker='.', c='red')
        plt.plot(mov0[2,1],mov0[3,1],marker='^', c='red')
        plt.plot(mov0[2,-1],mov0[3,-1],marker='o', c='red')
        plt.plot(mov0[4,1:],mov0[5,1:], marker='.', c='red')
        plt.plot(mov0[4,1],mov0[5,1],marker='^', c='red')
        plt.plot(mov0[4,-1],mov0[5,-1],marker='o', c='red')
        plt.plot(mov0[6,1:],mov0[7,1:], marker='.', c='red')
        plt.plot(mov0[6,1],mov0[7,1],marker='^', c='red')
        plt.plot(mov0[6,-1],mov0[7,-1],marker='o', c='red')

        plt.plot(mov1[0,1:],mov1[1,1:], marker='.', c='blue', label='A 1')
        plt.plot(mov1[0,1],mov1[1,1],marker='^', c='blue')
        plt.plot(mov1[0,-1],mov1[1,-1],marker='o', c='blue')
        plt.plot(mov1[2,1:],mov1[3,1:], marker='.', c='blue')
        plt.plot(mov1[2,1],mov1[3,1],marker='^', c='blue')
        plt.plot(mov1[2,-1],mov1[3,-1],marker='o', c='blue')
        plt.plot(mov1[4,1:],mov1[5,1:], marker='.', c='blue')
        plt.plot(mov1[4,1],mov1[5,1],marker='^', c='blue')
        plt.plot(mov1[4,-1],mov1[5,-1],marker='o', c='blue')
        plt.plot(mov1[6,1:],mov1[7,1:], marker='.', c='blue')
        plt.plot(mov1[6,1],mov1[7,1],marker='^', c='blue')
        plt.plot(mov1[6,-1],mov1[7,-1],marker='o', c='blue')

        plt.plot(mov2[0,1:],mov2[1,1:], marker='.', c='green', label='A 2')
        plt.plot(mov2[0,1],mov2[1,1],marker='^', c='green')
        plt.plot(mov2[0,-1],mov2[1,-1],marker='o', c='green')
        plt.plot(mov2[2,1:],mov2[3,1:], marker='.', c='green')
        plt.plot(mov2[2,1],mov2[3,1],marker='^', c='green')
        plt.plot(mov2[2,-1],mov2[3,-1],marker='o', c='green')
        plt.plot(mov2[4,1:],mov2[5,1:], marker='.', c='green')
        plt.plot(mov2[4,1],mov2[5,1],marker='^', c='green')
        plt.plot(mov2[4,-1],mov2[5,-1],marker='o', c='green')
        plt.plot(mov2[6,1:],mov2[7,1:], marker='.', c='green')
        plt.plot(mov2[6,1],mov2[7,1],marker='^', c='green')
        plt.plot(mov2[6,-1],mov2[7,-1],marker='o', c='green')

        plt.plot(mov3[0,1:],mov3[1,1:], marker='.', c='silver', label='A 3')
        plt.plot(mov3[0,1],mov3[1,1],marker='^', c='silver')
        plt.plot(mov3[0,-1],mov3[1,-1],marker='o', c='silver')
        plt.plot(mov3[2,1:],mov3[3,1:], marker='.', c='silver')
        plt.plot(mov3[2,1],mov3[3,1],marker='^', c='silver')
        plt.plot(mov3[2,-1],mov3[3,-1],marker='o', c='silver')
        plt.plot(mov3[4,1:],mov3[5,1:], marker='.', c='silver')
        plt.plot(mov3[4,1],mov3[5,1],marker='^', c='silver')
        plt.plot(mov3[4,-1],mov3[5,-1],marker='o', c='silver')
        plt.plot(mov3[6,1:],mov3[7,1:], marker='.', c='silver')
        plt.plot(mov3[6,1],mov3[7,1],marker='^', c='silver')
        plt.plot(mov3[6,-1],mov3[7,-1],marker='o', c='silver')

        plt.plot(mov4[0,1:],mov4[1,1:], marker='.', c='lightgreen', label='A 4')
        plt.plot(mov4[0,1],mov4[1,1],marker='^', c='lightgreen')
        plt.plot(mov4[0,-1],mov4[1,-1],marker='o', c='lightgreen')
        plt.plot(mov4[2,1:],mov4[3,1:], marker='.', c='lightgreen')
        plt.plot(mov4[2,1],mov4[3,1],marker='^', c='lightgreen')
        plt.plot(mov4[2,-1],mov4[3,-1],marker='o', c='lightgreen')
        plt.plot(mov4[4,1:],mov4[5,1:], marker='.', c='lightgreen')
        plt.plot(mov4[4,1],mov4[5,1],marker='^', c='lightgreen')
        plt.plot(mov4[4,-1],mov4[5,-1],marker='o', c='lightgreen')
        plt.plot(mov4[6,1:],mov4[7,1:], marker='.', c='lightgreen')
        plt.plot(mov4[6,1],mov4[7,1],marker='^', c='lightgreen')
        plt.plot(mov4[6,-1],mov4[7,-1],marker='o', c='lightgreen')

        plt.plot(mov5[0,1:],mov5[1,1:], marker='.', c='fuchsia', label='A 5')
        plt.plot(mov5[0,1],mov5[1,1],marker='^', c='fuchsia')
        plt.plot(mov5[0,-1],mov5[1,-1],marker='o', c='fuchsia')
        plt.plot(mov5[2,1:],mov5[3,1:], marker='.', c='fuchsia')
        plt.plot(mov5[2,1],mov5[3,1],marker='^', c='fuchsia')
        plt.plot(mov5[2,-1],mov5[3,-1],marker='o', c='fuchsia')
        plt.plot(mov5[4,1:],mov5[5,1:], marker='.', c='fuchsia')
        plt.plot(mov5[4,1],mov5[5,1],marker='^', c='fuchsia')
        plt.plot(mov5[4,-1],mov5[5,-1],marker='o', c='fuchsia')
        plt.plot(mov5[6,1:],mov5[7,1:], marker='.', c='fuchsia')
        plt.plot(mov5[6,1],mov5[7,1],marker='^', c='fuchsia')
        plt.plot(mov5[6,-1],mov5[7,-1],marker='o', c='fuchsia')

        plt.ylim((data.i_dim[0],0))
        plt.xlim((0,data.i_dim[1]))
        plt.ylabel("v, pixels")
        plt.xlabel("u, pixels")
        plt.legend(loc="best")
        plt.savefig(name + "_movement.eps" , format='eps')
        plt.savefig(name_pdf + "_movement.pdf" , format='pdf')
        plt.savefig(name_png + "_movement.png" , format='png')
        plt.show()
        plt.pause(0.1)
        #plt.draw()

######################################################################################
######################################################################################

def callback_IBVC_Cyl(data):
    # rospy.loginfo(rospy.get_caller_id() + 'I heard '+ error)
    
    global error0_
    global error1_
    global error2_
    global error3_
    global error4_
    global error5_
    global mov0
    global mov1
    global mov2
    global mov3
    global mov4
    global mov5
    
    if data.m_info[0].id != 0:
        error0_ = np.append(error0_, [[data.m_info[0].errors[0]], [data.t], [data.m_info[0].errors[1]], [data.t], [data.m_info[0].errors[2]], [data.t], [data.m_info[0].errors[3]], [data.t], [data.m_info[0].errors[4]], [data.t], [data.m_info[0].errors[5]], [data.t], [data.m_info[0].errors[6]], [data.t], [data.m_info[0].errors[7]], [data.t]], axis = 1)
        mov0 = np.append(mov0, [[data.m_info[0].point_movements[0]], [data.m_info[0].point_movements[1]], [data.m_info[0].point_movements[2]], [data.m_info[0].point_movements[3]], [data.m_info[0].point_movements[4]], [data.m_info[0].point_movements[5]], [data.m_info[0].point_movements[6]], [data.m_info[0].point_movements[7]]], axis = 1)

    if data.m_info[1].id != 0:
        error1_ = np.append(error1_, [[data.m_info[1].errors[0]], [data.t], [data.m_info[1].errors[1]], [data.t], [data.m_info[1].errors[2]], [data.t], [data.m_info[1].errors[3]], [data.t], [data.m_info[1].errors[4]], [data.t], [data.m_info[1].errors[5]], [data.t], [data.m_info[1].errors[6]], [data.t], [data.m_info[1].errors[7]], [data.t] ], axis = 1)
        mov1 = np.append(mov1, [[data.m_info[1].point_movements[0]], [data.m_info[1].point_movements[1]], [data.m_info[1].point_movements[2]], [data.m_info[1].point_movements[3]], [data.m_info[1].point_movements[4]], [data.m_info[1].point_movements[5]], [data.m_info[1].point_movements[6]], [data.m_info[1].point_movements[7]]], axis = 1)

    if data.m_info[2].id != 0:
        error2_ = np.append(error2_, [[data.m_info[2].errors[0]], [data.t], [data.m_info[2].errors[1]], [data.t], [data.m_info[2].errors[2]], [data.t], [data.m_info[2].errors[3]], [data.t], [data.m_info[2].errors[4]], [data.t], [data.m_info[2].errors[5]], [data.t], [data.m_info[2].errors[6]], [data.t], [data.m_info[2].errors[7]], [data.t] ], axis = 1)
        mov2 = np.append(mov2, [[data.m_info[2].point_movements[0]], [data.m_info[2].point_movements[1]], [data.m_info[2].point_movements[2]], [data.m_info[2].point_movements[3]], [data.m_info[2].point_movements[4]], [data.m_info[2].point_movements[5]], [data.m_info[2].point_movements[6]], [data.m_info[2].point_movements[7]]], axis = 1)

    if data.m_info[3].id != 0:
        error3_ = np.append(error3_, [[data.m_info[3].errors[0]], [data.t], [data.m_info[3].errors[1]], [data.t], [data.m_info[3].errors[2]], [data.t], [data.m_info[3].errors[3]], [data.t], [data.m_info[3].errors[4]], [data.t], [data.m_info[3].errors[5]], [data.t], [data.m_info[3].errors[6]], [data.t], [data.m_info[3].errors[7]], [data.t] ], axis = 1)
        mov3 = np.append(mov3, [[data.m_info[3].point_movements[0]], [data.m_info[3].point_movements[1]], [data.m_info[3].point_movements[2]], [data.m_info[3].point_movements[3]], [data.m_info[3].point_movements[4]], [data.m_info[3].point_movements[5]], [data.m_info[3].point_movements[6]], [data.m_info[3].point_movements[7]]], axis = 1)

    if data.m_info[4].id != 0:
        error4_ = np.append(error4_, [[data.m_info[4].errors[0]], [data.t], [data.m_info[4].errors[1]], [data.t], [data.m_info[4].errors[2]], [data.t], [data.m_info[4].errors[3]], [data.t], [data.m_info[4].errors[4]], [data.t], [data.m_info[4].errors[5]], [data.t], [data.m_info[4].errors[6]], [data.t], [data.m_info[4].errors[7]], [data.t] ], axis = 1)
        mov4 = np.append(mov4, [[data.m_info[4].point_movements[0]], [data.m_info[4].point_movements[1]], [data.m_info[4].point_movements[2]], [data.m_info[4].point_movements[3]], [data.m_info[4].point_movements[4]], [data.m_info[4].point_movements[5]], [data.m_info[4].point_movements[6]], [data.m_info[4].point_movements[7]]], axis = 1)

    if data.m_info[5].id != 0:
        error5_ = np.append(error5_, [[data.m_info[5].errors[0]], [data.t], [data.m_info[5].errors[1]], [data.t], [data.m_info[5].errors[2]], [data.t], [data.m_info[5].errors[3]], [data.t], [data.m_info[5].errors[4]], [data.t], [data.m_info[5].errors[5]], [data.t], [data.m_info[5].errors[6]], [data.t], [data.m_info[5].errors[7]], [data.t] ], axis = 1)
        mov5 = np.append(mov5, [[data.m_info[5].point_movements[0]], [data.m_info[5].point_movements[1]], [data.m_info[5].point_movements[2]], [data.m_info[5].point_movements[3]], [data.m_info[5].point_movements[4]], [data.m_info[5].point_movements[5]], [data.m_info[5].point_movements[6]], [data.m_info[5].point_movements[7]]], axis = 1)

    
    global position0
    position0.append(data.p[0])
    global position1
    position1.append(data.p[1])
    global position2
    position2.append(data.p[2])
    global position3
    position3.append(data.p[3])
    global position4
    position4.append(data.p[4])
    global position5
    position5.append(data.p[5])

    global velocity0
    velocity0.append(data.v[0])
    global velocity1
    velocity1.append(data.v[1])
    global velocity2
    velocity2.append(data.v[2])
    global velocity3
    velocity3.append(data.v[3])
    global velocity4
    velocity4.append(data.v[4])
    global velocity5
    velocity5.append(data.v[5])

    global velocity0_e
    velocity0_e.append(data.v_c[0])
    global velocity1_e
    velocity1_e.append(data.v_c[1])
    global velocity2_e
    velocity2_e.append(data.v_c[2])
    global velocity3_e
    velocity3_e.append(data.v_c[3])
    global velocity4_e
    velocity4_e.append(data.v_c[4])
    global velocity5_e
    velocity5_e.append(data.v_c[5])

    global joints0
    joints0.append(data.j[0])
    global joints1
    joints1.append(data.j[1])
    global joints2
    joints2.append(data.j[2])
    global joints3
    joints3.append(data.j[3])
    global joints4
    joints4.append(data.j[4])
    global joints5
    joints5.append(data.j[5])

    global time
    time.append(data.t)

    global name 
    global name_pdf
    global name_png

    #global mov
    #mov = np.append(mov, [[data.m[0]],[data.m[1]],[data.m[2]],[data.m[3]],[data.m[4]],[data.m[5]],[data.m[6]],[data.m[7]]], axis = 1)




    #rospy.loginfo( "The errors %f , %f , %f , %f , %f , %f", e[0], e[1], e[2], e[3], e[4], e[5])
    #plt.ion()
    if data.finish:
        
        plt.figure(1)
        plt.title("rho,theta error")

        plt.plot(error0_[1,1:], error0_[0,1:], marker=',', c='red', label='A 0')
        plt.plot(error0_[3,1:], error0_[2,1:], marker=',', c='red')
        plt.plot(error0_[5,1:], error0_[4,1:], marker=',', c='red')
        plt.plot(error0_[7,1:], error0_[6,1:], marker=',', c='red')
        plt.plot(error0_[9,1:], error0_[8,1:], marker=',', c='red')
        plt.plot(error0_[11,1:], error0_[10,1:], marker=',', c='red')
        plt.plot(error0_[13,1:], error0_[12,1:], marker=',', c='red')
        plt.plot(error0_[15,1:], error0_[14,1:], marker=',', c='red')

        plt.plot(error1_[1,1:], error1_[0,1:], marker=',', c='blue', label='A 1')
        plt.plot(error1_[3,1:], error1_[2,1:], marker=',', c='blue')
        plt.plot(error1_[5,1:], error1_[4,1:], marker=',', c='blue')
        plt.plot(error1_[7,1:], error1_[6,1:], marker=',', c='blue')
        plt.plot(error1_[9,1:], error1_[8,1:], marker=',', c='blue')
        plt.plot(error1_[11,1:], error1_[10,1:], marker=',', c='blue')
        plt.plot(error1_[13,1:], error1_[12,1:], marker=',', c='blue')
        plt.plot(error1_[15,1:], error1_[14,1:], marker=',', c='blue')

        plt.plot(error2_[1,1:], error2_[0,1:], marker=',', c='green', label='A 2')
        plt.plot(error2_[3,1:], error2_[2,1:], marker=',', c='green')
        plt.plot(error2_[5,1:], error2_[4,1:], marker=',', c='green')
        plt.plot(error2_[7,1:], error2_[6,1:], marker=',', c='green')
        plt.plot(error2_[9,1:], error2_[8,1:], marker=',', c='green')
        plt.plot(error2_[11,1:], error2_[10,1:], marker=',', c='green')
        plt.plot(error2_[13,1:], error2_[12,1:], marker=',', c='green')
        plt.plot(error2_[15,1:], error2_[14,1:], marker=',', c='green')

        plt.plot(error3_[1,1:], error3_[0,1:], marker=',', c='silver', label='A 3')
        plt.plot(error3_[3,1:], error3_[2,1:], marker=',', c='silver')
        plt.plot(error3_[5,1:], error3_[4,1:], marker=',', c='silver')
        plt.plot(error3_[7,1:], error3_[6,1:], marker=',', c='silver')
        plt.plot(error3_[9,1:], error3_[8,1:], marker=',', c='silver')
        plt.plot(error3_[11,1:], error3_[10,1:], marker=',', c='silver')
        plt.plot(error3_[13,1:], error3_[12,1:], marker=',', c='silver')
        plt.plot(error3_[15,1:], error3_[14,1:], marker=',', c='silver')

        plt.plot(error4_[1,1:], error4_[0,1:], marker=',', c='lightgreen', label='A 4')
        plt.plot(error4_[3,1:], error4_[2,1:], marker=',', c='lightgreen')
        plt.plot(error4_[5,1:], error4_[4,1:], marker=',', c='lightgreen')
        plt.plot(error4_[7,1:], error4_[6,1:], marker=',', c='lightgreen')
        plt.plot(error4_[9,1:], error4_[8,1:], marker=',', c='lightgreen')
        plt.plot(error4_[11,1:], error4_[10,1:], marker=',', c='lightgreen')
        plt.plot(error4_[13,1:], error4_[12,1:], marker=',', c='lightgreen')
        plt.plot(error4_[15,1:], error4_[14,1:], marker=',', c='lightgreen')

        plt.plot(error5_[1,1:], error5_[0,1:], marker=',', c='fuchsia', label='A 5')
        plt.plot(error5_[3,1:], error5_[2,1:], marker=',', c='fuchsia')
        plt.plot(error5_[5,1:], error5_[4,1:], marker=',', c='fuchsia')
        plt.plot(error5_[7,1:], error5_[6,1:], marker=',', c='fuchsia')
        plt.plot(error5_[9,1:], error5_[8,1:], marker=',', c='fuchsia')
        plt.plot(error5_[11,1:], error5_[10,1:], marker=',', c='fuchsia')
        plt.plot(error5_[13,1:], error5_[12,1:], marker=',', c='fuchsia')
        plt.plot(error5_[15,1:], error5_[14,1:], marker=',', c='fuchsia')
        
        plt.ylabel("rho,theta error, m and rad")
        plt.xlabel("Time, s")
        plt.legend(loc="best")
        plt.savefig(name + "_rho_theta_error.eps" , format='eps')
        plt.savefig(name_pdf + "_rho_theta_error.pdf" , format='pdf')
        plt.savefig(name_png + "_rho_theta_error.png" , format='png')
        plt.show()
        plt.pause(0.1)
        #plt.draw()

        plt.figure(2)
        plt.title("End-effector position")
        #plt.plot(time, position0, marker='_', c='yellow')
        plt.plot(time, position0, marker=',', c='red', label='x')
        plt.plot(time, position1, marker=',', c='blue', label='y')
        plt.plot(time, position2, marker=',', c='green', label='z')
        plt.plot(time, position3, marker=',', c='silver', label='roll')
        plt.plot(time, position4, marker=',', c='lightgreen', label='pitch')
        plt.plot(time, position5, marker=',', c='fuchsia', label='yaw')
        plt.ylabel("End-effector position, m and rad")
        plt.xlabel("Time, s")
        plt.legend(loc="best")
        plt.savefig(name + "_position_end_effector.eps" , format='eps')
        plt.savefig(name_pdf + "_position_end_effector.pdf" , format='pdf')
        plt.savefig(name_png + "_position_end_effector.png" , format='png')
        plt.show()
        plt.pause(0.1)
        #plt.draw()
        
        plt.figure(3)
        plt.title("Joint velocities")
        #plt.plot(time,velocity0, marker='_', c='yellow')
        plt.plot(time,velocity0, marker=',', c='red', label='v. j0')
        plt.plot(time,velocity1, marker=',', c='blue', label='v. j1')
        plt.plot(time,velocity2, marker=',', c='green', label='v. j2')
        plt.plot(time,velocity3, marker=',', c='silver', label='v. j3')
        plt.plot(time,velocity4, marker=',', c='lightgreen', label='v. j4')
        plt.plot(time,velocity5, marker=',', c='fuchsia', label='v. j5')
        plt.ylabel("Joint velocities, rad/s")
        plt.xlabel("Time, s")
        plt.legend(loc="best")
        plt.savefig(name + "_joint_velocities.eps" , format='eps')
        plt.savefig(name_pdf + "_joint_velocities.pdf" , format='pdf')
        plt.savefig(name_png + "_joint_velocities.png" , format='png')
        plt.show()
        plt.pause(0.1)
        #plt.draw()

        plt.figure(4)
        plt.title("Joint positions")
        #plt.plot(time,joints0, marker='_', c='yellow')
        plt.plot(time,joints0, marker=',', c='red', label='p. j0')
        plt.plot(time,joints1, marker=',', c='blue', label='p. j1')
        plt.plot(time,joints2, marker=',', c='green', label='p. j2')
        plt.plot(time,joints3, marker=',', c='silver', label='p. j3')
        plt.plot(time,joints4, marker=',', c='lightgreen', label='p. j4')
        plt.plot(time,joints5, marker=',', c='fuchsia', label='p. j5')
        plt.ylabel("Joint positions, rad")
        plt.xlabel("Time, s")
        plt.legend(loc="best")
        plt.savefig(name + "_joint_positions.eps" , format='eps')
        plt.savefig(name_pdf + "_joint_positions.pdf" , format='pdf')
        plt.savefig(name_png + "_joint_positions.png" , format='png')
        plt.show()
        plt.pause(0.1)
        #plt.draw()

        plt.figure(5)
        plt.title("Camera velocities")
        #plt.plot(time,joints0, marker='_', c='yellow')
        plt.plot(time,velocity0_e, marker=',', c='red', label='v. x')
        plt.plot(time,velocity1_e, marker=',', c='blue', label='v. y')
        plt.plot(time,velocity2_e, marker=',', c='green', label='v. z')
        plt.plot(time,velocity3_e, marker=',', c='silver', label='v. roll')
        plt.plot(time,velocity4_e, marker=',', c='lightgreen', label='v. pitch')
        plt.plot(time,velocity5_e, marker=',', c='fuchsia', label='v. yaw')
        plt.ylabel("Camera velocities, m/s and rad/s")
        plt.xlabel("Time, s")
        plt.legend(loc="best")
        plt.savefig(name + "_end_effector_velocities.eps" , format='eps')
        plt.savefig(name_pdf + "_end_effector_velocities.pdf" , format='pdf')
        plt.savefig(name_png + "_end_effector_velocities.png" , format='png')
        plt.show()
        plt.pause(0.1)
        #plt.draw(

        
        plt.figure(6)
        plt.title("Movement of the ArUco markers in the image")
        #plt.plot(time,joints0, marker='_', c='yellow')
        plt.plot(mov0[0,1:],mov0[1,1:], marker='.', c='red', label='A 0')
        plt.plot(mov0[0,1],mov0[1,1],marker='^', c='red')
        plt.plot(mov0[0,-1],mov0[1,-1],marker='o', c='red')
        plt.plot(mov0[2,1:],mov0[3,1:], marker='.', c='red')
        plt.plot(mov0[2,1],mov0[3,1],marker='^', c='red')
        plt.plot(mov0[2,-1],mov0[3,-1],marker='o', c='red')
        plt.plot(mov0[4,1:],mov0[5,1:], marker='.', c='red')
        plt.plot(mov0[4,1],mov0[5,1],marker='^', c='red')
        plt.plot(mov0[4,-1],mov0[5,-1],marker='o', c='red')
        plt.plot(mov0[6,1:],mov0[7,1:], marker='.', c='red')
        plt.plot(mov0[6,1],mov0[7,1],marker='^', c='red')
        plt.plot(mov0[6,-1],mov0[7,-1],marker='o', c='red')

        plt.plot(mov1[0,1:],mov1[1,1:], marker='.', c='blue', label='A 1')
        plt.plot(mov1[0,1],mov1[1,1],marker='^', c='blue')
        plt.plot(mov1[0,-1],mov1[1,-1],marker='o', c='blue')
        plt.plot(mov1[2,1:],mov1[3,1:], marker='.', c='blue')
        plt.plot(mov1[2,1],mov1[3,1],marker='^', c='blue')
        plt.plot(mov1[2,-1],mov1[3,-1],marker='o', c='blue')
        plt.plot(mov1[4,1:],mov1[5,1:], marker='.', c='blue')
        plt.plot(mov1[4,1],mov1[5,1],marker='^', c='blue')
        plt.plot(mov1[4,-1],mov1[5,-1],marker='o', c='blue')
        plt.plot(mov1[6,1:],mov1[7,1:], marker='.', c='blue')
        plt.plot(mov1[6,1],mov1[7,1],marker='^', c='blue')
        plt.plot(mov1[6,-1],mov1[7,-1],marker='o', c='blue')

        plt.plot(mov2[0,1:],mov2[1,1:], marker='.', c='green', label='A 2')
        plt.plot(mov2[0,1],mov2[1,1],marker='^', c='green')
        plt.plot(mov2[0,-1],mov2[1,-1],marker='o', c='green')
        plt.plot(mov2[2,1:],mov2[3,1:], marker='.', c='green')
        plt.plot(mov2[2,1],mov2[3,1],marker='^', c='green')
        plt.plot(mov2[2,-1],mov2[3,-1],marker='o', c='green')
        plt.plot(mov2[4,1:],mov2[5,1:], marker='.', c='green')
        plt.plot(mov2[4,1],mov2[5,1],marker='^', c='green')
        plt.plot(mov2[4,-1],mov2[5,-1],marker='o', c='green')
        plt.plot(mov2[6,1:],mov2[7,1:], marker='.', c='green')
        plt.plot(mov2[6,1],mov2[7,1],marker='^', c='green')
        plt.plot(mov2[6,-1],mov2[7,-1],marker='o', c='green')

        plt.plot(mov3[0,1:],mov3[1,1:], marker='.', c='silver', label='A 3')
        plt.plot(mov3[0,1],mov3[1,1],marker='^', c='silver')
        plt.plot(mov3[0,-1],mov3[1,-1],marker='o', c='silver')
        plt.plot(mov3[2,1:],mov3[3,1:], marker='.', c='silver')
        plt.plot(mov3[2,1],mov3[3,1],marker='^', c='silver')
        plt.plot(mov3[2,-1],mov3[3,-1],marker='o', c='silver')
        plt.plot(mov3[4,1:],mov3[5,1:], marker='.', c='silver')
        plt.plot(mov3[4,1],mov3[5,1],marker='^', c='silver')
        plt.plot(mov3[4,-1],mov3[5,-1],marker='o', c='silver')
        plt.plot(mov3[6,1:],mov3[7,1:], marker='.', c='silver')
        plt.plot(mov3[6,1],mov3[7,1],marker='^', c='silver')
        plt.plot(mov3[6,-1],mov3[7,-1],marker='o', c='silver')

        plt.plot(mov4[0,1:],mov4[1,1:], marker='.', c='lightgreen', label='A 4')
        plt.plot(mov4[0,1],mov4[1,1],marker='^', c='lightgreen')
        plt.plot(mov4[0,-1],mov4[1,-1],marker='o', c='lightgreen')
        plt.plot(mov4[2,1:],mov4[3,1:], marker='.', c='lightgreen')
        plt.plot(mov4[2,1],mov4[3,1],marker='^', c='lightgreen')
        plt.plot(mov4[2,-1],mov4[3,-1],marker='o', c='lightgreen')
        plt.plot(mov4[4,1:],mov4[5,1:], marker='.', c='lightgreen')
        plt.plot(mov4[4,1],mov4[5,1],marker='^', c='lightgreen')
        plt.plot(mov4[4,-1],mov4[5,-1],marker='o', c='lightgreen')
        plt.plot(mov4[6,1:],mov4[7,1:], marker='.', c='lightgreen')
        plt.plot(mov4[6,1],mov4[7,1],marker='^', c='lightgreen')
        plt.plot(mov4[6,-1],mov4[7,-1],marker='o', c='lightgreen')

        plt.plot(mov5[0,1:],mov5[1,1:], marker='.', c='fuchsia', label='A 5')
        plt.plot(mov5[0,1],mov5[1,1],marker='^', c='fuchsia')
        plt.plot(mov5[0,-1],mov5[1,-1],marker='o', c='fuchsia')
        plt.plot(mov5[2,1:],mov5[3,1:], marker='.', c='fuchsia')
        plt.plot(mov5[2,1],mov5[3,1],marker='^', c='fuchsia')
        plt.plot(mov5[2,-1],mov5[3,-1],marker='o', c='fuchsia')
        plt.plot(mov5[4,1:],mov5[5,1:], marker='.', c='fuchsia')
        plt.plot(mov5[4,1],mov5[5,1],marker='^', c='fuchsia')
        plt.plot(mov5[4,-1],mov5[5,-1],marker='o', c='fuchsia')
        plt.plot(mov5[6,1:],mov5[7,1:], marker='.', c='fuchsia')
        plt.plot(mov5[6,1],mov5[7,1],marker='^', c='fuchsia')
        plt.plot(mov5[6,-1],mov5[7,-1],marker='o', c='fuchsia')

        plt.ylim((data.i_dim[0],0))
        plt.xlim((0,data.i_dim[1]))
        plt.ylabel("v, pixels")
        plt.xlabel("u, pixels")
        plt.legend(loc="best")
        plt.savefig(name + "_movement.eps" , format='eps')
        plt.savefig(name_pdf + "_movement.pdf" , format='pdf')
        plt.savefig(name_png + "_movement.png" , format='png')
        plt.show()
        plt.pause(0.1)
        #plt.draw()      

######################################################################################
######################################################################################

def callback_HVC(data):
    # rospy.loginfo(rospy.get_caller_id() + 'I heard '+ error)
    

    global error0_
    global error1_
    global error2_
    global error3_
    global error4_
    global error5_
    global mov0
    global mov1
    global mov2
    global mov3
    global mov4
    global mov5
    
    if data.m_info[0].id != 0:
        error0_ = np.append(error0_, [[data.m_info[0].errors[0]], [data.t], [data.m_info[0].errors[1]], [data.t], [data.m_info[0].errors[2]], [data.t], [data.m_info[0].errors[3]], [data.t], [data.m_info[0].errors[4]], [data.t], [data.m_info[0].errors[5]], [data.t], [data.m_info[0].errors[6]], [data.t], [data.m_info[0].errors[7]], [data.t]], axis = 1)
        mov0 = np.append(mov0, [[data.m_info[0].point_movements[0]], [data.m_info[0].point_movements[1]], [data.m_info[0].point_movements[2]], [data.m_info[0].point_movements[3]], [data.m_info[0].point_movements[4]], [data.m_info[0].point_movements[5]], [data.m_info[0].point_movements[6]], [data.m_info[0].point_movements[7]]], axis = 1)

    if data.m_info[1].id != 0:
        error1_ = np.append(error1_, [[data.m_info[1].errors[0]], [data.t], [data.m_info[1].errors[1]], [data.t], [data.m_info[1].errors[2]], [data.t], [data.m_info[1].errors[3]], [data.t], [data.m_info[1].errors[4]], [data.t], [data.m_info[1].errors[5]], [data.t], [data.m_info[1].errors[6]], [data.t], [data.m_info[1].errors[7]], [data.t] ], axis = 1)
        mov1 = np.append(mov1, [[data.m_info[1].point_movements[0]], [data.m_info[1].point_movements[1]], [data.m_info[1].point_movements[2]], [data.m_info[1].point_movements[3]], [data.m_info[1].point_movements[4]], [data.m_info[1].point_movements[5]], [data.m_info[1].point_movements[6]], [data.m_info[1].point_movements[7]]], axis = 1)

    if data.m_info[2].id != 0:
        error2_ = np.append(error2_, [[data.m_info[2].errors[0]], [data.t], [data.m_info[2].errors[1]], [data.t], [data.m_info[2].errors[2]], [data.t], [data.m_info[2].errors[3]], [data.t], [data.m_info[2].errors[4]], [data.t], [data.m_info[2].errors[5]], [data.t], [data.m_info[2].errors[6]], [data.t], [data.m_info[2].errors[7]], [data.t] ], axis = 1)
        mov2 = np.append(mov2, [[data.m_info[2].point_movements[0]], [data.m_info[2].point_movements[1]], [data.m_info[2].point_movements[2]], [data.m_info[2].point_movements[3]], [data.m_info[2].point_movements[4]], [data.m_info[2].point_movements[5]], [data.m_info[2].point_movements[6]], [data.m_info[2].point_movements[7]]], axis = 1)

    if data.m_info[3].id != 0:
        error3_ = np.append(error3_, [[data.m_info[3].errors[0]], [data.t], [data.m_info[3].errors[1]], [data.t], [data.m_info[3].errors[2]], [data.t], [data.m_info[3].errors[3]], [data.t], [data.m_info[3].errors[4]], [data.t], [data.m_info[3].errors[5]], [data.t], [data.m_info[3].errors[6]], [data.t], [data.m_info[3].errors[7]], [data.t] ], axis = 1)
        mov3 = np.append(mov3, [[data.m_info[3].point_movements[0]], [data.m_info[3].point_movements[1]], [data.m_info[3].point_movements[2]], [data.m_info[3].point_movements[3]], [data.m_info[3].point_movements[4]], [data.m_info[3].point_movements[5]], [data.m_info[3].point_movements[6]], [data.m_info[3].point_movements[7]]], axis = 1)

    if data.m_info[4].id != 0:
        error4_ = np.append(error4_, [[data.m_info[4].errors[0]], [data.t], [data.m_info[4].errors[1]], [data.t], [data.m_info[4].errors[2]], [data.t], [data.m_info[4].errors[3]], [data.t], [data.m_info[4].errors[4]], [data.t], [data.m_info[4].errors[5]], [data.t], [data.m_info[4].errors[6]], [data.t], [data.m_info[4].errors[7]], [data.t] ], axis = 1)
        mov4 = np.append(mov4, [[data.m_info[4].point_movements[0]], [data.m_info[4].point_movements[1]], [data.m_info[4].point_movements[2]], [data.m_info[4].point_movements[3]], [data.m_info[4].point_movements[4]], [data.m_info[4].point_movements[5]], [data.m_info[4].point_movements[6]], [data.m_info[4].point_movements[7]]], axis = 1)

    if data.m_info[5].id != 0:
        error5_ = np.append(error5_, [[data.m_info[5].errors[0]], [data.t], [data.m_info[5].errors[1]], [data.t], [data.m_info[5].errors[2]], [data.t], [data.m_info[5].errors[3]], [data.t], [data.m_info[5].errors[4]], [data.t], [data.m_info[5].errors[5]], [data.t], [data.m_info[5].errors[6]], [data.t], [data.m_info[5].errors[7]], [data.t] ], axis = 1)
        mov5 = np.append(mov5, [[data.m_info[5].point_movements[0]], [data.m_info[5].point_movements[1]], [data.m_info[5].point_movements[2]], [data.m_info[5].point_movements[3]], [data.m_info[5].point_movements[4]], [data.m_info[5].point_movements[5]], [data.m_info[5].point_movements[6]], [data.m_info[5].point_movements[7]]], axis = 1)

    
    global position0
    position0.append(data.p[0])
    global position1
    position1.append(data.p[1])
    global position2
    position2.append(data.p[2])
    global position3
    position3.append(data.p[3])
    global position4
    position4.append(data.p[4])
    global position5
    position5.append(data.p[5])

    global velocity0
    velocity0.append(data.v[0])
    global velocity1
    velocity1.append(data.v[1])
    global velocity2
    velocity2.append(data.v[2])
    global velocity3
    velocity3.append(data.v[3])
    global velocity4
    velocity4.append(data.v[4])
    global velocity5
    velocity5.append(data.v[5])

    global velocity0_e
    velocity0_e.append(data.v_c[0])
    global velocity1_e
    velocity1_e.append(data.v_c[1])
    global velocity2_e
    velocity2_e.append(data.v_c[2])
    global velocity3_e
    velocity3_e.append(data.v_c[3])
    global velocity4_e
    velocity4_e.append(data.v_c[4])
    global velocity5_e
    velocity5_e.append(data.v_c[5])

    global joints0
    joints0.append(data.j[0])
    global joints1
    joints1.append(data.j[1])
    global joints2
    joints2.append(data.j[2])
    global joints3
    joints3.append(data.j[3])
    global joints4
    joints4.append(data.j[4])
    global joints5
    joints5.append(data.j[5])

    global time
    time.append(data.t)

    global name 
    global name_pdf
    global name_png

    #global mov
    #mov = np.append(mov, [[data.m[0]],[data.m[1]],[data.m[2]],[data.m[3]],[data.m[4]],[data.m[5]],[data.m[6]],[data.m[7]]], axis = 1)




    #rospy.loginfo( "The errors %f , %f , %f , %f , %f , %f", e[0], e[1], e[2], e[3], e[4], e[5])
    #plt.ion()
    if data.finish:
        
        plt.figure(1)
        plt.title("x,y error")

        plt.plot(error0_[1,1:], error0_[0,1:], marker=',', c='red', label='A 0')
        plt.plot(error0_[3,1:], error0_[2,1:], marker=',', c='red')
        plt.plot(error0_[5,1:], error0_[4,1:], marker=',', c='red')
        plt.plot(error0_[7,1:], error0_[6,1:], marker=',', c='red')
        plt.plot(error0_[9,1:], error0_[8,1:], marker=',', c='red')
        plt.plot(error0_[11,1:], error0_[10,1:], marker=',', c='red')
        plt.plot(error0_[13,1:], error0_[12,1:], marker=',', c='red')
        plt.plot(error0_[15,1:], error0_[14,1:], marker=',', c='red')

        plt.plot(error1_[1,1:], error1_[0,1:], marker=',', c='blue', label='A 1')
        plt.plot(error1_[3,1:], error1_[2,1:], marker=',', c='blue')
        plt.plot(error1_[5,1:], error1_[4,1:], marker=',', c='blue')
        plt.plot(error1_[7,1:], error1_[6,1:], marker=',', c='blue')
        plt.plot(error1_[9,1:], error1_[8,1:], marker=',', c='blue')
        plt.plot(error1_[11,1:], error1_[10,1:], marker=',', c='blue')
        plt.plot(error1_[13,1:], error1_[12,1:], marker=',', c='blue')
        plt.plot(error1_[15,1:], error1_[14,1:], marker=',', c='blue')

        plt.plot(error2_[1,1:], error2_[0,1:], marker=',', c='green', label='A 2')
        plt.plot(error2_[3,1:], error2_[2,1:], marker=',', c='green')
        plt.plot(error2_[5,1:], error2_[4,1:], marker=',', c='green')
        plt.plot(error2_[7,1:], error2_[6,1:], marker=',', c='green')
        plt.plot(error2_[9,1:], error2_[8,1:], marker=',', c='green')
        plt.plot(error2_[11,1:], error2_[10,1:], marker=',', c='green')
        plt.plot(error2_[13,1:], error2_[12,1:], marker=',', c='green')
        plt.plot(error2_[15,1:], error2_[14,1:], marker=',', c='green')

        plt.plot(error3_[1,1:], error3_[0,1:], marker=',', c='silver', label='A 3')
        plt.plot(error3_[3,1:], error3_[2,1:], marker=',', c='silver')
        plt.plot(error3_[5,1:], error3_[4,1:], marker=',', c='silver')
        plt.plot(error3_[7,1:], error3_[6,1:], marker=',', c='silver')
        plt.plot(error3_[9,1:], error3_[8,1:], marker=',', c='silver')
        plt.plot(error3_[11,1:], error3_[10,1:], marker=',', c='silver')
        plt.plot(error3_[13,1:], error3_[12,1:], marker=',', c='silver')
        plt.plot(error3_[15,1:], error3_[14,1:], marker=',', c='silver')

        plt.plot(error4_[1,1:], error4_[0,1:], marker=',', c='lightgreen', label='A 4')
        plt.plot(error4_[3,1:], error4_[2,1:], marker=',', c='lightgreen')
        plt.plot(error4_[5,1:], error4_[4,1:], marker=',', c='lightgreen')
        plt.plot(error4_[7,1:], error4_[6,1:], marker=',', c='lightgreen')
        plt.plot(error4_[9,1:], error4_[8,1:], marker=',', c='lightgreen')
        plt.plot(error4_[11,1:], error4_[10,1:], marker=',', c='lightgreen')
        plt.plot(error4_[13,1:], error4_[12,1:], marker=',', c='lightgreen')
        plt.plot(error4_[15,1:], error4_[14,1:], marker=',', c='lightgreen')

        plt.plot(error5_[1,1:], error5_[0,1:], marker=',', c='fuchsia', label='A 5')
        plt.plot(error5_[3,1:], error5_[2,1:], marker=',', c='fuchsia')
        plt.plot(error5_[5,1:], error5_[4,1:], marker=',', c='fuchsia')
        plt.plot(error5_[7,1:], error5_[6,1:], marker=',', c='fuchsia')
        plt.plot(error5_[9,1:], error5_[8,1:], marker=',', c='fuchsia')
        plt.plot(error5_[11,1:], error5_[10,1:], marker=',', c='fuchsia')
        plt.plot(error5_[13,1:], error5_[12,1:], marker=',', c='fuchsia')
        plt.plot(error5_[15,1:], error5_[14,1:], marker=',', c='fuchsia')
        plt.ylabel("x,y error, m")
        plt.xlabel("Time, s")
        plt.legend(loc="best")
        plt.savefig(name + "_error_x_y.eps" , format='eps')
        plt.savefig(name_pdf + "_error_x_y.pdf" , format='pdf')
        plt.savefig(name_png + "_error_x_y.png" , format='png')
        plt.show()
        plt.pause(0.1)
        #plt.draw()

        plt.figure(2)
        plt.title("End-effector Position")
        #plt.plot(time, position0, marker='_', c='yellow')
        plt.plot(time, position0, marker=',', c='red', label='x')
        plt.plot(time, position1, marker=',', c='blue', label='y')
        plt.plot(time, position2, marker=',', c='green', label='z')
        plt.plot(time, position3, marker=',', c='silver', label='roll')
        plt.plot(time, position4, marker=',', c='lightgreen', label='pitch')
        plt.plot(time, position5, marker=',', c='fuchsia', label='yaw')
        plt.ylabel("End-effector positions, m and rad")
        plt.xlabel("Time, s")
        plt.legend(loc="best")
        plt.savefig(name + "_position_end_effector.eps" , format='eps')
        plt.savefig(name_pdf + "_position_end_effector.pdf" , format='pdf')
        plt.savefig(name_png + "_position_end_effector.png" , format='png')
        plt.show()
        plt.pause(0.1)
        #plt.draw()
        
        plt.figure(3)
        plt.title("Joint velocities")
        #plt.plot(time,velocity0, marker='_', c='yellow')
        plt.plot(time,velocity0, marker=',', c='red', label='v. j0')
        plt.plot(time,velocity1, marker=',', c='blue', label='v. j1')
        plt.plot(time,velocity2, marker=',', c='green', label='v. j2')
        plt.plot(time,velocity3, marker=',', c='silver', label='v. j3')
        plt.plot(time,velocity4, marker=',', c='lightgreen', label='v. j4')
        plt.plot(time,velocity5, marker=',', c='fuchsia', label='v. j5')
        plt.ylabel("Joint velocities, rad/s")
        plt.xlabel("Time, s")
        plt.legend(loc="best")
        plt.savefig(name + "_joint_velocities.eps" , format='eps')
        plt.savefig(name_pdf + "_joint_velocities.pdf" , format='pdf')
        plt.savefig(name_png + "_joint_velocities.png" , format='png')
        plt.show()
        plt.pause(0.1)
        #plt.draw()

        plt.figure(4)
        plt.title("Joint positions")
        #plt.plot(time,joints0, marker='_', c='yellow')
        plt.plot(time,joints0, marker=',', c='red', label='p. j0')
        plt.plot(time,joints1, marker=',', c='blue', label='p. j1')
        plt.plot(time,joints2, marker=',', c='green', label='p. j2')
        plt.plot(time,joints3, marker=',', c='silver', label='p. j3')
        plt.plot(time,joints4, marker=',', c='lightgreen', label='p. j4')
        plt.plot(time,joints5, marker=',', c='fuchsia', label='p. j5')
        plt.ylabel("Joint positions, rad")
        plt.xlabel("Time, s")
        plt.legend(loc="best")
        plt.savefig(name + "_joint_positions.eps" , format='eps')
        plt.savefig(name_pdf + "_joint_positions.pdf" , format='pdf')
        plt.savefig(name_png + "_joint_positions.png" , format='png')
        plt.show()
        plt.pause(0.1)
        #plt.draw()

        plt.figure(5)
        plt.title("Camera velocities")
        #plt.plot(time,joints0, marker='_', c='yellow')
        plt.plot(time,velocity0_e, marker=',', c='red', label='v. x')
        plt.plot(time,velocity1_e, marker=',', c='blue', label='v. y')
        plt.plot(time,velocity2_e, marker=',', c='green', label='v. z')
        plt.plot(time,velocity3_e, marker=',', c='silver', label='v. roll')
        plt.plot(time,velocity4_e, marker=',', c='lightgreen', label='v. pitch')
        plt.plot(time,velocity5_e, marker=',', c='fuchsia', label='v. yaw')
        plt.ylabel("Camera velocities, m/s and rad/s")
        plt.xlabel("Time, s")
        plt.legend(loc="best")
        plt.savefig(name + "_end_effector_velocities.eps" , format='eps')
        plt.savefig(name_pdf + "_end_effector_velocities.pdf" , format='pdf')
        plt.savefig(name_png + "_end_effector_velocities.png" , format='png')
        plt.show()
        plt.pause(0.1)
        #plt.draw(

        
        plt.figure(6)
        plt.title("Movement of the ArUco markers in the image")
        #plt.plot(time,joints0, marker='_', c='yellow')
        plt.plot(mov0[0,1:],mov0[1,1:], marker='.', c='red', label='A 0')
        plt.plot(mov0[0,1],mov0[1,1],marker='^', c='red')
        plt.plot(mov0[0,-1],mov0[1,-1],marker='o', c='red')
        plt.plot(mov0[2,1:],mov0[3,1:], marker='.', c='red')
        plt.plot(mov0[2,1],mov0[3,1],marker='^', c='red')
        plt.plot(mov0[2,-1],mov0[3,-1],marker='o', c='red')
        plt.plot(mov0[4,1:],mov0[5,1:], marker='.', c='red')
        plt.plot(mov0[4,1],mov0[5,1],marker='^', c='red')
        plt.plot(mov0[4,-1],mov0[5,-1],marker='o', c='red')
        plt.plot(mov0[6,1:],mov0[7,1:], marker='.', c='red')
        plt.plot(mov0[6,1],mov0[7,1],marker='^', c='red')
        plt.plot(mov0[6,-1],mov0[7,-1],marker='o', c='red')

        plt.plot(mov1[0,1:],mov1[1,1:], marker='.', c='blue', label='A 1')
        plt.plot(mov1[0,1],mov1[1,1],marker='^', c='blue')
        plt.plot(mov1[0,-1],mov1[1,-1],marker='o', c='blue')
        plt.plot(mov1[2,1:],mov1[3,1:], marker='.', c='blue')
        plt.plot(mov1[2,1],mov1[3,1],marker='^', c='blue')
        plt.plot(mov1[2,-1],mov1[3,-1],marker='o', c='blue')
        plt.plot(mov1[4,1:],mov1[5,1:], marker='.', c='blue')
        plt.plot(mov1[4,1],mov1[5,1],marker='^', c='blue')
        plt.plot(mov1[4,-1],mov1[5,-1],marker='o', c='blue')
        plt.plot(mov1[6,1:],mov1[7,1:], marker='.', c='blue')
        plt.plot(mov1[6,1],mov1[7,1],marker='^', c='blue')
        plt.plot(mov1[6,-1],mov1[7,-1],marker='o', c='blue')

        plt.plot(mov2[0,1:],mov2[1,1:], marker='.', c='green', label='A 2')
        plt.plot(mov2[0,1],mov2[1,1],marker='^', c='green')
        plt.plot(mov2[0,-1],mov2[1,-1],marker='o', c='green')
        plt.plot(mov2[2,1:],mov2[3,1:], marker='.', c='green')
        plt.plot(mov2[2,1],mov2[3,1],marker='^', c='green')
        plt.plot(mov2[2,-1],mov2[3,-1],marker='o', c='green')
        plt.plot(mov2[4,1:],mov2[5,1:], marker='.', c='green')
        plt.plot(mov2[4,1],mov2[5,1],marker='^', c='green')
        plt.plot(mov2[4,-1],mov2[5,-1],marker='o', c='green')
        plt.plot(mov2[6,1:],mov2[7,1:], marker='.', c='green')
        plt.plot(mov2[6,1],mov2[7,1],marker='^', c='green')
        plt.plot(mov2[6,-1],mov2[7,-1],marker='o', c='green')

        plt.plot(mov3[0,1:],mov3[1,1:], marker='.', c='silver', label='A 3')
        plt.plot(mov3[0,1],mov3[1,1],marker='^', c='silver')
        plt.plot(mov3[0,-1],mov3[1,-1],marker='o', c='silver')
        plt.plot(mov3[2,1:],mov3[3,1:], marker='.', c='silver')
        plt.plot(mov3[2,1],mov3[3,1],marker='^', c='silver')
        plt.plot(mov3[2,-1],mov3[3,-1],marker='o', c='silver')
        plt.plot(mov3[4,1:],mov3[5,1:], marker='.', c='silver')
        plt.plot(mov3[4,1],mov3[5,1],marker='^', c='silver')
        plt.plot(mov3[4,-1],mov3[5,-1],marker='o', c='silver')
        plt.plot(mov3[6,1:],mov3[7,1:], marker='.', c='silver')
        plt.plot(mov3[6,1],mov3[7,1],marker='^', c='silver')
        plt.plot(mov3[6,-1],mov3[7,-1],marker='o', c='silver')

        plt.plot(mov4[0,1:],mov4[1,1:], marker='.', c='lightgreen', label='A 4')
        plt.plot(mov4[0,1],mov4[1,1],marker='^', c='lightgreen')
        plt.plot(mov4[0,-1],mov4[1,-1],marker='o', c='lightgreen')
        plt.plot(mov4[2,1:],mov4[3,1:], marker='.', c='lightgreen')
        plt.plot(mov4[2,1],mov4[3,1],marker='^', c='lightgreen')
        plt.plot(mov4[2,-1],mov4[3,-1],marker='o', c='lightgreen')
        plt.plot(mov4[4,1:],mov4[5,1:], marker='.', c='lightgreen')
        plt.plot(mov4[4,1],mov4[5,1],marker='^', c='lightgreen')
        plt.plot(mov4[4,-1],mov4[5,-1],marker='o', c='lightgreen')
        plt.plot(mov4[6,1:],mov4[7,1:], marker='.', c='lightgreen')
        plt.plot(mov4[6,1],mov4[7,1],marker='^', c='lightgreen')
        plt.plot(mov4[6,-1],mov4[7,-1],marker='o', c='lightgreen')

        plt.plot(mov5[0,1:],mov5[1,1:], marker='.', c='fuchsia', label='A 5')
        plt.plot(mov5[0,1],mov5[1,1],marker='^', c='fuchsia')
        plt.plot(mov5[0,-1],mov5[1,-1],marker='o', c='fuchsia')
        plt.plot(mov5[2,1:],mov5[3,1:], marker='.', c='fuchsia')
        plt.plot(mov5[2,1],mov5[3,1],marker='^', c='fuchsia')
        plt.plot(mov5[2,-1],mov5[3,-1],marker='o', c='fuchsia')
        plt.plot(mov5[4,1:],mov5[5,1:], marker='.', c='fuchsia')
        plt.plot(mov5[4,1],mov5[5,1],marker='^', c='fuchsia')
        plt.plot(mov5[4,-1],mov5[5,-1],marker='o', c='fuchsia')
        plt.plot(mov5[6,1:],mov5[7,1:], marker='.', c='fuchsia')
        plt.plot(mov5[6,1],mov5[7,1],marker='^', c='fuchsia')
        plt.plot(mov5[6,-1],mov5[7,-1],marker='o', c='fuchsia')


        plt.ylim((data.i_dim[0],0))
        plt.xlim((0,data.i_dim[1]))
        plt.ylabel("v, pixels")
        plt.xlabel("u, pixels")
        plt.legend(loc="best")
        plt.savefig(name + "_movement.eps" , format='eps')
        plt.savefig(name_pdf + "_movement.pdf" , format='pdf')
        plt.savefig(name_png + "_movement.png" , format='png')
        plt.show()
        plt.pause(0.1)
        #plt.draw()



def plotter():

    rospy.init_node('plotter', anonymous=True)

    rospy.Subscriber('all_plots', plot, callback)
    rospy.Subscriber('all_plots_IBVC', plot_IBVC, callback_IBVC)
    rospy.Subscriber('all_plots_IBVC_Cyl', plot_IBVC, callback_IBVC_Cyl)
    rospy.Subscriber('all_plots_HVC', plot_IBVC, callback_HVC)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    error0 = []
    error1 = []
    error2 = []
    error3 = []
    error4 = []
    error5 = []
    position0 = []
    position1 = []
    position2 = []
    position3 = []
    position4 = []
    position5 = []
    velocity0 = []
    velocity1 = []
    velocity2 = []
    velocity3 = []
    velocity4 = []
    velocity5 = []
    velocity0_e = []
    velocity1_e = []
    velocity2_e = []
    velocity3_e = []
    velocity4_e = []
    velocity5_e = []
    joints0 = []
    joints1 = []
    joints2 = []
    joints3 = []
    joints4 = []
    joints5 = []
    time = []
    name = '/home/ismael/catkin_ur10_ismael_ws/src/campero_ur10/images/' + sys.argv[1]
    name_pdf = '/home/ismael/catkin_ur10_ismael_ws/src/campero_ur10/images/pdf/' + sys.argv[1]
    name_png = '/home/ismael/catkin_ur10_ismael_ws/src/campero_ur10/images/png/' + sys.argv[1]
    mov = np.zeros((8,1))
    mov0 = np.zeros((8,1))
    mov1 = np.zeros((8,1))
    mov2 = np.zeros((8,1))
    mov3 = np.zeros((8,1))
    mov4 = np.zeros((8,1))
    mov5 = np.zeros((8,1))
    error0_ = np.zeros((16,1))
    error1_ = np.zeros((16,1))
    error2_ = np.zeros((16,1))
    error3_ = np.zeros((16,1))
    error4_ = np.zeros((16,1))
    error5_ = np.zeros((16,1))
    #array2 = np.append(array, [[1],[1],[1]], axis = 1)
    plotter()


