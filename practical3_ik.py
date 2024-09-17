# Inverse Kinematics practical
from env.leg_gym_env import LegGymEnv
import numpy as np
from practical2_jacobian import jacobian_rel

def angle_to_torque(angle_target,angle_current,J,vel_current,kp,kd):

    target_tau = np.zeros(2) ### 力矩初始化

    angle_error = angle_target - angle_current ### 计算角度误差

    target_force = kp @ angle_error + kd @ vel_current ### 计算目标力

    target_tau = J.T @ target_force

    return target_tau

def pseudoInverse(A,lam=0.001):
    """ Pseudo inverse of matrix A. 
        Make sure to take into account dimensions of A
            i.e. if A is mxn, what dimensions should pseudoInv(A) be if m>n 
        Also take into account potential singularities
    """
    m,n = np.shape(A)
    pinvA = None

    # [Todo]
    if m > n:
        pinvA = np.dot(np.linalg.inv(np.dot(A.T,A) + lam**2*np.eye(n)),A.T)
    else:
        pinvA = np.dot(A.T,np.linalg.inv(np.dot(A,A.T) + lam**2*np.eye(m)))

    return pinvA

def ik_geometrical(xz,angleMode="<",l1=0.209,l2=0.195):
    """ Inverse kinematics based on geometrical reasoning.
        Input: Desired foot xz position (array) 
               angleMode (whether leg should look like > or <) 
               link lengths
        return: joint angles 返回关节实际角度
    """
    q = np.zeros(2)

    x = xz[0]
    z = xz[1]
    r = np.sqrt(x**2+z**2)

    if angleMode == "<":
        theta_2 = -np.arccos((r**2-l1**2-l2**2)/(2*l1*l2)) # 余弦定理
    else:
        theta_2 = +np.arccos((r**2-l1**2-l2**2)/(2*l1*l2)) # 余弦定理

    theta_1 = +np.arctan2(-x,-z)-np.arctan2(l2*np.sin(theta_2),l1+l2*np.cos(theta_2))
    
    q[0] = theta_1
    q[1] = theta_2

    return q

def ik_numerical(q0,des_x,tol=1e-4):
    """ Numerical inverse kinematics
        Input: initial joint angle guess, desired end effector, tolerance
        return: joint angles
    """
    i = 0
    max_i = 100 # max iterations
    alpha = 0.5 # convergence factor
    lam = 0.001 # damping factor for pseudoInverse ### 说明需要使用 pseudoInverse 函数？
    joint_angles = q0

    # Condition to iterate: while fewer than max iterations, and while error is greater than tolerance
    while( i < max_i and 0 ):
        # Evaluate Jacobian based on current joint angles
        J, ee = jacobian_rel(q0) ### [TODO]

        # Compute pseudoinverse
        J_pinv = pseudoInverse(J,lam) ### [TODO]

        # Find end effector error vector
        ee_error = 0

        # update joint_angles
        joint_angles += 0

        # update iteration counter
        i += 1

    return joint_angles


env = LegGymEnv(render=True, 
                on_rack=True,    # set True to debug 
                motor_control_mode='TORQUE',
                action_repeat=1,
                )

NUM_STEPS = 5*1000   # simulate 5 seconds (sim dt is 0.001)
tau = np.zeros(2) # either torques or motor angles, depending on mode

IK_mode = "GEOMETRICAL"

# sample joint PD gains
# kpJoint = np.array([55,55])
# kdJoint = np.array([0.8,0.8])
kpJoint = np.diag([55,55])            ### 测试用
kdJoint = np.diag([10.0,10.0])        ### 测试用

# desired foot position (sample)
des_foot_pos = np.array([0.1,-0.2])   ### 目标位置

for counter in range(NUM_STEPS):
    # Compute inverse kinematics in leg frame 
    if IK_mode == "GEOMETRICAL":
        # geometrical
        qdes         = ik_geometrical(des_foot_pos)    ### 计算目标角度 # 根据 Fusion 运算 几何法正确
        angle_target = ik_geometrical(des_foot_pos)    ### 计算目标角度 # 根据 Fusion 运算 几何法正确
    else:
        # numerical
        qdes = env._robot_config.INIT_MOTOR_ANGLES     # ik_numerical
    
    # print 
    if counter % 500 == 0:
        J, ee_pos_legFrame = jacobian_rel(env.robot.GetMotorAngles())
        print('---------------', counter)
        print('q ik',qdes,'q real',env.robot.GetMotorAngles()) ### 打印计算角度和实际角度
        print('ee pos',ee_pos_legFrame)

    # determine torque with joint PD
    angle_current = env.robot.GetMotorAngles()          ### 获取当前角度
    vel_current   = J @ env.robot.GetMotorVelocities()  ### 获取当前速度

    tau = np.zeros(2)
    tau += -angle_to_torque(angle_target,angle_current,J,vel_current,kpJoint,kdJoint) ### 计算力矩
    tau += J.T @ np.array([0,-env.robot.total_mass*9.81])                             ### 重力补偿

    # apply control, simulate
    env.step(tau)
