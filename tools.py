# from env.leg_gym_env import LegGymEnv
import numpy as np
import matplotlib.pyplot as plt

def jacobian_abs(q,l1=0.209,l2=0.195):
    """ Jacobian based on absolute angles (like double pendulum)
        Input: motor angles (array), link lengths
        return: jacobian, foot position
    """
    # Jacobian
    J = np.zeros((2,2))
    # [TODO]
    J[0, 0] = l1*np.cos(q[0]) ###
    J[1, 0] = l1*np.sin(q[0]) ###
    J[0, 1] = l2*np.cos(q[1]) ###
    J[1, 1] = l2*np.sin(q[1]) ###

    # foot pos
    pos = np.zeros(2)
    # [TODO]
    pos[0] =  l1*np.sin(q[0]) + l2*np.sin(q[1]) ###
    pos[1] = -l1*np.cos(q[0]) - l2*np.cos(q[1]) ###

    return J, pos

def jacobian_rel(q,l1=0.209,l2=0.195):
    """ Jacobian based on relative angles (like URDF)
        Input: motor angles (array), link lengths
        return: jacobian, foot position
    """
    # Jacobian
    J = np.zeros((2,2))
    # [TODO]
    J[0, 0] = -l1*np.cos(q[0])-l2*np.cos(q[0]+q[1]) ###
    J[1, 0] = +l1*np.sin(q[0])+l2*np.sin(q[0]+q[1]) ###
    J[0, 1] = -l2*np.cos(q[0]+q[1]) ###
    J[1, 1] = +l2*np.sin(q[0]+q[1]) ###

    # foot pos
    pos = np.zeros(2)
    # [TODO]
    pos[0] = -l1*np.sin(q[0])-l2*np.sin(q[0]+q[1]) ###
    pos[1] = -l1*np.cos(q[0])-l2*np.cos(q[0]+q[1]) ###

    return J, pos

def plot(data_list):
    data_array = np.array(data_list)
    plt.figure(figsize=(10, 6))
    plt.plot(data_array[:, 0], label='Foot Pos Error X')
    plt.plot(data_array[:, 1], label='Foot Pos Error Y')
    plt.xlabel('Time Steps')
    plt.ylabel('Foot Position Error')
    plt.title('Foot Position Error Over Time')
    plt.legend()
    plt.grid(True)
    plt.show()
