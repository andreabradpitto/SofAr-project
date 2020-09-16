"""

@author: dipen

"""
import numpy as np

def JT_enhance(Jp, Ji, err_tran, alpha, beta, q_dot_pref):
    """! 
    Function that computes a Correction Term to be added to the Jacobian Transpose's
    formula, in order to escape from singularity (from "Advances in Robot Kinematics: 
    Analysis and Control", J.Lenarcic and M. L. Husty).
    @param Jp: matrix with inter-joints axes (by rows) used to detect singularity.
    @param Ji: matrix with rotation axes (by rows) used to detect singularity.
    @param err_tran: position error vector.
    @param alpha: tuning parameter (scalar).
    @param beta: tuning parameter (scalar).
    @param q_dot_pref: joint velocities to escape from singularity (elbow configuration), expected [7x1].
    @return: a numpy array containing the Joint Velocities for the correction term [7x1].
    """
    
    # Checking on dimensions of Jp and Ji
    if (len(Jp)/2!=len(Ji)):
        print("Wrong Dimensions for Jp and Ji: it must be Jp_rows = 2*Ji_rows\n")
        return
    
    # Initialization of the Normalized Vectors Matrix (Jp)
    Jp_norm = np.zeros(Jp.shape)
    
    # Initialization of the Normalized Vectors Matrix (Ji)
    Ji_norm = np.zeros(Ji.shape)
    
    # Initialization of the Mu Vector (one elementfor each pair)
    Mu = np.zeros((int(len(Jp)/2),1))
    
    # Initialization of the Gamma Vector
    Gamma = np.zeros((len(Ji),1))
    
    # Initialization of f Vector
    f = np.zeros((int(len(Jp)/2),1))
    
    # Initialization of the g Vector
    g = np.zeros(q_dot_pref.shape)
    
    # Indices Initialization
    k = 0
    j = 0
    
    # Normalization of the Jp Vectors
    for i in range(len(Jp)):
        Jp_norm[i,:] = Jp[i,:]/np.linalg.norm(Jp[i,:])
        
    # Normalization of the Ji Vectors
    for i in range(len(Ji)):
        Ji_norm[i,:] = Ji[i,:]/np.linalg.norm(Ji[i,:])
        
    # Mu Computation
    while (k<len(Jp)):
        Mu[j,:] = -np.dot(Jp_norm[k,:],Jp_norm[k+1,:])*np.dot(Jp_norm[k,:],err_tran)
        k = k+2
        j= j+1
        
    # f computation (to consider only worse singularity situations)
    f = alpha*np.power(Mu,4)
    
    # Gamma Computation
    for i in range(len(Ji_norm)):
        Gamma[i,:] = np.linalg.norm(np.cross(Ji_norm[i,:],err_tran.T))
        
    # Temporary Vector to be element-wise with the other terms: only 3 joints 
    # are considered, since only responsible ones for escaping the singularity
    temp = np.array([[1, Gamma[0]*f[0], 1, Gamma[1]*f[1], 1, Gamma[2]*f[2], 1]],dtype = float).T
    
    # g Enhancing Term Computation (element-wise product)
    g = np.multiply(temp, q_dot_pref*beta*np.linalg.norm(err_tran))
        
    return g

