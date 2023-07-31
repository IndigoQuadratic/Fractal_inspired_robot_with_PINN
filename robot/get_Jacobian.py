# This function is used to get the Jacobian matrix of 4-joints square fractal robot
from sympy import symbols, Matrix, sin, cos

def get_T(i):
    # Define the symbols for the i-th joint
    phi_i, theta_i, d_i, C_i = symbols(f'phi_{i} theta_{i} d_{i} C_{i}')

    # Define the transform matrix for the i-th joint
    Ti = Matrix([
        [-sin(theta_i/2)**2*cos(phi_i)+cos(phi_i)*cos(theta_i/2)**2, -2*sin(theta_i/2)*cos(phi_i)*cos(theta_i/2), sin(phi_i), -d_i*sin(theta_i/2)*cos(phi_i)],
        [2*sin(theta_i/2)*cos(theta_i/2), -sin(theta_i/2)**2+cos(theta_i/2)**2, 0, C_i+d_i*cos(theta_i/2)],
        [sin(phi_i)*sin(theta_i/2)**2-sin(phi_i)*cos(theta_i/2)**2, 2*sin(phi_i)*sin(theta_i/2)*cos(theta_i/2), cos(phi_i), d_i*sin(phi_i)*sin(theta_i/2)],
        [0, 0, 0, 1]
    ])

    return Ti

def get_jacobian(n):
    # Initialize the Jacobian matrix
    J = Matrix.zeros(6, 2*n)

    # Compute the transform matrices
    T = [get_T(i) for i in range(1, n+1)]

    # Compute the position and direction of each joint
    P = [T[i][:3, 3] for i in range(n)]
    Z = [T[i][:3, 2] for i in range(n)]

    # Compute the Jacobian matrix
    for i in range(n):
        J[:3, 2*i] = Z[i-1].cross(P[n-1] - P[i-1]) if i > 0 else Z[i-1].cross(P[n-1])
        J[3:, 2*i] = Z[i-1]

    return J

n = 4  # Number of joints
J = get_jacobian(n)
J
