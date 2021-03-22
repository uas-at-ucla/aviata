# Inspired by https://math.stackexchange.com/questions/2659023/minimal-ell-infty-solution-of-underdetermined-system-of-linear-equations

import numpy as np
import cvxpy as cvx

def maximize_setpoint(A, i, b, lower_bounds, upper_bounds, negative=False):
    # m = A.shape[0] # number of rows of A (setpoints)
    n = A.shape[1] # number of columns of A (rotors)

    # check that system has solution
    # if (np.linalg.matrix_rank(np.column_stack((A,b))) - np.linalg.matrix_rank(A)):
    #     return np.zeros(m)

    # constraints for equality Ax = b, excluding the value to maximize
    A_eq = np.delete(A, i, 0)
    b_eq = np.delete(b, i, 0)
    
    # set irrelevant rotors to 0
    b_nonzero = np.append(np.nonzero(b), i)
    A_bnonzero = A[b_nonzero]
    for j in range(n):
        if np.all(A_bnonzero[:,j] == 0):
            sel_j = np.zeros((1,n))
            sel_j[0,j] = 1
            A_eq = np.append(A_eq, sel_j, axis=0)
            b_eq = np.append(b_eq, 0)

    # multiply A[i] by -1 if value to maximize is negative (want to maximize magnitude)
    A_i_obj = A[i] * (-1 if negative else 1)

    # Construct the problem.
    x = cvx.Variable(n)
    objective = cvx.Maximize(A_i_obj @ x)
    constraints = [A_eq@x == b_eq, x >= lower_bounds, x <= upper_bounds ]
    prob = cvx.Problem(objective, constraints)

    # The optimal objective is returned by prob.solve().
    result = prob.solve()
    # The optimal value for x is stored in x.value.
    return x.value

def optimal_inverse(A):
    m = A.shape[0] # number of rows of A (setpoints)
    n = A.shape[1] # number of columns of A (rotors)
    B = np.zeros([n, m])
    thrust_sol = maximize_setpoint(A, 5, np.zeros(m), np.zeros(n), np.ones(n), negative=True)
    max_thrust = A[5] @ thrust_sol
    B[:,5] = thrust_sol / max_thrust

    # constrain torque control allocation at hover thrust
    hover_thrust = max_thrust / 2 # assume hover thrust is 50% of max thrust
    hover_thrust_vals = B[:,5] * hover_thrust
    inverse_hover_thrust_vals = 1 - hover_thrust_vals
    torque_contraints = np.minimum(hover_thrust_vals, inverse_hover_thrust_vals)

    for i in [0,1,2]:
        torque_sol = maximize_setpoint(A, i, np.zeros(m), -torque_contraints, torque_contraints)
        max_torque = A[i] @ torque_sol
        B[:,i] = torque_sol / max_torque
    
    return B


if __name__ == '__main__':
    # basic test
    print(optimal_inverse(np.array([
        [1, 1, -1, -1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 1, -1, -1],
        [1, -1, 1, -1, 1, -1, 1, -1],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [-1, -1, -1, -1, -1, -1, -1, -1]
    ])))
