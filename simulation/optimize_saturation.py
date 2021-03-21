# Based on https://math.stackexchange.com/questions/2659023/minimal-ell-infty-solution-of-underdetermined-system-of-linear-equations

import numpy as np
import cvxpy as cvx

# TODO this needs to do more than simply minimize infinity norm, because:
#    1) thrust cannot be negative
#    2) motors not used much for thrusting cannot be used much for attitude
#    An idea: optimize thrust first, then optimize for attitude saturation given 50% thrust

def min_sat_above_0_sol(A, i, val, ignore_z_thrust=False, B5=None):
    n = A.shape[0] # number of rows of A
    m = A.shape[1] # number of columns of A

    b = np.zeros(n)
    b[i] = val

    # check that system has solution
    if (np.linalg.matrix_rank(np.column_stack((A,b))) - np.linalg.matrix_rank(A)):
        return np.zeros(m)

    # matrix costraints for equality Ax = b
    A_eq = A
    b_eq = b
    n_eq = n
    if ignore_z_thrust:
        A_eq = A_eq[:-1] # last row is z thrust
        b_eq = b[:-1]
        n_eq = n-1
    A_eq = np.column_stack((A_eq, np.zeros(n_eq)))

    # matrix constraint for |x|<s
        # M_1 = np.column_stack((np.diag(np.ones(m)),  np.ones(m)))
        # b_1 = np.zeros(m)

        # M_2 = np.column_stack(( - np.diag(np.ones(m)),  np.ones(m)))
        # b_2 = np.zeros(m)
        # # contraint s > 0
        # M_3 = np.zeros(M_2.shape[1])
        # M_3[-1] = 1
        # b_3 = [0.]

        # A_ub = np.row_stack((M_1, M_2, M_3))
        # b_ub = np.concatenate((b_1, b_2, b_3))

    # Don't utilize rotors that don't contribute to rotation
    A_z = []
    b_z = []
    irrelevant_rotors = []
    if ignore_z_thrust:
        for j in range(m):
            if abs(A[i,j]) < 1e-6:
                irrelevant_rotors.append(j)
                sel_x_j = np.zeros(m)
                sel_x_j[j] = 1
                A_z.append(np.column_stack(([sel_x_j - A[5]*B5[j]], [[0]])))
                b_z.append([0])
    if len(A_z) > 0:
        A_eq = np.row_stack([A_eq] + A_z)
        b_eq = np.concatenate([b_eq] + b_z)

    # matrix constraint for x<s and x>0
    M_1 = np.column_stack(( - np.diag(np.ones(m)),  np.ones(m)))
    b_1 = np.zeros(m)
    M_1 = np.delete(M_1, irrelevant_rotors, 0)
    b_1 = np.delete(b_1, irrelevant_rotors, 0)
    # contraint x > 0
    M_2 = np.column_stack((np.diag(np.ones(m)),  np.zeros(m)))
    b_2 = np.zeros(m)
    M_2 = np.delete(M_2, irrelevant_rotors, 0)
    b_2 = np.delete(b_2, irrelevant_rotors, 0)

    A_ub = np.row_stack((M_1, M_2))
    b_ub = np.concatenate((b_1, b_2))

    # define target
    c = np.append(np.zeros(m), [1.])

    # Construct the problem.
    x = cvx.Variable(c.shape[0])
    objective = cvx.Minimize(c @ x)
    constraints = [A_eq@x == b_eq, A_ub@x >= b_ub ]
    prob = cvx.Problem(objective, constraints)

    # The optimal objective is returned by prob.solve().
    result = prob.solve()
    # The optimal value for x is stored in x.value.
    return x.value[:-1]

def optimal_inverse(A):
    B = np.zeros([A.shape[1], A.shape[0]])
    B[:,5] = -min_sat_above_0_sol(A, 5, -1)
    for i in [0,1,2]:
        torque_sol = min_sat_above_0_sol(A, i, 1, ignore_z_thrust=True, B5=B[:,5])
        thrust = A[5] @ torque_sol
        B[:,i] = torque_sol - thrust*B[:,5]
        assert(abs(A[5] @ B[:,i]) < 1e-6)
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
