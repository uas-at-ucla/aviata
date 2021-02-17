# Based on https://math.stackexchange.com/questions/2659023/minimal-ell-infty-solution-of-underdetermined-system-of-linear-equations

import numpy as np
import cvxpy as cvx

# TODO this needs to do more than simply minimize infinity norm, because:
#    1) thrust cannot be negative
#    2) motors not used much for thrusting cannot be used much for attitude
#    An idea: optimize thrust first, then optimize for attitude saturation given 50% thrust

def minimal_infinity_norm_solution_of_underdetermined_linear_system(A, b):
    n = A.shape[0] # number of rows of A
    m = A.shape[1] # number of columns of A

    # check that system has solution
    if (np.linalg.matrix_rank(np.column_stack((A,b))) - np.linalg.matrix_rank(A)):
        return np.zeros(m)

    # matrix costraints for equality Ax = b
    A_eq = np.column_stack((A, np.zeros(n)))
    b_eq = b

    # matrix constraint for |x|<s
    M_3 = np.column_stack((np.diag(np.ones(m)),  np.ones(m)))
    b_3 = np.zeros(m)

    M_4 = np.column_stack(( - np.diag(np.ones(m)),  np.ones(m)))
    b_4 = np.zeros(m)
    # contraint s > 0
    M_5 = np.zeros(M_4.shape[1])
    M_5[-1] = 1
    b_5 = [0.]

    A_ub = np.row_stack((M_3, M_4, M_5))
    b_ub = np.concatenate((b_3, b_4, b_5))

    # define target
    c = np.append(np.zeros(m), [1.])

    # Construct the problem.
    x = cvx.Variable(c.shape[0])
    objective = cvx.Minimize(c.T @ x)
    constraints = [A_eq@x == b_eq, A_ub@x >= b_ub ]
    prob = cvx.Problem(objective, constraints)

    # The optimal objective is returned by prob.solve().
    result = prob.solve()
    # The optimal value for x is stored in x.value.
    return x.value[:-1]

def optimal_inverse(A):
    B = np.zeros([A.shape[1], A.shape[0]])
    for i in range(A.shape[0]):
        b = np.zeros([A.shape[0]])
        b[i] = 1
        B[:,i] = minimal_infinity_norm_solution_of_underdetermined_linear_system(A, b)
    return B


if __name__ == '__main__':
    # basic test
    print(optimal_inverse(np.array([[1, 0, 0], [0, 1, 0]])))
