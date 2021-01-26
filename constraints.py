import numpy as np
from z3 import *

# auxiliary functions from https://theory.stanford.edu/~nikolaj/programmingz3.html#sec-maximizing-satisfying-assignments

'''
When initializing an oracle, ensure that all typing of input variables is correct (see docstrings)
'''
class UserOracle:

    def __init__(self, initial_constraint_dict, unknown_constraint_dict, discrete_vars, cont_vars):
        '''
        :param initial_constraint_dict: map names to boolean expressions containing z3 variables
        :param unknown_constraint_set: map names to boolean expressions containing z3 variables
        :param discrete_vars: a list of z3 variables in discrete format(s) (e.g. Int)
        :param cont_vars: a list of z3 variables in continuous format(s) (e.g. Float)
        '''
        self.solver = Solver()
        self.discrete_vars = discrete_vars
        self.continuous_vars = cont_vars
        # now, encode the constraints by name
        for constraint_name, constraint in initial_constraint_dict.items():
            self.solver.assert_and_track(constraint, Bool(constraint_name))
        for hidden_constraint_name, hidden_constraint in unknown_constraint_dict.items():
            self.solver.assert_and_track(hidden_constraint, Bool(hidden_constraint_name + "_hidden"))

        #sanity check: ensure that the solver is satisfiable in it's current form
        if self.solver.check() != sat:
            print("ERROR: original set of constraints provided to oracle is not satisfiable")
        self.solver.push()

    def get_id_response(self, soln_candidate):
        '''
        :param soln_candidate: a list of constraints that assigns all variables to values in the form of constraints
        :return: either a satisfactory response, or a list of constraints (by name) that were violated by the candidate
        '''
        for var_assign in soln_candidate:
            self.solver.add(var_assign)
        # now, check if the assignments is satisfactory
        if self.solver.check() == sat:
            print("proposed solution candidate is satisfactory:")
            print(self.solver.model())
            result = True
        else:
            result = self.solver.unsat_core()
            print("proposed solution violates the following constraints: {}".format(result))
        self.solver.pop()
        return result


    def get_counterexample_response(self, soln_candidate):
        result = self.get_id_response(soln_candidate)
        if not result:
            # meaning we have an unsat candidate
            # we want a solution that satisfies all provided constraints, so just use the current solver
            return self.solver.model()
        else:
            return result


def create_constraint_set_1():
    '''
    :return Z3 module with arbitrary constraint set encoded and solvable
    '''
    # add Z3 constraint variables to be solved for
    s = Solver()
    X = [Int('x%s' % i) for i in range(7)]
    s.add(X[0] >= 2 * X[3] - 4 * X[2] + 3,
           X[1] >= 4 * X[5] + 1 * X[4] + 2,
           X[2] < 3 * X[6] - 6 * X[5] - 1,
           X[3] < -1 * X[0] + 7 * X[6] - 9,
           X[4] > 5 * X[2] - 8 * X[1] + 9,
           X[5] > 6 * X[4] + 3 * X[3] - 6,
           X[6] < 9 * X[2] - 2 * X[0] + 5,
           X[2] < 4 * X[0] + 6 * X[3] + 5 * X[1],
           X[3] > 6 * X[2] - 2 * X[4] - 3 * X[5],
           X[0] < 100,
           X[1] < 100,
           X[1] > 0,
           X[2] < 100,
           X[2] > -50,
           X[3] < 100,
           X[3] > -100,
           X[4] < 100,
           X[4] > 0,
           X[5] < 100,
           X[5] > 0,
           X[6] < 100,
           X[6] > 0)
    return s


def addtional_constraint_set_1(slvr, X):
    slvr.add(X[5] > 6 * X[4] + 3 * X[3] - 6,
    X[6] < 9 * X[2] - 2 * X[0] + 5,
    X[2] < 4 * X[0] + 6 * X[3] + 5 * X[1],
    X[3] > 6 * X[2] - 2 * X[4] - 3 * X[5])
    return slvr


def block_model(s):
    m = s.model()
    s.add(Or([f() != m[f] for f in m.decls() if f.arity() == 0]))


def block_model(s, terms):
    m = s.model()
    s.add(Or([t != m.eval(t) for t in terms]))


def all_smt(s, terms):
    while sat == s.check():
       print(s.model())
       block_model(s, terms)


def all_smt_init(s, initial_terms):
    def block_term(s, m, t):
        s.add(t != m.eval(t))

    def fix_term(s, m, t):
        s.add(t == m.eval(t))

    def all_smt_rec(terms):
        if sat == s.check():
           m = s.model()
           yield m
           for i in range(len(terms)):
               s.push()
               block_term(s, m, terms[i])
               for j in range(i):
                   fix_term(s, m, terms[j])
               for m in all_smt_rec(terms[i+1:]):
                   yield m
               s.pop()
    for m in all_smt_rec(list(initial_terms)):
        yield m

orig_solver = create_constraint_set_1()
all_smt(orig_solver)

