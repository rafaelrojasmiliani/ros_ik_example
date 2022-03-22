#!/usr/bin/env python3
import pdb
from urdf_parser_py.urdf import URDF
import PyKDL as kdl
import numpy as np
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model

robot_urdf = URDF.from_parameter_server()

pdb.set_trace()
kdl_tree = kdl_tree_from_urdf_model(robot_urdf)


chain = kdl_tree.getChain()
pdb.set_trace()

fk_calculator = kdl.ChainFkSolverPos_recursive(chain)
jacobian_calculator = kdl.ChainJntToJacSolver(chain)


def get_jac(_q: np.array):

    q_kdl = kdl.JntArray(len(_q))
    for i, q_i in enumerate(_q):
        q_kdl[i] = q_i

    j_kdl = kdl.Jacobian(len(_q))
    jacobian_calculator.JntToJac(q_kdl, j_kdl)

    result = np.zeros((j_kdl.rows(), j_kdl.columns()))
    for i in range(result.shape[0]):
        for j in range(result.shape[1]):
            result[i, j] = j_kdl[i, j]
