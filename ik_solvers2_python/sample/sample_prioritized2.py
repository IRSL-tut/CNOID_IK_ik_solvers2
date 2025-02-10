## aim to do the same procedure as SampleSR1.cpp
## exec(open('sample.py').read())

import cnoid.IRSLUtil as iu
import irsl_choreonoid.sample_robot as sr
import irsl_choreonoid.robot_util as ru
from cnoid import IKSolvers
import numpy as np
import math

rr = sr.init_sample_robot()
rr.flush()
av_org = rr.angleVector()

rr.set_pose('default')
rr.flush()
rr.fix_leg_to_coords(iu.coordinates())
rr.flush()

robot = rr.robot

##// setup tasks
constraints0 = IKSolvers.Constraints()
constraints1 = IKSolvers.Constraints()

##// task: rleg to target
rl_constraint = IKSolvers.PositionConstraint()
rl_constraint.A_link = robot.link('RLEG_ANKLE_R')
rl_constraint.A_localpos = iu.cnoidPosition(np.array([0.0, 0.0, 0.0]))
#constraint.B_link() = nullptr;
rl_constraint.B_localpos = robot.link('RLEG_ANKLE_R').getPosition()
constraints0.push_back(rl_constraint)

##// task: lleg to target
ll_constraint = IKSolvers.PositionConstraint()
ll_constraint.A_link = robot.link('LLEG_ANKLE_R')
ll_constraint.A_localpos = iu.cnoidPosition(np.array([0.0, 0.0, 0.0]))
#constraint.B_link() = nullptr;
ll_constraint.B_localpos = robot.link('LLEG_ANKLE_R').getPosition()
constraints0.push_back(ll_constraint)

##// task: COM to target
com_constraint = IKSolvers.COMConstraint()
com_constraint.A_robot = robot
com_constraint.B_localp = np.array([0.0, 0.0, 0.0])
w = com_constraint.weight
w[2] = 0.0
com_constraint.weight = w
constraints1.push_back(com_constraint)
#
###// task: ????
rt_constraint = IKSolvers.PositionConstraint()
rt_constraint.A_link = robot.rootLink
rt_constraint.A_localpos = iu.cnoidPosition(np.array([0.0, 0.0, 0.0]))
rt_constraint.B_localpos = robot.rootLink.getPosition()
w = rt_constraint.weight
w[0] = 0.0
w[1] = 0.0
w[2] = 1.0
w[3] = 4.0
w[4] = 4.0
w[5] = 4.0
rt_constraint.weight = w
constraints1.push_back(rt_constraint)

tasks = IKSolvers.Tasks()
variables = []
variables.append(robot.rootLink)
for idx in range(robot.getNumJoints()):
    variables.append(robot.joint(idx))

constraints = [constraints0, constraints1 ]
for constl in constraints:
    for const in constl:
        const.debuglevel = 1

loop = IKSolvers.prioritized_solveIKLoop(variables,
                                         constraints,
                                         tasks,
                                         40,
                                         1e-6,
                                         1)

print('loop : {}'.format(loop))

cntr = 0
for constl in constraints:
    for const in constl:
        const.debuglevel = 0
        if const.checkConvergence():
            print('constraint %d (%s) : converged'%(cntr, const))
        else:
            print('constraint %d (%s) : NOT converged'%(cntr, const))
        cntr = cntr + 1
