#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed May 13 14:12:09 2026

@author: taix
"""

import pybullet as p
import pybullet_data
import numpy as np
import time

# --- Configuration des paramètres ---
START_POS = np.array([.4, 0.2, 0.5])  # [x, y, z]
END_POS = np.array([.6, 0.2, 0.5])
V_MAX = .10   # m/s
A_MAX = .10   # m/s^2
TARGET_FORCE = -10.0  # Force vers le bas en Newtons
DT = 1./240.

###################
def get_ee_state(robotId, ee_index):
    """Récupère la position et la vitesse actuelle de l'effecteur"""
    state = p.getLinkState(robotId, ee_index, computeLinkVelocity=1)
    pos = np.array(state[0])
    vel = np.array(state[6])
    return pos, vel
######################################################
# function to joint position, velocity and torque feedback
def getJointStates(robId, controllable_joints):
    joint_states = p.getJointStates(robId, controllable_joints)
    joint_positions = [state[0] for state in joint_states]
    joint_velocities = [state[1] for state in joint_states]
    joint_torques = [state[3] for state in joint_states]
    return joint_positions, joint_velocities, joint_torques
######################################################################
    # function for setting joint positions of robot
def setJointPosition(robId, position, controllable_joints,kp=1.1, kv=1.0):
#    print('Joint position controller',controllable_joints )
    zero_vec = [0.0] * len(controllable_joints)
    p.setJointMotorControlArray(robId,
                                controllable_joints,
                                p.POSITION_CONTROL,
                                targetPositions=position,
                                targetVelocities=zero_vec,
                                positionGains=[kp] * len(controllable_joints),
                                velocityGains=[kv] * len(controllable_joints))
    for _ in range(240): # to settle the robot to its position
        p.stepSimulation()
        time.sleep(0.001)
#####################################################################
        
def check_joint_equality(q1, q2, eps):
    """
    Compare deux configurations angulaires (en radians).
    
    Retourne True si toutes les articulations sont égales à 'eps' près,
    False sinon.
    """
    # 1. Conversion en tableaux numpy pour faciliter les calculs vectoriels
    q1_arr = np.array(q1)
    q2_arr = np.array(q2)
    
    # 2. Gestion de la périodicité des angles (Optionnel mais recommandé)
    # Calcule l'écart le plus court sur un cercle (évite l'erreur entre -pi et pi)
    diff = (q1_arr - q2_arr + np.pi) % (2 * np.pi) - np.pi
    
    # 3. Vérification si la valeur absolue de chaque écart est inférieure à eps
    return np.all(np.abs(diff) < eps)
######################################################################
def Position(robId, position, controllable_joints):
    print('Position controller')
    EPSILON = 0.001 # erreur <0.06 deg par axe
    KP_JOINT= [0.1,0.8,0.1,0.1,0.1,0.1]
    KD_JOINT= [1,1,1,1,1,1]
    p.setJointMotorControlArray(robId,
                                controllable_joints,
                                p.POSITION_CONTROL,
                                targetPositions=position,
                                positionGains=KP_JOINT,
                                velocityGains=KD_JOINT)
    # Sous-boucle dynamique : on attend que le robot atteigne la cible
    i=1
    current_q = getJointStates(robId)
    tfin = check_joint_equality(current_q[0], position, EPSILON)
    print("Tfin=",tfin)
    while i<100 and not tfin:
        p.stepSimulation()
        i=i+1
        print("i=",i)
        # Récupération de la position articulaire actuelle
        current_q = getJointStates(robId)
        tfin = check_joint_equality(current_q[0], position, EPSILON)
        time.sleep(1./1000)
        
    # function to solve forward kinematics
    
def solveForwardPositonKinematics(robId, joint_pos,end_eff_index):
    print('Forward position kinematics')

    # get end-effector link state
    eeState = p.getLinkState(robId, end_eff_index)
    link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot = eeState
    eePose = list(link_trn) + list(p.getEulerFromQuaternion(link_rot))
    print('End-effector pose:', eePose)
    return eePose        

### --- Préparation de la simulation ---
##Joints= np.array([0,np.pi/2,0,0,0,0])
##print("Joints=",Joints)
##time.sleep(1)
##print("Début de la simulation...")
###Joints= (-0.5001122022580431, 0.18830963578931062, 1.9054375269956523, -0.006859374789463249, 0.3318314108431607, 0.0)
##Position(robotId, Joints,controllable_joints)
##angles= getJointStates(robotId)
##print("Angles=",angles)
##ot= solveForwardPositonKinematics(robotId, Joints,ee_index)
##print("OT=",ot)

