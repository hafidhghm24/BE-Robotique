#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov  4 14:48:07 2025

@author: taix
"""
import numpy as np
import pybullet as p
import time
import affiche
  
F_CONTROL = 10000
sleep_time = 1/F_CONTROL 

def simulation(steps, sleep_time=0.0001):
# Simulation de steps pas de simulation avec une attente de sleep_time.
     for _ in range(steps):
         p.stepSimulation()
         time.sleep(sleep_time)

########################################################################        
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
####################################################################
# function to joint position, velocity and torque feedback
def getJointStates(robot_id,control_joints):
    joint_states = p.getJointStates(robot_id, control_joints)
    joint_positions = [state[0] for state in joint_states]
    joint_velocities = [state[1] for state in joint_states]
    joint_torques = [state[3] for state in joint_states]
    return joint_positions, joint_velocities, joint_torques


    # function for setting joint positions of robot
def setJointPosition(robot_id, control_joints, position, kp=1.0, kv=1.0):
    print('Joint position controller')
    zero_vec = [0.0] * len(control_joints)
    p.setJointMotorControlArray(robot_id,
                                control_joints,
                                p.POSITION_CONTROL,
                                targetPositions=position,
                                targetVelocities=zero_vec,
                                positionGains=[kp] * len(control_joints),
                                velocityGains=[kv] * len(control_joints))
#    frame_ee = []
    for _ in range(1000):
        p.stepSimulation()
        time.sleep(sleep_time)
        
#    frame_ee = affiche.update_joint_frame(robot_id,6 ,frame_ee)
########################################################################
def Position(robId, controllable_joints, position):
    print('Joint position controller')
    EPSILON = 0.001 # erreur <0.06 deg par axe
    KP_JOINT= [0.1,0.2,0.3,1,1,1]
    KD_JOINT= [1,1,1,1,1,1]
    p.setJointMotorControlArray(robId,
                                controllable_joints,
                                p.POSITION_CONTROL,
                                targetPositions=position,
                                positionGains=KP_JOINT,
                                velocityGains=KD_JOINT)
    # Sous-boucle dynamique : on attend que le robot atteigne la cible
    i=1
    current_q = getJointStates(robId, controllable_joints)
    tfin = check_joint_equality(current_q[0], position, EPSILON)
    print("Tfin=",tfin)
    while i<1000 and not tfin:
        p.stepSimulation()
        i=i+1
        print("i=",i)
        # Récupération de la position articulaire actuelle
        current_q = getJointStates(robId, controllable_joints)
        tfin = check_joint_equality(current_q[0], position, EPSILON)
        time.sleep(1./1000)
########################################################################
def PositionAffiche(robId, controllable_joints, position,frame_ee):
    print('Joint position controller')
    EPSILON = 0.001 # erreur <0.06 deg par axe
    KP_JOINT= [0.1,0.2,0.3,1,1,1]
    KD_JOINT= [1,1,1,1,1,1]
    frame_ee = affiche.update_joint_frame(robId,6 ,frame_ee,0.1)  
    p.setJointMotorControlArray(robId,
                                controllable_joints,
                                p.POSITION_CONTROL,
                                targetPositions=position,
                                positionGains=KP_JOINT,
                                velocityGains=KD_JOINT)
    # Sous-boucle dynamique : on attend que le robot atteigne la cible
    i=1
    current_q = getJointStates(robId, controllable_joints)
    tfin = check_joint_equality(current_q[0], position, EPSILON)
    print("Tfin=",tfin)
    while i<1000 and not tfin:
        p.stepSimulation()
        i=i+1
#        print("i=",i)
        # Récupération de la position articulaire actuelle
        current_q = getJointStates(robId, controllable_joints)
        tfin = check_joint_equality(current_q[0], position, EPSILON)
        time.sleep(1./1000)
        
             
####################################################################
    # function to solve forward kinematics
    
#################################################################
    #function to do joint velcoity control
def JointVelocityControl(robot_id, control_joints, joint_velocities, sim_time=0.01, max_force=200):
    print('Joint velocity controller')
    t=0
    while t<sim_time:
        p.setJointMotorControlArray(robot_id,
                                    control_joints,
                                    p.VELOCITY_CONTROL,
                                    targetVelocities=joint_velocities,
                                    forces = [max_force] * (len(control_joints)))
        p.stepSimulation()
        time.sleep(sleep_time)
        t += sleep_time
        
####################################################################        
# Fonction pour un controle en couple
def JointTorqueControl(robot_id, couple,control_joints):
        """
        sends torque commands (needs an array of desired joint torques)
        """
        zeroGains = [0.0] * len(control_joints)

        p.setJointMotorControlArray(robot_id, control_joints, 
                                    p.TORQUE_CONTROL, forces=couple, 
                                    positionGains=zeroGains, velocityGains=zeroGains)

