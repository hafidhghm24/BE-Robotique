###############################
# Import the necessary modules
###############################
import os
os.getcwd() 
import os.path
import sys
# The PyBullet physics simulation library
import pybullet as p
import pybullet_data

# Numpy for numerical calculations and manipulations
import numpy as np
import math
import time

# Matplotlib to create the necessary plots
import matplotlib.pyplot as plt
import threading

#################################
import control
import affiche
#import modeles
  
#############LANCEMENT PYBULLET et INITIALISATION du robot et environnement ############
# paramètres pour Pybullet
# the step time
dt = 0.001 # fréquence du simulateur
#
p.connect(p.GUI)
p.setGravity(0, 0, -0.81) #9.81
p.setPhysicsEngineParameter(fixedTimeStep= dt, numSubSteps=1)
# Zoom onto the robot.
p.resetDebugVisualizerCamera(2.0, 50, -35, (0., 0., 0.))
# Disable the gui controller as we don't use them.
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

#p.setRealTimeSimulation(1)

# récupérer le chemin du répertoire courant
path = os.getcwd()
print("Le répertoire courant est : " + path)
# récupérer le nom du répertoire courant
repn = os.path.basename(path)
print("Le nom du répertoire est : " + repn)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")  # Load the ground plane
#
p.setAdditionalSearchPath(repn)
# Chargement du modèle urdf du robot
urdf_path = "staubli/staubli_tx60_support/urdf/tx60l.urdf"
robot= p.loadURDF(urdf_path, [0,0,0], [0,0,0,1],flags=p.URDF_USE_INERTIA_FROM_FILE, useFixedBase=True)
# nj= variable du nombre total de joints du robot (attention: les artuculaires + les fixes)
nj = p.getNumJoints(robot)
##########################################################
# # Information sur le robot, à décommenter si vous voulez
# # nom des joints et des links (liaison et corps) du robot
# #
# print('the robot has: ' + str(nj) + ' joints\n')
# for i in range(nj):
#     info = p.getJointInfo(robot, i)
    
#     # L'index 1 correspond au nom du joint (bytes -> conversion en string)
#     joint_name = info[1].decode('utf-8')
    
#     # L'index 12 correspond au nom du lien enfant associé (bytes -> conversion en string)
#     link_name = info[12].decode('utf-8')
    
#     print(f"Indice du joint/link: {i} | Nom du joint: '{joint_name}' --> Nom du lien: '{link_name}'")
###########################################################################################
# L'urdf montre que seuls les (nj-3) joints correspondent aux articulations controllables
#
nja= nj -3
# On définit la liste des joints controllables
control_joints = list(range(0, nja))   
print("Robot controlable joint",control_joints)
print("nbr de moteur=",len(control_joints))
# On définit le coprs (correspondant au repère de l'OT
ee_index = control_joints[-1]+1
print("EE= ",ee_index)
###########################################################
pis2= np.pi/2
###########################################################
# Variables partagées pour la communication entre les threads
running = True
lock = threading.Lock()
# current_var: variable qui contient le controle envoyé au robot (angles, vitesses...)
current_var = [0.,0.,0.,0.,0.,0.]

##############################################################
# Fonction du Thread de Simulation qui va tourner en arière-plan
# Ici on simule le robot avec un controle en position en lui fournissant
# une liste des joints à atteindre dans la variable globale current_var
##########################################################
def simulation_thread():
    global current_target_pos, current_var, running
    dt = 1. / 240.
    
    while running:
        # Lecture sécurisée de la cible actuelle
        with lock:
            target= current_var
            kp=1.1
            kv=1.0
        # Commande en position
            zero_vec = [0.0] * len(control_joints)
            p.setJointMotorControlArray(robot,
                        control_joints,
                        p.POSITION_CONTROL,
                        targetPositions=target,
                        targetVelocities=zero_vec)
        
        p.stepSimulation()
        time.sleep(dt)

# Lancement du thread de simulation
sim_thread = threading.Thread(target=simulation_thread)
sim_thread.daemon = True
sim_thread.start()
#affiche.draw_coordinate_frame([0,0,0], [0,0,0,1], length=1.0)
affiche.create_coordinate_frame()
#########################################################
print("Simulation lancée en arrière-plan !")
#################################################################
# Création d'un repère visuel qu'on attache à ee_index
#M ultibody constitué de 3 fléches r,v,b
# il n'y à pas de test de collision pour ces 3 corps avec les objets
#
flange_index = ee_index
length = 0.1   
radius = 0.003 

# 1. Récupération de l'état initial
link_state = p.getLinkState(robot, flange_index, computeForwardKinematics=1)
link_pos = link_state[4]
link_ori = link_state[5]

# 2. Création des formes visuelles
ori_x = p.getQuaternionFromEuler([0, math.pi / 2, 0])
vis_x = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=length, rgbaColor=[1, 0, 0, 1], 
                            visualFramePosition=[length / 2, 0, 0], visualFrameOrientation=ori_x)

ori_y = p.getQuaternionFromEuler([-math.pi / 2, 0, 0])
vis_y = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=length, rgbaColor=[0, 1, 0, 1], 
                            visualFramePosition=[0, length / 2, 0], visualFrameOrientation=ori_y)

vis_z = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=length, rgbaColor=[0, 0, 1, 1], 
                            visualFramePosition=[0, 0, length / 2])

# 3. Création des corps (sans contraintes de force)
id_x = p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis_x, baseCollisionShapeIndex=-1, basePosition=link_pos, baseOrientation=link_ori)
id_y = p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis_y, baseCollisionShapeIndex=-1, basePosition=link_pos, baseOrientation=link_ori)
id_z = p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis_z, baseCollisionShapeIndex=-1, basePosition=link_pos, baseOrientation=link_ori)
##############################################################"
# ==========================================
# MISE A JOUR AFFICHAGE REPERE
# ==========================================
# À chaque fois que le robot bouge, il faut synchroniser la position des axes visuels du repère:
def update_repere_outil(robot, flange_index, id_x, id_y, id_z):
    state = p.getLinkState(robot, flange_index, computeForwardKinematics=1)
    pos = state[4]
    ori = state[5]
    
    p.resetBasePositionAndOrientation(id_x, pos, ori)
    p.resetBasePositionAndOrientation(id_y, pos, ori)
    p.resetBasePositionAndOrientation(id_z, pos, ori)###############################

############################################################################################
################ PROGRAMME DE TEST DU MGD avec CORRESPONDANCE q_modèle et q_tx
###############################################################################
target_joints_modele= np.array([[0.,0.,0.,0.,0.,0.],[0.6,0.3,0,0.,0.,0.], [0.3,0.2,pis2,0.,0.,0.], 
                                [0.,0., 0., pis2,0.,0.], [0.,0.,0.,0.,pis2,0.],[0.,0.,0.,0.,0.,pis2]])
#print("Q model=", target_joints_modele[2])
# A FAIRE cnversion Modèle => TX60l ?
target_joints_tx60l = target_joints_modele[1] 
########################################
target_joints = target_joints_tx60l 
try:
    time_start = time.time()

    while time.time() - time_start < 3.0: # Durée de la simulation en secondes
        t = time.time() - time_start
        
        # Mettre à jour la configuration désirée
        current_var = target_joints

        
        with lock:
            # Récupérer la position réelle du robot depuis la simulation
            ee_state = p.getLinkState(robot, ee_index)
#            print("EE=",ee_state)
            actual_pos = ee_state[0] # Coordonnées [x, y, z] mondiales
            
            update_repere_outil(robot, flange_index, id_x, id_y, id_z)
 

   
#    affiche.update_joint_frame(robot, ee_index, frame_ee,0.3)
    plt.pause(0.1) # Laisse respirer le thread principal


except KeyboardInterrupt:
    pass
#
#current_joints # Conversion TX60l => modèle
#####################################################
# A FAIRE: calcul du MGD qui correspond ici à T06

#T06= modeles.mgd_tx60l(current_joints)

############# La pose est donnée par Pybullet avec :
pos = ee_state[0]  # Position (x, y, z)
orn = ee_state[1]  # Orientation (quaternion [x, y, z, w])
print("Pos ee_index=",pos)
print("Ori=", orn)
  
situation, vel_sit= control.get_ee_state(robot, ee_index+1)
print("Situation=", situation)
ma= np.array(p.getMatrixFromQuaternion(orn))
#print(type(ma))
pp= np.array([pos[0],pos[1],pos[2]])
print(pp)
mat=np.array([ma[0:3],ma[3:6],ma[6:9]])
print(mat)
#print("T06=",T06)
# Arrêt propre
running = False
sim_thread.join()
p.disconnect()
plt.ioff()
plt.show()


