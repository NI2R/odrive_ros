#!/usr/bin/env python
# -*-coding:Latin-1 -*

from __future__ import print_function

import odrive
from odrive.enums import *  # a checker
import time
from math import pi
import param


def Config(odrv):

    odrv.axis0.motor.config.current_lim = 20
    odrv.axis1.motor.config.current_lim = 20

    # vmax en tick/s les encodeurs font 8192 tick/tours
    # controller.*.vel_limite prend le pas sur trap_traj.*.vel_limt
    odrv.axis0.controller.config.vel_limit = 10000
    odrv.axis1.controller.config.vel_limit = 10000

    # trap_traj parametrage des valeurs limit du comportement dynamique
    odrv.axis1.trap_traj.config.vel_limit = 10000
    odrv.axis0.trap_traj.config.vel_limit = 10000

    odrv.axis0.trap_traj.config.accel_limit = 7000
    odrv.axis1.trap_traj.config.accel_limit = 7000

    odrv.axis0.trap_traj.config.decel_limit = 7000
    odrv.axis1.trap_traj.config.decel_limit = 7000

def Calibration(odrv):
    # Lance la calibration moteur


    print("starting calibration...")
    odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    #odrv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    # Attente fin de la calib, et retour état par défaut IDLE_STATE
    while odrv.axis0.current_state != 1 and odrv.axis1.current_state != 1:
        time.sleep(0.2)

    # Met les moteurs en boucle fermée
    odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    #odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(3)


def Test_move_incremental(odrv, distance):

    # définition des données :
    nb_tics = 8192
    diametre_roue_mm = 80
    perimetre_roue_mm = diametre_roue_mm * pi
    distance_tics_G = - (nb_tics * distance) / perimetre_roue_mm
    distance_tics_D = (nb_tics * distance) / perimetre_roue_mm
    errorMax = 2.5

    print('test  move_incremental  allé')
    print("pos_estimate 0: %d" % axis0.encoder.shadow_count)
    print("pos_estimate 1: %d" % axis1.encoder.shadow_count)


    odrv.axis0.controller.move_incremental(distance_tics_G, False)
    odrv.axis1.controller.move_incremental(distance_tics_D, False)

    while axis0.encoder.shadow_count > (distance_tics_G - errorMax) or axis1.encoder.pos_estimate < (distance_tics_D + errorMax):
        sleep(0.001)

    print("pos_estimate 0: %d" % axis0.encoder.shadow_count)
    print("pos_estimate 1: %d" % axis1.encoder.shadow_count)
    print("j'attends 5sec avant de finir")
    time.sleep(5)

def Test_move_to_pos(odrv, distance):

    # définition des données :
    nb_tics = 8192
    diametre_roue_mm = 80
    perimetre_roue_mm = diametre_roue_mm * pi
    distance_tics_G = - (nb_tics * distance) / perimetre_roue_mm
    distance_tics_D = (nb_tics * distance) / perimetre_roue_mm

    print('test move_to_pos(distance_tics)  allé')

    odrv.axis0.controller.move_to_pos(distance_tics_G)
    #odrv.axis1.controller.move_to_pos(distance_tics_D)
    print("pos_estimate 0: %d" % odrv.axis0.encoder.pos_estimate)
    print("pos_estimate 1: %d" % odrv.axis1.encoder.pos_estimate)

    #print("attend idle state")
    #print("control_mode  gauche : ", odrv.axis0.controller.current_state)
    #print("control_mode droite : ", odrv.axis1.controller.current_state)
    #print("_______")
    #while odrv.axis0.controller.control_mode != 1 and odrv.axis1.controller.conrtrol_mode != 1:

    #    print("control_mode gauche : ", odrv.axis0.controller.current_state)
    #    print("control_mode droite : ", odrv.axis1.controller.current_state)
    #    time.sleep(0.2)

    print("j'attends 5sec avant de finir")
    time.sleep(5)
    print("pos_estimate 0: %d" % odrv.axis0.encoder.pos_estimate)
    print("pos_estimate 1: %d" % odrv.axis1.encoder.pos_estimate)

def Test_diametre_roue(odrv):

    print('lancement move_incremental 1 tour de roue')
    odrv.axis0.controller.move_incremental(-8192, False)
    odrv.axis1.controller.move_incremental(8192, False)
    time.sleep(5)


print("finding an odrive...")
odrv = odrive.find_any()
print('Odrive found ! ')
param = param.Param()
param.config()
param.raz_encoders()
Test_move_incremental(odrv,500)
#Test_move_to_pos(odrv,500)
# Test_diametre_roue(odrv)
