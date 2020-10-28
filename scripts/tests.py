#!/usr/bin/env python
# -*-coding:Latin-1 -*

from __future__ import print_function

import odrive
from odrive.enums import *  # a checker
import time
from math import pi
import param
import MCP3008


def Config(odrv):

    odrv.axis0.motor.config.current_lim = 20
    odrv.axis1.motor.config.current_lim = 20

    # vmax en tick/s les encodeurs font 8192 tick/tours
    # controller.*.vel_limite prend le pas sur trap_traj.*.vel_limt
    odrv.axis0.controller.config.vel_limit = 10000
    odrv.axis1.controller.config.vel_limit = 10000

    # trap_traj parametrage des valeurs limit du comportement dynamique
    odrv.axis1.trap_traj.config.vel_limit = 7000
    odrv.axis0.trap_traj.config.vel_limit = 7000

    odrv.axis0.trap_traj.config.accel_limit = 5000
    odrv.axis1.trap_traj.config.accel_limit = 5000

    odrv.axis0.trap_traj.config.decel_limit = 5000
    odrv.axis1.trap_traj.config.decel_limit = 5000


def Calibration(odrv):
    # Lance la calibration moteur
    # a faire si pas déjà prevu au demarrage


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
    distInitG_mm = (distance_tics_G * perimetre_roue_mm) / nb_tics
    distInitD_mm = (distance_tics_D * perimetre_roue_mm) / nb_tics
    errorMax = 2.5
    print('___________________________________________')
    print('--------- test  move_incremental ----------')
    print("pos_estimate 0: %d" % odrv.axis0.encoder.shadow_count)
    print("pos_estimate 1: %d" % odrv.axis1.encoder.shadow_count)
    print('---')
    distanceInit_G = odrv.axis0.encoder.shadow_count * perimetre_roue_mm / nb_tics
    print("Distance Initiale Roue Gauche (mm) : %.4f " % distanceInit_G)
    distanceInit_D = odrv.axis1.encoder.shadow_count * perimetre_roue_mm / nb_tics
    print("Distance Initiale Roue Droite (mm) : %.4f " % distanceInit_D)

    print('----- depart mvt -----')
    odrv.axis0.controller.move_incremental(distance_tics_G, False)
    odrv.axis1.controller.move_incremental(distance_tics_D, False)
    time.sleep(1)
    wd = 0
    while int(odrv.axis0.encoder.vel_estimate) != 0 and int(odrv.axis1.encoder.vel_estimate) != 0:
        time.sleep(0.01)
        wd += 1
        #print("watchdog = %d" % wd)
        if wd > 1000:
            break


    print('----- fin mvt -----')
    print("pos_estimate 0: %d" % odrv.axis0.encoder.shadow_count)
    print("pos_estimate 1: %d" % odrv.axis1.encoder.shadow_count)
    print('---')
    distanceFinaleG = odrv.axis0.encoder.shadow_count * perimetre_roue_mm / nb_tics
    print("Distance finale Roue Gauche (mm) : %.4f " % distanceFinaleG)
    distanceFinaleD = odrv.axis1.encoder.shadow_count * perimetre_roue_mm / nb_tics
    print("Distance Roue Droite (mm) : %.4f " % distanceFinaleD)
    print('---')
    time.sleep(1)
    print('Delta pos RG (mm) = %.3f' % (distanceFinaleG - distanceInit_G))
    print('Delta pos RD (mm) = %.3f' % (distanceFinaleD - distanceInit_D))


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


def Test_OAS(odrv, distance, senslist):

    # définition des données :
    nb_tics = 8192

    diametre_roue_mm = 80
    perimetre_roue_mm = diametre_roue_mm * pi
    distance_tics_G = - (nb_tics * distance) / perimetre_roue_mm
    distance_tics_D = (nb_tics * distance) / perimetre_roue_mm
    distInitG_mm = (distance_tics_G * perimetre_roue_mm) / nb_tics
    distInitD_mm = (distance_tics_D * perimetre_roue_mm) / nb_tics
    errorMax = 2.5

    # Définition données OAS ;
    OBS = False
    sharp = [0, 1, 2, 3, 4]  # liste des capteurs
    SenOn = [0 for i in range(len(sharp))]  # liste flag detection pour chaque capteur
    #Sen_count = 0 # conteur de detection
    limite_detection = 500

    print('___________________________________________')
    print('--------- test  move_incremental ----------')
    print("pos_estimate 0: %d" % odrv.axis0.encoder.shadow_count)
    print("pos_estimate 1: %d" % odrv.axis1.encoder.shadow_count)
    print('---')
    distanceInit_G = odrv.axis0.encoder.shadow_count * perimetre_roue_mm / nb_tics
    print("Distance Initiale Roue Gauche (mm) : %.4f " % distanceInit_G)
    distanceInit_D = odrv.axis1.encoder.shadow_count * perimetre_roue_mm / nb_tics
    print("Distance Initiale Roue Droite (mm) : %.4f " % distanceInit_D)

    print('----- depart mvt -----')
    odrv.axis0.controller.move_incremental(distance_tics_G, False)
    odrv.axis1.controller.move_incremental(distance_tics_D, False)
    time.sleep(.5)
    wd = 0
    while int(odrv.axis0.encoder.vel_estimate) != 0 and int(odrv.axis1.encoder.vel_estimate) != 0:
        time.sleep(0.01)
        # Watchdog pour sortir du while
        wd += 1
        if wd > 100:
            break

        for i in range(len(sharp)):
            if senslist[i] is True:
                if MCP3008.readadc(sharp[i]) > limite_detection :  # a voir:  600 trop de detection #  1000 test
                    SenOn[i] = 1
                    print("Obstacle détécté")
                    stop()
                    print("Valeur du capteur [%d] vaut : %d ", (sharp[i], MCP3008.readadc(sharp[i])))


def stop(self):
    # Met la vitessea des roues à 0.
    print("Le robot s'arrête")
    #self.odrv0.axis0.controller.speed(0)
    #self.odrv0.axis1.controller.speed(0)
    """ ou  POUR ARReTER LES MOTEURS : """

    self.odrv0.axis0.controller.set_vel_setpoint(0, 0)
    self.odrv0.axis1.controller.set_vel_setpoint(0, 0)
    self.odrv0.axis0.controller.pos_setpoint = self.odrv0.axis0.encoder.pos_estimate
    self.odrv0.axis1.controller.pos_setpoint = self.odrv0.axis1.encoder.pos_estimate


def main(param):

    param = param.Param()
    param.config()
    #param.raz_encoders()

    for i in range(0,10):
        #Test_move_incremental(param.odrv,500)
        Test_OAS(param.odrv, 500, [True, True, True, True, True])

        #Test_move_to_pos(odrv,500)
        # Test_diametre_roue(odrv)


if __name__ == '__main__':
    main()
