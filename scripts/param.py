#!/usr/bin/python
# -*-coding:Latin-1 -*

from __future__ import print_function

import odrive
from odrive.enums import *  # a checker
import time
from math import *


class Param:
    def __init__(self):
        print("finding an odrive...")
        self.odrv0 = odrive.find_any()
        odrv = self.odrv0
        print('Odrive found ! ')

    def config(self):
        # 40Amp max dans le moteur (gros couple et sécurité pour pas fumer le moteur)
        odrv.axis0.motor.config.current_lim = 10
        odrv.axis1.motor.config.current_lim = 10

        # vmax en tick/s les encodeurs font 8192 tick/tours
        # controller.*.vel_limite prend le pas sur trap_traj.*.vel_limt
        odrv.axis0.controller.config.vel_limit = 50000
        odrv.axis1.controller.config.vel_limit = 50000

        # trap_traj parametrage des valeurs limit du comportement dynamique
        odrv.axis1.trap_traj.config.vel_limit = 10000
        odrv.axis0.trap_traj.config.vel_limit = 10000

        odrv.axis0.trap_traj.config.accel_limit = 7000
        odrv.axis1.trap_traj.config.accel_limit = 7000

        odrv.axis0.trap_traj.config.decel_limit = 7000
        odrv.axis1.trap_traj.config.decel_limit = 7000

        # test avec  calib_saved.py
        # odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        # odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def raz(self):
        # Fonction de remise à zero des moteurs pour initialisation si calib déja faite
        flag = 'N'
        flag = input("Le robot est hors sol ? (Y or N)")
        odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        if flag == 'Y':
            """odrv.axis0.controller.config.vel_limit = 7000
            odrv.axis1.controller.config.vel_limit = 7000

            # trap_traj parametrage des valeurs limit du comportement dynamique
            odrv.axis1.trap_traj.config.vel_limit = 1000
            odrv.axis0.trap_traj.config.vel_limit = 1000

            odrv.axis0.trap_traj.config.accel_limit = 750
            odrv.axis1.trap_traj.config.accel_limit = 750

            odrv.axis0.trap_traj.config.decel_limit = 750
            odrv.axis1.trap_traj.config.decel_limit = 750"""
            # Remise en position 0 des moteurs pour initialisation
            odrv.axis0.controller.move_to_pos(0)
            odrv.axis1.controller.move_to_pos(0)
            print("Poser le robot au sol")
            time.sleep(10)

    def calib(self):
        # Fonction de calibration sans condition
        print("starting calibration...")
        odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        odrv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

        #"" fonction sauvegarde calibration""
        while odrv.axis0.current_state != 1 and odrv.axis1.current_state != 1:
            time.sleep(0.1)

        # Met les moteurs en boucle fermée
        odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        # flag calib Done !
        odrv0.axis0.motor.config.pre_calibrated = True
        odrv0.axis1.motor.config.pre_calibrated = True

        # permet d'attendre que le tunning soit "réglé", sinon robot dérive
        time.sleep(1)

    def calib_bis(self):

        # test 1 moteur, calib/Tun si pas déjà faite et saved en config
        if odrv.axis0.encoder.config.pre_calibrated is True and \
           odrv.axis0.motor.config.pre_calibrated is True :
            odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(1)
        else :
            # Fonction de calibration sans condition
            print("starting calibration...")
            odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

            # Met les moteurs en boucle fermée
            odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

            # flag calib Done !
            odrv.axis0.encoder.config.pre_calibrated = True
            odrv.axis0.motor.config.pre_calibrated = True


    def save_config(self):

        # test pour garder la config de tunning moteurs entre deux runs.
        while odrv.axis0.current_state != 1 and odrv.axis1.current_state != 1:
            time.sleep(0.1)

        odrv.save_configuration()


    def unlock_wheels(self):
        # AXIS_STATE_IDLE , libère le moteur : boucle ouverte
        odrv.axis0.requested_state = AXIS_STATE_IDLE
        odrv.axis1.requested_state = AXIS_STATE_IDLE
