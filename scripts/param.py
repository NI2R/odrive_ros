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
        self.axis0 = self.odrv0.axis0
        self.axis1 = self.odrv0.axis1

        print('Odrive found ! ')

    def config(self):
        # 40Amp max dans le moteur (gros couple et sécurité pour pas fumer le moteur)
        self.axis0.motor.config.current_lim = 10
        self.axis1.motor.config.current_lim = 10

        # vmax en tick/s les encodeurs font 8192 tick/tours
        # controller.*.vel_limite prend le pas sur trap_traj.*.vel_limt
        self.axis0.controller.config.vel_limit = 10000
        self.axis1.controller.config.vel_limit = 10000

        # trap_traj parametrage des valeurs limit du comportement dynamique
        self.axis1.trap_traj.config.vel_limit = 10000
        self.axis0.trap_traj.config.vel_limit = 10000

        self.axis0.trap_traj.config.accel_limit = 7000
        self.axis1.trap_traj.config.accel_limit = 7000

        self.axis0.trap_traj.config.decel_limit = 7000
        self.axis1.trap_traj.config.decel_limit = 7000

        # test avec  calib_saved.py
        # self.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        # self.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def raz(self):
        # Fonction de remise à zero des moteurs pour initialisation si calib déja faite
        flag = 'N'
        flag = input("Le robot est hors sol ? (Y or N)")
        self.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        if flag == 'Y':
            """self.axis0.controller.config.vel_limit = 7000
            self.axis1.controller.config.vel_limit = 7000

            # trap_traj parametrage des valeurs limit du comportement dynamique
            self.axis1.trap_traj.config.vel_limit = 1000
            self.axis0.trap_traj.config.vel_limit = 1000

            self.axis0.trap_traj.config.accel_limit = 750
            self.axis1.trap_traj.config.accel_limit = 750

            self.axis0.trap_traj.config.decel_limit = 750
            self.axis1.trap_traj.config.decel_limit = 750"""
            # Remise en position 0 des moteurs pour initialisation
            self.axis0.controller.move_to_pos(0)
            self.axis1.controller.move_to_pos(0)
            print("Poser le robot au sol")
            time.sleep(10)

    def calib(self):
        # Fonction de calibration sans condition
        print("starting calibration...")
        self.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        self.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

        #"" fonction sauvegarde calibration""
        while self.axis0.current_state != 1 and self.axis1.current_state != 1:
            time.sleep(0.1)

        # Met les moteurs en boucle fermée
        self.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        # flag calib Done !
        #self.axis0.motor.config.pre_calibrated = True
        #self.axis1.motor.config.pre_calibrated = True

        # permet d'attendre que le tunning soit "réglé", sinon robot dérive
        time.sleep(1)

    def calib_bis(self):

        # test 1 moteur, calib/Tun si pas déjà faite et saved en config
        if self.axis0.encoder.config.pre_calibrated is True and \
           self.axis0.motor.config.pre_calibrated is True:
            # print("ATTENTION! RAZ des moteurs, maintenir le robot en l'air")
            print("ATTENTION! RAZ des moteurs, maintenir le robot en l'air")
            enter = raw_input()
            print(enter)
            time.sleep(1)
            self.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
            self.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            while self.axis0.current_state != 1 and self.axis1.current_state != 1:
                time.sleep(0.1)

        else :
            #
            print("CALIBRATION ! : poser le robot au sol")
            print("starting calibration...")
            self.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            while self.axis0.current_state != 1 and self.axis1.current_state != 1:
                time.sleep(0.1)
            # Met les moteurs en boucle fermée
            self.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

            # flag calib Done !
            self.axis0.encoder.config.pre_calibrated = True
            self.axis0.motor.config.pre_calibrated = True
            self.axis0.config.startup_encoder_index_search = True



    def save_config(self):

        # test pour garder la config de tunning moteurs entre deux runs.
        self.odrv0.save_configuration()


    def unlock_wheels(self):
        # AXIS_STATE_IDLE , libère le moteur : boucle ouverte
        self.axis0.requested_state = AXIS_STATE_IDLE
        self.axis1.requested_state = AXIS_STATE_IDLE
