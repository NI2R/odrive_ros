#!/usr/bin/env python
# -*-coding:Latin-1 -*

from __future__ import print_function

import odrive
from odrive.enums import *  # a checker
import time
from math import *


class Param:
    def __init__(self):
        print("finding an odrive...")
        self.odrv = odrive.find_any()
        print('Odrive found ! ')

    def config(self):
        # 40Amp max dans le moteur (gros couple et sécurité pour pas fumer le moteur)
        self.odrv.axis0.motor.config.current_lim = 5
        self.odrv.axis1.motor.config.current_lim = 5

        # vmax en tick/s les encodeurs font 8192 tick/tours
        # controller.*.vel_limite prend le pas sur trap_traj.*.vel_limt
        self.odrv.axis0.controller.config.vel_limit = 10000
        self.odrv.axis1.controller.config.vel_limit = 10000

        # trap_traj parametrage des valeurs limit du comportement dynamique
        self.odrv.axis1.trap_traj.config.vel_limit = 10000
        self.odrv.axis0.trap_traj.config.vel_limit = 10000

        self.odrv.axis0.trap_traj.config.accel_limit = 7000
        self.odrv.axis1.trap_traj.config.accel_limit = 7000

        self.odrv.axis0.trap_traj.config.decel_limit = 7000
        self.odrv.axis1.trap_traj.config.decel_limit = 7000

        # test avec  calib_saved.py
        # self.odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        # self.odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def calib(self):
        # Fonction de calibration sans condition

        # Lance la calibration moteur si pas déjà faite
        print("starting calibration...")
        self.odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        self.odrv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

        # Attente fin de la calib, et retour état par défaut IDLE_STATE
        while self.odrv.axis0.current_state != 1 and self.odrv.axis1.current_state != 1:
            time.sleep(0.2)

        # Met les moteurs en boucle fermée
        self.odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
