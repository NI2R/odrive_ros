#!/usr/bin/env python
# -*-coding:Latin-1 -*

from __future__ import print_function

import odrive
from odrive.enums import *
from time import sleep
from math import *


class Param:
    def __init__(self):
        print("finding an odrive...")
        self.odrv = odrive.find_any(serial_number="365C33693037")
        print('Odrive found ! ')

    def config(self):
        # 40Amp max dans le moteur (gros couple et sécurité pour pas fumer le moteur)
        self.odrv.axis0.motor.config.current_lim = 40
        self.odrv.axis1.motor.config.current_lim = 40

        # vmax en tick/s les encodeurs font 8192 tick/tours
        # controller.*.vel_limite prend le pas sur trap_traj.*.vel_limt
        self.odrv.axis0.controller.config.vel_limit = 10000
        self.odrv.axis1.controller.config.vel_limit = 10000

        # trap_traj parametrage des valeurs limit du comportement dynamique
        self.odrv.axis1.trap_traj.config.vel_limit = 7000
        self.odrv.axis0.trap_traj.config.vel_limit = 7000

        self.odrv.axis0.trap_traj.config.accel_limit = 5000
        self.odrv.axis1.trap_traj.config.accel_limit = 5000

        self.odrv.axis0.trap_traj.config.decel_limit = 5000
        self.odrv.axis1.trap_traj.config.decel_limit = 5000

    def calib(self):

        # Calibration sauvegardée et lancée au démarrage...
        # A intégrer: Lance la calibration moteur si pas déjà faite
        print("starting calibration...")
        self.odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        self.odrv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while self.odrv.axis0.current_state != 1 and self.odrv.axis1.current_state != 1:
            sleep(0.2)

        # Met les moteurs en boucle fermée
        self.odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def raz_encoders(self):

        # Forcer la position zero des encoders
        print("Remise à zéro des moteurs")
        test = input("Le robot est-il en l'air ? : [ESPACE] pour continuer")
        if test == ' ':
            self.odrv.axis0.controller.move_to_pos(0)
            self.odrv.axis1.controller.move_to_pos(0)
            sleep(1)
            wd = 0
            while int(axis0.encoder.vel_estimate) != 0 and int(axis1.encoder.vel_estimate) != 0:
                sleep(0.01)
                wd += 1
                #print("watchdog = %d" % wd)
                if wd > 200:
                    break

            print("Axis0 pos_estimate = %0f" % self.odrv.axis0.encoder.pos_estimate)
            print("Axis1 pos_estimate = %0f" % self.odrv.axis1.encoder.pos_estimate)
            print('fin de la remise à zéro')

    def reboot(self):

        # redémarrage de la odrive :
        self.odrv.reboot()
