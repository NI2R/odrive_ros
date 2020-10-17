#!/usr/bin/env python
# -*-coding:Latin-1 -*

from __future__ import print_function

import odrive
from odrive.enums import *  # a checker
import time
from math import *


def Test_diametre_roue():

    print("finding an odrive...")
    odrv = odrive.find_any()
    print('Odrive found ! ')

    odrv.axis0.motor.config.current_lim = 5
    odrv.axis1.motor.config.current_lim = 5

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

    # Lance la calibration moteur si pas déjà faite
    print("starting calibration...")
    odrv.axis0.motor.config.pre_calibrated = False
    odrv.axis1.motor.config.pre_calibrated = False
    odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    odrv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    # Attente fin de la calib, et retour état par défaut IDLE_STATE
    while odrv.axis0.current_state != 1 and odrv.axis1.current_state != 1:
        time.sleep(0.2)

    # Met les moteurs en boucle fermée
    odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    print('lancement move_incremental')
    odrv.axis0.controller.move_incremental(8192, False)
    odrv.axis1.controller.move_incremental(8192, False)
    time.sleep(5)


Test_diametre_roue()
