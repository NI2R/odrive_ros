#!/usr/bin/env python3

from __future__ import print_function
import odrive
from odrive.enums import *
from time import sleep

print("Recherche Odrive...")
odrv0 = odrive.find_any()
print("Effacement de la configuration précédente")
odrv0.erase_configuration()
sleep(3)
print("Définition du mode Index Signal pour chaque encodeurs")
odrv0.axis0.encoder.config.use_index = True
odrv0.axis1.encoder.config.use_index = True
sleep(0.5)
print("Lancement de la recherche d'index pour chaque encodeur")
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
odrv0.axis1.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
while odrv0.axis0.requested_state is 7 and odrv0.axis1.requested_state is 7:
    sleep(0.5)
print("Définition de l'état pré-calibré pour chaque encodeur")
odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis1.encoder.config.pre_calibrated = True
sleep(0.5)
'''
print("Lancement de la recherche d'index sur chaque encodeur")
odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
odrv0.axis1.requested_state = AXIS_STATE_MOTOR_CALIBRATION
'''
print("Définition de l'état pré-calibré pour chaque moteur")
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis1.motor.config.pre_calibrated = True
sleep(0.5)
print("sauvegarde de la calibration et reboot")
odrv0.save_configuration()
sleep(2)
odrv0.reboot()

"""
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while odrv0.axis0.current_state != 1 and odrv0.axis1.current_state != 1:
    time.sleep(0.1)
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis1.motor.config.pre_calibrated = True
# Met les moteurs en boucle fermée
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
"""
