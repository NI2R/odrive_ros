#!/usr/bin/env python3
# -*-coding:Latin-1 -*

from __future__ import print_function
import odrive
from odrive.enums import *
from time import sleep

print("Recherche Odrive...")
odrv0 = odrive.find_any()
'''
print("Effacement de la configuration précédente")
odrv0.erase_configuration()
sleep(5)
'''

print("Lancement d'une calibration complète  moteurs + encoders ")
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while odrv0.axis0.current_state != 1 and odrv0.axis1.current_state != 1:
    sleep(0.2)

print("Activation état Boucle fermée")
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
while odrv0.axis0.current_state == 8 and odrv0.axis1.current_state == 8:
    sleep(0.1)

print("Lancement move_to_pos(0) ")
odrv0.axis0.controller.move_to_pos(0)
odrv0.axis1.controller.move_to_pos(0)
sleep(3)
print("Position de l'encoder 0 : %d " % odrv0.axis0.encoder.shadow_count)
print("Position de l'encoder 1 : %d " % odrv0.axis1.encoder.shadow_count)

'''
print("Lancement de la calibration moteurs")
odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
odrv0.axis1.requested_state = AXIS_STATE_MOTOR_CALIBRATION
while odrv0.axis0.current_state is 4 and odrv0.axis1.current_state is 4:
    time.sleep(0.1)
print("Définition de l'état pre-calibrated moteurs")
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis1.motor.config.pre_calibrated = True
sleep(0.5)
'''
print("Effacement de la configuration précédente")
odrv0.erase_configuration()
sleep(5)

print("Définition du mode Index Signal encoders")
odrv0.axis0.encoder.config.use_index = True
odrv0.axis1.encoder.config.use_index = True
sleep(0.5)
print("Lancement de la recherche d'index pour chaque encoders")
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
odrv0.axis1.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
while odrv0.axis0.current_state == 7 and odrv0.axis1.current_state == 7:
    sleep(0.1)
print("Définition de l'état pré-calibred encodeurs")
odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis1.encoder.config.pre_calibrated = True
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
