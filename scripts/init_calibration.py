#!/usr/bin/env python3
# -*-coding:Latin-1 -*

from __future__ import print_function
import odrive
import odrive.enums
from time import sleep

print("Recherche Odrive...")
odrv0 = odrive.find_any(serial_number="365C33693037")

""" /!\ PREREQUIS : dans odrivetools lancer 'odrv0.erase_configuration()' & 'odrv0.reboot()' """

print("Définition du mode Index Signal encoders = FALSE")
odrv0.axis0.encoder.config.use_index = False
odrv0.axis1.encoder.config.use_index = False
sleep(0.5)

print(" Définition des limites en courant et tension")
odrv0.axis0.motor.config.calibration_current = 20 # init: 10
odrv0.axis1.motor.config.calibration_current = 20
odrv0.axis0.motor.config.resistance_calib_max_voltage = 4 #init: 2
odrv0.axis0.motor.config.resistance_calib_max_voltage = 4 #init: 2


print("Lancement d'une calibration complète  moteurs + encoders ")
odrv0.axis0.requested_state = 3 #AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis1.requested_state = 3 #AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while odrv0.axis0.current_state != 1 and odrv0.axis1.current_state != 1:
    sleep(0.1)

print("Définition de l'état pre-calibrated moteurs")
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis1.motor.config.pre_calibrated = True
sleep(0.5)

print("Définition de l'état pré-calibred encodeurs")
odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis1.encoder.config.pre_calibrated = True
sleep(0.5)

print("Force Encoder_index_search au demarrage")
odrv0.axis0.config.startup_encoder_offset_calibration = True
odrv0.axis1.config.startup_encoder_offset_calibration = True

print("Force mode Boucle fermée au démarrage")
odrv0.axis0.config.startup_closed_loop_control = True
odrv0.axis1.config.startup_closed_loop_control = True

# Si besoin, pas indispensable
print("Force la calib moteur au démarrage")
odrv0.axis0.config.startup_motor_calibration = True
odrv0.axis1.config.startup_motor_calibration = True

print("sauvegarde de la calibration")
odrv0.save_configuration()
sleep(5)
print("REBOOT!")
odrv0.reboot()
