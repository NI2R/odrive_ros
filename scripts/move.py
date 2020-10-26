#!/usr/bin/env python
# -*-coding:Latin-1 -*

from __future__ import print_function
#import MCP3008
from interfaceROS import Robot_properties
from time import sleep
from math import pi, fabs


class Move:
    def __init__(self, odrv0):  # p1, p2
        # self.Treat = Treatment()
        # self.info_move = self.Treat.step(p1, p2)

        self.odrv = odrv0      # Assignation du odrive

        # Robot physical constant
        self.nbTics = 8192    # Nombre de tics pr un tour d'encoder
        self.diametreRoue = 80     # Diamètre roue en mm
        self.perimetreRoue = self.diametreRoue * pi  # Périmètre roue en mm
        self.distanceEntreAxe = 280    # entre-axe en mm

        # coding features
        self.errorMax = 1.5      # unité ?
        self.OBS = False        # Init  Ostacle Detecté
        self.actionFait = False     # Init Action Faite
        self.SenOn = list()
        self.done = False

        # Appel de la classe Robot_properties dans interfaseROS.py
        self.Robot = Robot_properties()
        #self.Angle_int = self.Robot.Angle_int
        #self.Angle_fi = self.Robot.Angle_fi
        #self.Distance_rect = self.Robot.Distance_rect

    def translation(self, distance, senslist):
        ''' [Fonction qui permet d'avancer droit pour une distance
            donnée en mm] '''

        # Défintion des Aliases
        axis0 = self.odrv.axis0
        axis1 = self.odrv.axis1

        # Définition du type de mouvement (Flag) :
        strMouv = "trans"

        # Def. de la distance parcouru par les roues avant nouveau deplacement
        distInit0_tics = axis0.encoder.pos_estimate
        distInit1_tics = axis1.encoder.pos_estimate

        distInit0_mm = (distInit0_tics * self.perimetreRoue) / self.nbTics
        distInit1_mm = (distInit1_tics * self.perimetreRoue) / self.nbTics

        print("Lancement d'une Translation de %.0f mm" % distance)

        # Définition de la distance à parcourir en tics vis à vis de la position actuelle avec le moteur de gauche:
        #target0 = - (axis0.encoder.shadow_count + self.nbTics * distance) / self.perimetreRoue
        #target1 = (axis0.encoder.shadow_count + self.nbTics * distance) / self.perimetreRoue
        target0 = - (self.nbTics * distance) / self.perimetreRoue
        target1 = (self.nbTics * distance) / self.perimetreRoue

        print("pos_estimate 0: %d" % axis0.encoder.shadow_count)
        print("target0 : %d" % target0)
        print("pos_estimate 1: %d" % axis1.encoder.shadow_count)
        print("target1 : %d" % target1)
        # Début de la translation :
        axis0.controller.move_incremental(target0, True)
        axis1.controller.move_incremental(target1, True)
        sleep(1)
        wd = 0
        while int(axis0.encoder.vel_estimate) != 0 and int(axis1.encoder.vel_estimate) != 0:
            sleep(0.01)
            wd += 1
            #print("watchdog = %d" % wd)
            if wd > 200:
                break

        # fonction pour réguler la fonction move_to_pos(nb_tics_distance)
        #self.wait_end_move(strMouv, axis0, target0, self.errorMax)
        #self.wait_end_move(strMouv, axis1, target1, self.errorMax)
        print("Translation Terminée !")
        print("pos_estimate 0: %d" % axis0.encoder.pos_estimate)
        print("pos_estimate 1: %d" % axis1.encoder.pos_estimate)

        sleep(1)

        # Distance parcourue par les roues
        distanceFinale0 = - distInit0_mm + (axis0.encoder.pos_estimate * self.perimetreRoue) / self.nbTics
        print("Distance Roue Gauche (mm) : %.4f " % distanceFinale0)
        distanceFinale1 = - distInit1_mm + (axis1.encoder.pos_estimate * self.perimetreRoue) / self.nbTics
        print("Distance Roue Droite (mm) : %.4f " % distanceFinale1)

    def rotation(self, angle, senslist):
        ''' [ Fonction qui fait tourner le robot sur lui même
            d'un angle donné en degré ] '''

        """ --- Variables Locales : --- """
        # Définition des Aliases :
        axis0 = self.odrv.axis0
        axis1 = self.odrv.axis1

        # Convertion angle en degré :
        angleDeg = angle * 180 / pi

        print("Lancement d'une Rotation de %.2f°" % angleDeg)
        # calcul des ticks/pas à parcourir pour tourner

        # distance angulaire en tics avec angle en radiant
        distAngulaire = (self.distanceEntreAxe/2) * angle * self.nbTics / self.perimetreRoue
        print("fraction de tour de roue = %.2f" % (distAngulaire / self.nbTics))
        #angleRobot = (distAngulaire * self.perimetreRoue * pi)/ ((self.distanceEntreAxe/2) * self.nbTics * angleDeg)
        #print("angle parcourue par le robot = %.2f" % angleRobot)

        # Assignation de values avec valeur du capteur IR
        # values = MCP3008.readadc(1)
        axis0.controller.move_incremental(distAngulaire, False)
        axis1.controller.move_incremental(distAngulaire, False)
        sleep(1)
        wd = 0
        while int(axis0.encoder.vel_estimate) != 0 and int(axis1.encoder.vel_estimate) != 0:
            sleep(0.01)
            wd += 1
            #print("watchdog = %d" % wd)
            if wd > 200:
                break

        print("pos_estimate 0: %d" % axis0.encoder.pos_estimate)
        print("pos_estimate 1: %d" % axis1.encoder.pos_estimate)
        #angleRobotFin = (axis0.encoder.pos_estimate * self.perimetreRoue * pi)/ ((self.distanceEntreAxe/2) * self.nbTics * angleDeg)
        #print("angle parcourue par le robot = %.2f" % angleRobotFin)


            # fonction lié à l'OAS
        # elif self.OBS is True and self.actionFait is False:
        #     self.stop()
        #     sleep(0.5)
        #     self.OBS = False
        #     print("Rotation : Obstacle")
        # else:
        #     print("Rotation Terminée !")
        #     self.actionFait = False
        #self.actionFait = False
        sleep(1)

    def stop(self):

        # Variables locales :
        axis0 = self.odrv0.axis0
        axis1 = self.odrv0.axis1

        # Met la vitessea des roues à 0.
        print("Le robot s'arrête")
        # axis0.controller.speed(0)
        # axis1.controller.speed(0)
        """ ou  POUR ARReTER LES MOTEURS : """

        axis0.controller.set_vel_setpoint(0, 0)
        axis1.controller.set_vel_setpoint(0, 0)
        axis0.controller.pos_setpoint = axis0.encoder.pos_estimate
        axis1.controller.pos_setpoint = axis1.encoder.pos_estimate

    def run(self):

        print("----------------<- 1 ROTATION ->----------------")
        self.rotation(self.Robot.Angle_int, [False, False, False, False, False])
        sleep(0.5)
        print("---------------<- 2 TRANSLATION ->---------------")
        self.translation(self.Robot.Dist_rect, [True, True, True, False, False])
        sleep(0.5)
        print("----------------<- 3 ROTATION ->----------------")
        self.rotation(self.Robot.Angle_fi, [False, False, False, False, False])
        print("=================================================")
