#!/usr/bin/env python
# -*-coding:Latin-1 -*

from __future__ import print_function
import MCP3008
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
        self.diametreRoue = 77     # Diamètre roue en mm
        self.perimetreRoue = self.diametreRoue * pi  # Périmètre roue en mm
        self.distanceEntreAxe = 280    # entre-axe en mm

        # coding features
        self.errorMax = 1.5      # unité ?
        self.actionFait = False     # Init Action Faite
        self.pos_init_move = 0  # position initialle avant le début d'un déplacement (TRA/ROT)
        self.compteur_deplacement = 0

        # Définition données évitement :
        self.OBS = False
        self.sharp = [0, 1, 2, 3, 4]  # liste des capteurs
        self.SenOn = [0 for i in range(len(self.sharp))]  # liste flag detection pour chaque capteur
        self.Sen_count = 0 # compteur de detection
        self.limite_detection = 500

        # Définition des distances et vitesses :
        self.distance0_mm = 0
        self.distance1_mm = 0
        self.distanceRobot_mm = 0
        self.vitesse0_ms = 0
        self.vitesse1_ms = 0

        # Appel de la classe Robot_properties dans interfaseROS.py :
        self.Robot = Robot_properties()

    def publication(self):

        # Variables locales :
        axis0 = self.odrv0.axis0
        axis1 = self.odrv0.axis1

        """ PUBLICATIONS """
        # Publication distance parcourue
        self.Robot.update_Distance_parc(self.distanceRobot_mm)
        # Publication vitesse moteurs:
        self.vitesse0_ms = - axis0.controller.vel_estimate / (self.perimetreRoue/1000)
        self.vitesse1_ms = axis1.controller.vel_estimate / (self.perimetreRoue/1000)
        print("Vitesse roue gauche (m/s) : %d " % self.vitesse0_ms)
        print("Vitesse roue droite (m/s) : %d " % self.vitesse1_ms)
        self.Robot.update_Vitesse0(self.vitesse0_ms)
        self.Robot.update_Vitesse1(self.vitesse1_ms)

    def stop(self):
        """   POUR ARReTER LES MOTEURS : """

        # Variables locales :
        axis0 = self.odrv0.axis0
        axis1 = self.odrv0.axis1

        # Met la vitessea des roues à 0.
        print("Le robot s'arrête")
        axis0.controller.set_vel_setpoint(0, 0)
        axis1.controller.set_vel_setpoint(0, 0)
        axis0.controller.pos_setpoint = axis0.encoder.pos_estimate
        axis1.controller.pos_setpoint = axis1.encoder.pos_estimate

        # Publication distance parcourue
        self.Robot.update_Distance_parc()

    def evitement(self):

        for i in range(len(self.sharp)):
            if self.senslist[i] is True:
                if MCP3008.readadc(self.sharp[i]) > self.limite_detection :  # a voir:  600 trop de detection #  1000 test
                    self.SenOn[i] = 1
                    self.OBS = True
                    print("Obstacle détécté")
                    self.stop(self)
                    print("Valeur du capteur [%d] vaut : %d ", (self.sharp[i], MCP3008.readadc(self.sharp[i])))

    def wait_end_move(self):

        # Définition des Aliases :
        axis0 = self.odrv.axis0
        axis1 = self.odrv.axis1

        sleep(1)
        wd = 0
        while int(axis0.encoder.vel_estimate) != 0 and int(axis1.encoder.vel_estimate) != 0:
            sleep(0.1)
            wd += 1
            #print("watchdog = %d" % wd)
            if wd > 100:
                break
            self.evitement()

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
        target0 = - (self.nbTics * distance) / self.perimetreRoue
        target1 = (self.nbTics * distance) / self.perimetreRoue

        print("pos_estimate 0: %d" % axis0.encoder.shadow_count)
        print("target0 : %d" % target0)
        print("pos_estimate 1: %d" % axis1.encoder.shadow_count)
        print("target1 : %d" % target1)
        # Début de la translation :

        axis0.controller.move_incremental(target0, True)
        axis1.controller.move_incremental(target1, True)
        self.wait_end_move()

        print("Translation Terminée !")
        print("pos_estimate 0: %d" % axis0.encoder.pos_estimate)
        print("pos_estimate 1: %d" % axis1.encoder.pos_estimate)

        # Distance parcourue par les roues
        self.distance0_mm = - distInit0_mm + (axis0.encoder.pos_estimate * self.perimetreRoue) / self.nbTics
        print("Distance Roue Gauche (mm) : %.4f " % self.distance0_mm)
        self.distance1_mm = - distInit1_mm + (axis1.encoder.pos_estimate * self.perimetreRoue) / self.nbTics
        print("Distance Roue Droite (mm) : %.4f " % self.distance1_mm)
        self.distanceRobot_mm = abs(self.distance0_mm + self.distance1_mm)/2

        # Publication :
        self.publication()

    def rotation(self, angle, senslist):
        ''' [ Fonction qui fait tourner le robot sur lui même
            d'un angle donné en radiant ] '''

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
        self.wait_end_move()

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


    def run(self):

        print("----------------<- 1 ROTATION ->----------------")
        self.rotation(self.Robot.Angle_int, [True, True, True, True, True])
        sleep(0.5)

        print("---------------<- 2 TRANSLATION ->---------------")
        self.translation(self.Robot.Dist_rect, [True, True, True, True, True])
        sleep(0.5)

        print("----------------<- 3 ROTATION ->----------------")
        self.rotation(self.Robot.Angle_fi, [True, True, True, True, True])
        print("====================== FIN DEPLACEMENT n°%d =======================" % self.compteur_deplacement)
        self.compteur_deplacement += 1
