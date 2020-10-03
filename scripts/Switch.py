#!/usr/bin/env python3
import Jetson.GPIO as GPIO
from time import sleep
# Set up du board (reférence n° port)
GPIO.setmode(GPIO.BOARD)

# definition de la pin comme pin d'entrée
GPIO.setup(7, GPIO.IN)
GPIO.setup(11, GPIO.IN)
#GPIO.setup(12, GPIO.OUT)

# initialisation de l'expérience => TEST
#GPIO.output(12, GPIO.LOW)

# Lecture de la Pin
# GPIO.input(7) # GPAIO 4
# GPIO.input(11) # GPAIO 17


def experience():
    GPIO.output(12, GPIO.HIGH)


# Attente de réponse => TODO: test
def tirette():
    Tirrette = [0 for i in range(10)]  # niveau bas = 0
    while 1:
        print("attente tirette")
        for i in range(0, 10):
            if GPIO.input(7) == 0:  # boucle de check tirette relevée
                Tirrette[i] = 0

            else:
                Tirrette[i] = 1

            if i == 9:
                compteur = 0
                for i in Tirrette:
                    compteur += i
                if compteur >= 5:
                    return
        sleep(1)


def cote():
    if(GPIO.input(11) == 1): # TODO: Validé
        print("Jaune")
        return True

    if(GPIO.input(11) == 0): # TODO: Validé
        print("Violet")
        return False

def main():
    tirette()
    cote()



if __name__ == '__main__':
    main()
