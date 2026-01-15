#!/usr/bin/env micropython
# coding: utf-8

# Import the EV3-robot library
from ev3dev2.motor import MediumMotor, LargeMotor
from ev3dev2.sensor.lego import ColorSensor, UltrasonicSensor, TouchSensor
from ev3dev2.button import Button
from ev3dev2.led import Leds 
from ev3dev2.sound import Sound
# Import time
import time
import json


# Appels des autres classes
from motorization import Motorization
from steering import Steering
from PID import PID



class Car:
    """
    Une classe représentant notre voiture suiveuse de ligne.
    """
    # Constructor
    def __init__(self, name = ''):
        self._name = name
        self.btn = Button()
        self.leds = Leds()
        self.sound = Sound()
        self.voice_options = '-a 200 -s 100 -v fr+f2' # voir https://sites.google.com/site/ev3devpython/learn_ev3_python/sound 
        # Capteur de couleur
        self.cs = ColorSensor()
        # Capteur de toucher
        self.ts = TouchSensor()
        self.light = None               # Initialisation de la partie claire à None
        self.dark = None                # Initialisation de la partie foncée à None
        # motorization
        self._motorization = Motorization(self)
        self._pid = PID(0, 0, 0)
        self._plots = {}
        
        # steering
        self._steering = Steering(self)
        
    def set_name(self, name):
        """
        Set the name of the follower
        """
        self.name = name

    def get_name(self):
        """Return the name of the car
        """
        return self._name

    def get_motorization(self):
        return self._motorization

    def get_steering(self):
        return self._steering

    def get_pid(self):
        return self._pid

    def get_threshold(self):
        """
        Retourne la valeur de consigne, None si non calculable
        """
        if self.light is not None and self.dark is not None:
            return (self.light+self.dark)//2

    def speak(self, message, display = True):
        """
        Speak the message.
        if display is True, print the message
        """
        if display:
            print(message)
        self.sound.speak(message, espeak_opts = self.voice_options)

    def launch(self, speed = 100, measures = False):
        pid = self.get_pid()
        cs = self.cs
        motorization = self.get_motorization()
        steering = self.get_steering()
        if not self.get_threshold():
            self.calibrate()
        pid.SetPoint = self.get_threshold()
        pid.setSampleTime(0.01)  # Limiter les updates PID à 100 Hz
        motorization.run(speed)
        if measures:
            self._plots['Kp'] = self.get_pid().Kp
            self._plots['Ki'] = self.get_pid().Ki
            self._plots['Kd'] = self.get_pid().Kd
            self._plots['threshold'] = self.get_threshold()
            self._plots['datas'] = []
        start = time.time()
        loop_count = 0  # Compteur pour mesurer la fréquence
        while not self.ts.is_pressed:
            motorization.run(speed)
            feedback = cs.reflected_light_intensity
            loop_count += 1  # Incrémenter le compteur
            if measures:
                now = time.time()
                self._plots['datas'].append((now, feedback))
            pid.update(feedback)
            steering.turn(pid.output, speed=100)
            time.sleep(0.01)  # Ajouter un petit délai pour éviter une boucle trop serrée
        elapsed_time = time.time() - start
        # Calcul et affichage de la fréquence
        if elapsed_time > 0:
            frequency = loop_count / elapsed_time
            print(f"Fréquence des appels à reflected_light_intensity : {frequency:.2f} Hz")
        #print("Durée : "+str(elapsed_time))
        motorization.stop()
        with open('json_data.json', 'w') as outfile:
            json_string = json.dumps(self._plots)
            json.dump(json_string, outfile)

    def show_config(self):
        """Affichage des paramètres de configuration du véhicule"""
        print("########################\n# PARAMETRES\n########################")
        print("\nDIRECTION\n------------------------")
        print("Angle maximal de rotation du moteur de direction : " + str(self.get_steering().get_max_angle()) + "°")
        print("Rapport de division entre l'angle du moteur et l'angle des roues : " + str(self.get_steering()._steering_divider))
        print("\nCAPTEUR DE COULEURS\n------------------------")
        print("Valeur de la partie claire : " + str(self.light))
        print("Valeur de la partie foncée : " + str(self.dark))
        print("\nPID\n------------------------")
        print("Valeur de consigne : " + str(self.get_threshold()))
        print("Kp : " + str(self.get_pid().Kp))
        print("Ki : " + str(self.get_pid().Ki))
        print("Kd : " + str(self.get_pid().Kd))


    ###############################################
    #
    #  Interface de configuration
    #
    ###############################################

    def configure(self):
        liste = (('1',"Configuration de la direction"),
                 ('2',"Vérification de la motorisation et du différentiel."),
                 ('3',"Configuration du capteur de lumière"),
                 ('4',"Configuration du PID"),
                 ('5',"Affichage de la configuration"),
                 ('6',"Executer le suiveur de ligne"),
                 ('Q',"Quitter"),
        )
        quitter = False
        self.get_motorization().stop()
        while not quitter:
            print("\n##################################\n Menu Principal \n##################################\n")
            for char, description in liste:
                print(char + ' : ' +description)
            choix = input("Votre choix : ")

            if choix == '1':
                self._configure_steering()
            elif choix == '2':
                self._configure_motorization()
            elif choix == '3':
                self._configure_light_sensor()
            elif choix == '4':
                self._configure_PID()
            elif choix == '5':
                self.show_config()
            elif choix == '6':
                self._run_line_follower()
            elif choix == 'Q' or choix == 'q':
                quitter = True
            time.sleep(0.1)


    def _configure_steering(self):
        print("\n" + 30*'-' + "\nConfiguration de la direction\n" + 30*'-')

        steering = self.get_steering()
        steering.set_max_angle(720)
        max_angle = False
        while not max_angle:
            angle = int(input("Entrez l'angle de rotation du moteur de direction : "))
            steering.turn(angle)
            time.sleep(1)
            steering.turn(-angle)
            time.sleep(1)
            steering.turn(0)
            new_angle = input("Voulez vous tester un nouvel angle (Y/n)? : ")
            if new_angle == 'N' or new_angle == 'n':
                steering.set_max_angle(abs(angle))
                print("=> Affectation de l'angle maximal du moteur de direction : " + str(steering.get_max_angle()))
                max_angle = True

        max_angle = steering.get_max_angle()
        steering.turn(0)
        time.sleep(1)
        print("Angle maximal de rotation du moteur de direction : " + str(max_angle) )
        steering.turn(max_angle)
        print("Mesurer l'angle de rotation roues.")
        wheel_angle = int(input("Entrer l'angle de rotation des roues : "))
        divider = steering.get_max_angle()/wheel_angle
        steering._steering_divider = divider
        print("=> Rapport de division : " + str(divider))
        steering.turn(0)
        self.configure()

    def _configure_motorization(self):
        print("\n" + 30*'-' + "\nVérification de la motorisation et du différentiel\n" + 30*'-')
        motors = self.get_motorization()
        ml = motors.get_left_motor()
        mr = motors.get_right_motor()
        print("\n### Vérification du moteur gauche")
        for v in range(0, 110, 10):
            print(str(v)+"%")
            ml.on_for_seconds(speed=-v, seconds=1)
        print("\n### Vérification du moteur droit")
        for v in range(0, 110, 10):
            print(str(v)+"%")
            mr.on_for_seconds(speed=-v, seconds=1)
        print("\n### Vérification du différentiel\n")
        steering = self.get_steering()
        max_angle = steering.get_max_angle()
        divider = steering.get_steering_divider()
        if divider == None:
            print(" /!\ Vous devez configurer la direction /!\ ")
        else:
            print("Vérifiez que la roue arrière gauche tourne plus vite.")
            steering.turn(max_angle)
            motors.run(100)
            time.sleep(5)
            print("Vérifiez que la roue arrière droite tourne plus vite.")
            steering.turn(-max_angle)
            motors.run(100)
            time.sleep(5)
            print("Arrêt des moteurs.")
            steering.turn(0)
            motors.stop()
        self.configure()

    def _configure_light_sensor(self):
        calibration_OK = False
        while not calibration_OK:
            self.calibrate()
            answer_calibration = input("Voulez vous refaire une calibration (Y/n)? : ")
            if answer_calibration == 'N' or answer_calibration == 'n':
                calibration_OK = True
        self.configure()

    def _configure_PID(self):
        set_PID = False
        while not set_PID:
            default_kp = self.get_pid().Kp
            kp = input("Entrer la valeur du KP [%s] : "%(default_kp))
            if kp == '':
                kp = default_kp
            kp = float(kp)
            self.get_pid().Kp = kp
            default_ki = self.get_pid().Ki
            ki = input("Entrer la valeur du KI [%s] : "%(default_ki))
            if ki == '':
                ki = default_ki
            ki = float(ki)
            self.get_pid().Ki = ki
            default_kd = self.get_pid().Kd
            kd = input("Entrer la valeur du KD [%s] : "%(default_kd))
            if kd == '':
                kd = default_kd
            kd = float(kd)
            self.get_pid().Kd = kd
            answer_pid = input("Voulez vous reconfigurer le PID (Y/n)? : ")
            if answer_pid == 'N' or answer_pid == 'n':
                set_PID = True
        self.configure()

    def _run_line_follower(self):
        print("\n" + 30*'-' + "\nExecution du suiveur de ligne\n" + 30*'-')
        speed = int(input("Entrer la vitesse entre 0 et 100 : "))
        self.launch(speed)

    

    def calibrate(self):
        """
        Launch the calibration of the color sensor
        """

        def calibrate_light(state):
            if state:
                self._calibrate_zone(1)

        def calibrate_dark(state):
            if state:
                self._calibrate_zone(0)

        self.cs.MODE_REFLECT = 'REFLECT'

        self.leds.all_off()
        self.speak("Calibration du capteur de couleur.")
        self.leds.set_color('LEFT', 'GREEN')
        print("Mettez le capteur de couleur sur la partie claire et appuyez sur le bouton gauche pour démarrer.")
        self.light = None
        while self.light == None:
            self.btn.on_left = calibrate_light
            self.btn.process()
            time.sleep(0.01)
        print("Mettez le capteur de couleur sur la partie foncée et appuyez sur le bouton gauche pour demarrer.")
        self.dark = None
        while self.dark == None:
            self.btn.on_left = calibrate_dark
            self.btn.process()
            time.sleep(0.01)
        if self.get_name():
            self.speak("Calibration de " + self.get_name() + " terminée")
        else:
            self.speak("Calibration terminée")
        self.speak("La valeur de consigne est "+str(self.get_threshold()))

    def _calibrate_zone(self, zone):
        """
        Calibration de la zone.
        zone : 1 pour la partie claire, 0 pour la partie foncée
        """
        self.leds.all_off()
        measures = []
        for i in range(5):
            self.leds.set_color('RIGHT', 'GREEN')
            time.sleep(0.2)
            measure = self.cs.reflected_light_intensity
            print(measure)
            measures.append(measure)
            self.leds.set_color('RIGHT', 'BLACK')
            time.sleep(0.2)
        # calcul de la moyenne des mesures
        average = int(sum(measures)/len(measures))
        # Partie claire
        if zone == 1:
            self.light = average
            message = "Partie claire : "+str(self.light)
        # Partie foncée
        elif zone == 0:
            self.dark = average
            message = "Partie foncée : "+str(self.dark)
        self.speak(message)
