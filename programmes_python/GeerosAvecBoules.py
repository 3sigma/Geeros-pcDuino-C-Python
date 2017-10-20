#!/usr/bin/python
# -*- coding: utf-8 -*-

##################################################################################
# Programme de pilotage du robot Geeros (avec boules stabilisatrices),
# disponible à l'adresse:
# http://boutique.3sigma.fr/12-robots
#
# Auteur: 3Sigma
# Version 3.0 - 01/10/2017
##################################################################################

# Importe les fonctions Arduino pour Python
from pyduino_pcduino import *

# Imports pour l'IMU sur bus i2c
import smbus
import FaBo9Axis_MPU9250
import struct

import time, sched
import os
import threading
import signal
import json
import sys

# Pour la détection d'adresse IP
import socket
import fcntl
import struct

# Pour le serveur de Websocket
import tornado.httpserver
import tornado.ioloop
from tornado.ioloop import PeriodicCallback
import tornado.web
import tornado.websocket
import tornado.template

# Pour le client Websocket
import websocket

# Nom de l'hostname (utilisé ensuite pour savoir sur quel système
# tourne ce programme)
hostname = socket.gethostname()

# Imports pour la communication i2c avec l'Arduino Pro Mini
from uno import Uno
uno = Uno(hostname = hostname)

# Entete declarative
directionMoteurDroit = 7
pwmMoteurDroit = 6

directionMoteurGauche = 4
pwmMoteurGauche = 5

Nmoy = 10
codeurDroitDeltaPos = 0
codeurDroitDeltaPosPrec = 0
codeurGaucheDeltaPos = 0
codeurGaucheDeltaPosPrec = 0
omegaDroit = 0
omegaGauche = 0


servocam = Servo()

tensionBatterie = 7.4

# Les moteurs sont asservis en vitesse grâce à un régulateur de type PID
# On déclare ci-dessous les variables et paramètres nécessaires à l'asservissement et au régulateur
R = 0.045 # Rayon d'une roue
W = 0.14 # Largeur du robot
umax = 6. # valeur max de la tension de commande du moteur
umin = -6. # valeur min (ou max en négatif) de la tension de commande du moteur
vxmes = 0. # vitesse longitudinale mesurée
ximes = 0. # vitesse de rotation mesurée
Tf = 0.02 # constante de temps de filtrage de l'action dérivée du PID
Kpvx = 1. # gain proportionnel pour l'asservissement de vitesse longitudinale
Kivx = 10. # gain intégral pour l'asservissement de vitesse longitudinale
Kdvx = 0.00 # gain dérivé pour l'asservissement de vitesse longitudinale
Kpxi = 0.1 # gain proportionnel pour l'asservissement de rotation
Kixi = 1. # gain intégral pour l'asservissement de rotation
Kdxi = 0.000 # gain dérivé pour l'asservissement de rotation
commande_avant_sat_vx = 0. # commande avant la saturation pour l'asservissement de vitesse longitudinale
commande_vx = 0. # commande pour l'asservissement de vitesse longitudinale
commande_avant_sat_xi = 0. # commande avant la saturation pour l'asservissement de rotation
commande_xi = 0. # commande pour l'asservissement de rotation
P_vx = 0. # action proportionnelle pour l'asservissement de vitesse longitudinale
I_vx = 0. # action intégrale pour l'asservissement de vitesse longitudinale
D_vx = 0. # action dérivée pour l'asservissement de vitesse longitudinale
P_xi = 0. # action proportionnelle pour l'asservissement de rotation
I_xi = 0. # action intégrale pour l'asservissement de rotation
D_xi = 0. # action dérivée pour l'asservissement de rotation
commandeDroit = 0. # commande en tension calculée par le PID pour le moteur droit
commandeGauche = 0. # commande en tension calculée par le PID pour le moteur gauche
yprecvx = 0. # Mesure de la vitesse longitudinale au calcul précédent
yprecxi = 0. # Mesure de la vitesse de rotation au calcul précédent
codeurDroitDeltaPosPrec = 0.
codeurGaucheDeltaPosPrec = 0.

# Variables intermédiaires
Ti = 0.
ad = 0.
bd = 0.

# Variables utilisées pour les données reçues
x1 = 0.
x2 = 0.
servoref = 45.
Kp2 = 1.
Ki2 = 1.
Kd2 = 1.
Kpxi2 = 1.
Kixi2 = 1.
Kdxi2 = 1.

# Déclarations pour les consignes de mouvement
vxref = 0.
xiref = 0.


# Time out de réception des données
timeout = 2
timeLastReceived = 0
timedOut = False

T0 = time.time()
dt = 0.01
tprec = time.time()
i = 0
# Création d'un scheduler pour exécuter des opérations à cadence fixe
s = sched.scheduler(time.time, time.sleep)

# Lecture de la tension d'alimentation
idecimLectureTension = 0
decimLectureTension = 6000
tensionAlim = 7.4
# Sécurité sur la tension d'alimentation
tensionAlimMin = 6.4;


#--- setup --- 
def setup():
    global servoref, servocam
    
    pinMode(directionMoteurDroit, OUTPUT)
    pinMode(pwmMoteurDroit, OUTPUT)
    
    pinMode(directionMoteurGauche, OUTPUT)
    pinMode(pwmMoteurGauche, OUTPUT)
    
    pinMode(13, OUTPUT)

    # Attache le servomoteur à la broche 10
    servocam.attach(10)
    servocam.write(servoref)
            
    
  
    CommandeMoteurDroit(0, tensionBatterie)
    CommandeMoteurGauche(0, tensionBatterie)
    
    # La mesure de la tension d'alimentation se fait via un pont diviseur de rapport 3 / 13
    # Les entrées analogiques A0 et A1 sont sur 6 bits avant une plage de variation de 2V
    # La résolution obtenue avec la commande suivante est donc très mauvaise
    #tensionAlimBrute = analogReadmV(A0)
    #tensionAlim = max(tensionAlimMin, float(tensionAlimBrute) * 13. / 3.) / 1000.
    # On préfère lire la tension à partir de la mesure faire par la carte Iteaduino Uno
    try:
        tensionAlimBrute = uno.analog_read(0)
        tensionAlimAvantMax = 3.3 * tensionAlimBrute * (13. / 3.) / 1024.;
        tensionAlim = max(tensionAlimMin, tensionAlimAvantMax);
        print "Tension d'alimentation", tensionAlim
    except:
        print "Probleme lecture tension d'alimentation"
        pass

    
# -- fin setup -- 
 
# -- loop -- 
def loop():
    global i
    i = i+1
    s.enterabs( T0 + (i * dt), 1, CalculVitesse, ())
    s.run()
# -- fin loop --


def CalculVitesse():
    global ticksCodeurDroit, ticksCodeurGauche, indiceTicksCodeurDroit, indiceTicksCodeurGauche, started, \
        omegaDroit, omegaGauche, ticksCodeurDroitTab, ticksCodeurGaucheTab, codeurDroitDeltaPosPrec, codeurGaucheDeltaPosPrec, \
        ad, P_vx, I_vx, D_vx, P_xi, I_xi, D_xi, bd, Ti, yprecvx, yprecxi, timeLastReceived, timeout, timedOut, \
        codeurDroitDeltaPos, codeurGaucheDeltaPos, commandeDroit, commandeGauche, vxmes, ximes, vxref, xiref, dt2, tprec, \
        idecimLectureTension, decimLectureTension, tensionAlim
        
        
    # Mesure de la vitesse du moteur grâce aux codeurs incrémentaux
    try:        
        codeurDroitDeltaPos = uno.read_codeurDroitDeltaPos()
        
        codeurDroitDeltaPosPrec = codeurDroitDeltaPos
    except:
        #print "Erreur lecture codeur droit"
        codeurDroitDeltaPos = codeurDroitDeltaPosPrec
        pass
    
    try:        
        codeurGaucheDeltaPos = uno.read_codeurGaucheDeltaPos()
        
        codeurGaucheDeltaPosPrec = codeurGaucheDeltaPos
    except:
        #print "Erreur lecture codeur gauche"
        codeurGaucheDeltaPos = codeurGaucheDeltaPosPrec
        pass
    
    # C'est bien dt qu'on utilise ici et non pas dt2 (voir plus loin l'explication de dt2)
    # car codeurDroitDeltaPos et codeurGaucheDeltaPos sont mesurés en temps-réel par le micro-contrôleur
    # Atmega328 qui est présent dans Geeros
    omegaDroit = -2 * ((2 * 3.141592 * codeurDroitDeltaPos) / 1632) / (Nmoy * dt)  # en rad/s
    omegaGauche = 2 * ((2 * 3.141592 * codeurGaucheDeltaPos) / 1632) / (Nmoy * dt)  # en rad/s

    # Application de la consigne lue
    vxref = x1
    xiref = x2

    # Définition des entrées de la fonction d'asservissement
    vxmes = (omegaDroit + omegaGauche)*R/2
    ximes = -(omegaDroit - omegaGauche)*R/W

    # La suite des calculs se fait avec dt2, qui correspond au "vrai" pas de temps d'échantillonnage
    # de cette fonction (le pcDuino n'est pas un système temps-réel et au début de l'exécution du programme,
    # dt2 peut être jusqu'à deux fois plus petit que dt)
    dt2 = time.time() - tprec
    tprec = time.time()

    # Calcul du PID sur vx
    # Paramètres intermédiaires
    Ti = Ki2 * Kivx/(Kp2 * Kpvx + 0.01)
    ad = Tf/(Tf+dt2)
    bd = Kd2 * Kdvx/(Tf+dt2)
    
    # Terme proportionnel
    P_vx = Kpvx * Kp2 * (vxref - vxmes)

    # Terme dérivé
    D_vx = ad * D_vx - bd * (vxmes - yprecvx)
    
    # Calcul de la commande
    commande_vx = P_vx + I_vx


    # Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
    I_vx = I_vx + Kivx * Ki2 * dt2 * (vxref - vxmes)

    # Stockage de la mesure courante pour utilisation lors du pas d'échantillonnage suivant
    yprecvx = vxmes
    
    # Fin Calcul du PID sur vx

    # Calcul du PID sur xi
    # Paramètres intermédiaires
    Ti = Kixi2 * Kixi/(Kpxi2 * Kpxi + 0.01)
    ad = Tf/(Tf+dt2)
    bd = Kdxi2 * Kdxi/(Tf+dt2)
    
    # Terme proportionnel
    P_xi = Kpxi * Kpxi2 * (xiref - ximes)

    # Terme dérivé
    D_xi = ad * D_xi - bd * (ximes - yprecxi)
    
    # Calcul de la commande
    commande_xi = P_xi + I_xi + D_xi


    # Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
    I_xi = I_xi + Kixi * Kixi2 * dt2 * (xiref - ximes)

    # Stockage de la mesure courante pour utilisation lors du pas d'échantillonnage suivant
    yprecxi = ximes
    
    # Fin Calcul du PID sur xi


    # Calcul des commandes des moteurs
    commandeDroit = (commande_vx - commande_xi);
    commandeGauche = (commande_vx + commande_xi);
      
    CommandeMoteurDroit(commandeDroit, tensionBatterie)
    CommandeMoteurGauche(commandeGauche, tensionBatterie)
    
    # Lecture de la tension d'alimentation
    if idecimLectureTension >= decimLectureTension:
        # La mesure de la tension d'alimentation se fait via un pont diviseur de rapport 3 / 13
        # Les entrées analogiques A0 et A1 sont sur 6 bits avant une plage de variation de 2V
        # La résolution obtenue avec la commande suivante est donc très mauvaise
        #tensionAlimBrute = analogReadmV(A0)
        #tensionAlim = max(tensionAlimMin, float(tensionAlimBrute) * 13. / 3.) / 1000.
        # On préfère lire la tension à partir de la mesure faire par la carte Iteaduino Uno
        try:
            tensionAlimBrute = uno.analog_read(0)
            tensionAlimAvantMax = 3.3 * tensionAlimBrute * (13. / 3.) / 1024.;
            tensionAlim = max(tensionAlimMin, tensionAlimAvantMax);
            idecimLectureTension = 0
        except:
            print "Probleme lecture tension d'alimentation"
            pass
    else:
        idecimLectureTension = idecimLectureTension + 1

        
    
def CommandeMoteurDroit(commande, tensionAlim):
    # Cette fonction calcule et envoi les signaux PWM au pont en H
    # en fonction des tensions de commande et d'alimentation

    # L'ensemble pont en H + moteur pourrait ne pas être linéaire
    tension = commande

    # Normalisation de la tension d'alimentation par
    # rapport à la tension d'alimentation
    tension_int = int(255 * tension / tensionAlim)

    # Saturation par sécurité
    if (tension_int > 255):
        tension_int = 255

    if (tension_int < -255):
        tension_int = -255

    # Commande PWM
    if (tension_int >= 0):
        digitalWrite(directionMoteurDroit, 0)
        analogWrite(pwmMoteurDroit, tension_int)

    if (tension_int < 0):
        digitalWrite(directionMoteurDroit, 1)
        analogWrite(pwmMoteurDroit, -tension_int)


    
def CommandeMoteurGauche(commande, tensionAlim):
    # Cette fonction calcule et envoi les signaux PWM au pont en H
    # en fonction des tensions de commande et d'alimentation

    # L'ensemble pont en H + moteur pourrait ne pas être linéaire
    tension = commande

    # Normalisation de la tension d'alimentation par
    # rapport à la tension d'alimentation
    tension_int = int(255 * tension / tensionAlim)

    # Saturation par sécurité
    if (tension_int > 255):
        tension_int = 255

    if (tension_int < -255):
        tension_int = -255

    # Commande PWM
    if (tension_int >= 0):
        digitalWrite(directionMoteurGauche, 1)
        analogWrite(pwmMoteurGauche, tension_int)

    if (tension_int < 0):
        digitalWrite(directionMoteurGauche, 0)
        analogWrite(pwmMoteurGauche, -tension_int)


            
def emitData():
    global tprec
    # Délai nécessaire pour que le serveur ait le temps de démarrer
    delay(5000)
    servocam.detach()
    servocam.attach(10)
    digitalWrite(13,HIGH)
        
    # Démarrage du client Websocket (indispensable d'avoir toujours un client connecté)
    try:
        websocket.create_connection("ws://" + get_ip_address('eth0') + ":" + "9090" + "/ws")
    except:
        pass
        
    try:
        websocket.create_connection("ws://" + get_ip_address('wlan0') + ":" + "9090" + "/ws")
    except:
        pass

    tprec = time.time()
    while not noLoop: loop() # appelle fonction loop sans fin

    
class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        global socketOK
        print 'connection opened...'
        socketOK = True
        self.callback = PeriodicCallback(self.sendToSocket, 100)
        self.callback.start()
    
    def on_message(self, message):
        global x1, x2, Kp2, Ki2, Kd2, Kpxi2, Kixi2, Kdxi2, servocam, timeLastReceived, timedOut, servoref

        jsonMessage = json.loads(message)
        
        # Annulation du timeout de réception des données
        timeLastReceived = time.time()
        timedOut = False;
      
        if jsonMessage.get('vref') != None:
            x1 = float(jsonMessage.get('vref')) / 100
            #print ("x1: %.2f" % x1)
        if jsonMessage.get('xiref') != None:
            x2 = (float(jsonMessage.get('xiref'))) * 3.141592 / 180
            #print ("x2: %.2f" % x2)
        if jsonMessage.get('servoref') != None:
            servorefprec = servoref
            servoref = float(jsonMessage.get('servoref'))
            if servoref != servorefprec:
                servocam.attach(10)
                servocam.write(servoref)
                servocam.write(servoref)
                servocam.write(servoref)
            else:
                servocam.detach()
            #print ("servoref: %d" % servoref)
        if jsonMessage.get('Kp2ref') != None:
            Kp2 = float(jsonMessage.get('Kp2ref'))
            #print ("Kp2: %.2f" % Kp2)
        if jsonMessage.get('Ki2ref') != None:
            Ki2 = float(jsonMessage.get('Ki2ref'))
            #print ("Ki2: %.2f" % Ki2)
        if jsonMessage.get('Kd2ref') != None:
            Kd2 = float(jsonMessage.get('Kd2ref'))
            #print ("Kd2: %.2f" % Kd2)
        if jsonMessage.get('Kpxi2ref') != None:
            Kpxi2 = float(jsonMessage.get('Kpxi2ref'))
            #print ("Kpxi2: %.2f" % Kpxi2)
        if jsonMessage.get('Kixi2ref') != None:
            Kixi2 = float(jsonMessage.get('Kixi2ref'))
            #print ("Kixi2: %.2f" % Kixi2)
        if jsonMessage.get('Kdxi2ref') != None:
            Kdxi2 = float(jsonMessage.get('Kdxi2ref'))
            #print ("Kdxi2: %.2f" % Kdxi2)
        

    def on_close(self):
        global socketOK, commandeDroit, commandeGauche
        print 'connection closed...'
        socketOK = False
        commandeDroit = 0.
        commandeGauche = 0.

    def sendToSocket(self):
        global started, codeurDroitDeltaPos, codeurGaucheDeltaPos, socketOK, commandeDroit, commandeGauche, vxref, xiref, vxmes, ximes

        tcourant = time.time() - T0
        aEnvoyer = json.dumps( {'Temps':("%.2f" % tcourant),
                                'Consigne vitesse longitudinale':("%.2f" % x1),
                                'Consigne vitesse de rotation':("%.2f" % x2),
                                'Vitesse longitudinale':("%.2f" % vxmes),
                                'Vitesse de rotation':("%.2f" % (180 * ximes/3.141592)),
                                'omegaDroit':("%.2f" % omegaDroit),
                                'omegaGauche':("%.2f" % omegaGauche),
                                'commandeDroit':("%.2f" % commandeDroit),
                                'commandeGauche':("%.2f" % commandeGauche),
                                'Raw':("%.2f" % tcourant) + "," +
                                ("%.2f" % x1) + "," +
                                ("%.2f" % x2) + "," +
                                ("%.2f" % vxmes) + "," +
                                ("%.2f" % (180 * ximes/3.141592)) + "," +
                                ("%.2f" % omegaDroit) + "," +
                                ("%.2f" % omegaGauche) + "," +
                                ("%.2f" % commandeDroit) + "," +
                                ("%.2f" % commandeGauche)})
        if socketOK:
            try:
                self.write_message(aEnvoyer)
            except:
                pass
            
    def check_origin(self, origin):
        # Voir http://www.tornadoweb.org/en/stable/websocket.html#tornado.websocket.WebSocketHandler.check_origin
        # et http://www.arundhaj.com/blog/tornado-error-during-websocket-handshake.html
        return True        

    
def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])
    
application = tornado.web.Application([
    (r'/ws', WSHandler)
])

def startTornado():
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(9090)
    tornado.ioloop.IOLoop.instance().start()
    

# Gestion du CTRL-C
def signal_handler(signal, frame):
    global commandeDroit, commandeGauche, servovam
    print 'You pressed Ctrl+C!'
    commandeDroit = 0.
    commandeGauche = 0.
    CommandeMoteurDroit(0, 5)
    CommandeMoteurGauche(0, 5)
    servocam.detach()
    digitalWrite(13, LOW)
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# Gestion des segmentation fault
def signal_handler2(signal, frame):
    print 'Received signal ' + str(sig) + ' on line ' + str(frame.f_lineno) + ' in ' + frame.f_code.co_filename

signal.signal(signal.SIGSEGV, signal_handler2)

#--- obligatoire pour lancement du code -- 
if __name__=="__main__": # pour rendre le code executable 

    # Test pour savoir si le firmware est présent sur la carte Uno
    firmwarePresent = False
    for i in range(1, 11):
        time.sleep(0.1)
        print "Test presence du firmware de la carte Uno, tentative " + str(i) + " / 10"
        try:
            firmwarePresent = uno.firmwareOK()
            if firmwarePresent:
                break
        except:
            print "Firmware absent"
        
    if firmwarePresent:
        print "Firmware present, on continue..."
        started = False
        startedDroit = False
        startedGauche = False
        setup() # appelle la fonction setup
        print "Setup done."
        
        th = threading.Thread(None, emitData, None, (), {})
        th.daemon = True
        th.start()
        
        print "Starting Tornado."
        try:
            print "Connect to ws://" + get_ip_address('eth0') + ":9090/ws with Ethernet."
        except:
            pass
            
        try:
            print "Connect to ws://" + get_ip_address('wlan0') + ":9090/ws with Wifi."
        except:
            pass
            
        socketOK = False
        startTornado()
    else:
        print "Firmware absent, on abandonne ce programme."
        print "Veuillez charger le firmware sur la carte Uno pour exécuter ce programme."


