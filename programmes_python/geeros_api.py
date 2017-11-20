# -*- coding: utf-8 -*-

from websocket import create_connection
import math
import time

class geeros_api:

    def __init__(self):

        self.ws = create_connection("ws://192.168.0.199:9090/ws")
        

    def Tourner(self, vitesseRotation, duree = -1):
        tdebut = time.time()
        t = time.time() - tdebut
        while t < duree:
            xiref = eval(str(vitesseRotation))
            # Saturations min et max
            xiref = max(min(360, xiref), -360)
            # Envoi de la consigne au programme principal par Websocket
            self.ws.send('{"xiref": ' + str(xiref) + '}')
            time.sleep(0.1)
            t = time.time() - tdebut
            
        if duree == -1:
            xiref = eval(str(vitesseRotation))
            # Saturations min et max
            xiref = max(min(360, xiref), -360)
            # Envoi de la consigne au programme principal par Websocket
            self.ws.send('{"xiref": ' + str(xiref) + '}')
        else:
            self.ws.send('{"xiref": 0}')
    
    
    def Avancer(self, vitesseLongitudinale, duree = -1):
        tdebut = time.time()
        t = time.time() - tdebut
        while t < duree:
            vref = eval(str(vitesseLongitudinale))
            # Saturations min et max
            vref = max(min(50, vref), -50)
            # Envoi de la consigne au programme principal par Websocket
            self.ws.send('{"vref": ' + str(vref) + '}')
            time.sleep(0.1)
            t = time.time() - tdebut
            
        if duree == -1:
            vref = eval(str(vitesseLongitudinale))
            # Saturations min et max
            vref = max(min(50, vref), -50)
            # Envoi de la consigne au programme principal par Websocket
            self.ws.send('{"vref": ' + str(vref) + '}')
        else:
            self.ws.send('{"vref": 0}')
    
    
    def Mouvement(self, vitesseLongitudinale, vitesseRotation, duree = -1):
        tdebut = time.time()
        t = time.time() - tdebut
        while t < duree:
            vref = eval(str(vitesseLongitudinale))
            xiref = eval(str(vitesseRotation))
            # Saturations min et max
            vref = max(min(0.5, vref), -0.5)
            xiref = max(min(360, xiref), -360)
            # Envoi de la consigne au programme principal par Websocket
            self.ws.send('{"vref": ' + str(vref) + ', "xiref": ' + str(xiref) + '}')
            time.sleep(0.1)
            t = time.time() - tdebut
            
        if duree == -1:
            vref = eval(str(vitesseLongitudinale))
            xiref = eval(str(vitesseRotation))
            # Saturations min et max
            vref = max(min(0.5, vref), -0.5)
            xiref = max(min(360, xiref), -360)
            # Envoi de la consigne au programme principal par Websocket
            self.ws.send('{"vref": ' + str(vref) + ', "xiref": ' + str(xiref) + '}')
        else:
            self.ws.send('{"vref": 0, "xiref": 0}')
    
    
    def AngleServo(self, angle, duree = -1):
        tdebut = time.time()
        t = time.time() - tdebut
        while t < duree:
            servoref = eval(str(angle))
            # Saturations min et max
            servoref = max(min(30, servoref), -30)
            # Envoi de la consigne au programme principal par Websocket
            self.ws.send('{"servoref": ' + str(servoref) + '}')
            time.sleep(0.1)
            t = time.time() - tdebut
            
        if duree == -1:
            servoref = eval(str(angle))
            # Saturations min et max
            servoref = max(min(30, servoref), -30)
            # Envoi de la consigne au programme principal par Websocket
            self.ws.send('{"servoref": ' + str(servoref) + '}')
    
    
    
    def Terminer(self):
        self.ws.send('{"vref": 0, "xiref": 0}')
        self.ws.close()
        
