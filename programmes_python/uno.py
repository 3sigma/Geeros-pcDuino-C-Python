#!/usr/bin/python
# -*- coding: utf-8 -*-

##################################################################################
# Programme de communication en i2c avec la carte Ireaduino Uno du système
# de commande de moteur électrique disponible à l'adresse:
# http://boutique.3sigma.fr/12-robots
#
# Auteur: 3Sigma
# Version 1.0 - 15/06/2017
##################################################################################

import smbus
import struct

class Uno(object):
    def __init__(self, hostname = "pcduino"):
        if (hostname == "pcduino"):
            self.bus = smbus.SMBus(2)
        elif (hostname == "raspberrypi"):
            self.bus = smbus.SMBus(1)
        else:
            # pcDuino par défaut
            self.bus = smbus.SMBus(2)

    def command(self, cmd, byte_array):
        bytes = [cmd] + byte_array
        self.bus.write_i2c_block_data(20, 1, bytes)
        while(not self.command_done()):
            pass

    def command_done(self):
        self.bus.write_byte(20,0)
        return self.bus.read_byte(20) == 0

    def read_unpack(self, size, format):
        self.bus.write_byte(20,2)
        byte_list = []
        for n in range(0,size):
            byte_list.append(self.bus.read_byte(20))
        return struct.unpack(format,bytes(bytearray(byte_list)))

    def shiftu2s8(self, x):
        return x - 128

    def shiftu2s16(self, x):
        return x - 32768

    def read_codeurDroitDeltaPos(self):
        self.command(1, [])
        codeurDroitDeltaPos = self.shiftu2s16(self.read_unpack(2, "H")[0])
        return codeurDroitDeltaPos

    def read_codeurGaucheDeltaPos(self):
        self.command(2, [])
        codeurGaucheDeltaPos = self.shiftu2s16(self.read_unpack(2, "H")[0])
        return codeurGaucheDeltaPos

    def read_codeurDroitPos(self):
        self.command(3, [])
        codeurDroitPos = self.shiftu2s16(self.read_unpack(2, "H")[0])
        return codeurDroitPos

    def read_codeurGauchePos(self):
        self.command(4, [])
        codeurGauchePos = self.shiftu2s16(self.read_unpack(2, "H")[0])
        return codeurGauchePos

    def codeurDroitPos(self, value):
        self.command(5, map(ord, list(struct.pack('h', value))))

    def codeurGauchePos(self, value):
        self.command(6, map(ord, list(struct.pack('h', value))))

    def analog_read(self, port):
        self.command(7, [port])
        return self.read_unpack(2, "H")[0]

    def firmwareOK(self):
        self.command(8, [])
        if self.read_unpack(2, "H")[0] == 1:
            return True
        else:
            return False

    def test_read8(self):
        try:
            self.read_unpack(8, "cccccccc")
        except:
            print "Erreur test_read8"

    def test_write8(self):
        try:
            self.bus.write_i2c_block_data(20, 0, [0,0,0,0,0,0,0,0])
        except:
            print "Erreur test_write8"

