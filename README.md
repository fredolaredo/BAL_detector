# BAL_detector

## objectif

recevoir une notification sur ouverture de la boite aux lettre qui se trouve en bas de l'immeuble

basse consommation pour tourner avec une batterie sur plusieurs mois 

## principe

1. ouverture de la porte de la BAL par capteur magnétique

2. envoi d'un packet lora contenant des données ambiantes et le voltage de la batterie

3. notification IFTTT sur message TTN

## composants

atmega328 package DIP28 en 3,3V / 8Mhz

LoRa 1276

DHT22 (pour le fun)

une baterie Lithium 3,7v 1000mAh

une Led 

quelques composants passifs : resistances, capa de découplage, un push button, ...

