# Tutoriel LiDAR LD19

[english version](./README.md)

Le LD19 est un LiDAR peu coûteux et très efficace, mais il n’existe pas de documentation ou de tutoriel adéquat sur le web. J’ai néanmoins réussi à le faire fonctionner après de longues journées de travail. Voici comment j'y suis arrivé !

> [!TIP]
> Ce tutoriel a pour but de vous guider dans la récupération des données LiDAR à partir d’un microcontrôleur, dans un format permettant un traitement immédiat à bord. Si vous souhaitez simplement vérifier que votre LiDAR fonctionne, [ces documents](#liens) fournissent des instructions pour utiliser leur logiciel afin de visualiser les données sur votre ordinateur.

> [!NOTE]
> Ceci n’est pas une documentation officielle.
>
> Toute cette documentation et le code sont sous licence CC BY-SA 4.0, à l’exception des images et autres fichiers.

Si vous remarquez un oubli ou avez des suggestions, n’hésitez pas à ouvrir une [issue sur GitHub](https://github.com/LudovaTech/lidar-LD19-tutorial/issues/new).

## Aperçu

![LiDAR LD19](./images/lidar-LD19.jpg)

Le LiDAR LD19 utilise la technologie Direct Time-of-Flight (DTOF), qui mesure l’intervalle de temps entre l’émission et la réception d’un signal. Selon la documentation du fabricant, le LD19 peut effectuer jusqu’à 5 000 mesures par seconde.

> [!NOTE]
> J'ai observé qu’une rotation complète du LiDAR prend environ 100 millisecondes, ce qui donne environ 450 points de mesure par tour.

## Communication interface

Vous pouvez utiliser un connecteur `JST ZH à 4 broches` pour relier le LiDAR à d’autres composants, ce qui permet à la fois l’alimentation et la réception des données. Les détails de l’interface sont présentés dans le tableau ci-dessous :

De gauche à droite, en tenant le LiDAR avec la partie circulaire orientée vers le haut.

| Nom et type |  Tension  |   Commentaires   |
| :------------ | :-------: | :----------- |
| **Tx** (sortie UART), données du LiDAR à un baud rate `230400` | 0V - 3.5V </br> typique: 3.3V | Ce LiDAR envoie uniquement des données et n’en reçoit aucune, d’où l’absence de port Rx. |
| **PWM** (entrée), contrôle la vitesse du moteur intégré. La vitesse actuelle du LiDAR est indiquée dans les données transmises | 0V - 3.3V | Si le contrôle manuel de la vitesse du LiDAR n’est pas nécessaire, la broche dédiée peut être reliée à la masse (GND) lors de l’activation du dispositif LiDAR, et maintenue dans cet état pendant toute la durée de son fonctionnement. (Plus d’informations sur le contrôle manuel de la vitesse dans la [documentation réelle](#liens))|
| **Ground** (alimentation) | 0V | - |
| **5V** (alimentation) | 4.5V - 5.5V </br> typique: 5V | - |

Le LD19 utilise le protocole UART pour la communication des données avec les paramètres suivants :

- **Baud Rate:** 230400
- **Data Length:** 8 bits
- **Stop Bit:** 1
- **Parity:** None
- **Flow Control:** None

> [!TIP]
> Pour les utilisateurs d’Arduino, il suffit de régler la vitesse de transmission (baud rate) sur `230400`. Les autres paramètres sont configurés par défaut aux valeurs correctes.

> [!IMPORTANT]
> Le LiDAR LD19 commence à transmettre les données de mesure dès que sa rotation se stabilise, ce qui prend généralement deux à trois secondes. Il n’est pas nécessaire d’envoyer des commandes pour lancer ce processus. En réalité, il est impossible d’envoyer des commandes pour cela.

## Protocole de données

### Format du paquet de données

Le LD19 utilise une communication unidirectionnelle. Une fois qu’il fonctionne de manière stable, il commence automatiquement à envoyer des paquets de données de mesure, sans nécessiter de commandes. Chaque paquet contient 12 points de mesure. Le format de ces paquets de mesure est illustré dans le tableau ci-dessous.

|  Nom  | Longueur | Type ou Valeur | Description |
| :----: | :----: | :-----------: | :---------- |
| Header | 1 Octet | Toujours `0x54` | Indiquant le début du paquet de données. |
| VerLen | 1 Octet | Toujours `0x2C` | Les trois bits supérieurs de l’octet spécifient le type de paquet, qui est actuellement fixé à 1. Les cinq bits inférieurs représentent le nombre de points de mesure dans un paquet, qui est fixé à 12. |
| Vitesse  | 2 Octets | [Bit de poids faible][LSB] avant, </br> *unité: degrés par seconde* | Indique la vitesse du LiDAR. |
| Angle de départ | 2 Octets | [Bit de poids faible][LSB] avant, </br> *unité: 0.01 degrés* | Indique l’angle de départ du paquet de données. |
| **Données** | 3 * 12 Octets | ... | Référez-vous à la [section suivante](#Comprendre-le-paquet-de-données) pour plus de détails. |
| Angle d'arrivée | 2 Octets | [Bit de poids faible][LSB] avant, </br> *unité: 0.01 degrés* | Indique l’angle d'arrivée du paquet de données. |
| Timestamp | 2 Octets | [Bit de poids faible][LSB] avant, </br> *unité: millisecondes*, </br> Remise à zéro lorsqu’il atteint `30000` | Indique la valeur du timestamp du paquet de données. |
| CRC check | 1 Octet | Vérification de toutes les données précédentes | Vérifie le transfert des données pour en garantir l’exactitude et l’intégralité, assurant ainsi des résultats sans erreur. |

> [!IMPORTANT]
> Nous recevons les angles initial et final pour chaque série de 12 points. La documentation recommande d’utiliser une interpolation linéaire pour déterminer l’angle de chaque point individuel. Pour les étapes détaillées de mise en œuvre, référez-vous à la [section implémentation](#Implémentation). (Ne vous inquiétez pas, c’est très simple.)

### Comprendre le paquet de données

Chacune des 12 mesures de chaque paquet est composé de 2 valeurs :

|   Nom   | Longueur  | Type ou Valeur | Description |
| :------: | :-----: | :-----------: | :---------- |
| Distance | 2 Octets | [Bit de poids faible][LSB] avant, </br> *unité: mm* | La distance au point détecté. |
| Intensité | 1 Octet | Représente l'intensité de la lumière refletée | L’intensité lumineuse est proportionnelle à la valeur d’intensité du signal. Pour un objet blanc situé à moins de 6 mètres, la valeur typique de la puissance du signal est d’environ 200. |

> [!NOTE]
> Le LD19 utilise un système de coordonnées gaucher avec le centre de rotation à l’origine. L’avant du capteur est défini comme la direction zéro degré, et l’angle de rotation augmente dans le sens horaire, comme illustré dans la figure ci-dessous. </br>
> ![Système de coordonnées du LiDAR](./images/lidar-coordinate-system.jpg)

## Implémentation

### Interpolation Linéaire

L’interpolation linéaire est dans ce cas une méthode permettant d’estimer des valeurs situées entre deux valeurs connues. Ici, elle suppose que tous les points sont à égale distance les uns des autres.
Voici ce que vous devez faire :

- Calculez la distance `angleStep` entre chaque point : `(endAngle - startAngle) / nbr_points`. `nbr_points` est toujours égal à 12 avec ce LiDAR.
- Calculez l’angle du point `n` : `startAngle + (angleStep * n)`.
- Les calculs sont en réalité un peu plus complexes que ceux indiqués dans la documentation, notamment pour gérer la transition de 359° à 0°.

Voici notre implémentation en C++, qui inclut la gestion de la transition 359° - 0° :


```c++
// Calcule la taille du pas entre startAngle et endAngle (en dixièmes de degré).
// divisé par lenMinusOne, qui représente le nombre de pas moins un.
// Suppose que les angles sont compris entre 0 et 3599 (représentant 0,0° à 359,9°).
uint16_t angleStep(uint16_t startAngle, uint16_t endAngle, unsigned int lenMinusOne) {
  if (startAngle <= endAngle) {
    return (endAngle - startAngle) / lenMinusOne;
  } else {
    return (36000 + endAngle - startAngle) / lenMinusOne;
  }
}

// Calcule l’angle (en dixièmes de degré) correspondant à un indice de pas donné.
// à partir de startAngle, chaque pas valant step dixièmes de degré.
// Retourne l’angle normalisé entre 0 et 3599 (représentant 0,0° à 359,9°).
uint16_t angleFromStep(uint16_t startAngle, uint16_t step, unsigned int indice) {
  return (startAngle + (step * indice)) % 36000;
}
```

### Lire une valeur sur 2 octets avec l’octet de poids faible en premier

Voici comment interpréter une valeur sur 2 octets (LSB/MSB) à partir d’un tableau `buffer` contenant les octets, où `index` indique la position du bit de poids faible (LSB, le premier) :

```c++
uint16_t _get2BytesLsbMsb(byte buffer[], int index) {
  return (buffer[index + 1] << 8) | buffer[index];
}
```

### Implémenter le CRC check

Voici comment implémenter la vérification CRC pour contrôler la validité des données :
`p` est un tableau contenant les octets récupérés depuis le LiDAR.
`lenWithoutCRCCheckValue` représente la longueur totale des données envoyées par le LiDAR, hors contrôle CRC. Pour ce LiDAR, elle est de 44 octets.

```c++
static const uint8_t crcTable[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
    0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
    0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
    0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
    0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
    0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
    0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
    0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
    0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
    0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
    0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
    0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
    0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
    0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
    0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
    0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
    0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
    0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
    0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
    0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
    0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
    0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8};

uint8_t _calCRC8FromBuffer(uint8_t* p, uint8_t lenWithoutCRCCheckValue) {
  uint8_t crc = 0xD8;                                       // pre-calculated header and verlen values (crc = crcTable[(crc ^ 0x54) & 0xff];crc = crcTable[(crc ^ 0x2C) & 0xff];)
  for (uint16_t i = 0; i < lenWithoutCRCCheckValue; i++) {  // ignores the last value of the p array (which contains the crc check value)
    crc = crcTable[(crc ^ *p++) & 0xff];
  }
  return crc;
}
```

Pour vérifier si les valeurs sont valides, il suffit de comparer le résultat de cette fonction avec les données reçues du LiDAR.

> [!CAUTION]
> Dans certains documents de la documentation du LiDAR LD19, la table crcTable est incomplète, ce qui entraîne un dysfonctionnement. Cette version inclut toutes les lignes de code nécessaires.

### Voici l’implémentation complète en C++ pour interpréter les données reçues du LiDAR :

`lidar_reader.h`

```c++
#ifndef LIDAR_H
#define LIDAR_H

#include <Arduino.h>

class LidarPoint {
 public:
  LidarPoint(uint16_t distance, uint8_t intensity, float angle);

  LidarPoint &operator=(const LidarPoint &) = delete;

  inline uint16_t distance() const { return _distance; }  // distance from the center of the lidar
  inline uint8_t intensity() const { return _intensity; }
  inline float angle() const { return _angle; }

  String toString() const;

 private:
  const uint16_t _distance;
  const uint8_t _intensity;
  const float _angle;
};

static const uint8_t crcTable[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
    0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
    0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
    0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
    0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
    0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
    0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
    0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
    0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
    0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
    0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
    0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
    0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
    0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
    0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
    0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
    0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
    0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
    0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
    0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
    0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
    0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8};

std::vector<LidarPoint> getPoints();

uint8_t _calCRC8FromBuffer(uint8_t *p, uint8_t lenWithoutCRCCheckValue);
uint16_t _get2BytesLsbMsb(byte buffer[], int index);

uint16_t angleStep(uint16_t startAngle, uint16_t endAngle, unsigned int lenMinusOne = 11);
uint16_t angleFromStep(uint16_t startAngle, uint16_t step, unsigned int indice);

#endif
```

`lidar_reader.cpp`

```c++
#include "lidar_reader.hpp"

//////LIDARPOINT

LidarPoint::LidarPoint(uint16_t distance, uint8_t intensity, float angle)
    : _distance(distance), _intensity(intensity), _angle(angle) {}

String LidarPoint::toString() const {
  String result = "(distance=";
  result += String(_distance);
  result += ", intensity=";
  result += String(_intensity);
  result += ", angle=";
  result += String(_angle);
  result += ")";
  return result;
}

//////FONCTIONS

std::vector<LidarPoint> getPoints() {
  std::vector<LidarPoint> points;
  if (!SerialLidar.find("T,")) {  // equivalent en char de 84 44 (decimal)
    Serial.println("lidar_reader.getPoints : error, no header-verlen found in RX for the LiDAR LD19");
  } else {
    // The previous instruction (find) jumped to the beginning of the information
    // Now the stream is aligned
    byte buffer[45];
    size_t nbrBytesReceived = SerialLidar.readBytes(buffer, 45);
    if (nbrBytesReceived != 45) {
      Serial.println("lidar_reader.getPoints : error, wrong number of bytes received (" + String((uint32_t) nbrBytesReceived) + ")");
    } else {
      uint16_t speed = _get2BytesLsbMsb(buffer, 0);
      uint16_t startAngle = _get2BytesLsbMsb(buffer, 2);

      LidarPoint data[] = {// no for loop possible due to 'const' in LidarPoint class
                            LidarPoint(_get2BytesLsbMsb(buffer, 4), buffer[6], 0),
                            LidarPoint(_get2BytesLsbMsb(buffer, 7), buffer[9], 0),
                            LidarPoint(_get2BytesLsbMsb(buffer, 10), buffer[12], 0),
                            LidarPoint(_get2BytesLsbMsb(buffer, 13), buffer[15], 0),
                            LidarPoint(_get2BytesLsbMsb(buffer, 16), buffer[18], 0),
                            LidarPoint(_get2BytesLsbMsb(buffer, 19), buffer[21], 0),
                            LidarPoint(_get2BytesLsbMsb(buffer, 22), buffer[24], 0),
                            LidarPoint(_get2BytesLsbMsb(buffer, 25), buffer[27], 0),
                            LidarPoint(_get2BytesLsbMsb(buffer, 28), buffer[30], 0),
                            LidarPoint(_get2BytesLsbMsb(buffer, 31), buffer[33], 0),
                            LidarPoint(_get2BytesLsbMsb(buffer, 34), buffer[36], 0),
                            LidarPoint(_get2BytesLsbMsb(buffer, 37), buffer[39], 0)};

      uint16_t endAngle = _get2BytesLsbMsb(buffer, 40);
      uint16_t timestamp = _get2BytesLsbMsb(buffer, 42);
      uint8_t crcCheck = buffer[44];

      if (_calCRC8FromBuffer(buffer, 44) == crcCheck) {
        uint16_t step = angleStep(startAngle, endAngle);
        for (unsigned int i = 0; i < 12; i++) {
          points.push_back(
              LidarPoint(
                  data[i].distance(),
                  data[i].intensity(),
                  angleFromStep(startAngle, step, i)));
        }
      }
    }
  }
  return points;
}

uint8_t _calCRC8FromBuffer(uint8_t* p, uint8_t lenWithoutCRCCheckValue) {
  uint8_t crc = 0xD8;                                       // pre-calculated header and verlen values (crc = crcTable[(crc ^ 0x54) & 0xff];crc = crcTable[(crc ^ 0x2C) & 0xff];)
  for (uint16_t i = 0; i < lenWithoutCRCCheckValue; i++) {  // ignores the last value of the p array (which contains the crc check value)
    crc = crcTable[(crc ^ *p++) & 0xff];
  }
  return crc;
}

uint16_t _get2BytesLsbMsb(byte buffer[], int index) {
  return (buffer[index + 1] << 8) | buffer[index];
}

uint16_t angleStep(uint16_t startAngle, uint16_t endAngle, unsigned int lenMinusOne) {
  if (startAngle <= endAngle) {
    return (endAngle - startAngle) / lenMinusOne;
  } else {
    return (36000 + endAngle - startAngle) / lenMinusOne;
  }
}

uint16_t angleFromStep(uint16_t startAngle, uint16_t step, unsigned int indice) {
  return (startAngle + (step * indice)) % 36000;
}
```

Pour utiliser le LiDAR LD19 avec votre carte et ce code, il suffit de définir `SerialLidar` (avec `#define`) comme l’objet Serial adapté à votre carte. Ensuite, appelez la fonction `getPoints` pour recevoir un `std::vector` contenant tous les points détectés par le LiDAR.

Vous pouvez retrouver l’intégralité de notre code source disponible [ici](https://github.com/LudovaTech/robot-prog-public).

## Liens

- [Ce qui semble être la documentation officielle](https://wiki.youyeetoo.com/en/Lidar/D300) ***Contient des erreurs***
- [Une meilleure documentation.](https://www.elecrow.com/download/product/SLD06360F/LD19_Development%20Manual_V2.3.pdf) ([version locale](./documents/LD19_Development_Manual_v2.5.pdf) si le site enlève le document)

<p xmlns:cc="http://creativecommons.org/ns#" xmlns:dct="http://purl.org/dc/terms/"><a property="dct:title" rel="cc:attributionURL" href="https://github.com/LudovaTech/lidar-LD19-tutorial">lidar-LD19-tutorial</a> (only the text and code of this document, not the images or the other files) by <a rel="cc:attributionURL dct:creator" property="cc:attributionName" href="https://github.com/LudovaTech">LudovaTech (D'Artagnant et RemyMagnon)</a> is licensed under <a href="https://creativecommons.org/licenses/by-sa/4.0/?ref=chooser-v1" target="_blank" rel="license noopener noreferrer" style="display:inline-block;">CC BY-SA 4.0<img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/cc.svg?ref=chooser-v1" alt=""><img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/by.svg?ref=chooser-v1" alt=""><img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/sa.svg?ref=chooser-v1" alt=""></a></p>

[LSB]: https://fr.wikipedia.org/wiki/Num%C3%A9rotation_des_bits
