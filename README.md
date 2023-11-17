<h1 align="center">MATRS</h1>

<p align="center">
<img src="https://camo.githubusercontent.com/97d4586afa582b2dcec2fa8ed7c84d02977a21c2dd1578ade6d48ed82296eb10/68747470733a2f2f6261646765732e66726170736f66742e636f6d2f6f732f76312f6f70656e2d736f757263652e7376673f763d313033"/>
<img src="https://img.shields.io/badge/lang-C%2B%2B-blue" />
<img src="https://img.shields.io/badge/3D%20Printing-orange">
<img src="https://img.shields.io/badge/license-GNU%20GPLv3-green" />
</p>

MATRS (Motion Actuated Turret Remote System) ist ein quelloffenes persönliches Projekt. Ein Miniaturgeschütz wird kabellos durch die im Handschuh verbauten Sensoren gesteuert.

<img src="assets/Turret.png" width="400" /> <img src="assets/Glove.png" width="400" />

## Hintergrund

Was eigentlich nur ein Wochenend-Projekt, zum wiederverwenden einiger Komponenten, werden sollte artete in mein wohl umfangreichstes Mechatronik Projekt aus. Vom Getriebe bis zum Handschuh habe ich so ziemlich alles entworfen, programmiert und fabriziert.

## Features

### Geschoss

MATRS feuert sechs Millimeter Airsoft BBs. Andere Größen könnten jedoch durch einfache Modifizierung am Lauf und Hopper benutzt werden.

### Modi

MATRS hat zwei programmierte Modi, welche mittels Knopfdruck am Handschuh ausgewählt werden können:

#### Stationärer Modus

In diesem Modus rotiert die Turret nicht. Lediglich der Abzug kann betätigt werden, um ein Geschoss abzufeuern.

#### Dynamischer Modus

Hier rotiert die Turret, sodass die Orientierung gleich dem des Handschuhs/Controllers ist (Gieren). Auch die Neigung (Nicken) des Handschuhs wird berücksichtigt.

## Umsetzung

### Gieren

Das Gieren des Geschützes übernimmt ein 39,5:1 **Spannungswellengetriebe**. Als _Flex-Spline_ wurde ein Zahnriemen vom Typ GT2 verwendet, welcher von zwei Kugellagern gespannt wird. Den Antrieb übernimmt in meiner Installation ein Nema 23 Schrittmotor (Closed-Loop), welcher zugegeben etwas überqualifiziert ist.

### Nicken

Das Nicken wird durch ein Zahnriemengetriebe mit einem Übersetzungsverhältnis von 3⅓:1 realisiert. Als Antrieb habe ich einen SG90 Mikroservo benutzt. Des weiteren sorgt ein Idler mit einstellbarer Spannung für eine optimale Kraftübersetzung.

### Schießen

Den Schuss-Mechanismus habe ich in dieser Version nicht selbst entworfen. Es handelt sich um eine einfache Airsoft-Gearbox. Der durch die Gearbox freigelassene Luftdruck mündet in dem 3D-gedruckten vorderen Lauf. Die Kugeln werden hierbei von einem Trichter in den Lauf gefüllt.

### Bewegungserfassung

Das **AHRS** (_Attitude Heading Reference System_) wird mittels Sensor-Fusion eines 6-Achsen Beschleunigungssensor und Gyroskops inmitten des Handschuhs umgesetzt. Ein Arduino Nano 33 BLE sorgt für die Beschaffung und Verarbeitung dieser Informationen. Eine **Kalibrierung** und **Homing-Prozedur** direkt nach dem Verbindungsaufbau zwischen Handschuh und Geschütz sorgt für zuverlässige, konstante Nutzung.

### Drahtlose Kommunikation

Des weiteren ist dank des verbauten Chipsets Bluetooth® Low Energy und somit eine kabellose Kommunikation zwischen Controller (Handschuh) und Turret möglich. Die Kalibrierung, das Homing, wie die Auswahl der verschiedenen Modi erfolgt über einen Taster am Handschuh. Zudem erzeugt die Basisstation der Turret Audio-Signale zu bestimmten Ereignissen (wie z.B. beim Wechsel des Modus) und eine LED im Handschuh liefert Einblick in den Zustand der Kommunikation. So bedeutet ein blinkendes Signal bspw. das Daten für die Kalibrierung gesammelt werden.

Es gibt einen **BLE-Service** mit folgenden Charakteristiken:

```
BLECharacteristic headingInt("b3de23b1-12fe-4266-a590-53528c11116d", BLERead, 3);
BLEIntCharacteristic buttonPress("6541935a-5c22-464b-b150-c335eaa92910", BLERead | BLEWrite | BLENotify);
BLEIntCharacteristic triggerPull("626488b2-70d1-4b89-ad13-45e8ee61ebd8", BLERead | BLENotify);
```

Wobei `headingInt` aus drei Byte besteht (platzbedingt, da z.B. 359 (°) > 255):

- Byte 1 und 2 für die Gierungsrichtung
- Byte 3 für das Nicken

## Hürden

### AHRS

Ursprüngliche Versuche die Sensor-Daten zu Fusionieren fanden im Zahlenbereich der Quaternionen statt. Letztendlich waren normale Euler-Winkel leichter umzusetzen. Da ein Gimbal-Lock für den Betrieb des Systems irrelevant scheint, ist diese Lösung pragmatischer.

### Asynchronizität

Anfangs war für die Kontrolle der Basisstation lediglich ein Arduino Nano 33 BLE vorgesehen. Dieser sollte Kommunikation und Steuerung des Geschützes vornehmen (Gieren & Nicken). Diese Realisierung stellte sich jedoch ziemlich schnell als unpraktisch heraus. Die Limitierung ergab sich aus der ungenügenden Spannung der digitalen Pins des Nanos für die Steuerung des Schrittmotor-Treibers. Daher wurden die Steuerungsaufgaben auf einen zusätzlichen Arduino Mega umgelegt. Der Arduino Nano der Basisstation übernimmt also lediglich Kommunikationsaufgaben.

Die eingehenden AHRS Daten des Handschuhs übermittelt der Arduino Nano der Basisstation an den Arduino Mega per serieller Schnittstelle.

### Stromversorgung

## Weiterführendes

Der ganze Artikel über das Projekt steht auf meiner <a href="https://philipsi.de">Portfolio Website</a> zur Verfügung. In diesem finden Sie auch Hardware Liste & Co.
