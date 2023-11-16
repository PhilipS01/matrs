<h1 align="center">MATRS</h1>

<p align="center">
<img src="https://camo.githubusercontent.com/97d4586afa582b2dcec2fa8ed7c84d02977a21c2dd1578ade6d48ed82296eb10/68747470733a2f2f6261646765732e66726170736f66742e636f6d2f6f732f76312f6f70656e2d736f757263652e7376673f763d313033"/>
<img src="https://img.shields.io/pypi/pyversions/notion_client" />
<img src="https://img.shields.io/badge/license-GNU%20GPLv3-green" />
</p>

MATRS (Motion Actuated Turret Remote System) ist ein quelloffenes persönliches Projekt. Ein Miniaturgeschütz wird kabellos durch die im Handschuh verbauten Sensoren gesteuert.

<img src="assets/greeter_screenshot.png" />

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

Das Gieren des Geschützes übernimmt ein 39,5:1 Spannungswellengetriebe.

## Weiterführendes

Der ganze Artikel über das Projekt steht auf meiner <a href="https://philipsi.de">Portfolio Website</a> zur Verfügung. In diesem finden Sie auch Hardware Liste & Co.
