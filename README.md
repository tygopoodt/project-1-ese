<p align="center">
  <img src="Documentatie/Content/Q-Dat%20Systems%20Banner.png" alt="Q-Dat Systems" width="820">
</p>

<h1 align="center">Robot-car</h1>

<p align="center">
  <em>Project 1 NL &middot; Embedded Systems Engineering &middot; HAN</em>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/MCU-ATmega328P-000000?style=flat-square" alt="ATmega328P">
  <img src="https://img.shields.io/badge/taal-C-000000?style=flat-square" alt="C">
  <img src="https://img.shields.io/badge/H--brug-L298N-000000?style=flat-square" alt="L298N">
  <img src="https://img.shields.io/badge/status-in%20ontwikkeling-lightgrey?style=flat-square" alt="Status">
</p>

---

## Over het project

Transport wordt steeds verder geautomatiseerd. Voor opdrachtgever Robot-car-Gadgets
ontwikkelt **Q-Dat Systems** een prototype van de **Robot-car**: een autonoom rijdende
modelauto op basis van een ATmega328P.

De auto rijdt vooruit, achteruit en draait om zijn as, volgt een zwarte lijn op de vloer,
houdt afstand tot een voorganger en is via remote control te besturen. De motoren worden
met PWM via een L298N H-brug aangestuurd; bediening en tests lopen over de USART.

Het project beslaat twee perioden. In de eerste periode verkennen we de functionaliteit met
rapid prototyping in de Arduino IDE. In de tweede periode gaat dezelfde microcontroller
opnieuw op de tekentafel, nu in C met directe registermanipulatie.

## Repository

| Map | Inhoud |
| --- | --- |
| `Software/` | Firmware voor de ATmega328P |
| `Hardware/` | Schema's, componentkeuzes en het 3D-model van het chassis |
| `Documentatie/Templates/` | Huisstijl-templates: agenda, notulen, productrapport |
| `Documentatie/Content/` | Logo's en beeldmateriaal Q-Dat Systems |
| `Documentatie/School Docs/` | Projecthandleiding, beoordelingsformulieren en richtlijnen |

## Kernvereisten

- **T1** ATmega328P als microcontroller
- **T2/T3** Programmeren in C, volgens de C-stijl- en programmeerrichtlijnen, code in het Engels
- **T4** DC-motoren via een L298N H-brug, snelheid geregeld met PWM
- **T5** Besturing en tests via de USART
- **T6** Voeding uit batterijen, met polariteits- en kortsluitbeveiliging
- **B1** Maximaal &euro;50,- aan aanvullende componenten

## Team

Projectgroep **Q-Dat Systems**:

Mohammed &middot; Tygo &middot; Floris &middot; Milan &middot; Mathijs &middot; Nathan

---

<p align="center">
  <sub>Q-Dat Systems &middot; Academie Engineering en Automotive &middot; Hogeschool van Arnhem en Nijmegen</sub>
</p>
