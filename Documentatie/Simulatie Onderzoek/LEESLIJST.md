# Leeslijst — simulatie + automatisch afstellen van de lijnvolger

Zij-onderzoek naast Project 1 NL (Robot-car). Valt **buiten** de gestelde
projecteisen; raakt alleen F3.1 ("een lijn kunnen volgen").

Alle gegevens hieronder zijn nagekeken bij de uitgever, arXiv, IEEE, ACM of ASME
(september 2026). PDF's kun je in `Papers/` zetten.

Legenda: **[gratis]** = vrij te downloaden · **[HAN]** = achter betaalmuur, via
de HAN-bibliotheek waarschijnlijk wel binnen te halen.

---

## Blok A — Waarom simulaties liegen

Dit blok beschrijft precies het risico van jouw plan: een regelaar die in
simulatie prachtig rondrijdt en op de echte vloer meteen de bocht uit vliegt.
Als je maar drie dingen leest, lees dan hieruit.

**A1. Mouret & Chatzilygeroudis (2017) — "20 Years of Reality Gap: a few
Thoughts about Simulators in Evolutionary Robotics"** **[gratis]**
GECCO '17 Companion, ACM, pp. 1121–1124 · DOI `10.1145/3067695.3082052`
PDF: http://www.cmap.polytechnique.fr/~nikolaus.hansen/proceedings/2017/GECCO/companion/companion_files/wksp147s1-file1.pdf

> Vier pagina's, twintig jaar ervaring, nuchter opgeschreven. Hoogste
> informatiedichtheid van de hele lijst. **Begin hier.**

**A2. Jakobi, Husbands & Harvey (1995) — "Noise and the Reality Gap: The Use of
Simulation in Evolutionary Robotics"** **[HAN]**
ECAL 1995, LNCS 929, Springer, pp. 704–720 · DOI `10.1007/3-540-59496-5_337`

> Het startpunt van het vakgebied; introduceert de term *reality gap*.
> Meenemen: **voeg bewust ruis toe** aan je gesimuleerde sensoren en motoren.
> Een regelaar die alleen werkt bij perfecte metingen is waardeloos. Goedkoopste
> en belangrijkste ingreep in je hele opzet.

**A3. Jakobi (1997) — "Evolutionary Robotics and the Radical
Envelope-of-Noise Hypothesis"** **[HAN]**
Adaptive Behavior 6(2), pp. 325–368 · DOI `10.1177/105971239700600205`

> Werkt A2 uit tot een recept: bepaal wat je écht vertrouwt aan je simulatie, en
> varieer al het andere zo hard dat de optimizer er niet op kan leunen.
> Voor jou: vertrouwde basis ≈ {wielbasis, wieldiameter, sensorafstanden} —
> dingen die je met een schuifmaat nameet. Wrijving, motor-deadband,
> tapereflectie en batterijspanning laat je juist willekeurig variëren.

**A4. Tobin et al. (2017) — "Domain Randomization for Transferring Deep Neural
Networks from Simulation to the Real World"** **[gratis]**
IROS 2017, IEEE, pp. 23–30 · arXiv:`1703.06907` · DOI `10.1109/IROS.2017.8202133`
https://arxiv.org/abs/1703.06907

> Neem het *idee* over, niet de methode — zij trainen neurale netwerken op
> beelden, jij tunet vier getallen. Vertaling: optimaliseer niet op één baan,
> maar op honderd willekeurige banen met per run andere wrijving, deadband en
> accuspanning. Wat daar gemiddeld goed uit komt, overleeft de echte vloer.

**A5. Muratore et al. (2022) — "Robot Learning From Randomized Simulations:
A Review"** **[gratis, open access]**
Frontiers in Robotics and AI 9:799893 · DOI `10.3389/frobt.2022.799893`
https://arxiv.org/abs/2111.00956

> Overzichtsartikel. Vooral bruikbaar als bronnenkaart. Grote delen gaan over
> deep reinforcement learning en zijn voor jou niet relevant — lees selectief.

---

## Blok B — De regelaar zelf

**B1. Wescott (2000) — "PID Without a PhD"** **[gratis]**
Embedded Systems Programming, oktober 2000
https://www.wescottdesign.com/articles/pid/pidWithoutAPhd.pdf

> Klassieker, geschreven voor precies jouw situatie: PID in C op een kleine
> microcontroller, zonder regeltechniek vooraf. Windup, afgeleide op de meting
> i.p.v. op de fout, vaste-komma-rekenwerk.
>
> **Belangrijkste detail voor je simulatie:** de ATmega328P heeft geen FPU, dus
> je regelaar wordt integerwerk. Simuleer dan ook de *integer*-versie inclusief
> afkapping — anders transfereren je getunede waarden niet en heb je voor niets
> gerekend.

**B2. Ziegler & Nichols (1942) — "Optimum Settings for Automatic Controllers"**
**[HAN]**
Transactions of the ASME 64(8), pp. 759–765 · DOI `10.1115/1.4019264`

> Je **referentiepunt**. "Onze optimizer vond goede waarden" is een lege
> bewering zonder vergelijking. Met een Ziegler–Nichols-afstelling ernaast wordt
> het een meetbaar resultaat — en dat is wat een tutor onder "onderbouwing van
> ontwerpkeuzes" wil zien.

---

## Blok C — Lijnvolgers concreet

**C1. Oguten & Kabas (2021) — "PID Controller Optimization for Low-cost Line
Follower Robots"** **[gratis]**
arXiv:`2111.04149` · https://arxiv.org/abs/2111.04149

> Het dichtst bij jouw project van alles hier: PID-optimalisatie voor een
> goedkope differentieel aangedreven lijnvolger met reflectiesensor en
> motordriver — dezelfde klasse hardware als de Robot-car. Inclusief publieke
> broncode.
>
> **Lees dit vóór je begint te bouwen.** Hun handmatige/heuristische aanpak is
> je ondergrens: komt jouw simulatie + optimizer daar niet overheen, dan is de
> extra complexiteit niet te verdedigen. Preprint, geen peer review — kritisch
> lezen.

**C2. Pakdaman & Sanaatiyan (2009) — "Design and Implementation of Line
Follower Robot"** **[HAN]**
ICCEE 2009, vol. 2, IEEE, pp. 585–590 · DOI `10.1109/ICCEE.2009.43`

> Ouderwets en juist daarom bruikbaar: complete bouwbeschrijving met een
> IR-sensorarray, inclusief de plaatsingsproblemen. De geometrie van je
> sensorbalk (onderlinge afstand, afstand tot de wielas) bepaalt hoe je je
> positiefout berekent — die maten moeten in de simulatie kloppen met de echte
> auto.

---

## Blok D — Optimalisatie en kinematica

**D1. Hansen (2016) — "The CMA Evolution Strategy: A Tutorial"** **[gratis]**
arXiv:`1604.00772` · https://arxiv.org/abs/1604.00772

> **Nog niet toepassen.** Begin met random search over vier parameters — twintig
> regels code, werkt in zo'n kleine zoekruimte verrassend goed. Pak dit er pas
> bij als je aantoonbaar tegen de grenzen daarvan aanloopt. Zo niet: noem het in
> je rapport als afweging (overwogen, niet nodig gebleken). Dat leest beter dan
> een ongebruikt algoritme.

**D2. Siegwart, Nourbakhsh & Scaramuzza (2011) — *Introduction to Autonomous
Mobile Robots*, 2e druk, MIT Press** **[HAN]**

> Hoofdstuk 3 geeft het kinematisch model van een differentieel aangedreven
> robot — de kern van je simulator. In essentie: `v = (v_r + v_l) / 2` en
> `ω = (v_r − v_l) / L`, met `L` de wielbasis. Meer heb je voor een 2D-simulatie
> niet nodig. Check de HAN-bibliotheek op een digitale licentie voor je dit boek
> koopt.

**D3. Dudek & Jenkin (2010) — *Computational Principles of Mobile Robotics*,
2e druk, Cambridge University Press** **[HAN]**

> Alternatief voor D2, met meer aandacht voor odometrie en foutenopbouw.
> Optioneel — pak D2 óf D3, niet allebei.

---

## Leesvolgorde

1. **A1** — vier pagina's, meteen het hele probleem helder
2. **C1** — jouw project, al eens gedaan door iemand anders
3. **B1** — hoe je de regelaar fatsoenlijk in C schrijft
4. **D2 hoofdstuk 3** — de vergelijkingen voor je simulator
5. **A2** en **A3** — pas lezen als de simulator draait en de eerste resultaten
   verdacht goed zijn. Dat moment komt.
6. De rest naar behoefte.

---

## Vragen om tijdens het lezen bij te houden

- Welke grootheden ga ik aan de echte auto **meten** i.p.v. schatten?
  (PWM→toerental-curve, deadband, wielbasis, wieldiameter, sensorafstanden,
  sensorrespons op zwart tape op linoleum.)
- Met welke tijdstap draait de simulatie, en komt die overeen met de echte
  regellus-frequentie op de ATmega328P?
- Hoe genereer ik willekeurige banen die lijken op de echte baan in de gang van
  R29 — welke bochtstralen komen daar voor?
- Wat is mijn fitness-functie precies? Rondetijd alleen is niet genoeg; er moet
  straf op lijnverlies en op oscillatie.
- **Hoe groot is het verschil tussen simulatie en werkelijkheid, in meetbare
  eenheden?** Dit is het eigenlijke resultaat van het onderzoek — interessanter
  dan de getunede waarden zelf, en het enige deel dat een echte onderzoeksvraag
  beantwoordt.

---

`referenties.bib` in `Literatuur/` bevat dezelfde bronnen als BibTeX, voor als
je hier later in het productrapport naar verwijst. Weggooien mag.
