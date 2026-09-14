# Functioneel ontwerp — aanvulling hoofdstuk 2.2 en 2.3

**Status:** concept, uitgewerkt voor F1, F2 en F3. Wordt aangevuld zodra F4 tot en met F7 zijn vastgesteld.
**Bron van de eisen:** Projecthandleiding PROJ1-V v1.0, §3.5 (concept technische eisen T1–T7 en budgeteis B1) en §3.7 en §3.11.
**Doel:** de twee subparagrafen die het Template productrapport v1.4 voor hoofdstuk 2 voorschrijft, maar die nu nog ontbreken.

---

## 2.2 Technische specificaties

Naast de functionele specificaties uit paragraaf 2.1 stelt de opdrachtgever een aantal aanvullende eisen aan het product zelf: de te gebruiken microcontroller, de programmeertaal, de manier waarop de motoren worden aangedreven en de wijze van voeding. Deze technische specificaties beschrijven waaraan het product moet voldoen, niet hoe wij dat gaan realiseren; die uitwerking volgt in hoofdstuk 3.

De nummering T1 tot en met T7 komt overeen met de concept technische eisen uit de projecthandleiding, zodat de opdrachtgever zijn eigen eisen terugvindt. T8 tot en met T11 zijn door ons toegevoegd omdat de functionele specificaties uit paragraaf 2.1 daar niet zonder kunnen. De kolom *Herleidbaar naar* verwijst naar de functionele specificatie die de betreffende technische eis mogelijk maakt.

### Tabel 2 — SMART technische specificaties

| # | MoSCoW | Omschrijving | Herleidbaar naar |
|---|---|---|---|
| T1 | M | De besturing van de robot-car wordt gerealiseerd met een ATmega328(p)-microcontroller. | F1, F2, F3 |
| T1.1 | M | In periode 1 wordt het tussenproduct via rapid prototyping gerealiseerd met de Arduino IDE op dezelfde ATmega328(p); in periode 2 wordt de microcontroller aangestuurd door rechtstreeks de registers te bewerken. | F1, F2, F3 |
| T2 | M | De software voor de microcontroller wordt geschreven in de programmeertaal C. | F1, F2, F3 |
| T3 | M | De C-code voldoet aan de programmeerrichtlijnen zoals aangeleerd in de opleiding (C-style programming guidelines v3.1). | F1, F2, F3 |
| T4 | M | De dc-motoren van de robot-car worden aangedreven via een H-brug, zodat beide draairichtingen per motor mogelijk zijn. | F1.1, F1.2 |
| T4.1 | M | Voor het regelen van de motorsnelheid wordt gebruikgemaakt van de PWM van de microcontroller; er wordt geen aanvullende hardware voor snelheidsregeling toegepast. | F1.3 |
| T4.2 | M | Voor de H-brug wordt gebruikgemaakt van een L298N dual H-bridge driverboard. | F1.1, F1.2 |
| T4.3 | M | De robot-car wordt aangedreven door de vier dc-motoren van het door de opdrachtgever geleverde chassis. | F1 |
| T5 | M | Voor testdoeleinden kan de robot-car bestuurd worden via de seriële interface, de USART van de ATmega328(p). | F1, F2.5 |
| T5.1 | M | Via de USART kan de robot-car vooruit en achteruit rijden. | F1.1 |
| T5.2 | M | Via de USART kan de robot-car links- en rechtsom draaien. | F1.2 |
| T5.3 | M | Via de USART kan de rijsnelheid van de robot-car worden ingesteld. | F1.3 |
| T5.4 | M | Via de USART kan de autonome mode worden gestart en gestopt. | F2.5 |
| T6 | M | Voor de voeding van de robot-car wordt gebruikgemaakt van batterijen. | F1, F2, F3 |
| T6.1 | M | De schakeling is beveiligd tegen het verkeerd om aansluiten van de voeding. | – |
| T6.2 | M | De batterijen zijn beveiligd tegen kortsluiting door een beveiliging die zo dicht mogelijk bij de batterij is geplaatst. | – |
| T6.3 | M | Een LED geeft aan dat de robot-car wordt gevoed; deze LED brandt continu zolang de robot-car is ingeschakeld. | F1, F2, F3 |
| T6.4 | M | De voedingsspanning van de microcontroller blijft binnen het door de fabrikant toegestane bereik, ook wanneer alle vier de motoren gelijktijdig maximaal worden belast. | F1, F2, F3 |
| T6.5 | C | De batterijspanning is voor de microcontroller meetbaar, zodat het batterijniveau kan worden weergegeven. | F5.4 |
| T7 | M | De achterzijde van de robot-car is voorzien van een reflector, zodat deze robot-car op zijn beurt door een andere robot-car gevolgd kan worden. | F3.2 |
| T8 | M | De robot-car meet de vrije ruimte recht vooruit met een afstandssensor. | F2.1 t/m F2.4, F3.2 |
| T8.1 | M | De afstandssensor heeft een meetbereik van ten minste 10 tot 50 cm. | F2.2, F3.2 |
| T8.2 | M | De afstandssensor levert ten minste 5 metingen per seconde. | F2.2 |
| T9 | M | De robot-car detecteert de te volgen lijn met een lijnsensor aan de onderzijde van het chassis. | F3.1 |
| T9.1 | M | De lijnsensor detecteert zwart tape van 1 tot 2 cm breed op het linoleum van de gang op de begane grond van gebouw R29. | F3.1 |
| T9.2 | M | De lijnsensor kan onderscheiden of de lijn zich links van, onder, of rechts van het midden van de robot-car bevindt, zodat de afwijking kan worden bijgestuurd. | F3.1 |
| T10 | M | De robot-car is voorzien van handbediening op de auto zelf, bestaande uit een display en bedieningstoetsen. | F5, F6 |
| T10.1 | M | Het display geeft vier regels gelijktijdig weer, volgens de regelindeling van figuur 2. | F5.1 t/m F5.4 |
| T10.2 | S | De bediening bestaat uit drie toetsen: omhoog, omlaag en OK. | F6 |
| T11 | C | Aan de achterzijde van de robot-car zitten twee LED's, links en rechts, die de draairichting aangeven. | F1.4 |

### Tabel 3 — Budgeteisen

| # | MoSCoW | Omschrijving | Herleidbaar naar |
|---|---|---|---|
| B1 | M | Het chassis met de vier dc-motoren wordt geleverd door de opdrachtgever. | T4.3 |
| B2 | M | De aanvullende kosten voor het prototype bedragen maximaal € 50,- aan onderdelen. Willen wij meer besteden, dan wordt dit vooraf met de tutor overlegd. | T8, T9, T10 |
| B3 | M | Kosten voor het gebruik van het Fablab worden meegenomen in de berekening van de uiteindelijke productiekosten, ook wanneer ze voor het project zelf niet in rekening worden gebracht. | – |

### Open punten bij de technische specificaties

1. **T5.4** — In de concepteisen staat T5.4 zonder omschrijving. Wij hebben deze ingevuld als "de autonome mode kan via de USART worden gestart en gestopt", aansluitend op F2.5. Graag bevestigen of dit is wat u bedoelde.
2. **H-brug** — In §3.5 van de projecthandleiding staat een L298N, in implementatietip 5 een L294n. Wij gaan uit van de L298N; graag bevestigen.
3. **Snelheidsmeting** — Op het beoordelingsformulier staat dat de robot-car sensoren heeft voor het meten van afstand *en snelheid*. In de concepteisen komt snelheidsmeting niet voor. Moet de werkelijke rijsnelheid gemeten worden, of volstaat het instellen van de snelheid via PWM (T4.1)?
4. **T10.1** — Wij stellen een display van vier regels voor, omdat F5.1 tot en met F5.4 samen vier gegevens moeten tonen. Gaat u hiermee akkoord?
5. **T6.5** — Het meten van de batterijspanning staat op Could, in lijn met F5.4. Als u het batterijniveau wel als vaste eis wilt, wordt dit een Must.

---

## 2.3 User interface

In deze paragraaf staat hoe de robot-car er voor de gebruiker uitziet en hoe hij zich gedraagt: welke bedieningselementen er zijn, wat er op het display verschijnt, en wat er aan de uitvoer verandert wanneer de gebruiker of de omgeving iets doet. De schetsen zijn een tweede weergave van dezelfde functionaliteit die in paragraaf 2.1 in tabelvorm staat, en zijn bedoeld om samen met de opdrachtgever vast te stellen dat wij hetzelfde beeld voor ogen hebben. De schetsen zijn niet op schaal en leggen geen componentkeuze vast; die volgt in hoofdstuk 3.

### Figuur 1 — Bedieningselementen en sensoren van de robot-car

*Bestand: `Documentatie/Schetsen/UI-01 Bedieningselementen Robot-car.svg`*

Bovenaanzicht en achteraanzicht van de robot-car met de plaats van de afstandssensor (T8), de lijnsensor (T9), het display en de bedieningstoetsen (T10), de batterijen (T6), de richting-LED's (T11), de voedings-LED (T6.3) en de reflector (T7). De genummerde legenda koppelt elk element aan de specificatie waaruit het voortkomt.

### Figuur 2 — Displayschermen en menustructuur

*Bestand: `Documentatie/Schetsen/UI-02 Displayschermen en menustructuur.svg`*

Deel A legt de vaste regelindeling van het display vast: regel 1 toont de actieve mode en het batterijniveau, regel 2 de rijrichting of de status, regel 3 de tijd in de huidige mode en regel 4 de totale gebruikstijd. Deze indeling is in elke mode gelijk; alleen de inhoud van regel 1 en 2 verandert. Deel B toont het hoofdmenu waarmee de gebruiker een mode kiest (F6) en de schermen die daarbij horen: Autonoom rijdend en Autonoom bij stilstand voor een obstakel (F2), en Slave volgend en Slave wachtend op de voorganger (F3). De schermen voor de handbediening (F4) volgen dezelfde indeling en worden bij F4 uitgewerkt.

### Figuur 3 — Invoer, uitvoer en gedrag van de richting-LED's

*Bestand: `Documentatie/Schetsen/UI-03 Invoer-uitvoer en LED-gedrag.svg`*

Deel A geeft in één overzicht weer welke invoer de robot-car krijgt (bedieningstoetsen, afstandssensor, lijnsensor, USART-commando's) en welke uitvoer hij daarop geeft (display, richting-LED's, voedings-LED, aandrijving). Deel B laat zien wat er met de uitvoer gebeurt als de invoer verandert: zolang de robot-car naar links draait knippert alleen de linker-LED met 50 ms aan en 50 ms uit, en die LED gaat direct uit zodra het draaien stopt (F1.4). De voedings-LED brandt onafhankelijk daarvan continu (T6.3).

### Figuur 4 — Gedrag op het parcours bij F2 en F3

*Bestand: `Documentatie/Schetsen/UI-04 Gedrag F2 autonoom en F3 slave.svg`*

Deel A toont de drie situaties van het autonoom rijden met de bijbehorende maten: doorrijden zolang er recht vooruit ten minste 30 cm vrij is (F2.1), remmen en stoppen bij een obstakel binnen 20 cm met minimaal 2 cm ruimte over (F2.3), en binnen 4 seconden een nieuwe richting kiezen door te draaien of maximaal 30 cm achteruit te rijden (F2.4). Deel B toont de slave mode: de robot-car blijft binnen 5 cm van de lijn (F3.1) en houdt 10 tot 50 cm afstand tot de voorganger, die aan de achterzijde een reflector draagt (F3.2, T7). Bij elke situatie staat wat het display op dat moment toont.

### Open punten bij de user interface

1. **Plaats van display en menu** — Wij gaan ervan uit dat het display en de toetsen op de auto zelf zitten, zoals in figuur 1. Als u ze liever in de afstandsbediening ziet, verandert figuur 1 en figuur 2.
2. **Regel 2 van het display** — In de concepteisen staat de snelheid op het display (F5.2); wij tonen daar nu de rijrichting en de mode. Wilt u de snelheid er alsnog bij, dan is een vijfde regel of een tweede scherm nodig.
3. **Wisselen van mode** — Mag de gebruiker tijdens het rijden van mode wisselen, of moet de robot-car daarvoor eerst stilstaan?
4. **Parcours** — Voor figuur 4 hebben wij de afmetingen van het testparcours en het lijnparcours nodig, en de afmetingen en het materiaal van de obstakels.

---

## Verwerken in het Word-document

De vier schetsen zijn SVG-bestanden. Word 2016 en later ondersteunt SVG rechtstreeks: **Invoegen → Afbeeldingen → Dit apparaat**, en daarna een bijschrift toevoegen via **Verwijzingen → Bijschrift invoegen**, zodat de figuurnummering automatisch meeloopt met de rest van het rapport. De tabellen staan ook als CSV in `Documentatie/Technische eisen MoSCoW.csv` en `Documentatie/Budgeteisen MoSCoW.csv`, in dezelfde opzet als de bestaande `Functionele eisen MoSCoW.csv`.
