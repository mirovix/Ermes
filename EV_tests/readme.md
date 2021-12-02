# ESPERIMENTO LABORATORIO
Test delle elettrovalvole per valutare le capacità di apertura chiusura e le possibilità di regolazione del PWM.

## Circuito
Il circuito è il seguente:
![Circuito](/imgs/circuito.jpg)

**Nota Bene:** 
 - Il mosfet è N non P quindi l'elettrovalvola NON va collegata come nel disegno, ma il negativo dell'elettrovalvola va al source del transistor, e il drain del transistor va direttamente a massa.
 - I tre pin I/O per leggere il PWM attuale vanno collegati a una resistenza di pull-down
 - Se si automatizza l'esperimento per consumare meno CO2 possibile, va collegato un altro pin da Nano a Mega che dia le info di sperimentazione. Con annessa resistenza di pull-down.

## Codice

PWM_generator carica un PWM hardcoded oppure con gli oscillatori di arduino, c'è una variabile preprocessor da dichiarare per cambiare stato

**RICORDA DI:**
 - settare i PIN PWM giusti nell'arduino PWM in modo da comunicare correttamente col Mega
 - Settare il PIN di sperimentazione
 - Vedere il formato dati
 - Decidere dopo quante oscillazioni passare da un PWM all'altro

## In Laboratorio

 1) Finisci il circuito collegando i pin mancanti.
 2) Collega la cella di carico, i fili VANNO TWISTATI
 3) Nel codice scrivi la PORTA CORRETTA da leggere, le info a questo link:
[ARDUINO MEGA PINMAP](https://www.arduino.cc/en/Hacking/PinMapping2560).
 4) Scrivi il PWM usato in arduino nano nella porta giusta: [ARDUINO NANO PINMAP](http://www.micheleardito.info/ma/it/arduino-it/arduino-nano-pinout/).
 5) Testa il codice senza CO2
 6) Leggi l'output da terminale e ridirezionalo su un file
 7) L'alimentatore dell'elettrovalvola va a 24V
 8) per testare scrivi in seriale un log da arduino nano, TOGLI IL SERIALE quando fai il test.

### Check list:
 1) debug commentato
 2) high frequency in PWM_gen commentato
 3) loop di controllo del PWM
 4) filo per startare l'esperimento con resistenza pull up
 5) Controlla il collegamento dell'HX711 se è corretto secondo i pin del codice
 6) console che reindirizza su file
 7) cortocircuita il filo di start

 ### Comandi utili
Redirect dev/ttyacm0 su received.txt:

 cat > received.txt < /dev/ttyACM0