# Esperimento

## Utilizzo compressore

Dovrebbero esserci già 2 bombole su 3 cariche di aria, c'è un regolatore di pressione a monte già regolato a 4 bar, e uno a valle già impostato a 2 bar.

Per attivare il sistema va solo aperto il rubinetto vicino il regolatore a monte (quello col filtro sotto). E poi aperto eventualmente il rubinetto dell'esperimento.

## Test valvola accesa con l'alimentatore senza circuito

 - Collegare l'elettrovalvola **DIRETTAMENTE** all'alimentatore.
 - Controllare che l'alimentatore sia regolato a 24V e con limite Ampere maggiore di 0.5
 - Fissare i tubi, fissare l'ugello al tavolo con la morsa

Per questo esperimento conviene utilizzare solo il programma di calibrazione, esso non fa altro che leggere l'output dell'elettrovalvola.

Il programma si trova in:

./original/Load_Cell_calibration/calibration.ino

Per leggere l'output serve una shell UART a frequenza 57600.

Su linux uso il programma "screen", su windows mi pare esista PuTTY.

Non è necessario salvare i dati su file, basta che superi gli 840 e arrivi a circa 847 che dovrebbero corrispondere a circa 100 mN, ovviamente se c'è un modo semplice per salvare i dati con putty meglio salvarli su file.

Una volta avviato il programma, nella seriale appare la lettura. Aprire l'elettrovalvola dandole corrente dalll'alimentatore e leggere il nuovo valore.

SE esso non è il valore atteso di 100 mN non ha senso proseguire i test, bisogna capire perché non dà 100 mN

è quindi necessario analizzare i dati man mano che vengono letti, oppure salvarli in un file ed elaborarli con Matlab o simili.

## Test PWM a 30 Hz

 - Spegnere l'alimentatore in modo che l'elettrovalvola stia chiusa.
 - Collegare di nuovo l'elettrovalvola alla porta sulla breadboard.

Caricare in arduino mega il programma dentro:

./freq_experiment/Load_Cell_Reader/load_cell_reader.ino

Caricare in arduino nano il programma che genera un PWM:

./freq_experiment/PWM_generator/pwm_gen.ino

Al tempo della scrittura di questo file il programma sarà già impostato per generare un PWM a 30Hz e registrare 16 step: da 0 a 15.

A riga 24 c'è:
```
#define DELAY 500
```

Vuol dire che ad ogni step di PWM tiene il valore fisso per 500 ms. Consiglio di mettere 1000.

Una volta caricato su arduino nano il pwm_gen, si hanno 10 secondi per controllare tutto prima che parta il programma. L'output in seriale sarà solo quello necessario all'esperimento.

## Libreria per leggere da HX711

Al link trovate la libreria:
https://github.com/FedericoFavotto/HX711_ADC

Va installata in Arduino/libraries (non ricordo la cartella di windows, basta fare un git clone di quel repository dentro la cartella giusta di windows).