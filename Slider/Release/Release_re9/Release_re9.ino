
// SLIDER 1 - Target
#define PIN_POS1_1 2    // fino a metà slider
#define PIN_POS2_1 3    // distanza di rilascio
#define PIN_HOME_1 4    // home = inizio slider
#define PIN_EM_1   5    // rilascio EM

//SLIDER 2 - Chaser
#define PIN_POS1_2 8    // fino a metà slider
#define PIN_POS2_2 9    // distanza di rilascio
#define PIN_HOME_2 10    // home = inizio slider
#define PIN_EM_2   11    // rilascio EM


unsigned long t_start, dt_st;

bool AZ_ST1 = false;
bool AZ_ST2 = false;
bool AZ_ST3 = false;
bool AZ_ST4 = false;
bool AZ_ST5 = false;
bool AZ_ST6 = false;
bool AZ_SU = false;
bool AZ_COMPL = true;


// Caso Start1 - H --> P2.1 --> EM --> P2.2 --> H
int AttesaStart1 =     6100;          // tempo di attesa per inizio rilascio dopo l'arrivo del comando START
int DurataRilascio1 =  1900;          // durata manovra di rilascio da H --> P2
int AttesaEM1 =        1600;          // tempo di attesa dopo per il rilascio magnetico dopo l'arrivo del comando START

// Caso Start2 - H --> P1 --> P2.1 --> EM --> P2.2 --> H
int AttesaStart2 =     4000;          // tempo di attesa per inizio preposizionamento dopo l'arrivo del comando START
int DurataRilascio2 =  900;           // durata manovra di rilascio da P1 --> P2
int AttesaEM2 =        600;           // tempo di attesa dopo per il rilascio magnetico dopo l'arrivo del comando START
int DurataPrePos2 =    2000;          // durata preposizionamento per rilascio corto
int AttesaPostPP2 =    1100;          // attesa dopo preposizionamento

// Caso Start3 - H --> P1.1 --> EM --> P1.2 --> H
int AttesaStart3 =     2000;          // tempo di attesa per inizio preposizionamento dopo l'arrivo del comando START
int DurataRilascio3 =  900;           // durata manovra di rilascio da P1 --> P2
int AttesaEM3 =        400;           // tempo di attesa dopo per il rilascio magnetico dopo l'arrivo del comando START
int DurataPrePos3 =    1000;          // durata preposizionamento per rilascio corto
int AttesaPostPP3 =    2000;          // attesa dopo preposizionamento

// Caso Start4 - H --> P2 --> EM --> H
int AttesaStart4 =     5200;          // tempo di attesa per inizio rilascio dopo l'arrivo del comando START
int DurataPrePos4 =    1900;          // durata preposizionamento per rilascio lungo
int AttesaEM4 =        900;           // tempo di attesa dopo per il rilascio magnetico dopo l'arrivo del comando START

// Caso Start5 - H --> P1 --> EM --> H
int AttesaStart5 =     5200;          // tempo di attesa per inizio preposizionamento dopo l'arrivo del comando START
int DurataPrePos5 =    1900;          // durata preposizionamento per rilascio corto
int AttesaEM5 =        900;           // tempo di attesa dopo per il rilascio magnetico dopo l'arrivo del comando START


// Caso Start6 - Chaser: s3, Target: s5
int AttesaStart6 =     4000;          // tempo di attesa per inizio preposizionamento dopo l'arrivo del comando START
int DurataPrePos6 =    2000;          // durata manovra di preposizionamento da H --> P1
int DurataPostPP6 =    1100;          // Attesa prima dell'inizio della manovra
int DurataRilascio6 =   900;          // durata manovra di rilascio da H --> P1
int AttesaEM6 =         600;          // tempo di attesa dopo per il rilascio magnetico dopo preposizionamento



int AttesaRip =       1500;           // tempo di attesa per il riposizionamento dopo il rilascio
int DurataHoming =    6000;           // durata manovra di homing P1 --> H
int DurataRip =       5000;           // durata riposizionamento generico

char ch;
String line;




void setup() {

  Serial.begin(9600);
  
  
  // SLIDER 1
  pinMode(PIN_POS1_1, OUTPUT);
  pinMode(PIN_POS2_1, OUTPUT);
  pinMode(PIN_HOME_1, OUTPUT);
  pinMode(PIN_EM_1  , OUTPUT);
  

  // SLIDER 2
  pinMode(PIN_POS1_2, OUTPUT);
  pinMode(PIN_POS2_2, OUTPUT);
  pinMode(PIN_HOME_2, OUTPUT);
  pinMode(PIN_EM_2  , OUTPUT);

  digitalWrite(PIN_EM_1, HIGH); // con HIGH apri il p-mosfet 
  digitalWrite(PIN_EM_2, HIGH); // con HIGH apri il p-mosfet 

  digitalWrite(PIN_HOME_1, LOW); // con LOW chiudi il p-mosfet
  digitalWrite(PIN_HOME_2, LOW); // con LOW chiudi il p-mosfet
  delay(5000);
  digitalWrite(PIN_HOME_1, HIGH); // con HIGH chiudi il p-mosfet
  digitalWrite(PIN_HOME_2, HIGH); // con HIGH chiudi il p-mosfet
  delay(500);


  Serial.println("Setup completed - ready for business");
  Serial.println("type...");

}

void loop() {
  
  
  dt_st = millis() - t_start; //timer



  // CHECK POST OGNI AZIONE COMPLETATA --> evito che si muova a caso 
  if(AZ_COMPL){

    // SLIDER 1
    digitalWrite(PIN_POS1_1, HIGH); // con HIGH apri il p-mosfet 
    digitalWrite(PIN_POS2_1, HIGH); // con HIGH apri il p-mosfet
    digitalWrite(PIN_HOME_1, HIGH); // con HIGH apri il p-mosfet

      // SLIDER 2
    digitalWrite(PIN_POS1_2, HIGH); // con HIGH apri il p-mosfet 
    digitalWrite(PIN_POS2_2, HIGH); // con HIGH apri il p-mosfet
    digitalWrite(PIN_HOME_2, HIGH); // con HIGH apri il p-mosfet

}


  // ----------------------------------------------------------------------------

  // MANOVRA DI RILASCIO 1 = H --> P2.1 --> EM --> P2.2 --> H  ------------- stessa per entrambi gli slider     
  if (AZ_ST1 && (dt_st >= AttesaStart1)){  // Se il comando START1 è stato ricevuto e sono passati 'AttesaStart1' secondi
      
    // esegui la manovra
    digitalWrite(PIN_POS2_1, LOW);  // con LOW chiudi il p-mosfet
    digitalWrite(PIN_POS2_2, LOW);  // con LOW chiudi il p-mosfet
    Serial.println("Initialize POS2");
    
    delay(AttesaEM1);
    digitalWrite(PIN_EM_1, HIGH); // con HIGH apri il p-mosfet
    digitalWrite(PIN_EM_2, HIGH); // con HIGH apri il p-mosfet
    Serial.println("MAGNETIC RELEASE");
    
    delay(DurataRilascio1 - AttesaEM1);
    digitalWrite(PIN_POS2_1, HIGH);  // con HIGH apri il p-mosfet
    digitalWrite(PIN_POS2_2, HIGH);  // con HIGH apri il p-mosfet
    
    delay(AttesaRip);
    digitalWrite(PIN_HOME_1, LOW); // con LOW chiudi il p-mosfet
    digitalWrite(PIN_HOME_2, LOW); // con LOW chiudi il p-mosfet
    Serial.println("Initialize HOMING");
    delay(DurataHoming);
    Serial.println("HOMING completed");

    AZ_COMPL = true;
    AZ_ST1 = false;  // sovrascrivi il comando per evitare che lo ripeta  
    Serial.println("type...");   
    }


  // ----------------------------------------------------------------------------

  // MANOVRA DI RILASCIO 2 = H --> P1 --> P2.1 --> EM --> P2.2 --> H         
  if (AZ_ST2 && (dt_st >= AttesaStart2)){  // Se il comando START2 è stato ricevuto e sono passati 'AttesaStart2' secondi
      
    // esegui il posizionamento iniziale 
    digitalWrite(PIN_POS1_1, LOW);  // con LOW chiudi il p-mosfet
    digitalWrite(PIN_POS1_2, LOW);  // con LOW chiudi il p-mosfet
    Serial.println("Initialize POS1");
    
    delay(DurataPrePos2);
    digitalWrite(PIN_POS1_1, HIGH);  // con HIGH apri il p-mosfet
    digitalWrite(PIN_POS1_2, HIGH);  // con HIGH apri il p-mosfet
    Serial.println("Prepositioning (POS1) completed");
    delay(AttesaPostPP2);

    // esegui la manovra
    digitalWrite(PIN_POS2_1, LOW);  // con LOW chiudi il p-mosfet
    digitalWrite(PIN_POS2_2, LOW);  // con LOW chiudi il p-mosfet
    Serial.println("Initialize POS2");
    
    delay(AttesaEM2);
    digitalWrite(PIN_EM_1, HIGH); // con HIGH apri il p-mosfet
    digitalWrite(PIN_EM_2, HIGH); // con HIGH apri il p-mosfet
    Serial.println("MAGNETIC RELEASE");
    
    delay(DurataRilascio2 - AttesaEM2);
    digitalWrite(PIN_POS2_1, HIGH);  // con HIGH apri il p-mosfet
    digitalWrite(PIN_POS2_2, HIGH);  // con HIGH apri il p-mosfet
    
    delay(AttesaRip);
    digitalWrite(PIN_HOME_1, LOW); // con LOW chiudi il p-mosfet
    digitalWrite(PIN_HOME_2, LOW); // con LOW chiudi il p-mosfet
    Serial.println("Initialize Homing");
    delay(DurataHoming);
    Serial.println("HOMING completed");
    
    AZ_COMPL = true;
    AZ_ST2 = false;  // sovrascrivi il comando per evitare che lo ripeta  
    Serial.println("type...");   
    }
 
  // MANOVRA DI RILASCIO 3 = H --> P1.1 --> EM --> P1.2 --> H  ------------- stessa per entrambi gli slider     
  if (AZ_ST3 && (dt_st >= AttesaStart3)){  // Se il comando START3 è stato ricevuto e sono passati 'AttesaStart3' secondi
      
    // esegui la manovra
    digitalWrite(PIN_POS1_1, LOW);  // con LOW chiudi il p-mosfet
    digitalWrite(PIN_POS1_2, LOW);  // con LOW chiudi il p-mosfet
    Serial.println("Initialize POS1");
    
    delay(AttesaEM1);
    digitalWrite(PIN_EM_1, HIGH); // con HIGH apri il p-mosfet
    digitalWrite(PIN_EM_2, HIGH); // con HIGH apri il p-mosfet
    Serial.println("MAGNETIC RELEASE");
    
    delay(DurataRilascio1 - AttesaEM1);
    digitalWrite(PIN_POS1_1, HIGH);  // con HIGH apri il p-mosfet
    digitalWrite(PIN_POS1_2, HIGH);  // con HIGH apri il p-mosfet
    
    delay(AttesaRip);
    digitalWrite(PIN_HOME_1, LOW); // con LOW chiudi il p-mosfet
    digitalWrite(PIN_HOME_2, LOW); // con LOW chiudi il p-mosfet
    Serial.println("Initialize HOMING");
    delay(DurataHoming);
    Serial.println("HOMING completed");

    AZ_COMPL = true;
    AZ_ST3 = false;  // sovrascrivi il comando per evitare che lo ripeta  
    Serial.println("type...");   
    }


  // ----------------------------------------------------------------------------

  // MANOVRA DI RILASCIO 4 = H --> P2 --> EM --> H      
  if (AZ_ST4 && (dt_st >= AttesaStart4)){  // Se il comando START4 è stato ricevuto e sono passati 'AttesaStart4' secondi
      
    // esegui la manovra
    digitalWrite(PIN_POS2_1, LOW);  // con LOW chiudi il p-mosfet
    digitalWrite(PIN_POS2_2, LOW);  // con LOW chiudi il p-mosfet
    Serial.println("Initialize POS2");
    delay(DurataPrePos4);
    
    delay(AttesaEM4);
    digitalWrite(PIN_EM_1, HIGH); // con HIGH apri il p-mosfet
    digitalWrite(PIN_EM_2, HIGH); // con HIGH apri il p-mosfet
    Serial.println("MAGNETIC RELEASE");
    
    delay(100);
    digitalWrite(PIN_POS2_1, HIGH);  // con HIGH apri il p-mosfet
    digitalWrite(PIN_POS2_2, HIGH);  // con HIGH apri il p-mosfet
    
    delay(AttesaRip);
    digitalWrite(PIN_HOME_1, LOW); // con LOW chiudi il p-mosfet
    digitalWrite(PIN_HOME_2, LOW); // con LOW chiudi il p-mosfet
    Serial.println("Initialize Homing");
    delay(DurataHoming);
    Serial.println("HOMING completed");
    
    AZ_COMPL = true;
    AZ_ST4 = false;  // sovrascrivi il comando per evitare che lo ripeta  
    Serial.println("type...");   
    }

  // MANOVRA DI RILASCIO 5 = H --> P1 --> EM --> H      
  if (AZ_ST5 && (dt_st >= AttesaStart5)){  // Se il comando START5 è stato ricevuto e sono passati 'AttesaStart5' secondi
      
    // esegui la manovra
    digitalWrite(PIN_POS1_1, LOW);  // con LOW chiudi il p-mosfet
    digitalWrite(PIN_POS1_2, LOW);  // con LOW chiudi il p-mosfet
    Serial.println("Initialize POS1");
    delay(DurataPrePos5);
    
    delay(AttesaEM5);
    digitalWrite(PIN_EM_1, HIGH); // con HIGH apri il p-mosfet
    digitalWrite(PIN_EM_2, HIGH); // con HIGH apri il p-mosfet
    Serial.println("MAGNETIC RELEASE");
    
    delay(100);
    digitalWrite(PIN_POS1_1, HIGH);  // con HIGH apri il p-mosfet
    digitalWrite(PIN_POS1_2, HIGH);  // con HIGH apri il p-mosfet
    
    delay(AttesaRip);
    digitalWrite(PIN_HOME_1, LOW); // con LOW chiudi il p-mosfet
    digitalWrite(PIN_HOME_2, LOW); // con LOW chiudi il p-mosfet
    Serial.println("Initialize Homing");
    delay(DurataHoming);
    Serial.println("HOMING completed");
    
    AZ_COMPL = true;
    AZ_ST5 = false;  // sovrascrivi il comando per evitare che lo ripeta  
    Serial.println("type...");   
    }

  // MANOVRA DI RILASCIO 6 = Chaser: s3, Target: s5
  if (AZ_ST6 && (dt_st >= AttesaStart6)){  // Se il comando START6 è stato ricevuto e sono passati 'AttesaStart6' secondi

    // Preposizionamento del Target
    digitalWrite(PIN_POS1_1, LOW);  // con LOW chiudi il p-mosfet
    Serial.println("T: Initialize POS1");
    delay(DurataPrePos6);

    // esegui la manovra
    delay(DurataPostPP6);
    digitalWrite(PIN_POS1_2, LOW);  // con LOW chiudi il p-mosfet
    Serial.println("Ch: Initialize POS1");
    
    delay(AttesaEM6);
    digitalWrite(PIN_EM_1, HIGH); // con HIGH apri il p-mosfet
    digitalWrite(PIN_EM_2, HIGH); // con HIGH apri il p-mosfet
    Serial.println("MAGNETIC RELEASE");
    
    delay(DurataRilascio6 - AttesaEM6);
    digitalWrite(PIN_POS1_1, HIGH);  // con HIGH apri il p-mosfet
    digitalWrite(PIN_POS1_2, HIGH);  // con HIGH apri il p-mosfet
    
    delay(AttesaRip);
    digitalWrite(PIN_HOME_1, LOW); // con LOW chiudi il p-mosfet
    digitalWrite(PIN_HOME_2, LOW); // con LOW chiudi il p-mosfet
    Serial.println("Initialize HOMING");
    delay(DurataHoming);
    Serial.println("HOMING completed");
    

    AZ_COMPL = true;
    AZ_ST6 = false;  // sovrascrivi il comando per evitare che lo ripeta  
    Serial.println("type...");   
    }


  // ----------------------------------------------------------------------------

  // MANOVRA DI SETUP = P --> H --> EM on   
  if (AZ_SU){  // Se il comando SETUP è stato ricevuto
      
    // Prepara gli slider per la prox parabola
    digitalWrite(PIN_HOME_1, LOW);  // con LOW chiudi il p-mosfet
    digitalWrite(PIN_HOME_2, LOW);  // con LOW chiudi il p-mosfet
    Serial.println("Initialize HOMING");
    delay(DurataHoming);
    digitalWrite(PIN_HOME_1, HIGH);  // con HIGH apri il p-mosfet
    digitalWrite(PIN_HOME_2, HIGH);  // con HIGH apri il p-mosfet
    
    digitalWrite(PIN_EM_1, LOW); // con HIGH chiudi il n-mosfet
    digitalWrite(PIN_EM_2, LOW); // con HIGH chiudi il n-mosfet
    Serial.println("EM activated");
    delay(500);

    AZ_COMPL = true; 
    AZ_SU = false; // sovrascrivi il comando per evitare che lo ripeta   
    Serial.println("type...");    
    }


  // ----------------------------------------------------------------------------
   
    
    // LETTURA SERIALE

    if(Serial.available()){
     line = "";
     
     do{
       if(Serial.available()){
        ch = Serial.read();
        if(ch != '\n');
        line += ch;
       }
     }while(ch != '\n');
           
        // START1
        if((line == "s1\n") && (!AZ_ST1)){
        AZ_COMPL = false;
        AZ_ST1 = true;  // inizializza la manovra
        Serial.println("comando: START - mod1");
        AZ_SU = false;
        AZ_ST2 = false;
        AZ_ST3 = false;
        AZ_ST4 = false;
        AZ_ST5 = false;
        AZ_ST6 = false;
        // start timer per rilascio
        t_start = millis(); // dtST == 0 quando legge start e la manovra non è ancora iniziata
        delay(100); // delay di sicurezza di 100 ms 
        }

        // START2
        else if((line == "s2\n") && (!AZ_ST2)){
        AZ_COMPL = false;
        AZ_ST2 = true;  // inizializza la manovra
        Serial.println("comando: START - mod2");
        AZ_SU = false;
        AZ_ST1 = false;
        AZ_ST3 = false;
        AZ_ST4 = false;
        AZ_ST5 = false;
        AZ_ST6 = false;
        // start timer per rilascio
        t_start = millis(); // dtST == 0 quando legge start e la manovra non è ancora iniziata
        delay(100); // delay di sicurezza di 100 ms 
        }

        // START3
        else if((line == "s3\n") && (!AZ_ST3)){
        AZ_COMPL = false;
        AZ_ST3 = true;  // inizializza la manovra
        Serial.println("comando: START - mod3");
        AZ_SU = false;
        AZ_ST1 = false;
        AZ_ST2 = false;
        AZ_ST4 = false;
        AZ_ST5 = false;
        AZ_ST6 = false;
        // start timer per rilascio
        t_start = millis(); // dtST == 0 quando legge start e la manovra non è ancora iniziata
        delay(100); // delay di sicurezza di 100 ms 
        }

        // START4
        else if((line == "s4\n") && (!AZ_ST4)){
        AZ_COMPL = false;
        AZ_ST4 = true;  // inizializza la manovra
        Serial.println("comando: START - mod4");
        AZ_SU = false;
        AZ_ST1 = false;
        AZ_ST2 = false;
        AZ_ST3 = false;
        AZ_ST5 = false;
        AZ_ST6 = false;
        // start timer per rilascio
        t_start = millis(); // dtST == 0 quando legge start e la manovra non è ancora iniziata
        delay(100); // delay di sicurezza di 100 ms 
        }

        // START5
        else if((line == "s5\n") && (!AZ_ST5)){
        AZ_COMPL = false;
        AZ_ST5 = true;  // inizializza la manovra
        Serial.println("comando: START - mod5");
        AZ_SU = false;
        AZ_ST1 = false;
        AZ_ST2 = false;
        AZ_ST3 = false;
        AZ_ST4 = false;
        AZ_ST6 = false;
        // start timer per rilascio
        t_start = millis(); // dtST == 0 quando legge start e la manovra non è ancora iniziata
        delay(100); // delay di sicurezza di 100 ms 
        }
        
        // START6
        else if((line == "s6\n") && (!AZ_ST6)){
        AZ_COMPL = false;
        AZ_ST6 = true;  // inizializza la manovra
        Serial.println("comando: START - mod3");
        AZ_SU = false;
        AZ_ST1 = false;
        AZ_ST2 = false;
        AZ_ST3 = false;
        AZ_ST4 = false;
        AZ_ST5 = false;
        // start timer per rilascio
        t_start = millis(); // dtST == 0 quando legge start e la manovra non è ancora iniziata
        delay(100); // delay di sicurezza di 100 ms 
        }


        // SETUP
        else if((line == "SETUP\n") && (!AZ_SU)){
        AZ_COMPL = false;
        AZ_SU = true;  // inizializza la manovra
        Serial.println("comando: SETUP");
        AZ_ST1 = false;
        AZ_ST2 = false;
        AZ_ST3 = false;
        AZ_ST4 = false;
        AZ_ST5 = false;
        AZ_ST6 = false;
        }   
    }
}
