#define PIN_POS1 2    // fino a metà slider
#define PIN_POS2 3    // distanza di rilascio
#define PIN_HOME 5    // home = inizio slider
#define PIN_EM 9      // rilascio EM 


unsigned long t_start, dt_st;

bool AZ_ST1 = false;
bool AZ_ST2 = false;
bool AZ_ST3 = false;
bool AZ_SU = false;
bool AZ_COMPL = true;


// Caso Start1
int AttesaStart1 =     2000;          // tempo di attesa per inizio rilascio dopo l'arrivo del comando START
int DurataRilascio1 =  1900;          // durata manovra di rilascio da H --> P2
int AttesaEM1 =        900;           // tempo di attesa dopo per il rilascio magnetico dopo l'arrivo del comando START

// Caso Start2
int AttesaStart2 =     2000;          // tempo di attesa per inizio preposizionamento dopo l'arrivo del comando START
int DurataRilascio2 =  900;           // durata manovra di rilascio da P1 --> P2
int AttesaEM2 =        400;           // tempo di attesa dopo per il rilascio magnetico dopo l'arrivo del comando START
int DurataPrePos2 =    1000;          // durata preposizionamento per rilascio corto
int AttesaPostPP2 =    2000;          // attesa dopo preposizionamento

// Caso Start3
int AttesaStart3 =     2000;          // tempo di attesa per inizio preposizionamento dopo l'arrivo del comando START
int DurataPrePos3 =    1900;          // durata manovra di rilascio da H --> P2
int AttesaRilEM3 =     900;           // tempo di attesa dopo per il rilascio magnetico dopo preposizionamento


int AttesaRip =       1500;           // tempo di attesa per il riposizionamento dopo il rilascio
int DurataHoming =    8000;           // durata manovra di homing P1 --> H


char ch;
String line;




void setup() {

  Serial.begin(9600);
  Serial.println("type...");
  
  pinMode(PIN_POS1, OUTPUT);
  pinMode(PIN_POS2, OUTPUT);
  pinMode(PIN_HOME, OUTPUT);
  pinMode(PIN_EM, OUTPUT);
  
  digitalWrite(PIN_EM, HIGH);  // con HIGH apri il p-mosfet --> setup iniziale con tutto spento
}

void loop() {
  
  dt_st = millis() - t_start; //timer



  // CHECK POST OGNI AZIONE COMPLETATA --> evito che si muova a caso 
  if(AZ_COMPL){
    digitalWrite(PIN_POS1, HIGH); // con HIGH apri il p-mosfet 
    digitalWrite(PIN_POS2, HIGH); // con HIGH apri il p-mosfet
    digitalWrite(PIN_HOME, HIGH); // con HIGH apri il p-mosfet
}


  // ----------------------------------------------------------------------------

  // NB: Prima di ogni manovra bisogna fare un SETUP perchè così accendi i magneti e riposizioni ad HOME

  // MANOVRA DI RILASCIO1 = H --> P2.1 --> EM --> P2.2 --> P1       
  if (AZ_ST1 && (dt_st >= AttesaStart1)){  // Se il comando START1 è stato ricevuto e sono passati 'AttesaStart1' secondi
      
    // esegui la manovra
    digitalWrite(PIN_POS2, LOW);  // con LOW chiudi il p-mosfet
    Serial.println("Initialize POS2");
    
    delay(AttesaEM1);
    digitalWrite(PIN_EM, HIGH); // con HIGH apri il p-mosfet
    Serial.println("MAGNETIC RELEASE");
    
    delay(DurataRilascio1 - AttesaEM1);
    digitalWrite(PIN_POS2, HIGH);  // con HIGH apri il p-mosfet
    
    delay(AttesaRip);
    digitalWrite(PIN_POS1, LOW); // con LOW chiudi il p-mosfet
    Serial.println("Initialize POS1");
    delay(DurataRip);

    AZ_COMPL = true;
    AZ_ST1 = false;  // sovrascrivi il comando per evitare che lo ripeta  
    Serial.println("type...");   
    }


  // ----------------------------------------------------------------------------

  // MANOVRA DI RILASCIO2 = H --> P1 --> P2.1 --> EM --> P2.2 --> P1         
  if (AZ_ST2 && (dt_st >= AttesaStart2)){  // Se il comando START2 è stato ricevuto e sono passati 'AttesaStart2' secondi
      
    // esegui il posizionamento iniziale 
    digitalWrite(PIN_POS1, LOW);  // con LOW chiudi il p-mosfet
    Serial.println("Initialize POS1");
    delay(DurataPrePos);
    digitalWrite(PIN_POS1, HIGH);  // con HIGH apri il p-mosfet
    Serial.println("Prepositioning (POS1) completed");
    delay(AttesaPostPP);

    // esegui la manovra
    digitalWrite(PIN_POS2, LOW);  // con LOW chiudi il p-mosfet
    Serial.println("Initialize POS2");
    
    delay(AttesaEM2);
    digitalWrite(PIN_EM, HIGH); // con HIGH apri il p-mosfet
    Serial.println("MAGNETIC RELEASE");
    
    delay(DurataRilascio2 - AttesaEM2);
    digitalWrite(PIN_POS2, HIGH);  // con HIGH apri il p-mosfet
    
    delay(AttesaRip);
    digitalWrite(PIN_POS1, LOW); // con LOW chiudi il p-mosfet
    Serial.println("Initialize POS1");
    delay(DurataRip);

    AZ_COMPL = true;
    AZ_ST2 = false;  // sovrascrivi il comando per evitare che lo ripeta  
    Serial.println("type...");   
    }
 
  
  // ----------------------------------------------------------------------------

  // MANOVRA DI RILASCIO3 = H --> P2 --> EM --> P1      
  if (AZ_ST3 && (dt_st >= AttesaStart3)){  // Se il comando START3 è stato ricevuto e sono passati 'AttesaStart3' secondi
      
    // esegui la manovra
    digitalWrite(PIN_POS2, LOW);  // con LOW chiudi il p-mosfet
    Serial.println("Initialize POS2");
    delay(DurataPrePos3);
    
    digitalWrite(PIN_EM, HIGH); // con HIGH apri il p-mosfet
    Serial.println("MAGNETIC RELEASE");
    
    delay(100);
    digitalWrite(PIN_POS2, HIGH);  // con HIGH apri il p-mosfet
    
    delay(AttesaRip);
    digitalWrite(PIN_POS1, LOW); // con LOW chiudi il p-mosfet
    Serial.println("Initialize POS1");
    delay(DurataRip);

    AZ_COMPL = true;
    AZ_ST3 = false;  // sovrascrivi il comando per evitare che lo ripeta  
    Serial.println("type...");   
    }


  // ----------------------------------------------------------------------------

  // MANOVRA DI SETUP = P2 --> H      
  if (AZ_SU){  // Se il comando SETUP è stato ricevuto
      
    // Prepara gli slider per la prox parabola
    digitalWrite(PIN_HOME, LOW);  // con LOW chiudi il p-mosfet
    Serial.println("Initialize HOMING");
    delay(DurataHoming);
    digitalWrite(PIN_HOME, HIGH);  // con HIGH apri il p-mosfet
    
    digitalWrite(PIN_EM, LOW); // con HIGH chiudi il p-mosfet
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
        delay(100); // delay di sicurezza di 100 ms 
        }   
    }
}
