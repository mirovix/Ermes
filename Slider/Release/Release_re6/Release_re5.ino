#define PIN_POS1 2    // fino a metà slider
#define PIN_POS2 3    // distanza di rilascio
//#define PIN_POS3 4
#define PIN_HOME 5    // home = inizio slider
//#define PIN_START 6
//#define PIN_PAUSE 7
//#define PIN_POSCOMP 8
//#define PIN_HCOMP 9

#define PIN_EM 9      // rilascio EM 


unsigned long t_start, dt_st;

bool AZ_ST = false;
bool AZ_SU = false;
bool AZ_COMPL = true;

int AttesaStart =     2000;          // tempo di attesa per inizio rilascio dopo l'arrivo del comando START
int DurataRilascio =  1900;          // durata manovra di rilascio da H --> P2
int AttesaEM =        900;           // tempo di attesa dopo per il rilascio magnetico dopo l'arrivo del comando START
int AttesaRip =       1500;          // tempo di attesa per il riposizionamento dopo il rilascio
int DurataHoming =    8000;          // durata manovra di homing P1 --> H


char ch;
String line;




void setup() {

  Serial.begin(9600);
  Serial.println("type...");
  
  pinMode(PIN_POS1, OUTPUT);
  pinMode(PIN_POS2, OUTPUT);
  pinMode(PIN_HOME, OUTPUT);
  pinMode(PIN_EM, OUTPUT);
  
  digitalWrite(PIN_EM, HIGH);
}

void loop() {
  
  dt_st = millis() - t_start; //timer



 // CHECK POST OGNI AZIONE COMPLETATA --> evito che si muova a caso 
  if(AZ_COMPL){
    digitalWrite(PIN_POS1, HIGH); // con HIGH apri il p-mosfet 
    digitalWrite(PIN_POS2, HIGH); // con HIGH apri il p-mosfet
    digitalWrite(PIN_HOME, HIGH); // con HIGH apri il p-mosfet
}



 // MANOVRA DI RILASCIO = H --> P2.1 --> EM --> P2.2 --> P1       
  if (AZ_ST && (dt_st >= AttesaStart)){  // Se il comando START è stato ricevuto e sono passati 'AttesaStart' secondi
      
    // esegui la manovra
    digitalWrite(PIN_POS2, LOW);  // con LOW chiudi il p-mosfet
    Serial.println("Initialize POS2");
    
    delay(AttesaEM);
    digitalWrite(PIN_EM, LOW); // con LOW apri il n-mosfet
    Serial.println("MAGNETIC RELEASE");
    
    delay(DurataRilascio - AttesaEM);
    digitalWrite(PIN_POS2, HIGH);  // con HIGH apri il p-mosfet
    
    delay(AttesaRip);
    digitalWrite(PIN_POS1, LOW); // con LOW chiudi il p-mosfet
    Serial.println("Initialize POS1");
    AZ_COMPL = true;
    AZ_ST = false;  // sovrascrivi il comando per evitare che lo ripeta  
        
    }

 
 // MANOVRA DI SETUP = P2 --> H      
  if (AZ_SU){  // Se il comando SETUP è stato ricevuto
      
    // Prepara gli slider per la prox parabola
    digitalWrite(PIN_HOME, LOW);  // con LOW chiudi il p-mosfet
    Serial.println("Initialize HOMING");
    delay(DurataHoming);
    digitalWrite(PIN_HOME, HIGH);  // con HIGH apri il p-mosfet
    
    digitalWrite(PIN_EM, HIGH); // con HIGH chiudi il n-mosfet
    Serial.println("EM activated");
    delay(500);
    AZ_COMPL = true; 
    AZ_SU = false; // sovrascrivi il comando per evitare che lo ripeta   
        
    }


   
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
           
        // START
        if((line == "START\n") && (!AZ_ST)){
        AZ_COMPL = false;
        AZ_ST = true;  // inizializza la manovra
        Serial.println("comando: START");
        AZ_SU = false;
        // start timer per rilascio
        t_start = millis(); // dtST == 0 quando legge start e la manovra non è ancora iniziata
        delay(100); // delay di sicurezza di 100 ms 
        //line = "";
        }

        
        // SETUP
        else if((line == "SETUP\n") && (!AZ_SU)){
        AZ_COMPL = false;
        AZ_SU = true;  // inizializza la manovra
        Serial.println("comando: SETUP");
        AZ_ST = false;
        //line = "";
        }   
    }
}
