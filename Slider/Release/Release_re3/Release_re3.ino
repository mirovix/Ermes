#define PIN_POS1 2
#define PIN_POS2 3
//#define PIN_POS3 4
#define PIN_HOME 5
//#define PIN_START 6
//#define PIN_PAUSE 7
#define PIN_POSCOMP 8
//#define PIN_HCOMP 9

#define PIN_EM 9

void setup() {

  Serial.begin(9600);
  Serial.println("type...");
  
  pinMode(PIN_POS1, OUTPUT);
  pinMode(PIN_POS2, OUTPUT);
  pinMode(PIN_HOME, OUTPUT);
  pinMode(PIN_EM, OUTPUT);

  pinMode(PIN_POSCOMP, INPUT);
  
  digitalWrite(PIN_EM, HIGH);
  //digitalWrite(PIN_EM, HIGH);
}

unsigned long t_start, dt;

bool REL_ST = false;
bool POS_COM = true;
bool REL_EM = false;
bool REL_COMPL = false;

int AttesaStart = 2000; // tempo di attesa per inizio rilascio dopo l'arrivo del comando di inizio parabola
int AttesaEM = 3000; // tempo di attesa dopo per il rilascio magnetico dopo l'arrivo del comando di inizio parabola
int AttesaRip = 8000; // tempo di attesa per il riposizionamento dopo l'arrivo del comando di inizio parabola


String line = "";

void loop() {
  
  dt = millis() - t_start; //timer


// SET UP PRIMA DELLA MANOVRA
  if(!REL_ST){  // Freno iniziale prima della manovra
    digitalWrite(PIN_POS1, HIGH); // con HIGH apri il p-mosfet
  }
  
  if(!REL_EM){ // Mantiene l'EM acceso durante l'inizio di manovra
    digitalWrite(PIN_EM, HIGH) // con HIGH chudi il n-mosfet
  }
  

 // MANOVRA DI RILASCIO = H --> P1.1 --> EM --> P1.2 --> RIPOSIZIONAMENTO
      
  if (REL_ST && (dt >= AttesaStart)){  // Se il comando START è stato ricevuto e sono passati 'AttesaStart' secondi
      
    // esegui la manovra
    digitalWrite(PIN_POS1, LOW);  // con LOW chiudi il p-mosfet
    delay(100);
        
    // spegnimento EM
    if ((dt >= AttesaEM)) {
    digitalWrite(PIN_EM, LOW); // con LOW apri il n-mosfet
    delay(100);
    
    }

    // Riposizionamento
    if ((dt >= AttesaRip)) {
    digitalWrite(PIN_POS1, HIGH);  // con HIGH apri il p-mosfet
    delay(500);
    digitalWrite(PIN_HOME, LOW); // con LOW chiudi il p-mosfet
    REL_COMPL = true;
    
    
    }
  } 



   
// LETTURA SERIALE

    if(Serial.available()){
     char ch = Serial.read();
    
    if((ch == '\n') || (ch == '\r')){  // Se leggi START e la manovra non è iniziata ancora (!REL_ST) esegui quanto segue:
     //process(line)
     //line == "";
        
        if((line = "START") && (!REL_ST)){
        REL_ST = true;  // inizializza la manovra
        REL_EM = false; // Mantiene l'EM acceso
        Serial.println("START");
        // start timer per rilascio
        t_start = millis(); // dtST == 0 quando legge start e la manovra non è ancora iniziata
        delay(100); // delay di sicurezza di 100 ms 
        }
    }   
     else{
      line += ch;
      
    }
   }

}
