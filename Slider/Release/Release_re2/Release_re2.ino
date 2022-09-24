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
  
  //digitalWrite(PIN_POS1, HIGH);
  //digitalWrite(PIN_EM, HIGH);
}

unsigned long t1, dt;

bool REL_ST = false;

int AttesaStart = 3000; // tempo di attesa per inizio rilascio dopo l'arrivo del comando di inizio parabola
int AttesaEM = 5000; // tempo di attesa dopo l'arrivo del comando di fine posizionamento

String line = "";

void loop() {
  dt = millis() - t1;
  if(!REL_ST){
    digitalWrite(PIN_POS1, HIGH);
  }
      
    if ((dt >= AttesaStart) && REL_ST){
      
      // start della manovra
      while(digitalRead(PIN_POSCOMP)== LOW) {
        digitalWrite(PIN_POS1, LOW);  // con LOW apri il mosfet
        Serial.println("pos1");
        
        // spegnimento EM
        if ((dt >= AttesaEM)) {
          digitalWrite(PIN_EM, LOW);
          //Serial.println("em rel");
        }
        
      else{
        digitalWrite(PIN_POS1, HIGH);  // con HIGH chiudi il mosfet
        Serial.println("pos1 conclusa");
      
      }
    }

    }

   if(Serial.available()){
    char ch = Serial.read();
    
    if((ch == '\n') || (ch == '\r')){  // Se leggi START e la manovra non è iniziata ancora (!REL_ST) esegui quanto segue:
      //process(line)
      //line == "";
        
        if((line = "START") && (!REL_ST)){
        REL_ST = true;  // inizializza la manovra
     
        // start timer per rilascio
        t1 = millis(); // dtST == 0 quando legge start e la manovra non è ancora iniziata
        delay(100); // delay di sicurezza di 100 ms 
        }
    }   
     else{
      line += ch;
      
    }
   }    
}
