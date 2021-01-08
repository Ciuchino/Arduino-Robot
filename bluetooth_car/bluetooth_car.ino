// BAL BLA BLA
 //www.elegoo.com
#include <Servo.h>
#include <Stepper.h>
// Prima di caricare il codice, rimuovere la componente bluetooth dal robot

#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11
#define LED 13
#define STEPS  32
#define ITERAZIONIOSTACOLI 3

Stepper small_stepper(STEPS, 2, 10, 4, 12);
Servo head;
  
int  Steps2Take; 
unsigned char carSpeed = 150;
bool state = LOW;
char getstr;
int Echo = A4;  
int Trig = A5; 
void timer(unsigned long differenzaTempo, boolean flagObstacle, boolean flagTimer);

boolean headLeftProva(unsigned long differenzaTempo, boolean flagObstacle, boolean flagTimer){
  unsigned long tempoIniziale;
  unsigned long tempoCorrente;
  tempoIniziale = millis(); //calcola il tempo iniziale in millisecondi
  while (true) {
    
    if (flagObstacle == false and obstacle(ITERAZIONIOSTACOLI, flagTimer, 15)== true) { // Rilevato un ostacolo

      Serial.println("OSTACOLO!");
      if (flagTimer == false){ // funzione timer invocata da funzione indipendent e non da timer per ricorsione
       stop();
       delay(500);      
       stateChange();     
      }
      avoidObstacle();
      // timer(0, false, true); // se c'è un ostacolo gira a destra finchè vede un ostacolo e passa alla funzione il parametro false, cosi se la funzione continua a rilevare un ostacolo mentre gira, obastacle sa che sta girando, e non che sta compiendo una azione
      if (flagTimer == false) stateChange();
      return;
    }
    tempoCorrente = millis();
    if ((tempoCorrente - tempoIniziale) > differenzaTempo) break;// Calcola il tempo passato dall'inizio della chiamata alla funzione, se maggiore di quell dato come input esce
    
  }
  stop(); //Quando scade il timer, esce e si ferma
}

//Ultrasonic distance measurement Sub function
int Distance_test() {
  digitalWrite(Trig, LOW);   
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  
  delayMicroseconds(20);
  digitalWrite(Trig, LOW);   
  float Fdistance = pulseIn(Echo, HIGH);  
  Fdistance= Fdistance / 58.2;       
  return (int)Fdistance;
}

void armup(){
  small_stepper.setSpeed(500);
  Steps2Take  =  500; 
  small_stepper.step(Steps2Take);
  delay(1000); 
  Serial.println("Arm Up")  ;
  }
  

void armdown(){
  small_stepper.setSpeed(500);
  Steps2Take  =  -500; 
  small_stepper.step(Steps2Take);
  delay(1000);
  Serial.println("Arm Down")  ;
  }

void headCenter(){
  head.write(90);
  Serial.println("Head Center");
}

void back(){ 
  digitalWrite(ENA,HIGH);
  digitalWrite(ENB,HIGH);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  Serial.println("Back");
}

void forward(){
  digitalWrite(ENA,HIGH);
  digitalWrite(ENB,HIGH);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  Serial.println("Forward");
}

void right(){
  digitalWrite(ENA,HIGH);
  digitalWrite(ENB,HIGH);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW); 
  Serial.println("Right");
}

void left(){
  digitalWrite(ENA,HIGH);
  digitalWrite(ENB,HIGH);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);

  Serial.println("Left");
}

void stop(){
  digitalWrite(ENA,LOW);
  digitalWrite(ENB,LOW);
  Serial.println("Stop!");
}

void stateChange(){
  state = !state;
  digitalWrite(LED, state);
  Serial.println("Light");  
}

boolean obstacle(int contatore, boolean flagNotRicorsione, int distanzaMassima) { //funzione per determinare se c'è un ostacolo. si invoca per tre volte, con uno stacco di 0,05 secondi, effettuando per tre volte il controllo
  int distanza;                                      //Il secondo parametro ci dice se la funzione è stata invocata per la prima volta(quindi dal robot mentre compiva un'azione, = false) oppure dalla funzione timer dopo che era già stata riscontrato un ostacolo
  distanza = Distance_test(); // distanza misurata   //In questa maniera i 3 controlli sull'ostacolo vengono eseguiti solo se il robot sta compiendo un'azione, come andare avanti, e non se sta girando per evitare un altro ostacolo

  if (distanza < distanzaMassima) { // distanza minore di 20 cm
    if (contatore == 1 or flagNotRicorsione == true) return (true); // restituisce vero se c'è un ostacolo, altrimenti è falso
    timer(50, true, true);
    return obstacle(contatore-1, false, 20);
  }
  return (false);
}

boolean headLeft(){
    Serial.println("head left");
    int headAngle = head.read() + 50;
    head.write(headAngle);
    while (headAngle < 170) { //gira la testa a sinistra e se vede un ostacolo lo riferisce alla funzione avoidObastacle, che provvederà a continuare a girare. Questo finchè la testa non si è girata di 90 gradi
      headAngle=head.read();
      Serial.println(headAngle);
      headAngle=headAngle + 5;
      head.write(headAngle);
      delay(1000);
      if (obstacle(1, true, 20) == true) {
        return (true);
      }
      delay(500);
    }
   return (false);
}

boolean headRight(){
    Serial.println("head right");
    int headAngle = head.read() - 50;    
    head.write(headAngle);    
    while (headAngle > 10) {
      Serial.println(headAngle);
      headAngle=head.read();
      headAngle=headAngle - 5;
      head.write(headAngle);
      delay(500);
      if (obstacle(1, true, 20) == true){
        return (true);
      }
      delay(500);
    }
   return (false);
}

void avoidObstacle() { // Se c'è un ostacolo gira in maniera saucle a destra o a sinistra e continua a girare finchè non si è girato di 90 gradi in pratica
  boolean obstaclePresence;
  int sceltaAzione = random(1,3);
  while (true) {
    if (sceltaAzione == 1) obstaclePresence = headRight();
    else obstaclePresence = headLeft();
    headCenter();
    if (obstaclePresence == true) {
      sceltaAzione = 3 - sceltaAzione; // Se vedo un ostacolo a destra mi giro a sinistra e vicecersa     
      if (sceltaAzione == 1) obstaclePresence = headRight();
      else obstaclePresence = headLeft();
      headCenter();
      if (obstaclePresence == true) { // faccio il controllo dall'altro lato. Se rilevo un ostacolo torno un po' indietro e ci riprovo
        back();
        timer(1000, true, false);
        continue; // torno su per rifare il controllo
        } 
      } 
    if (sceltaAzione == 1) right(); // gira a destra
    else left();
    timer(1500, true, false);
    break;
  }
  stop();
}

//funzione che rimane attiva per un certo tempo dato come parametro (in millisecondi) e compie il controllo sulla presenza di ostacoli. Terzo parametro = true se la funzione è stata invocata da un'altra funzione timer per ricorsione e il secondo parametro = true se è stata invocata da una funzione obstacle (altrimenti false)
//Il terzo parametro viene passato alla funzione obstacle e serve per dirgli se è la prima volta che viene invocata, oppure viene invocata dopo una ricorsione di timer
void timer(unsigned long differenzaTempo, boolean flagObstacle, boolean flagTimer) {
  unsigned long tempoIniziale;
  unsigned long tempoCorrente;
  tempoIniziale = millis(); //calcola il tempo iniziale in millisecondi
  while (true) {
    
    if (flagObstacle == false and obstacle(ITERAZIONIOSTACOLI, flagTimer, 15)== true) { // Rilevato un ostacolo

      Serial.println("OSTACOLO!");
      if (flagTimer == false){ // funzione timer invocata da funzione indipendent e non da timer per ricorsione
       stop();
       delay(500);      
       stateChange();     
      }
      avoidObstacle();
      // timer(0, false, true); // se c'è un ostacolo gira a destra finchè vede un ostacolo e passa alla funzione il parametro false, cosi se la funzione continua a rilevare un ostacolo mentre gira, obastacle sa che sta girando, e non che sta compiendo una azione
      if (flagTimer == false) stateChange();
      return;
    }
    tempoCorrente = millis();
    if ((tempoCorrente - tempoIniziale) > differenzaTempo) break;// Calcola il tempo passato dall'inizio della chiamata alla funzione, se maggiore di quell dato come input esce
    
  }
  stop(); //Quando scade il timer, esce e si ferma
}

void indipendent(){
int azione; // prossima azione da svolgere h
  while (true) {
    if (Serial.available() > 0) {
      getstr = Serial.read();
      if (getstr == 'r') {
      stop();
      break;
      }
    }
    /*if (obstacle(ITERAZIONIOSTACOLI, false, 20)== true) {
      Serial.println("Rilevato ostacolo!");
      stop();
      delay(500);
      avoidObstacle();
      // timer(0, false, true); // se c'è un ostacolo torna indietro per 2 secondi
    } */
    delay(500);
    azione = random(1,4); // 1 va avanti, 2 gira a destra e e 3 gira a sinistra
    if (azione == 1) {
      forward();
      timer(random(3000, 5000), false, false); // funzione per compiere l'azione per un eterminato tempo passato come parametro
    }
    else {
      if (azione == 2) {
        left();
        timer(random(3000, 6000), false, false); //gira a destra per un tempo casuale compreso tra 1 e 8 secondi
      }
      else {
        right();
        timer(random(3000, 6000), false, false); // gira a sinistra per un tempo casuale compreso tra 1 e 8 secondi
      }
    }
  }
}


void setup() {
  digitalWrite(8, LOW); //arm
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW); 
  head.attach(3);  //head
  head.write(90);
  Serial.begin(9600);
  pinMode(Echo, INPUT); 
  pinMode(Trig, OUTPUT); 
  pinMode(LED, OUTPUT);  //motor 
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  stop();
}

void loop() {
    getstr = Serial.read();
    switch(getstr){
    case 'e': armup(); break;
    case 'g': armdown(); break;
    case 'H': headLeftProva(20000, true, true); break;
    case 'F': headRight(); break;
    case 'E': headCenter(); break;
    case 'A': forward(); break;
    case 'C': back(); break;    
    case 'B': left(); break;
    case 'D': right();break;
    case 'S': stop();   break;
    case 'G': stateChange(); break;
    case 'r': indipendent(); break;
    }
    
  }
