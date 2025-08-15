String chaine = ""; 

void setup() {
  Serial.begin(9600);  
}

void loop() {
  Serial.flush(); 
  if (Serial.available() > 0) {
    String lettre = Serial.readStringUntil('\n');
    chaine += lettre;
    delay(5); 
  }

  if (receivedString.length() > 0) {
    if ((chaine.indexOf('H') != -1)||(chaine.indexOf('h') != -1)) {
      drop();
    } 
    else if ((chaine.indexOf('S') != -1)||(chaine.indexOf('s') != -1)) {
      drop();
    } 
    else if ((chaine.indexOf('U') != -1)||(chaine.indexOf('u') != -1)) {
 
    }

    chaine = "";
  }

}

void drop() {
  
}
