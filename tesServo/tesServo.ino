#include <Servo.h>

Servo jepit;
Servo angkat; 

void setup() {
  jepit.attach(9);  
//  jepit.write(65);
  angkat.attach(10); 
//  angkat.write(65); 
}

void loop() {
  ambilBarang();
  delay(2000);
  simpanBarang();
  delay(2000);
}

void ambilBarang(){
  for (int pos1 = 80; pos1 >= 0; pos1 -= 1) { //buka
    jepit.write(pos1);              
    delay(15);                       
  }
  
  for (int pos2 = 65; pos2 <= 180; pos2 += 1) { //turun
    angkat.write(pos2);             
    delay(15);                       
  }
  
  for (int pos1 = 0; pos1 <= 80; pos1 += 1) { //tutup
    jepit.write(pos1);           
    delay(15);                     
  }
  
  for (int pos2 = 180; pos2 >= 65; pos2 -= 1) { //naik
    angkat.write(pos2);           
    delay(15);                      
  }
}

void simpanBarang(){
  for (int pos2 = 65; pos2 <= 180; pos2 += 1) { //turun
    angkat.write(pos2);             
    delay(15);                       
  }
  
  for (int pos1 = 80; pos1 >= 0; pos1 -= 1) { //buka
    jepit.write(pos1);              
    delay(15);                       
  }
  
  for (int pos2 = 180; pos2 >= 65; pos2 -= 1) { //naik
    angkat.write(pos2);           
    delay(15);                      
  }
  
  for (int pos1 = 0; pos1 <= 80  ; pos1 += 1) { //tutup
    jepit.write(pos1);           
    delay(15);                     
  }
}
