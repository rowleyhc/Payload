float dc=.5;
int hz=20;
float pwmtimer=millis();
int fwdPin=10;
int bckPin=9;

void setup() {
  // put your setup code here, to run once:
  pinMode(fwdPin,OUTPUT);
  pinMode(bckPin,OUTPUT);

}

void loop() {
  if((millis()-pwmtimer)<(1000*abs(dc)/hz)){
    if(dc>0){
      digitalWrite(fwdPin,HIGH);
      digitalWrite(bckPin,LOW);
    }
    else if(dc<0){
      digitalWrite(fwdPin,LOW);
      digitalWrite(bckPin,HIGH);
    }
    else{
      digitalWrite(fwdPin,LOW);
      digitalWrite(bckPin,LOW);    
    }
  }
  else if(millis()-pwmtimer<(1000*1/hz)){
    digitalWrite(fwdPin,LOW);
    digitalWrite(bckPin,LOW); 
  }
  else{
    pwmtimer=millis();
  }
  

}
