String d="";
int ENA=5,IN1=7,IN2=6;
int ENB=9,IN3=4,IN4=3;
int L=0,R=0;

void setup(){
  Serial.begin(115200);
  pinMode(ENA,1);
  pinMode(IN1,1);
  pinMode(IN2,1);
  pinMode(ENB,1);
  pinMode(IN3,1);
  pinMode(IN4,1);
}

void loop(){
  while(Serial.available()){
    char c=Serial.read();
    if(c=='\n'){
      parse();
      d="";
    }
    else d+=c;
  }

  drive(L,R);
}

void parse(){

  int l=d.indexOf("L:");
  int r=d.indexOf("R:");
  if(l!=-1&&r!=-1){

    L=d.substring(l+2,r).toInt();
    R=d.substring(r+2).toInt();
  }
}

void drive(int l,int r){
  l=constrain(l,0,100);
  r=constrain(r,0,100);
  l=map(l,0,100,0,255);
  r=map(r,0,100,0,255);
  digitalWrite(IN1,1);
  digitalWrite(IN2,0);
  digitalWrite(IN3,1);
  digitalWrite(IN4,0);
  analogWrite(ENA,l);
  analogWrite(ENB,r);
}
