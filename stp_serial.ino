#define dirPin 2
#define stepPin 3
#define stepsPerRevolution 98
#define turn 10

char prev_a = "0";
int whole_rev    = turn;
int rest_rev     = 0;
int possible_rev = turn;

void setup() {
  Serial.begin(9600);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() > 0){
  char a = Serial.read();
//  if(a != '1' or a != '2')
//  a = prev_a;
  //On-Off 형식으로 해서 신호 받으면 잡고, 풀고만 나눠서 한다면
  //stepsPerRevolution 값 조절해서 하면 되기는 함
  //최대 감길 수 있는 Step수 계산해서(그리퍼 탭 길이 고려)
  // 그거 남은 만큼만 감길 수 있도록 변 하나 설정해야 함
  // 전체 = 남은거 + 감을 수 있는 수

  //1 오므림
  if(a == '1'){
  Serial.println("Clockwise!");
  digitalWrite(dirPin, HIGH);
  
  for (int j = 1; j < possible_rev + 1; j++){
  for (int i = 0; i < stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);}
    rest_rev = whole_rev - j;}
  }
  //2 벌림
  else if(a == '2'){
  Serial.println("C-Clockwise!");
  digitalWrite(dirPin, LOW);
  
  for (int j = 1; j < possible_rev + 1; j++){
  for (int i = 0; i < stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);}
    rest_rev = whole_rev - j;}
  }
  
  else{
   Serial.println("Stop!");
   digitalWrite(stepPin, LOW);
   delayMicroseconds(1000);
  }
  possible_rev = turn;
//  prev_a = a;
}
}
