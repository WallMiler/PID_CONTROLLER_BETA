#define fwd 5 //pino pwm
#define tac 0 //pino tacogerador
#define pot 4 //pino potenciometro


float kp = 1.1, //constante proporcional
      ki = 0.32, //constante integrativa
      erro,
      proporcional,
      integral, 
      PID,
      setpoint;

unsigned long timeold; //armazena tempo decorrido desde a inicialização

int var_setpoint; //guarda valor do setpoint adquirido pelo potenciometro
int speed; //velocidade adquirida pelo tacogerador

void setup(){
timeold = 0; //inicia tempo de inicialização em 0

pinMode(fwd, OUTPUT);
Serial.begin(115200);
}

void loop(){

  var_setpoint = analogRead(pot); //adquire setpoint variável pelo potenciometro
  setpoint = map(var_setpoint, 0, 1023, 325, 900); //normalização do valor do setpoint, 325 foi a velocidade minima, 900 a máxima

  speed = analogRead(tac); //adqire dados do tacogerador

  erro = speed - setpoint; //calcula o erro
  proporcional = erro * kp; //calcula ação proporcional
  integral += erro * ki; //calcula ação integral
  PID = proporcional + integral; //calcula somatório PI

//se o PID for menor que zero, normaliza o pwm para aumentar a velocidade, valor adquiridos empiricamente
  if(PID < 0){
    PID = map(PID, 0, -setpoint, 161, 255);
  }
  //senão normaliza o pwm para a diminuição da velocidade, valores também adquiridos empiricamente
  else {
  {
    PID = map(PID, 0, 800, 161, 255);
    //limita o valor máximo do PID para 255 afim de evitar erros de quadrante
    if(PID > 255) PID = 255;
  }
  }

  analogWrite(fwd, PID);//ativa o pwm para o pino designado

// esquema para plotar os gráficos seriais
  if(millis() - timeold >= 1000){
  timeold = millis(); 
  Serial.print("Var_setpoint ");
  Serial.print(var_setpoint);  
  Serial.print("  ");
  Serial.print("speed ");
  Serial.print(speed);
  Serial.print("  ");
  // Serial.println("erro");
  // Serial.print(erro);
  // Serial.print("  ");
  Serial.print("proporcional ");
  Serial.print(proporcional);
  Serial.print("  ");
  Serial.print("integral ");
  Serial.print(integral);
  Serial.print("  ");
  Serial.print("pid ");
  Serial.println(PID);

  }
  //delay que define a taxa de amostragem, valor também obtido de forma empírica
  delay(20);
}
