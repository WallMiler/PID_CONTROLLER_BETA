//Algoritmo para controle PID de um motor DC com realimentação de velocidade via tacogerador
//Foi usado o método das Equações das Diferenças para obtenção do algoritmo PID:
//cv(n) = cv*(n-1) + (kp +(kd/ts))*e*(n)+(-kp+(ki*ts)-(2*(kd/ts)))*e*(n-1)+((kd/ts)*e*(n-2))

//Definições das constantes
#define fwd 5 //pino pwm
#define tac 0 //pino tacogerador
#define pot 4 //pino potenciometro
#define n 20 //número de iterações

//-----------------------------------------------------------------------------------------------------------------------------------------------------------
//Declaração para a função do filtro de média móvel
long moving_average(); //declaração do protótipo da função do filtro

//variáveis do algoritmo das Equações das Diferenças
float cv, //  cv(n-1)
      cv_1,
      erro, // e(n)
      erro_1, // e(n-1)
      erro_2, // e(n-2)
      setpoint;

//variáveis dos parâmetros do PID
float kp = 5;
float ki = 2;
float kd = 3.31;
float ts = 0.1;

unsigned long timeold; //armazena tempo decorrido desde a inicialização

int var_setpoint; //guarda valor do setpoint adquirido pelo potenciometro
int speed; //velocidade adquirida pelo tacogerador

//variáveis para filtro de média móvel
int       original;      //armazena aquisição original   
int       numbers[n];  //vetor volátil para o armazenamento dos dados das iterações
//-----------------------------------------------------------------------------------------------------------------------------------------------------------
void setup(){
timeold = 0; //inicia tempo de inicialização em 0

pinMode(fwd, OUTPUT);
Serial.begin(115200);
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void loop(){

  var_setpoint = analogRead(pot); //adquire setpoint variável pelo potenciometro
  setpoint = map(var_setpoint, 0, 1023, 350, 900); //normalização do valor do setpoint, 325 foi a velocidade minima, 900 a máxima

  original = analogRead(tac); //adqire dados do tacogerador
  
  speed = moving_average(); //armazena o valor retornado pela função do filtro

  erro = setpoint - speed;

  cv = cv_1 + (kp + kd/ts)*erro + (-kp + ki*ts - 2*kd/ts)*erro_1 + (kd/ts) * erro_2; //Equação das diferenças
  cv_1 = cv;
  erro_2 = erro_1;
  erro_1 = erro;
//---------------------------------------------------------------------------------------------------------------------------------------------------------
//se o PID for menor que zero, normaliza o pwm para aumentar a velocidade, valores adquiridos empiricamente
  if(cv > setpoint){
    cv = setpoint;
  }
  //senão normaliza o pwm para a diminuição da velocidade, valores também adquiridos empiricamente
  if(cv < 131) {
     cv = 131;
  
  }

  if(speed < 340) cv = setpoint; //Se a velocidade for menor que o valor mínimo, cv será = setpoint

  int pwm = map(cv, 0,setpoint, 106, 255);
  //analogWrite(fwd, cv*(255.0/800.0));//ativa o pwm para o pino designado
  analogWrite(fwd, pwm);//ativa o pwm para o pino designado

//----------------------------------------------------------------------------------------------------------------------------------------------------------
// Algoritmo de plotagem do gráfico serial e monitor serial
  if(millis() - timeold >= 10){
  timeold = millis(); 
  Serial.print("Var_setpoint ");
  Serial.print(setpoint);  
  Serial.print("  ");
  Serial.print("speed ");
  Serial.print(speed);
  Serial.print("  ");
  // Serial.println("erro");
  // Serial.print(erro);
  // Serial.print("  ");
  //Serial.print("proporcional ");
  //Serial.print(proporcional);
  //Serial.print("  ");
  Serial.print("pwm ");
  Serial.print(pwm);
  Serial.print("  ");
  Serial.print("cv ");
  Serial.println(cv);

  }
  //delay que define a taxa de amostragem, valor também obtido de forma empírica
  delay(10);
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------
//Corpo da função do filtro de média móvel
long moving_average()
{

   for(int i= n-1; i>0; i--) numbers[i] = numbers[i-1]; //armazena os dados no vetor de n posições consecutivamente

   numbers[0] = original; //inicia o calculo no vetor de posição zero

   long acc = 0;  //inicializa a variável de acumulação em 0       

   for(int i=0; i<n; i++) acc += numbers[i]; // move a posição dos vetores, adquirindo um novo na ultima posição e removendo o da primeira posição


   return acc/n; // realiza o retorna da operação de média, em que a soma acumulada dos vetores é dividida pelo número dos vetores

 
} 
