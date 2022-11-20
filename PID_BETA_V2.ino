//Algoritmo para controle PID de um motor DC com realimentação de velocidade via tacogerador
//Foi usado o método das Equações das Diferenças para obtenção do algoritmo PID:
//cv(n) = cv*(n-1) + (kp +(kd/ts))*e*(n)+(-kp+(ki*ts)-(2*(kd/ts)))*e*(n-1)+((kd/ts)*e*(n-2))

//Definições das constantes
#define fwd 5 //pino pwm
#define rev 6 //pino pwm
#define tac 0 //pino tacogerador
#define pot 1 //pino potenciometro
#define bot 4 //botão para inversão da rotação
#define rele 8
#define rele_led 13 //led para sinalização dos relés
#define n 20 //número de iterações

//-----------------------------------------------------------------------------------------------------------------------------------------------------------
//Declaração para a função do filtro de média móvel
int mode(); //declaração da função para reversão do sentido de giro do motor
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
float ts = 0.01;

unsigned long timeold; //armazena tempo decorrido desde a inicialização

int var_setpoint; //guarda valor do setpoint adquirido pelo potenciometro
int relative_setpoint; //variável ponteiro para o setpoint
int speed; //velocidade adquirida pelo tacogerador
int pwm; //variável global para pwm

//variáveis para filtro de média móvel
int       original;      //armazena aquisição original   
int       numbers[n];  //vetor volátil para o armazenamento dos dados das iterações

bool last_botstate = 1; //estado anterior do botão
bool bot_state; //estado atual do botão
bool rele_state = 1; //estado atual do relé

unsigned int last_time = 0; //ultimo tempo do millis() para o debounce
unsigned int debounce_delay = 50; //tempo de delay do debounce
//-----------------------------------------------------------------------------------------------------------------------------------------------------------
void setup(){
timeold = 0; //inicia tempo de inicialização em 0

pinMode(fwd, OUTPUT);//pino para ativar sentido horário
pinMode(rev, OUTPUT);// pino para ativar sentido anti-horário
pinMode(rele_led, OUTPUT); //saída para ativar relés da meia ponte h
pinMode(bot, INPUT_PULLUP); //botão para ativar reversão
Serial.begin(115200);
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void loop(){

  var_setpoint = analogRead(pot); //adquire setpoint variável pelo potenciometro
  relative_setpoint = mode(); //adquire novo valor de setpoint e inversão de rotação
  setpoint = map(relative_setpoint, 0, 1023, 350, 900); //normalização do valor do setpoint, 325 foi a velocidade minima, 900 a máxima
  

  mode(); //chama a função para inveter o sentido de giro e alterar a velocidade

  original = analogRead(tac); //adqire dados do tacogerador
  
  speed = moving_average(); //armazena o valor retornado pela função do filtro

  erro = setpoint - speed; //erro atual do sistema

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

  pwm = map(cv, 0,setpoint, 106, 255);
  //analogWrite(fwd, cv*(255.0/800.0));//ativa o pwm para o pino designado
  //analogWrite(fwd, pwm);//ativa o pwm para o pino designado
  
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

int mode(){
  int setpoint_mode; //variavel que guarda a velocidade do sistema
  bool read_bot = digitalRead(bot); //variavel que faz a leitura e guarda o estado do botão
  if(read_bot != last_botstate){ //faz a negação do estado do botão
  last_time = millis(); //atualiza o tempo 
  }
 if ((millis() - last_time) > debounce_delay) { //compara os tempos para o debounce
    if (read_bot != bot_state) { //
      bot_state = read_bot;

      if (bot_state == 1) {
        rele_state = !rele_state;
      }
    }
  }
  digitalWrite(rele_led, rele_state); 
  last_botstate = read_bot;

  //Se o estado do relé for LOW o sentido de rotação do motor é horário
  if(rele_state == LOW){ 
      //setpoint_mode = 500;
      digitalWrite(rele, LOW);
      //digitalWrite(rev_led, LOW);
      //digitalWrite(fwd_led, HIGH);
      analogWrite(fwd, pwm);
      analogWrite(rev, 0);
      digitalWrite(rele_led, LOW);
      setpoint_mode = var_setpoint - 150;
  }

  //Se o estado do relé for HIGH o sentido de rotação do motor é anti-horário
  if(rele_state == HIGH){
      //setpoint_mode = 500;
      digitalWrite(rele, HIGH);
      //digitalWrite(rev_led, HIGH);
      //digitalWrite(fwd_led, LOW);
      analogWrite(fwd, 0);
      analogWrite(rev, pwm);
      digitalWrite(rele_led, HIGH);
      setpoint_mode = var_setpoint;
  }
  return setpoint_mode; //retorna novo setpoint
}
