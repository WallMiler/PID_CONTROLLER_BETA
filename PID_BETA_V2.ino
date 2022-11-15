  //Algoritmo para controle PID de um motor DC com realimentação de velocidade via tacogerador
  //Foi usado o método das Equações das Diferenças para obtenção do algoritmo PID:
  //cv(n) = cv*(n-1) + (kp +(kd/ts))*e*(n)+(-kp+(ki*ts)-(2*(kd/ts)))*e*(n-1)+((kd/ts)*e*(n-2))

  //Bibliotecas para trabalhar com o LCD OLED I2C 128x32
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>

  //Constantes do OLED
  #define SCREEN_WIDTH 128 // Largura do display OLED (pixels)
  #define SCREEN_HEIGHT 32 // Altura do display OLED (pixels)
  #define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
  #define SCREEN_ADDRESS 0x3C ///Endereço do LCD 0x3C for 128x32
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


  //Definições das constantes
  #define fwd 5 //pino pwm
  #define tac 0 //pino tacogerador
  //#define pot 4 //pino potenciometro
  #define n 20 //número de iterações

  //-----------------------------------------------------------------------------------------------------------------------------------------------------------


  //Definições das variáveis do LCD
  bool config = false;
  int opcao = 0;

  const int BUTTON1_PIN = 2;
  const int BUTTON2_PIN = 3;

  int lastState_btn1 = LOW;
  int lastState_btn2 = LOW;
  int currentState_btn1;
  int currentState_btn2;
  int endTime_btn1 = 0;
  int endTime_btn2 = 0;

  int totalTime_btn1 =0;
  int totalTime_btn2 =0;
  int startTime_btn1 =0;
  int startTime_btn2 =0;

  //Posição da casa decimal do setpoint
  int dec_pos = 0;

  // Alterar o valor do setpoint
  int setpoint = 0;


  //Declaração para a função do filtro de média móvel
  long moving_average(); //declaração do protótipo da função do filtro

  //variáveis do algoritmo das Equações das Diferenças
  float cv, //  cv(n-1)
        cv_1,
        erro, // e(n)
        erro_1, // e(n-1)
        erro_2;// e(n-2)

  //setpoint;

  //variáveis dos parâmetros do PID
  float kp = 5;
  float ki = 2;
  float kd = 3.31;
  float ts = 0.1;

  unsigned long timeold; //armazena tempo decorrido desde a inicialização

  // int var_setpoint; //guarda valor do setpoint adquirido pelo potenciometro
  int speed; //velocidade adquirida pelo tacogerador

  //variáveis para filtro de média móvel
  int       original;      //armazena aquisição original   
  int       numbers[n];  //vetor volátil para o armazenamento dos dados das iterações

  //-----------------------------------------------------------------------------------------------------------------------------------------------------------
  void setup(){
    /*
    * Configuração do display OLED 
    */

    pinMode(BUTTON1_PIN, INPUT_PULLUP);
    pinMode(BUTTON2_PIN, INPUT_PULLUP);

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }
    display.display();
    delay(1000);
    display.clearDisplay();
    delay(1000);
    display.display();



    timeold = 0; //inicia tempo de inicialização em 0

    pinMode(fwd, OUTPUT);
    Serial.begin(115200);
  }
  //----------------------------------------------------------------------------------------------------------------------------------------------------------
  void loop(){

    //Verifica quando o botao saiu do estado HIGH para LOW  
    //Pressionou o botao
    currentState_btn1 = !digitalRead(BUTTON1_PIN);
    currentState_btn2 = !digitalRead(BUTTON2_PIN);

    //Se algum dos botões sair de LOW para HIGH (for pressionado)
    //Começa o timer 
    if (lastState_btn1 == LOW && currentState_btn1 == HIGH)  startTime_btn1 = millis();
    if (lastState_btn2 == LOW && currentState_btn2 == HIGH)  startTime_btn2 = millis();

    
    //Se soltou o botão, para o timer e reseta o startTime
    if (lastState_btn1 == HIGH && currentState_btn1 == LOW) {
      endTime_btn1 = millis();
      totalTime_btn1 = endTime_btn1 - startTime_btn1;
      startTime_btn1 = endTime_btn1 = 0;
    } 

    if (lastState_btn2 == HIGH && currentState_btn2 == LOW) {
      endTime_btn2 = millis();
      totalTime_btn2 = endTime_btn2 - startTime_btn2;
      startTime_btn2 = endTime_btn2 = 0;
    }
    
    //Entra no menu de configuração do setpoint
    if (opcao == 0 && totalTime_btn1 > 550 && config==false){
      config = true;
      totalTime_btn1 = 0;
      dec_pos = 0;
    }

    //Configurando o setpoint
    if (config == true){
      configurar_setpoint(totalTime_btn1, totalTime_btn2);
      totalTime_btn2 = 0;
      totalTime_btn1 = 0; 
    }
    
    //Navege entre as opções
    if(totalTime_btn1< 550 && totalTime_btn1 > 60 && config ==false ){
      opcao++;
      totalTime_btn1 = 0;
      if(opcao > 3) opcao = 3;
    }
    
    if(totalTime_btn2< 550 && totalTime_btn2 > 60 && config ==false) {
      opcao--;
      totalTime_btn2 =0 ;
      if(opcao < 0) opcao =0;
    }
   
    lastState_btn1 = currentState_btn1;
    lastState_btn2 = currentState_btn2;


  //----------------------------------------------------------------------------------------------------------------------------------------------------------


    //var_setpoint = analogRead(pot); //adquire setpoint variável pelo potenciometro
    //setpoint = map(var_setpoint, 0, 1023, 350, 900); //normalização do valor do setpoint, 325 foi a velocidade minima, 900 a máxima

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
    
    //Plota as informações no display OLED
    mostrar_valores(speed, pwm, cv, opcao, config);

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



  /*
  *  Essa função realiza a configuração do setpoint através
  * dos botões conectados ao arduino
  */
  void configurar_setpoint(int totalTime_bnt1, int totalTime_btn2){
    Serial.println(totalTime_btn1);
    Serial.println(totalTime_btn2);
    Serial.println("Dec-pos\t\t");
    Serial.println(dec_pos);

    //Se o botão direito for pressionado por um tempo
    if (totalTime_btn1 >= 550) {
      dec_pos++;
      if(dec_pos > 5) dec_pos = 0; 
    }
      
    //Se o botão direito for pressionado por um tempo
    if (totalTime_btn2 >= 550){
      dec_pos--;
      if(dec_pos < 0) dec_pos = 5;
    }

    
    //Se o botão direito for pressionado rapidamente
    if(totalTime_btn1<550 && totalTime_btn1 > 20){
      if (dec_pos == 0) setpoint += 1000;
      if (dec_pos == 1) setpoint += 100;
      if (dec_pos == 2) setpoint += 10;
      if (dec_pos == 3) setpoint ++;
      if (setpoint >= 7000) setpoint = 7000;
    }
    //Se o botão esquerdo for pressionado rapidamente
    if(totalTime_btn2<550 && totalTime_btn2 > 20){
      if (dec_pos == 0) setpoint -= 1000;
      if (dec_pos == 1) setpoint -= 100;
      if (dec_pos == 2) setpoint -= 10;
      if (dec_pos == 3) setpoint --;
      if (setpoint <= 0) setpoint = 0;
    }

    atualiza_display(dec_pos, setpoint, totalTime_btn1);

  }

  /*
  * É chamado toda vez para atualizar a tela OLED com as informações
  */
  void atualiza_display(int dec_pos,int setpoint, int totalTime_btn1){
    
    //Configurações iniciais
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0,0);
    display.cp437(true);

    display.println("SETPOINT: ");
    display.setTextSize(2);

    //Exibe os '0' das casas decimais
    if(setpoint < 1000 && setpoint >= 100) display.print("0");
    else if (setpoint < 100 && setpoint >= 10) display.print("00"); 
    else if (setpoint < 10 && setpoint >= 0) display.print("000");

    display.print(setpoint);
    display.print("      ");
    display.write(24);
    display.println("");

    //Ponteiro decimal
    for (int i = 1; i <= (dec_pos) ; i++) display.print(" ");

    //Ponteiro abaixo do simbolo de confirma  
    if(dec_pos == 4) {
      display.print("     ");
      display.write(30);
    
    }else if (totalTime_btn1 >= 550 && dec_pos == 5){
        config = false;
        dec_pos = 0;
        opcao = 0;

    } else display.write(30);
    display.display();

  }


  /*
  *  Essa função irá exibir na tela as informações.
  *  SETPOINT deve ser declarado como variável global 
  */
  void mostrar_valores(int speed , int pwm, int cv, int opcao, bool config) {
    //Se não estiver na tela de configuração
    if(config == false){ 
      //Limpa a tela e imprime as informações
      display.clearDisplay();
      display.setTextSize(2);             
      display.setCursor(0,0);
      switch(opcao){
        case 0:
          display.println(F("setpoint: "));
          display.print(setpoint);
          display.print(F( " RPM"));
          break;
        case 1:
          display.println(F("speed: "));
          display.print(speed);
          display.print(F(" RPM"));
          break;
        case 2:
          display.println(F("PWM: "));
          display.print(pwm);
          display.print(F(" %")); 
          break;
        case 3:
          display.println(F("cv: "));
          display.print(cv);
          break;
        }
      display.display();
    }
  }
