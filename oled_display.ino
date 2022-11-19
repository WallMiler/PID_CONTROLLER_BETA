#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define led 13
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


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
int dec_pos = 0;

int setpoint = 0;

void setup() {
  pinMode(13, OUTPUT);
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  Serial.begin(9600);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  display.display();
  delay(1000);

}

void loop(){

  // Criar flags para quando estiver configurando o setpoint
  // Verificar quando o botao saiu do estado HIGH para LOW  
  //Pressionou o botao
  currentState_btn1 = !digitalRead(BUTTON1_PIN);
  currentState_btn2 = !digitalRead(BUTTON2_PIN);

 

//------------- VERIFICAR BUGS AO PRESSIONAR UM BOTÃO E DEPOIS PRESSIONAR E SOLTAR O OUTRO


  //Se algum dos botões sair de LOW para HIGH (for pressionado)
  //Começa o timer
  
  if  (lastState_btn1 == LOW && currentState_btn1 == HIGH)  startTime_btn1 = millis();
  if  (lastState_btn2 == LOW && currentState_btn2 == HIGH)  startTime_btn2 = millis();

  
  //se soltou o botão, para o timer e reseta o startTime
  if  (lastState_btn1 == HIGH && currentState_btn1 == LOW) {
    endTime_btn1 = millis();
    totalTime_btn1 = endTime_btn1 - startTime_btn1;
    startTime_btn1 = endTime_btn1 = 0;
  } 
  if  (lastState_btn2 == HIGH && currentState_btn2 == LOW) {
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


  if (config == true){
    config_setpoint(totalTime_btn1, totalTime_btn2);
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


  //attachInterrupt(digitalPinToInterrupt(2), increment, RISING);
  mostrar_valores(300, 20, 100, opcao, config);
  lastState_btn1 = currentState_btn1;
  lastState_btn2 = currentState_btn2;


}


//Atualiza a tela toda vez que altera o setpoint
void update_screen(int dec_pos,int setpoint, int totalTime_btn1){
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.cp437(true);
  display.println("SETPOINT: ");
  display.setTextSize(2);
  if(setpoint < 1000 && setpoint >= 100) display.print("0");
  else if (setpoint < 100 && setpoint >= 10) display.print("00"); 
  else if (setpoint < 10 && setpoint >= 0) display.print("000");
  display.print(setpoint);
  display.print("      ");
  display.write(24);
  display.println("");
  for (int i = 1; i <= (dec_pos) ; i++) display.print(" ");

//  Ponteiro ficara em cima da opção de confirmar  
  if(dec_pos == 4) {
    display.print("     ");
    display.write(30);
  }else if (totalTime_btn1 >= 550 && dec_pos == 5){
      config = false;
      dec_pos = 0;
      opcao = 0;
    }else display.write(30);
    

  
  display.display();
}


//Esse função será chamada toda vez
void config_setpoint(int totalTime_bnt1, int totalTime_btn2){


  Serial.println(totalTime_btn1);
  Serial.println(totalTime_btn2);
  Serial.println("Dec-pos\t\t");
  Serial.println(dec_pos);

  if (totalTime_btn1 >= 550) {
    dec_pos++;
    if(dec_pos > 5) dec_pos = 0; 
  }
    
  if (totalTime_btn2 >= 550){
    dec_pos--;
    if(dec_pos < 0) dec_pos = 5;
  }

  
  //Se o tempo pressionado for menor que meio segundo:
  if(totalTime_btn1<550 && totalTime_btn1 > 20){
    if(dec_pos == 0) setpoint += 1000;
    if(dec_pos == 1) setpoint += 100;
    if(dec_pos == 2) setpoint += 10;
    if(dec_pos == 3) setpoint ++;
    if(setpoint >= 7000) setpoint = 7000;
  }
  if(totalTime_btn2<550 && totalTime_btn2 > 20){
    if(dec_pos == 0) setpoint -= 1000;
    if(dec_pos == 1) setpoint -= 100;
    if(dec_pos == 2) setpoint -= 10;
    if(dec_pos == 3) setpoint --;
    if (setpoint <= 0) setpoint = 0;
  }

   
  
  update_screen(dec_pos, setpoint, totalTime_btn1);
}


void mostrar_valores(int rpm , int pwm, int cv, int opcao, bool config) {
  if(config == false){ 
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);
  switch(opcao){
    case 0:
      display.println(F("setpoint: "));
      display.print(setpoint);
      display.print(F( " RPM"));
      break;
    case 1:
      display.println(F("speed: "));
      display.print(rpm);
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
