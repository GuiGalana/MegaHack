#include <Arduino.h>
#include "BluetoothSerial.h"
#include <mySD.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include "esp_task_wdt.h"
#include <SoftwareSerial.h>
#include <WiFi.h>
#include "mcp_can.h"
#include "mcp_can_dfs.h"
#include <SPI.h>
#include <math.h>

//versão 1.0

//************************************** ASSINATURAS ***************************************
//******************************************************************************************
void resetESP();
void resetSIM808();
void beginSim808();
void connectGPRS();
void enviaGPRS();
String getValue(String data, char separator, int index);
boolean tcp_process(void);
void monta_trama();
float dataProcess(unsigned char input[8], int position, double factor, int offset, int dataLenght);
void getData(unsigned long int id, unsigned char data);
void sendInfo();
void colisao (float frotaColisao,double latColisao, double longiColisao, int spinColisao,int veloColisao);

//************************************** VARIAVEIS & DEFINES *******************************
//******************************************************************************************
// ----------:> identificação do bordo
float vFrota = 101;              //MINHA FROTA
char vEstado[1];
char vOperacao[20];
int vCodigoOperacao=0;
boolean vTelaConnected =0;

// ----------:> Bluetooth
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error ERRO BLUETOOTH!
#endif
BluetoothSerial BT;
char DataBtRecived[350];
char DataBtSend[350];

// ----------:> Serial
#define BAUD_RATE_SERIAL 9600
#define baudrate 9600
SoftwareSerial sim808(32, 33); //  RX,  TX modem sim808
#define DEBUG   1              //print serial sim808

// ----------:> GPS
TinyGPSPlus gps;
#define RXgps 16
#define TXgps 17
HardwareSerial gps_serial(2);
unsigned long int Data = 0;
unsigned long int Hora = 0;
unsigned int velocidade = 0; // velocidade em km/h
unsigned int velocidade_nos = 0; // velocidade em nós
double Lat = 0;
double Longi = 0;
double vSpin = 0;
#define velocidade_minima 1.5 //velocidade minima km/h

// ----------:> ARQUIVO
File root;
char NomeArquivo[17];

// -----------:> TasK
static uint8_t taskCoreZero = 0; // nucleo
static uint8_t taskCoreOne  = 1; // nucleo

// -----------:> SIM900
int conectadoTCP = 0;
char response[300];
int tamPacote = 0;
#define PINreset 35

// ----------:> Wifi
#define MAX_CLIENTS 20 //quantidade max de esp8266
WiFiServer Server(80);
WiFiClient Client[MAX_CLIENTS];
WiFiClient Client1;
char ComandoRec[200];
char IpRec[200];
char IdRec[200];
char LastCampo[200];
char DataRec[200];

// ----------:> CAN
unsigned long int LastTime = 0;
#define TimeInfo 1000 // 1 segundo
long unsigned int rxId;
long unsigned int pgn;
unsigned char len = 0;
unsigned char rxBuf[8];
float vRPM = -1.0;
float vTorque = -1.0;
float vNivelTanque = -1.0;
float vConsumo = -1.0;
MCP_CAN CAN0(10);
#define INT_CAN 2

//************************************** FUNÇÕES *******************************************
//******************************************************************************************

void TaskBT(void *pvParameters)
{
  int i=0;
  while (true)
  {
    if (BT.available() > 0) {
      while(BT.available() > 0){
          char c = BT.read();
          DataBtRecived[i++] = c;

          if (DataBtSend != 0){
             BT.println(DataBtSend);
          }
      }
      Serial.print(DataBtRecived);
      Serial.println(DataBtSend);
      //Serial.println(strlen(DataBtRecived));
      //Serial.println(strcmp (DataBtRecived, "CONNECT\r\n"));
    }
    
    if (strcmp (DataBtRecived, "CONNECT\r\n") == 0){
        vTelaConnected =1;
    }

    DataBtRecived[i]=0x00;
    DataBtSend[i]=0x00;
    i=0;
  }
  vTaskDelay(10);
  esp_task_wdt_reset();
}

void Start_BT(){
  BT.begin("MeuBordo-"+(String)vFrota);
  Serial.println("Bluetooth inicializado");
}

int8_t sendATcommand(char *ATcommand, char *expected_answer1, unsigned int timeout)
{
  //char response[300];
  uint8_t x = 0, answer = 0;
  unsigned long previous;

  memset(response, '\0', 100); // inicializa string

  delay(100);

  while (sim808.available() > 0)
    sim808.read(); // limpa a serial

  sim808.println(ATcommand);

  x = 0;
  previous = millis();

  // aguarda resposta
  do
  {
    if (sim808.available() != 0)
    {
      response[x] = sim808.read();
      x++;
      // check respost do modulo
      if (strstr(response, expected_answer1) != NULL)
      {
        answer = 1;
      }
    }
    // espera timeout
  } while ((answer == 0) && ((millis() - previous) < timeout));

  if (DEBUG == 1)
  {
    Serial.println(response); //debug resposta
  }
  return answer;
}

void GPS(void *pvParameters)
{
  while (true)
  {
    while (gps_serial.available() > 0)
    {
      //Serial.println(gps_serial.readString());
      gps.encode(gps_serial.read());
      Lat = gps.location.lat();
      Longi = gps.location.lng();
      Data = gps.date.value(); //DDMMAA
      Hora = gps.time.value(); // HHMMSS
      velocidade = gps.speed.kmph();
      velocidade_nos = gps.speed.knots();
      if(velocidade > velocidade_minima)
      {
       vSpin = gps.course.deg();
      }
    }
    vTaskDelay(10);
    esp_task_wdt_reset();
  }
}

void printDirectory(File dir, int numTabs)
{

  while (true)
  {
    File entry = dir.openNextFile();
    if (!entry)
    {
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++)
    {
      Serial.print('\t'); // we'll have a nice indentation
    }
    // Print the name
    Serial.print(entry.name());
    if (entry.isDirectory())
    {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    }
    else
    {
      Serial.print("\t\t");
      Serial.println(entry.size());
    }
    entry.close();
  }
}

void hora_no_arquivo()
{
  char hora_Pre[9];
  sprintf(hora_Pre, "%i", Hora);
  String HORA_PRE = "";
  HORA_PRE = hora_Pre;
  HORA_PRE = HORA_PRE.substring(0, 6);

  sprintf(NomeArquivo, "GL%s.txt\0", HORA_PRE);
  Serial.print("Nome Arquivo: ");
  Serial.println(NomeArquivo);
}

void inicia_SD()
{
  Serial.println("Initializing SD card...");
  /* initialize SD library with Soft SPI pins, if using Hard SPI replace with this SD.begin() | SD.begin(26, 14, 13, 27)*/
  if (!SD.begin())
  {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  root = SD.open(NomeArquivo, FILE_WRITE);
  /* if open succesfully -> root != NULL 
    then write string "Hello world!" to it
  */
  if (root)
  {
    root.print("Inicio LOG: ");
    root.print(Data);
    root.print(" - ");
    root.println(Hora);
    root.flush();
    root.close();
  }
  else
  {
    Serial.print("error creating ");
    Serial.println(NomeArquivo);
  }
  delay(1000);

  Serial.println("done!");
}

String getValue(String data, char separator, int index)
{
  int maxIndex = data.length() - 1;
  int j = 0;
  String chunkVal = "";

  for (int i = 0; i <= maxIndex && j <= index; i++)
  {
    chunkVal.concat(data[i]);

    if (data[i] == separator)
    {
      j++;

      if (j > index)
      {
        chunkVal.trim();
        return chunkVal;
      }

      chunkVal = "";
    }
    else if ((i == maxIndex) && (j < index))
    {
      chunkVal = "";
      return chunkVal;
    }
  }
}

void enviaGPRS()
{
  char aux_str[100];
  int i = 0;
  for (i = 0; i < 100; i++)
  {
    aux_str[i] = NULL;
  }

  Serial.println("Enviando pacote");
  snprintf(aux_str, sizeof(aux_str), "AT+CIPSEND=%d", tamPacote);
  sendATcommand(aux_str, ">", 2000);
  //sendATcommand(pacote, "SEND OK", 2000);
}

void connectGPRS()
{
  Serial.println("Conectando TCP/ip ");
  uint8_t answer = 0;
  int tentativa = 0;

  while (answer == 0 && tentativa < 3)
  {
    answer = sendATcommand("AT+CIPSHUT", "SHUT OK", 2000);
    tentativa++;
    Serial.print(".");
  }
  answer = 0;

  while (answer == 0 && tentativa < 5)
  {
    answer = sendATcommand("AT+CGATT=1", "OK", 2000);
    tentativa++;
    Serial.print(".");
  }
  answer = 0;

  while (answer == 0 && tentativa < 6)
  {
    answer = sendATcommand("AT+CIPSTART=\"TCP\",\"3.15.197.22\",\"9998\"", "OK", 2000); // TIPO TCP, IP, PORTA
    tentativa++;
    if (answer == 1)
    {
      Serial.println("Conectdo TCP");
      conectadoTCP = 1;
    }
    else
    {
      Serial.println("Falha Conexão");
      conectadoTCP = 0;
    }
  }
}

void resetESP()
{
  Serial.println("Resetando ESP32");
  ESP.restart();
}

void resetSIM808()
{
  digitalWrite(PINreset, HIGH);
  delay(3000);
  digitalWrite(PINreset, LOW);
  Serial.println("reset Sim808 ok");
}

void beginSim808()
{
  uint8_t answer = 0;
  // testa AT
  answer = sendATcommand("AT", "OK", 2000);
  if (answer == 0)
  {
    digitalWrite(PINreset, HIGH);
    delay(3000);
    digitalWrite(PINreset, LOW);
    Serial.println("reset Sim808 ok");
    while (answer == 0)
    {
      answer = sendATcommand("AT", "OK", 2000);
      Serial.println("Teste AT ok");
    }
  }
}

/*
void salva_trama()
{
  sprintf(trama, "%d,%d,%f,%f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", Data, Hora, Lat, Longi, velocidade_nos, vSpin, vRPM, vTorque, vNivelTanque, vConsumo);
  Serial.println(trama);

  root = SD.open(NomeArquivo, FILE_WRITE);
  if (root)
  {
    root.println(trama);
    root.flush();
    root.close();
  }
  else
  {
    Serial.print("Erro ao tentar escrever ");
    Serial.println(NomeArquivo);
  }
}/*/

void monta_trama(char quemChama[100], char pacoteTrama[300]) //função que chama a trama, pacote da msg
{
  /*------------------cabeçalhos 
  //bluetooth :  comando,id,frota, 
  //zig
  //GTFEW  
  //-------------------------*/
  char DataSend [350];
  int id=0;
  String comando;
  tamPacote = 0;
  tamPacote = strlen(pacoteTrama);

  if(strcmp (quemChama, "tela_connectada") == 0){   //id 1
      comando = "e32bt";
      id=1;
      //bt
      sprintf(DataSend,"%s,%d,%f,%s",comando,id,vFrota,pacoteTrama);
  }

  if(strcmp (quemChama, "colisao") == 0){   //id 2
      comando = "e32bt";
      id=2;
      //bt
      sprintf(DataSend,"%s,%d,%f,%s",comando,id,vFrota,pacoteTrama);
      if (DEBUG == 1){
          Serial.println(DataSend);
      }
    //GTFEW
  }
/*
  if (ComandoRec[0] == 'S' || ComandoRec[0] == 'I')
  { //protege para não subir lixo
    sprintf(pacote, "%d,%s,%s,%s,%s,%s", FROTA, Data, Hora, Lat, Longi, DataRec);
    Serial.println(pacote);
    enviaGPRS();
  }*/
}

void tela_connectada(void *pvParameters){
  //estado, operacao, data, hora,time
  vEstado[1] = 'E';
  //vOperacao[20];
  vCodigoOperacao= 999;

  int redundancia =0;
  char texto [200];
  while (true){

      sprintf(texto,"%c,%s,%d,",vEstado,vOperacao,vCodigoOperacao);
      Serial.println(".");

      if (vTelaConnected == 1){
        Serial.println("Entrou com connect do bt");
          redundancia =0;
          monta_trama("tela_connectada",texto);
      }

      if(redundancia < 10){
          redundancia++;
          monta_trama("tela_connectada",texto);
          vTelaConnected =0;
      }
  }
  vTaskDelay(10);
  esp_task_wdt_reset();
}

void limpa_variavel()
{
  vRPM = 0;
  vTorque = 0;
  vNivelTanque = 0;
  vConsumo = 0;
}

void colisao (float frotaColisao, double latColisao, double longiColisao, int spinColisao,int veloColisao)
{
//quem pode chamar a funcao de colisao é outra embarcacao ou um ponto fixo.

  int i=0, raiocolisao =5; //raio para colisão 5 m
  float toleranciaImprecisao = 1.2,resultRaio =0;; //20% de tolerancia para o erro do gps
  double resultLat =0, resultLong = 0;
  //valores para teste
  Lat = -21.222722;
  Longi = -50.419890;
  vSpin = 115;
  //velocidade_nos = velocidade_nos*0,514444; // converte para m/s
  velocidade_nos = 13.88;
  int tempoProjecao [4]= {3,5,7,10}; //s
  int PontColisao [4] = {0,0,0,0};

  double LatProjetada[4] = {0,0,0,0};
  double LongProjetada[4] = {0,0,0,0};
  double LatProjetadaColisao[4] = {0,0,0,0};
  double LongProjetadaColisao[4] = {0,0,0,0};

for(i=0;i<4;i++){ //projeta meus pontos futuros
    LatProjetada[i] = Lat+((velocidade_nos*tempoProjecao[i]*(cos(vSpin)))/100000);
    LongProjetada[i] = Longi+((velocidade_nos*tempoProjecao[i]*(sin(vSpin)))/100000);
  }


for(i=0;i<4;i++){ // projeta de quem chama a funcao
    LatProjetadaColisao[i] = latColisao+(((veloColisao/3.6)*tempoProjecao[i]*(cos(spinColisao)))/100000);
    LongProjetadaColisao[i] = longiColisao+(((veloColisao/3.6)*tempoProjecao[i]*(sin(spinColisao)))/100000);
  }

  for(i=0;i<4;i++){ // verifica se vai colidir algum ponto
    resultLat = (LatProjetada[i]-LatProjetadaColisao[i])*100000;
    resultLong = (LongProjetada[i]-LongProjetadaColisao[i])*100000;
    resultRaio = (sqrt(pow(resultLat,2) + pow(resultLong,2)))*toleranciaImprecisao;
    
    if(resultRaio < raiocolisao){
      PontColisao[i] = 1;
      char texto [200];
      sprintf(texto,"%f,%f,%f",frotaColisao,LatProjetada[i],LongProjetada[i]);
      monta_trama("colisao",texto);
      //manda por bt para tela a colisão
    }
    else{
      PontColisao[i]=0;
    }
      if (DEBUG == 1){
        Serial.printf("colisão [%d] - lat: %f , Long: %f, chance colidir: %d\n",i,LatProjetadaColisao[i],LongProjetadaColisao[i],PontColisao[i]);
      }
  }
}

//************************************** SETUP *********************************************
//******************************************************************************************
void setup()
{
  Serial.begin(BAUD_RATE_SERIAL);
  Serial.setTimeout(100);
  
  Start_BT();

  gps_serial.begin(9600, SERIAL_8N1, RXgps, TXgps);
  gps_serial.setTimeout(250);

  sim808.begin(BAUD_RATE_SERIAL);
  pinMode(PINreset, OUTPUT); // reset a placa processo padrão quando inicia
  digitalWrite(PINreset, LOW);

  Serial.println("iniciando...");

  //resetSIM808(); // liga sim808 sem precisar do botão
  //beginSim808(); // testa placa on

  xTaskCreatePinnedToCore( //GPS
      GPS,                 // função que implementa a tarefa /
      "TaskGPS",           // nome da tarefa /
      10000,               // número de palavras a serem alocadas para uso com a pilha da tarefa /
      NULL,                // parâmetro de entrada para a tarefa (pode ser NULL) /
      0,                   // prioridade da tarefa (0 a N). maior mais alto /
      NULL,                // referência para a tarefa (pode ser NULL) /
      taskCoreZero);       //nucleo esp 32 -(0 ou 1)

  xTaskCreatePinnedToCore( //Bluetooth
      TaskBT,                 // função que implementa a tarefa /
      "TaskBT",           // nome da tarefa /
      10000,               // número de palavras a serem alocadas para uso com a pilha da tarefa /
      NULL,                // parâmetro de entrada para a tarefa (pode ser NULL) /
      2,                   // prioridade da tarefa (0 a N). maior mais alto /
      NULL,                // referência para a tarefa (pode ser NULL) /
      taskCoreOne);       //nucleo esp 32 -(0 ou 1)

  xTaskCreatePinnedToCore( //Bluetooth
      tela_connectada ,                 // função que implementa a tarefa /
      "tela_connectada",           // nome da tarefa /
      10000,               // número de palavras a serem alocadas para uso com a pilha da tarefa /
      NULL,                // parâmetro de entrada para a tarefa (pode ser NULL) /
      1,                   // prioridade da tarefa (0 a N). maior mais alto /
      NULL,                // referência para a tarefa (pode ser NULL) /
      taskCoreOne);       //nucleo esp 32 -(0 ou 1)
     

  delay(1000);
  //connectGPRS(); // connecta TCP

  Serial.println("Aguarde");
  DataBtSend[0]=0;
  delay(1000);

  //hora_no_arquivo();
  //inicia_SD();
}

//************************************** LOOP **********************************************
//******************************************************************************************

void loop()
{
  //teste colisão
  //colisao(211,-21.222722,-50.419890,115,50);//frota do outro, lat, long,spin, velo km/h
  delay(1000);
}