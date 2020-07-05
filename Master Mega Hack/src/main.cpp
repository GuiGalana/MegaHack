#include <Arduino.h>
#include "BluetoothSerial.h"
#include <mySD.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include "esp_task_wdt.h"
#include <SoftwareSerial.h>
#include <WiFi.h>
#include <CAN.h>
#include <SPI.h>
#include <math.h>

//versão 1.0

//************************************** ASSINATURAS ***************************************
//******************************************************************************************
void Leitura_CAN();
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
void colisao(float frotaColisao, double latColisao, double longiColisao, int spinColisao, int veloColisao);
void limpa_variavel();
//************************************** VARIAVEIS & DEFINES *******************************
//******************************************************************************************
// ----------:> identificação do bordo
float vFrota = 101; //MINHA FROTA

// ----------:> Bluetooth
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error ERRO BLUETOOTH!
#endif
BluetoothSerial BT;
char DataBtRecived[350];

// ----------:> Serial
#define BAUD_RATE_SERIAL 500000
#define baudrate 500000
SoftwareSerial sim808(32, 33); //  RX,  TX modem sim808
#define DEBUG 1                //print serial sim808

// ----------:> GPS
TinyGPSPlus gps;
#define RXgps 16
#define TXgps 17
HardwareSerial gps_serial(2);
unsigned long int Data = 0;
unsigned long int Hora = 0;
unsigned int velocidade = 0;     // velocidade em km/h
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
static uint8_t taskCoreOne = 1;  // nucleo

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
#define Baud_CAN 250000
#define Baud_Serial 500000
#define PGN_Mask 0x00FFFF00
#define ID_Mask 0x00FFFFFF

long id = 0x00000000;
uint8_t DataCAN[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
boolean CANSuccess = false;
float vRPM = -1.0;
float vNivelTanque = -1.0;
float vTorque = -1.0;
float vConsumo = -1.0;

//************************************** FUNÇÕES *******************************************
//******************************************************************************************
float dataProcess(uint8_t entrada_data[8], int Byte_inicial, double fator, int offset, int Tamanho_Byte)
{
  unsigned long int aux1;
  unsigned long int aux2;
  unsigned long int aux3;
  unsigned long int aux4;
  double output = 0;

  if (Tamanho_Byte == 1)
  {
    output = (entrada_data[Byte_inicial] * fator) + offset;
    return output;
  }
  else if (Tamanho_Byte == 2)
  {                                        //Concatenate and Sort byte input[0] = 1F        input[1]= 3C
    aux1 = entrada_data[Byte_inicial];     //aux1 = 1F
    aux2 = entrada_data[Byte_inicial + 1]; //aux2 = 3C
    aux2 = aux2 << 8;                      //aux2 = 3C00

    aux1 = aux2 | aux1; //aux1 = 3C1F

    output = (aux1 * fator) + offset;
    return output;
  }
  else if (Tamanho_Byte == 3)
  {                                        //Concatenate and Sort byte input[0] = 1F        input[1]= 3C       input[2] = 4F
    aux1 = entrada_data[Byte_inicial];     //aux1 = 1F
    aux2 = entrada_data[Byte_inicial + 1]; //aux2 = 3C
    aux3 = entrada_data[Byte_inicial + 2]; //aux3 = 4F

    aux3 = aux3 << 16; // aux3 = 4F 00 00
    aux2 = aux2 << 8;  //  aux2 = 00 3C 00

    aux1 = aux3 | aux2 | aux1; //aux1 = 4F3C1F

    output = (aux1 * fator) + offset;
    return output;
  }
  else if (Tamanho_Byte == 4)
  {
    // EXAMPLE INFORMMATION 1E 48 64 75
    aux1 = entrada_data[Byte_inicial];     //1E
    aux2 = entrada_data[Byte_inicial + 1]; //48
    aux3 = entrada_data[Byte_inicial + 2]; //64
    aux4 = entrada_data[Byte_inicial + 3]; //75

    //32 BITS DE INFORMAÇÃO
    //Concatenate and Sort byte ## input[0] = 1E  ## input[1]= 48  ## input[2] = 64 ## input[3] = 75##
    aux4 = aux4 << 24; //aux4 = 41 00 00 00
    aux3 = aux3 << 16; //aux3 = 00 3F 00 00
    aux2 = aux2 << 8;  //aux2 = 00 00 3C 00

    aux1 = aux4 | aux3 | aux2 | aux1; // aux1 = 41 3F 3C 1F
    output = (aux1 * fator) + offset;
    return output;
  }
  return -1;
}

void config_CAN()
{
  if (!CAN.begin(Baud_CAN))
  {
    Serial.println("Falha ao configura a CAN");
    CANSuccess = false;
  }
  else
  {
    Serial.println("CAN configurada com sucesso!");
    CANSuccess = true;
  }
}

void Debug_CAN(long identificador, uint8_t dados[8])
{
  /*int ax;
  Serial.print(String(identificador,HEX) + ": ");
  for(ax = 0; ax < 8; ax++){
    Serial.print(dados[ax], HEX);
    if (ax < 7)
    {
      Serial.print(",");
    }
  }
  Serial.println();*/
  Serial.printf("-> RPM,Torque,NivelTanque,vConsumo: %.2f, %.2f, %.2f, %.2f;\n", vRPM, vTorque, vNivelTanque, vConsumo);
  //limpa_variavel();
}

void Converte_dados_CAN(long identificador, uint8_t dados[8])
{
  identificador = identificador & ID_Mask;
  long pgn = identificador & PGN_Mask;
  pgn = pgn >> 8;

  if (pgn == 0xF004)
  {
    vRPM = dataProcess(dados, 3, 0.125, 0, 2);
    vTorque = dataProcess(dados, 2, 1, -125, 1);
  }
  if (pgn == 0xFEFC)
  {
    vNivelTanque = dataProcess(dados, 1, 0.4, 0, 1);
  }
  if (pgn == 0xFEF2)
  {
    vConsumo = dataProcess(dados, 0, 0.05, 0, 2);
  }
}

void Leitura_CAN(void *pvParameters)
{
  while (true)
  {
      if (CAN.parsePacket())
      {
        id = CAN.packetId();
        if (!CAN.packetRtr())
        {
          int pos = 0;
          while (CAN.available())
          {
            DataCAN[pos] = CAN.read();
            pos++;
          }
          esp_task_wdt_reset(); 
        }
        Converte_dados_CAN(id, DataCAN);
      }
      Debug_CAN(id, DataCAN);
      vTaskDelay(10);
  }
}

void Start_BT()
{
  BT.begin("MeuBordo-" + (String)vFrota);
  Serial.println("Bluetooth inicializado");
}

void Escreve_BT(char DataBtSend[350])
{
  if (BT.available())
  {
    BT.println(DataBtSend);
  }
}

void Leitura_BT()
{
  int i = 0;

  if (BT.available() > 0)
  {
    while (BT.available() > 0)
    {
      char c = BT.read();
      DataBtRecived[i++] = c;
    }
    DataBtRecived[i] = 0x00;
    Serial.println(DataBtRecived);
  }
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
      if (velocidade > velocidade_minima)
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
  char DataSend[350];
  int id = 0;
  String comando;
  tamPacote = 0;
  tamPacote = strlen(pacoteTrama);
  if (strcmp(quemChama, "colisao") == 0)
  { //id 2
    comando = "e32bt";
    id = 2;
    //bt
    sprintf(DataSend, "%s,%d,%f,%s", comando, id, vFrota, pacoteTrama);
    Serial.println(DataSend);
    Escreve_BT(DataSend);

    //bt
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
/*
void inicia_WiFi()
{
  WiFi.mode(WIFI_AP);                     // configura como master
  WiFi.softAP("GalaInZe", "paodequeijo"); // rede, senha
  Server.begin();                         // inicia servidor wifi
  IPAddress IP = WiFi.softAPIP();         //request ip server
  Serial.print("Wifi configurado ...");
  Serial.print("IP address: ");
  Serial.println(IP);
}*/

void limpa_variavel()
{
  vRPM = -1.0;
  vTorque = -1.0;
  vNivelTanque = -1.0;
  vConsumo = -1.0;
}

void colisao(float frotaColisao, double latColisao, double longiColisao, int spinColisao, int veloColisao)
{
  //quem pode chamar a funcao de colisao é outra embarcacao ou um ponto fixo.

  int i = 0, raiocolisao = 5; //raio para colisão 5 m
  float toleranciaImprecisao = 1.2, resultRaio = 0;
  ; //20% de tolerancia para o erro do gps
  double resultLat = 0, resultLong = 0;
  Lat = -21.222722;
  Longi = -50.419890;
  vSpin = 115;
  //velocidade_nos = velocidade_nos*0,514444; // converte para m/s
  velocidade_nos = 13.88;
  int tempoProjecao[4] = {3, 5, 7, 10}; //s
  int PontColisao[4] = {0, 0, 0, 0};

  double LatProjetada[4] = {0, 0, 0, 0};
  double LongProjetada[4] = {0, 0, 0, 0};
  double LatProjetadaColisao[4] = {0, 0, 0, 0};
  double LongProjetadaColisao[4] = {0, 0, 0, 0};

  for (i = 0; i < 4; i++)
  { //projeta meus pontos futuros
    LatProjetada[i] = Lat + ((velocidade_nos * tempoProjecao[i] * (cos(vSpin))) / 100000);
    LongProjetada[i] = Longi + ((velocidade_nos * tempoProjecao[i] * (sin(vSpin))) / 100000);
  }

  for (i = 0; i < 4; i++)
  { // projeta de quem chama a funcao
    LatProjetadaColisao[i] = latColisao + (((veloColisao / 3.6) * tempoProjecao[i] * (cos(spinColisao))) / 100000);
    LongProjetadaColisao[i] = longiColisao + (((veloColisao / 3.6) * tempoProjecao[i] * (sin(spinColisao))) / 100000);
  }

  for (i = 0; i < 4; i++)
  { // verifica se vai colidir algum ponto
    resultLat = (LatProjetada[i] - LatProjetadaColisao[i]) * 100000;
    resultLong = (LongProjetada[i] - LongProjetadaColisao[i]) * 100000;
    resultRaio = (sqrt(pow(resultLat, 2) + pow(resultLong, 2))) * toleranciaImprecisao;

    if (resultRaio < raiocolisao)
    {
      PontColisao[i] = 1;
      char texto[200];
      sprintf(texto, "%f,%f,%f", frotaColisao, LatProjetada[i], LongProjetada[i]);
      monta_trama("colisao", texto);
      //manda por bt para tela a colisão
    }
    else
    {
      PontColisao[i] = 0;
    }
    if (DEBUG == 1)
    {
      Serial.printf("colisão [%d] - lat: %f , Long: %f, chance colidir: %d\n", i, LatProjetadaColisao[i], LongProjetadaColisao[i], PontColisao[i]);
    }
  }
}

//************************************** SETUP *********************************************
//******************************************************************************************
void setup()
{
  Serial.begin(BAUD_RATE_SERIAL);
  Serial.setTimeout(100);
  config_CAN();
  /*
  Start_BT();

  gps_serial.begin(9600, SERIAL_8N1, RXgps, TXgps);
  gps_serial.setTimeout(250);

  sim808.begin(BAUD_RATE_SERIAL);
  pinMode(PINreset, OUTPUT); // reset a placa processo padrão quando inicia
  digitalWrite(PINreset, LOW);
*/
  Serial.println("iniciando...");

  //inicia_WiFi();
  //resetSIM808(); // liga sim808 sem precisar do botão
  //beginSim808(); // testa placa on
  /*
  xTaskCreatePinnedToCore( //GPS
      GPS,                 // função que implementa a tarefa /
      "TaskGPS",           // nome da tarefa /
      10000,               // número de palavras a serem alocadas para uso com a pilha da tarefa /
      NULL,                // parâmetro de entrada para a tarefa (pode ser NULL) /
      0,                   // prioridade da tarefa (0 a N). maior mais alto /
      NULL,                // referência para a tarefa (pode ser NULL) /
      taskCoreZero);       //nucleo esp 32 -(0 ou 1)

  delay(1000);
*/
  xTaskCreatePinnedToCore( //can
      Leitura_CAN,         // função que implementa a tarefa /
      "TaskCAN",           // nome da tarefa /
      100000,               // número de palavras a serem alocadas para uso com a pilha da tarefa /
      NULL,                // parâmetro de entrada para a tarefa (pode ser NULL) /
      0,                   // prioridade da tarefa (0 a N). maior mais alto /
      NULL,                // referência para a tarefa (pode ser NULL) /
      taskCoreOne);        //nucleo esp 32 -(0 ou 1)

  //connectGPRS(); // connecta TCP

  //incia_CAN();

  Serial.println("Aguarde");
  delay(3000);

  //hora_no_arquivo();

  //inicia_SD();
}

//************************************** LOOP **********************************************
//******************************************************************************************

void loop()
{
  /*
  colisao(211,-21.222722,-50.419890,115,50);//frota do outro, lat, long,spin, velo km/h
  Leitura_BT();*/

  //  Leitura_CAN();
}