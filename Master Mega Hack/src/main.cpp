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

//versão 2.1
/*
GPIO 04 -   RX    -    CAN
GPIO 05 -   TX    -    CAN
GPIO 32 -   RX    -    GPRS 
GPIO 33 -   TX    -    GPRS
GPIO 16 -   RX    -    ZIGB
GPIO 17 -   TX    -    ZIGB
GPIO 35 -   TX    -    GPS
GPIO 34 -   RX    -    GPS
GPIO 14 -   MOSI  -    SD 
GPIO 13 -   MISO  -    SD 
GPIO 27 -   SCK   -    SD 
GPIO 26 -    CS    -    SD 
GPIO 3 -    TX    -    ESP SERIAL
GPIO 1 -    RX    -    ESP SERIAL
*/
char StringTeste[350] = "$GTFW55,52,COLIDASAO,FROTA,PLAY";
//************************************** ASSINATURAS ***************************************
//******************************************************************************************
int GPRS(String dados_GPRS);
void Escrita_SD(char *dados_SD);
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
void tela_connectada();

//************************************** VARIAVEIS & DEFINES *******************************
//******************************************************************************************
// ----------:> identificação do bordo
float vFrota = 101; //MINHA FROTA
char vEstado[1];
String vOperacao;
int vCodigoOperacao = 0;
boolean vTelaConnected = 0;

// ----------:> Bluetooth
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error ERRO BLUETOOTH!
#endif
BluetoothSerial BT;
char DataBtRecived[350];
char DataBtSend[350];

// ----------:> Serial
#define BAUD_RATE_SERIAL 500000
SoftwareSerial sim808(32, 33); //  RX,  TX modem sim808
#define DEBUG 1                //print serial sim808

// ----------:> GPS
TinyGPSPlus gps;
#define RXgps 34
#define TXgps 35
SoftwareSerial gps_serial(RXgps, TXgps);
//HardwareSerial gps_serial(2);
unsigned long int Data = 0;
unsigned long int Hora = 0;
unsigned int velocidade = 0;     // velocidade em km/h
unsigned int velocidade_nos = 0; // velocidade em nós
double Lat = 0;
double Longi = 0;
int vSpin = 0;
#define velocidade_minima 1.5 //velocidade minima km/h

// ----------:> ARQUIVO
File root;
char NomeArquivo[17] = "Trama.txt";

// -----------:> TasK
static uint8_t taskCoreZero = 0; // nucleo
static uint8_t taskCoreOne = 1;  // nucleo

// -----------:> SIM900
int conectadoTCP = 0;
char response[300];
int tamPacote = 0;

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
#define PGN_Mask 0x00FFFF00
#define ID_Mask 0x00FFFFFF

long id = 0x00000000;
uint8_t DataCAN[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
boolean CANSuccess = false;
float vRPM = -1.0;
float vNivelTanque = -1.0;
float vTorque = -1.0;
float vConsumo = -1.0;
boolean LFE1_CAN = false;
#define TimeInfo 2000 // 10 segundo
unsigned long int LastTime = 0;
boolean Pacote_CAN = false;
int conta_CAN = 0;
//************************************** FUNÇÕES *******************************************
//******************************************************************************************

void TaskBT(void *pvParameters)
{
  int i = 0, count = 0, qtdtramabt = 20;
  while (true)
  {
    if (BT.available() > 0)
    {
      while (BT.available() > 0)
      {
        char c = BT.read();
        DataBtRecived[i++] = c;

        if (DataBtSend != 0)
        {
          BT.println(DataBtSend);
        }
      }
      Serial.print(DataBtRecived);
    }

    if (strcmp(DataBtRecived, "CONNECT\r\n") == 0)
    {
      vTelaConnected = 1;
      tela_connectada();
    }
    if (vTelaConnected == 1 && count < qtdtramabt)
    {
      count++;
      tela_connectada();
    }
    if (count == qtdtramabt)
    {
      count = 0;
      vTelaConnected = 0;
    }

    DataBtRecived[i] = 0x00;
    DataBtSend[i] = 0x00;
    i = 0;
    vTaskDelay(1000);
    esp_task_wdt_reset();
  }
}

void Start_BT()
{
  BT.begin("MeuBordo-" + (String)vFrota);
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
  if (!SD.begin(26, 14, 13, 27))
  {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  root = SD.open("/");
  if (root)
  {
    printDirectory(root, 0);
    root.close();
  }
  else
  {
    Serial.println("Erro ao acessar ROOT");
  }

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

void beginSim808()
{
  int tentativa = 0;
  uint8_t answer = 0;
  // testa AT
  answer = sendATcommand("AT", "OK", 2000);
<<<<<<< HEAD

    while (answer == 0 && tentativa <3)
    {
      answer = sendATcommand("AT", "OK", 2000);
      //Serial.println("Teste AT ok");
      tentativa++;
    }
=======
  while (answer == 0 && tentativa < 3)
  {
    answer = sendATcommand("AT", "OK", 2000);
    //Serial.println("Teste AT ok");
    tentativa++;
  }
>>>>>>> 1517cad7be4f14c0a529eaba5bea759641ec8fee
  Serial.println("Modem GPRS iniciado");
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
  //zig       :  
  //GTFEW     :
  //-------------------------*/
  char DataSend[350];
  int id = 0;
  String comando;
  tamPacote = 0;

  if (strcmp(quemChama, "tela_connectada") == 0)
  { //id 1
    comando = "e32bt";
    id = 1;
    //bt
    sprintf(DataSend, "%s,%d,%f,%s", comando, id, vFrota, pacoteTrama);
    BT.println(DataSend);
    //GTFEW
  }

  if (strcmp(quemChama, "colisao") == 0)
  { //id 2
    comando = "e32bt";
    id = 2;
    //bt
    sprintf(DataSend, "%s,%d,%f,%s", comando, id, vFrota, pacoteTrama);
    if (DEBUG == 1)
    {
      Serial.println(DataSend);
      BT.println(DataSend);
    }
    //GTFEW
<<<<<<< HEAD
    comando ="GTFEW";
    id =1;
    sprintf(DataSend, "%s,%d,%f,%s", comando, id, vFrota, pacoteTrama);
    tamPacote = strlen(DataSend);
=======
    comando = "GTFEW";
    id = 1;
    sprintf(DataSend, "%s,%d,%f,%s", comando, id, vFrota, pacoteTrama);
    tamPacote = strlen(DataSend);
    Escrita_SD(DataSend);
>>>>>>> 1517cad7be4f14c0a529eaba5bea759641ec8fee
    /*se(manda GPRS(dataSend) == 0){
      salvacartão(dataSend);
    }*/
  }
<<<<<<< HEAD
   if (strcmp(quemChama, "monitoramento") == 0)
    //GTFEW
    comando ="GTFEW";
    id =0;
    sprintf(DataSend, "%s,%d,%f,%s", comando, id, vFrota, pacoteTrama);
    tamPacote = strlen(DataSend);
    /*se(manda GPRS(dataSend) == 0){
      salvacartão(dataSend);
    }*/
=======
  if (strcmp(quemChama, "monitoramento") == 0)
    //GTFEW
    comando = "GTFEW";
  id = 0;
  sprintf(DataSend, "%s,%d,%f,%s", comando, id, vFrota, pacoteTrama);
  tamPacote = strlen(DataSend);
  Escrita_SD(DataSend);
  /*se(manda GPRS(dataSend) == 0){
      salvacartão(dataSend);
    }*/
}

int GPRS(String dados_GPRS)
{
  return 0;
}

void Escrita_SD(char *Data_proSD)
{
      int aux = strlen(Data_proSD);
      char Dada_proSD[aux];
      int j = 0;
      for (j=0; j < (aux); j++)
      {
        Dada_proSD[j] = Data_proSD[j];
      }

      root = SD.open(NomeArquivo, FILE_WRITE);

      if (root)
      {
        // Serial.print("Writing to test.txt...");
        root.println(Dada_proSD);
        root.close();
        Serial.println("done.");
      }
      else
      {
        // if the file didn't open, print an error:
        Serial.println("error opening test.txt");
      }
>>>>>>> 1517cad7be4f14c0a529eaba5bea759641ec8fee
}

void tela_connectada()
{
  //estado, operacao, data, hora,time
  vEstado[0] = 'E';
  //vOperacao[20];
  vCodigoOperacao = 999;
  char texto[200];
  sprintf(texto, "%c,%s,%d,%d,%d,%f,%f,%d,%f,%f,%f,%f", vEstado, vOperacao, vCodigoOperacao, Data, Hora, velocidade_nos, vSpin, Lat, Longi, vNivelTanque, vConsumo);
  monta_trama("tela_connectada", texto);
}

void limpa_variavel()
{
  vRPM = 0;
  vTorque = 0;
  vNivelTanque = 0;
  vConsumo = 0;
}

void colisao(float frotaColisao, double latColisao, double longiColisao, int spinColisao, int veloColisao)
{
  //quem pode chamar a funcao de colisao é outra embarcacao ou um ponto fixo.

  int i = 0, raiocolisao = 5;                       //raio para colisão 5 m
  float toleranciaImprecisao = 1.2, resultRaio = 0; //20% de tolerancia para o erro do gps
  double resultLat = 0, resultLong = 0;
  //valores para teste
  Lat = -21.222722;
  Longi = -50.419890;
  vSpin = 115;
  velocidade_nos = velocidade_nos*0,514444; // converte para m/s
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

  //if(veloColisao > 1.5){
  for (i = 0; i < 4; i++)
  { // projeta de quem chama a funcao
    LatProjetadaColisao[i] = latColisao + (((veloColisao / 3.6) * tempoProjecao[i] * (cos(spinColisao))) / 100000);
    LongProjetadaColisao[i] = longiColisao + (((veloColisao / 3.6) * tempoProjecao[i] * (sin(spinColisao))) / 100000);
  }
  //}
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

void colisao_manobra(float frotaManobra, double latManobra, double longManobra, int spinManobra)
{
}

void maquina_estado()
{
  //torque, velocidade, colisao, rpm, ponto fixo
  //----------------------------estados
  //  Efeivo -- Puxando ou empurrando
  //  Parada -- cod de parada ou ponto fixo
  //  Deslocamento

  int inicioManobra = 0;
  int motor_ligado = 200;
  int quantidadePF = 0;
  int i = 0;
  double resultLat = 0, resultLong = 0;
  float toleranciaImprecisao = 1.2, resultRaio = 0;
  int raiocolisao = 5;
  //rotina corre lista de ponto fixo e retorna a quantidade
  //para teste valor fixo
  quantidadePF = 9;
  double latPontoFixo[quantidadePF];
  double longPontoFixo[quantidadePF];

  //teste parada
  /*
  latPontoFixo[0] = -21.222722;
  longPontoFixo[0] - 50.419890;
  */
  if (velocidade_nos > velocidade_minima && vRPM > motor_ligado)
  { //Deslocamento motor ligado e velocidade maior que 1.5
    vEstado[0] = 'D';
  }
  if (velocidade_nos < velocidade_minima)
  { // parada com validação de ponto fixo
    vEstado[0] = 'F';
    //valida ponto fixo
    for (i = 0; i <= quantidadePF; i++)
    {
      resultLat = (Lat - latPontoFixo[i]) * 100000;
      resultLong = (Longi - longPontoFixo[i]) * 100000;
      resultRaio = (sqrt(pow(resultLat, 2) + pow(resultLong, 2))) * toleranciaImprecisao;

      if (resultRaio < raiocolisao)
      {
        vOperacao = "Nome Parada"; // esse nome vem da lista de parada
      }
      else
      {
        //solicita parada > manda comando por bt para tela
      }
    }
  } // fim do estado parada
}

void alarme_operacionais()
{
}

void calcula_consumo()
{
  if(LFE1_CAN == false){
  LFE1_CAN = false;
  float reg_coef0 = 24.905251702;
  float reg_coef1 = 11.04978200;
  float reg_intercept_ = 2.9715984714;

  vConsumo = reg_coef0 * (velocidade / 36.48) + (reg_coef1 * pow(vRPM, 2) / pow(2280, 2)) + reg_intercept_;
  }
}

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
  /*int i;
  Serial.print(String(identificador, HEX) + ": ");
  for (i = 0; i < 8; i++)
  {
    Serial.print(dados[i], HEX);
    if (i < 7)
    {
      Serial.print(",");
    }
  }*/
  Serial.printf("RPM,Torque,NivelTanque,Consumo: %.2f, %.2f, %.2f, %.2f;", vRPM, vTorque, vNivelTanque, vConsumo);
  Serial.println();
}

void Converte_dados_CAN(long identificador, uint8_t dados[8])
{
  identificador = identificador & ID_Mask;
  long pgn = identificador & PGN_Mask;
  pgn = pgn >> 8;
  if (pgn == 0xF004)
  {
    conta_CAN = 0;
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
    LFE1_CAN = true;
  }
}

void Leitura_CAN()
{
      //Serial.println("Entro Pacote");
      Pacote_CAN = true;
      id = CAN.packetId();
      if (!CAN.packetRtr())
      {
        int pos = 0;
        while (CAN.available())
        {
          DataCAN[pos] = CAN.read();
          pos++;
        }
        
      }
     // limpa_variavel();
      Converte_dados_CAN(id, DataCAN); 
      //id = 0x00000000;
}

//************************************** SETUP *********************************************
//******************************************************************************************

void setup()
{
  Serial.begin(BAUD_RATE_SERIAL);
  Serial.setTimeout(100);
  Serial.println("Serial configurada!");
  config_CAN();
  Start_BT();

  gps_serial.begin(BAUD_RATE_SERIAL);
  gps_serial.setTimeout(250);

  sim808.begin(BAUD_RATE_SERIAL);
  beginSim808(); // testa placa on

  inicia_SD();

  
  xTaskCreatePinnedToCore( //GPS
      GPS,                 // função que implementa a tarefa /
      "TaskGPS",           // nome da tarefa /
      10000,               // número de palavras a serem alocadas para uso com a pilha da tarefa /
      NULL,                // parâmetro de entrada para a tarefa (pode ser NULL) /
      0,                   // prioridade da tarefa (0 a N). maior mais alto /
      NULL,                // referência para a tarefa (pode ser NULL) /
      taskCoreZero);       //nucleo esp 32 -(0 ou 1)

  xTaskCreatePinnedToCore( //Bluetooth
      TaskBT,              // função que implementa a tarefa /
      "TaskBT",            // nome da tarefa /
      10000,               // número de palavras a serem alocadas para uso com a pilha da tarefa /
      NULL,                // parâmetro de entrada para a tarefa (pode ser NULL) /
      2,                   // prioridade da tarefa (0 a N). maior mais alto /
      NULL,                // referência para a tarefa (pode ser NULL) /
      taskCoreOne);        //nucleo esp 32 -(0 ou 1)


  delay(1100);
  //inicia_SD();
}

//************************************** LOOP **********************************************
//******************************************************************************************

void loop()
{
    
  if (CAN.parsePacket())
    {
      Leitura_CAN();
    }else{
      conta_CAN++;
    }
    if(vRPM > 0){
      velocidade = 50;  //simulação km/h
      velocidade_nos = 27;
      calcula_consumo();
    }else{
      velocidade = 0;  //simulação km/h
      velocidade_nos = 0;
      vConsumo = 0;
    }
    if(conta_CAN > 100){
      limpa_variavel();
    }
    
    Debug_CAN(id, DataCAN);
    //if(!CAN.available()){ limpa_variavel();}



  // hora_no_arquivo(); 
  //teste colisão
  colisao(211,-21.222722,-50.419890,115,50);//frota do outro, lat, long,spin, velo km/h
  //não aumentar valor do delay
  delay(10);//
}