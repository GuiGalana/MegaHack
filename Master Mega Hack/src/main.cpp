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

//versão 1

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

//************************************** VARIAVEIS & DEFINES *******************************
//******************************************************************************************
// ----------:> Bluetooth
BluetoothSerial SerialBT;

// ----------:> Serial
#define BAUD_RATE_SERIAL 9600
#define baudrate 9600
SoftwareSerial sim808(32, 33); //  RX,  TX modem sim808
#define FROTA 101              //MINHA FROTA
#define DEBUG 0                //print serial sim808

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
char trama[200];

// -----------:> TasK
static uint8_t taskCoreZero = 0; // nucleo
//static uint8_t taskCoreOne  = 1;

// -----------:> SIM900
int conectadoTCP = 0;
char pacote[300];
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
  sendATcommand(pacote, "SEND OK", 2000);
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

boolean tcp_process(void)
{ // recebe dados wifi
  int j = 0;
  for (int i = 0; i < MAX_CLIENTS; i++)
  {
    if (Client[i].connected()) //identifica clientes
    {
      if (Client[i].available() > 0) //Verifica se tem dados
      {
        while (Client[i].available() > 0)
        {
          char c = Client[i].read();
          DataRec[j++] = c;
        }
        DataRec[j] = 0x00;
        if (DEBUG == 1)
        {
          Serial.println("Recebeu dados por WiFi:");
          Serial.println(DataRec);
        }
        return true;
      }
    }
    else //Se não tem ngm conectado
    {
      Client[i] = Server.available(); //libera posição para novo cliente
      delay(1);
      return false;
    }
  }
  return false;
}

void getData(unsigned long int id, unsigned char data[8])
{

  pgn = (id & 0xffff00);
  pgn = pgn >> 8;

  id = (id & 0x00FFFFFF);

  if (pgn == 0xF004)
  {
    vRPM = dataProcess(data, 3, 1, 0, 2);
    vRPM = vRPM * 0.125;
    vTorque = dataProcess(data, 2, 1, -125.00, 1);
  }
  if (pgn == 0xFEFC)
  {
    vNivelTanque = dataProcess(data, 1, 0.4, 0, 1);
  }

  if (millis() - LastTime >= TimeInfo)
  {
    LastTime = millis();
  }
}

void incia_CAN()
{
  Serial.println("CANConnect Diagnostico");

  if (CAN0.begin(MCP_STDEXT, CAN_250KBPS, MCP_8MHZ) == CAN_OK)
  {
    Serial.print("CANConnect Started\r\n");
  }
  else
  {
    Serial.print("CANConnect Failed\r\n");
  }

  CAN0.init_Filt(0, 1, 0x00);
  CAN0.init_Mask(1, 1, 0x00);
  CAN0.setMode(MCP_NORMAL);
  Serial.println("");
}

void interrup_CAN()
{
  if (!digitalRead(INT_CAN))
  {
    //Leitura do Barramento CAN
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    getData(rxId, rxBuf);
  }
}

float dataProcess(unsigned char input[8], int position, double factor, int offset, int dataLenght)
{
  unsigned long int aux1;
  unsigned long int aux2;
  unsigned long int aux3;
  unsigned long int aux4;
  double output = 0;

  if (dataLenght == 1)
  {
    output = (input[position] * factor) + offset;
    return output;
  }
  else if (dataLenght == 2)
  {                             //Concatenate and Sort byte input[0] = 1F        input[1]= 3C
    aux1 = input[position];     //aux1 = 1F
    aux2 = input[position + 1]; //aux2 = 3C
    aux2 = aux2 << 8;           //aux2 = 3C00

    aux1 = aux2 | aux1; //aux1 = 3C1F

    output = (aux1 * factor) + offset;
    return output;
  }
  else if (dataLenght == 3)
  {                             //Concatenate and Sort byte input[0] = 1F        input[1]= 3C       input[2] = 4F
    aux1 = input[position];     //aux1 = 1F
    aux2 = input[position + 1]; //aux2 = 3C
    aux3 = input[position + 2]; //aux3 = 4F

    aux3 = aux3 << 16; // aux3 = 4F 00 00
    aux2 = aux2 << 8;  //  aux2 = 00 3C 00

    aux1 = aux3 | aux2 | aux1; //aux1 = 4F3C1F

    output = (aux1 * factor) + offset;
    return output;
  }
  else if (dataLenght == 4)
  {
    // EXAMPLE INFORMMATION 1E 48 64 75
    aux1 = input[position];     //1E
    aux2 = input[position + 1]; //48
    aux3 = input[position + 2]; //64
    aux4 = input[position + 3]; //75

    //32 BITS DE INFORMAÇÃO
    //Concatenate and Sort byte ## input[0] = 1E  ## input[1]= 48  ## input[2] = 64 ## input[3] = 75##
    aux4 = aux4 << 24; //aux4 = 41 00 00 00
    aux3 = aux3 << 16; //aux3 = 00 3F 00 00
    aux2 = aux2 << 8;  //aux2 = 00 00 3C 00

    aux1 = aux4 | aux3 | aux2 | aux1; // aux1 = 41 3F 3C 1F
    output = (aux1 * factor) + offset;
    return output;
  }
  return -1;
}

void salva_trama()
{
  sprintf(trama, "%d,%d,%f,%f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", Data, Hora, Lat, Longi, velocidade_nos, vSpin, vRPM, vTorque, vNivelTanque, vConsumo);
  //appendFile(SPIFFS, "/GPSLOG.txt", trama);
  Serial.println(trama);

  root = SD.open(NomeArquivo, FILE_WRITE);
  /* if open succesfully -> root != NULL 
        then write string "Hello world!" to \
    */

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
}

/*void monta_trama()
{
  Serial.println("Monta pacote");
  tamPacote = 0;
  tamPacote = strlen(pacote);

  int i = 0;
  while (DataRec[i] != ',')
  {
    ComandoRec[i] = DataRec[i];
    i++;
  }
  if (ComandoRec[0] == 'S' || ComandoRec[0] == 'I')
  { //protege para não subir lixo
    sprintf(pacote, "%d,%s,%s,%s,%s,%s", FROTA, Data, Hora, Lat, Longi, DataRec);
    Serial.println(pacote);
    enviaGPRS();
  }
}*/

void inicia_WiFi()
{
  WiFi.mode(WIFI_AP);                     // configura como master
  WiFi.softAP("GalaInZe", "paodequeijo"); // rede, senha
  Server.begin();                         // inicia servidor wifi
  IPAddress IP = WiFi.softAPIP();         //request ip server
  Serial.print("Wifi configurado ...");
  Serial.print("IP address: ");
  Serial.println(IP);
}

void limpa_variavel()
{
  vRPM = 0;
  vTorque = 0;
  vNivelTanque = 0;
  vConsumo = 0;
}

//************************************** SETUP *********************************************
//******************************************************************************************
void setup()
{
  Serial.begin(BAUD_RATE_SERIAL);
  Serial.setTimeout(100);

  gps_serial.begin(9600, SERIAL_8N1, RXgps, TXgps);
  gps_serial.setTimeout(250);

  sim808.begin(BAUD_RATE_SERIAL);
  pinMode(PINreset, OUTPUT); // reset a placa processo padrão quando inicia
  digitalWrite(PINreset, LOW);

  Serial.println("iniciando...");

  inicia_WiFi();
  resetSIM808(); // liga sim808 sem precisar do botão
  beginSim808(); // testa placa on

  xTaskCreatePinnedToCore( //GPS
      GPS,                 // função que implementa a tarefa /
      "TaskGPS",           // nome da tarefa /
      10000,               // número de palavras a serem alocadas para uso com a pilha da tarefa /
      NULL,                // parâmetro de entrada para a tarefa (pode ser NULL) /
      0,                   // prioridade da tarefa (0 a N). maior mais alto /
      NULL,                // referência para a tarefa (pode ser NULL) /
      taskCoreZero);       //nucleo esp 32 -(0 ou 1)

  delay(1000);

  connectGPRS(); // connecta TCP

  incia_CAN();

  Serial.println("Aguarde");
  delay(1000);

  hora_no_arquivo();

  inicia_SD();
}

//************************************** LOOP **********************************************
//******************************************************************************************

void loop()
{
  interrup_CAN();
  salva_trama();

  delay(1000); //ciclo 10s
  limpa_variavel();
}