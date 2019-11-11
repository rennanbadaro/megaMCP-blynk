/**
 * MegaMCP-Blynk
 * Curso: Engenharia Eletrônica
 * Autor: Grupo C
 * Versão: 1.0.0
 * Revisao: 09/11/2019
 * Microcontrolador: ATMega328P
 */

/**
* Import das bibliotecas do ESP8266, Blynk e criacao de portas seriais, respectivamente
*/
#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>
#include <SoftwareSerial.h>

/**
* Definindo OPCODES (comandos) do MCP39F501 (informações do datasheet)
*/
#define        NO                      0
#define        YES                     1
#define        ON                      0
#define        OFF                     1
#define        HEADER_BYTE             0xA5
#define        ACK                     0x06
#define        READ_16                 0x52
#define        READ_32                 0x44
#define        WRITE_16                0x57
#define        WRITE_32                0x45
#define        READ_N                  0x4E
#define        WRITE_N                 0x4D
#define        SELECT_DEVICE           0x4C
#define        DE_SELECT_DEVICE        0x4B
#define        SET_ADDRESS             0x41
#define        SAVE_FLASH              0x53
#define        READ_EEPROM             0x42
#define        WRITE_EEPROM            0x50
#define        ERASE_EEPROM            0x4F
#define        AC_GAIN                 0x5A
#define        AC_REACTIVE_GAIN        0x7A
#define        AC_FREQUENCY            0x76

/**
* Definicao do baud rate do ESP8266
*/
#define      ESP8266_BAUD        9600

/**
* Definicao dos pinos de protecao e carga
*/
#define        PROTECTION_ALERT         2
#define        LOAD                     7
#define        OK_PIN                   3
#define        FAIL_PIN                 4

/**
* Criacao de porta serial para o ESP8266
*/
SoftwareSerial EspSerial(8, 9);

/**
* Variaveis que armazerao os dados recebidos do MCP39F501 e os de envio ao Blynk
*/
unsigned long receivedData[29];
double blynkData[8];

/**
* Constantes que armazeram os possiveis erros mapeados do projeto
*/
unsigned int errorFlag = 0;
const unsigned int overCurrent = 32768;                // 1000 0000 0000 0000
const unsigned int voltageSag = 128;                   // 0000 0000 1000 0000
const unsigned int voltageSurge = 64;                  // 0000 0000 0100 0000
const unsigned int overCurrentVoltageSag = 32896;      // 1000 0000 1000 0000
const unsigned int overCurrentVoltageSurge = 32832;    // 1000 0000 0100 0000

/**
* Variaveis utilizadas como flag para evitar o envio repetido de notificacoes no app
*/
bool overCurrentNotified = false;
bool voltageSagNotified = false;
bool voltageSurgeNotified = false;
bool overCurrentVoltageSagNotified = false;
bool overCurrentVoltageSurgeNotified = false;

/**
* Variaveis que armazenarao os dados coletados pelo MCP39F501 apos devido processamento
*/
double Vrms = 0;
double Irms = 0;
double ActivePower = 0;
double ReactivePower = 0;
double ApparentPower = 0;
double PowerFactor = 0;
double Frequency = 0;
double ThermistorTemperature = 0;

/**
* Token de autenticacao do app e credenciais de conexao Wi-Fi
*/
char auth[] = "blink-app-token";
char ssid[] = "wifi-name";
char pass[] = "wifi-password";

ESP8266 wifi(&EspSerial);

#pragma region SKETCH_FUNCTIONS

/**
* Funcao executada automaticamente pelo micro assim que o mesmo e inicializado
*/
void setup()
{
    /**
    * Baud rate com o MCP39F501 (recomendacao do datasheet do MCP39F501)
    */
    Serial.begin(4800, SERIAL_8N1);

    /**
    * Definindo modo dos pinos de falha
    */
    pinMode(FAIL_PIN, OUTPUT);
    pinMode(OK_PIN, OUTPUT);
    pinMode(LOAD, OUTPUT);
    pinMode(PROTECTION_ALERT, INPUT);


    /**
    * Estabelecendo estado inicial dos pinos de alerta de falha
    */
    digitalWrite(OK_PIN, HIGH);
    digitalWrite(FAIL_PIN, LOW);

    /**
    * Baud rate com o ESP8266
    */
    EspSerial.begin(ESP8266_BAUD);
    delay(5);


    /**
    * Abertura de conexao entre o hardware e o app Blynk
    */
    Blynk.begin(auth, wifi, ssid, pass);

    clearMCPEvents();
}

/**
* Funcao executada automaticamente pelo micro repetidamente durante seu funcionamento
*/
void loop()
{
    checkForProtectionAlerts();
    Blynk.run();                    // Funcao responsavel pela manutencao da conexao com o app
    sendDataToMSP();
    proccessMCPData();
    sendDataToBlynk();
}

#pragma endregion SKETCH_FUNCTIONS


#pragma region CUSTOM_FUNCTIONS

/**
* Realiza a escrita de dados nos pinos virtuais mapeados no app
*/
void sendDataToBlynk()
{
    Blynk.virtualWrite(V0, blynkData[0]);    // Corrente RMS
    Blynk.virtualWrite(V1, blynkData[1]);    // Tensao RMS
    Blynk.virtualWrite(V2, blynkData[2]);    // Potencia Ativa
    Blynk.virtualWrite(V3, blynkData[3]);    // Potencia Reativa
    Blynk.virtualWrite(V4, blynkData[4]);    // Potencia Aparente
    Blynk.virtualWrite(V5, blynkData[5]);    // Fator de Potencia
    Blynk.virtualWrite(V6, blynkData[6]);    // Frequencia
    Blynk.virtualWrite(V7, blynkData[7]);    // Temperatura
    Blynk.virtualWrite(V10, errorFlag);       // Erro encontrado
}


/**
* Envia o frame correspondente a leitura dos dados de interesse
*/
void sendDataToMSP()
{
    /**
    * Frame correspondente a leitura dos registradores que armazenam as leituras desejadas
    */
    uint8_t readDataFrame[8] = {
        HEADER_BYTE,   // Byte inicial
        0x08,          // Numero de bytes do frame
        SET_ADDRESS,   // Comando -> Estabelece posicao do ponteiro de escrita/leitura
        0x00, 0x04,     // Dados -> endereco 0x0004
        READ_N,        // Comando -> Leitura de N bytes
        0x1A,          // Dados -> Numero de bytes para leitura
        0x5A           // Checksum -> Resto da soma dos bytes enviados por 256
    };

    /**
    * Os bytes do frame sao enviados um a um ao MCP39F501 precedidos de um byte de confirmacao
    */
    Serial.write(0xA4);
    delay(200);
    for (int count = 0; count < 8; count++)
    {
        Serial.write(readDataFrame[count]);
        delay(100);
    }
}


/**
* Verifica e estabelece o estado dos pinos de seguranca
*/
void checkForProtectionAlerts()
{
    if (digitalRead(PROTECTION_ALERT) == HIGH)
    {
        /**
        * No caso do pino de alerta de protecao estar em estado alto ativamos devidamente os
        * leds de sinalizacao, inativamos a carga e chamamos a funcao para processar os dados de erro
        */
        digitalWrite(LOAD, LOW);
        digitalWrite(OK_PIN, LOW);
        digitalWrite(FAIL_PIN, HIGH);
        Blynk.virtualWrite(V11, 0);      // seta em zero para sincronizar o estado do botao de acionamento no app

        checkMCPErrorEvents();
    }
    else
    {
        digitalWrite(OK_PIN, HIGH);
        digitalWrite(FAIL_PIN, LOW);
    }
}


/**
* Processa e alimenta o array de doubles a ser enviado para o Blynk
*/
void proccessMCPData()
{
    /**
    * Verifica se o numero de bytes armazenados na porta serial e maior que 29
    * Sao um total de 29 bytes enviados pelo MCP39F501 expressando as grandezas de interesse, alem
    * da resposta do byte de confirmacao
    */
    if (Serial.available() > 29)
    {
        Serial.read();
        for(int i = 0; i < 29; i++)
        {
            receivedData[i] = Serial.read();
        }

        if (receivedData[0] == ACK)
        {
            /**
            * As grandezas representadas pelos bytes de resposta do MCP39F501 são de 32 ou 16 bits
            * Para decodificar esses valores é necessario le-los de tras para frente sendo o ultimo
            * byte recebido daquela grandeza o mais significativo. Para isso realiza-se o byte-shift
            * conforme a necessidade
            */
            Irms = (receivedData[5] << 24) + (receivedData[4] << 16) + (receivedData[3] << 8) + (receivedData[2]);
            Vrms = (receivedData[7] << 8) + (receivedData[6]);
            ActivePower = (receivedData[11] << 24) + (receivedData[10] << 16) + (receivedData[9] << 8) + (receivedData[8]);
            ReactivePower = (receivedData[15] << 24) + (receivedData[14] << 16) + (receivedData[13] << 8) + (receivedData[12]);
            ApparentPower = (receivedData[19] << 24) + (receivedData[18] << 16) + (receivedData[17] << 8) + (receivedData[16]);
            PowerFactor = (receivedData[21] << 8) + (receivedData[20]);
            Frequency = (receivedData[23] << 8) + (receivedData[22]);
            ThermistorTemperature = (receivedData[25] << 8) + (receivedData[24]);


            /**
            * Alimentando o array de dados a serem enviados ao blynk devidamente convertidos
            */
            blynkData[0] = ((double)(Irms ))* 0.0001;
            blynkData[1] = ((double)(Vrms)) * 0.1;
            blynkData[2] = ((double)(ActivePower)) * 0.01;
            blynkData[3] = ((double)(ReactivePower)) * 0.01;
            blynkData[4] = ((double)(ApparentPower)) * 0.01;

            /**
            * Calculo do FP
            */
            double temporary = PowerFactor <= 32767
                ? ((double)(PowerFactor)) / 32767
                : (((double)(PowerFactor)) / 32767 - 2) * -1;

            blynkData[5] = temporary < 0.00
                ? 0
                : temporary;

            blynkData[6] = ((double)(Frequency)) * 0.001;
            blynkData[7] = ((double)(ThermistorTemperature)) * 125.0 / 1023.0;
            blynkData[8] = (double)(receivedData[1]);

            /**
            * Os dois ultimos bytes recebidos representam o erro encontrado em 16 bits
            * O mesmo e decodificado e atribuido a variavel utilizada para analise de erros
            */
            errorFlag = (receivedData[27] << 8) + (receivedData[26]);
        }
    }
}


/**
* Envia o frame correspondente a limpeza dos registrados que armazenam erros
*/
void clearMCPEvents()
{
    uint8_t acknowledge;

    uint8_t totalNumberOfBytes = 0x09;
    uint8_t clearEventsAddress[2] = { 0x00, 0x92 };
    uint8_t clearEventBytes[2] = {0x80, 0xC0 };
    uint8_t clearEventsChecksum = 0x18;

    while(acknowledge != ACK)
    {
      uint8_t clearEventFrame[9] = {
          HEADER_BYTE,                                      // Byte inicial
          totalNumberOfBytes,                               // Numero de bytes do frame
          SET_ADDRESS,                                      // Comando -> Posicao do ponteiro de escrita/leitura
          clearEventsAddress[0], clearEventsAddress[1],     // Endereco 0x0092
          WRITE_16,                                         // Comando -> Escrita de 16 bits
          clearEventBytes[0], clearEventBytes[1],           // Endereco 0x80C0
          clearEventsChecksum                               // Checksum -> Resto da soma dos bytes enviados por 256
      };

      Serial.write(0xA4);
      delay(250);
      byte count = 0;
      for (count = 0; count < 9; count++)
      {
          Serial.write(clearEventFrame[count]);
          delay(100);
      }

      delay(350);

      Serial.read();                                       // retira byte de confirmacao da serial
      acknowledge = Serial.read();
    }
}


/**
* Interpreta a flag de erro notificando no app a devida falha, se existente
*/
void checkMCPErrorEvents()
{
    /**
    * Nao havendo falha, redundantemente define as flags de notificacao como FALSE
    * Em caso de falha, a flag de erro e comparada com os erros mapeados e uma notificacao
    * e enviada ao app e a flag de notificacao e definida como TRUE para evitar o envio
    * de notificacoes duplicadas
    */
    if (errorFlag == 0)
    {
        overCurrentNotified = false;
        voltageSagNotified = false;
        voltageSurgeNotified = false;
        overCurrentVoltageSagNotified = false;
        overCurrentVoltageSurgeNotified = false;

        return;
    } else if (errorFlag == overCurrent) {
        if (overCurrentNotified == false) {
            Blynk.notify("Sobre corrente!");
        }
        overCurrentNotified = true;
    } else if (errorFlag == voltageSag) {
        if (voltageSagNotified == false) {
            Blynk.notify("Sub tensão!");
        }
        voltageSagNotified = true;
    } else if (errorFlag == voltageSurge) {
        if (voltageSurgeNotified == false) {
            Blynk.notify("Sobre tensão!");
        }
        voltageSurgeNotified = true;
    } else if (errorFlag == overCurrentVoltageSag) {
        if (overCurrentVoltageSagNotified == false) {
            Blynk.notify("Sobre corrente e sub tensão!");
        }
        overCurrentVoltageSagNotified = true;
    }

    /**
    * Chamada a funcao para limpeza do registrador de erro para que o mesmo possa ser verificado
    * e consequemente atualizado pelo MCP39F501
    */
    clearMCPEvents();

    return;
}
#pragma endregion CUSTOM_FUNCTIONS


#pragma region BLYNK_HOOK_FUNCTIONS

/**
* Funcao executada sempre que a conexao e estabelecida com o app
*/
BLYNK_CONNECTED() {
    Blynk.syncVirtual(V8); // Sincroniza o micro com o atual valor no app do botao acionador
    Blynk.virtualWrite(V11, digitalRead(LOAD)); // Sincroniza o app com o atual valor da carga
}


/**
* Funcao executada sempre que e solicitada alteracao pelo app das configuracoes da rede (120V/240V)
* Opcoes de configuracao mapeadas no app e hardware no pino virtual V8
*/
BLYNK_WRITE(V8) {
    int configTension = param.asInt();
    int _120ConfigOption = 1;
    int _240ConfigOption = 2;

    /**
    * Frame para habilitar configuracoes
    */
    uint8_t enableConfig[9] {
        0xA5,
        0x09,
        0x41,
        0x00,
        0x8C,
        0x57,
        0x80,
        0xC0,
        0x12
    };

    /**
    * Frame de definicao dos limites para configuracao de 120VA
    */
    uint8_t _120ConfigFrame[52] {
        0xA5,
        0x34,
        0x41,
        0x00,
        0x5E,
        0x4D,
        0x2C,
        0xF0,
        0x49,
        0x02,
        0x00,
        0x00,
        0x00,
        0xC0,
        0xB6,
        0x06,
        0x00,
        0x00,
        0x00,
        0x08,
        0xCF,
        0x80,
        0xBB,
        0xA4,
        0x01,
        0x96,
        0x00,
        0xE8,
        0x03,
        0x32,
        0x05,
        0x04,
        0x00,
        0x04,
        0x00,
        0x04,
        0x00,
        0x04,
        0x00,
        0x04,
        0x00,
        0x04,
        0x00,
        0x04,
        0x00,
        0x04,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x37
    };

    /**
    * Frame de definicao dos limites para configuracao de 240VA
    */
    uint8_t _240ConfigFrame[52] {
        0xA5,
        0x34,
        0x41,
        0x00,
        0x5E,
        0x4D,
        0x2C,
        0xF0,
        0x49,
        0x02,
        0x00,
        0x00,
        0x00,
        0xC0,
        0xB6,
        0x06,
        0x00,
        0x00,
        0x00,
        0x08,
        0xCF,
        0x80,
        0xBB,
        0xA4,
        0x01,
        0x96,
        0x00,
        0xD0,
        0x07,
        0xD8,
        0x09,
        0x04,
        0x00,
        0x04,
        0x00,
        0x04,
        0x00,
        0x04,
        0x00,
        0x04,
        0x00,
        0x04,
        0x00,
        0x04,
        0x00,
        0x04,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0xCD
    };

    /**
    * E comparada a selecao da configuracao e enviada de acordo para o MCP39F501 e na sequencia
    * enviado o frame para habilitacao da nova configuracao de limites
    */
    if (configTension == _120ConfigOption) {
        uint8_t acknowledge;
        while(acknowledge != ACK) {
            Serial.write(0xA4);
            delay(200);
        for (int count = 0; count < 52; count++)
        {
            Serial.write(_120ConfigFrame[count]);
            delay(50);
        }

        Serial.read();
        acknowledge = Serial.read();
        }

        acknowledge = 0x00;
        while(acknowledge != ACK)
        {
            Serial.write(0xA4);
            delay(200);

        for (int count = 0; count < 9; count++)
        {
            Serial.write(enableConfig[count]);
            delay(50);
        }

        Serial.read();
        acknowledge = Serial.read();
        }
    } else if(configTension == _240ConfigOption) {
        uint8_t acknowledge;
        while(acknowledge != ACK) {
            Serial.write(0xA4);
            delay(200);
            for (int count = 0; count < 52; count++)
            {
                Serial.write(_240ConfigFrame[count]);
                delay(50);
            }

            Serial.read();
            acknowledge = Serial.read();
        }

        acknowledge = 0x00;
        while(acknowledge != ACK)
        {
            Serial.write(0xA4);
            delay(200);
            for (int count = 0; count < 9; count++)
            {
                Serial.write(enableConfig[count]);
                delay(50);
            }

            Serial.read();
            acknowledge = Serial.read();
        }
    }
}


/**
* Funcao executada sempre que e solicitado o acionamento da carga pelo app
* Acionamento e desacionamento mapeados no app e hardware no pino V11
*/
BLYNK_WRITE(V11) {
    /**
    * Acionamento = 1
    * Desacionamento = 0
    */
   int loadButton = param.asInt();

    /**
    * Antes que seja realizado o acionamento da carga e verificado o pino de alerta de protecao
    * Caso o mesmo esteja em HIGH o app e notificado que ha falhas e a carga nao e acionada
    */
    if (loadButton == 1) {
        if (digitalRead(PROTECTION_ALERT) == HIGH) {
            Blynk.notify("Falhas identificadas!");
            delay(50);
            Blynk.virtualWrite(V11, 0);
        }

        if(digitalRead(PROTECTION_ALERT) == LOW) {
            digitalWrite(LOAD, HIGH);
        }
    }

    if (loadButton == 0) {
        digitalWrite(LOAD, LOW);
    }
}

#pragma endregion BLYNK_HOOK_FUNCTIONS
