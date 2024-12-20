#include "LoRaWan_APP.h"
#include "Arduino.h"

// Configurações LoRa
#define RF_FREQUENCY                                927000000 // Hz
#define TX_OUTPUT_POWER                             10        // dBm
#define LORA_BANDWIDTH                              0         // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reservado]
#define LORA_SPREADING_FACTOR                       11        // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Mesmo para Tx e Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Símbolos
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define RX_TIMEOUT_VALUE                            1000      // Tempo limite de RX
#define BUFFER_SIZE                                 12        // 4 bytes fluxo, 4 bytes bateria, 4 bytes solar

// Configurações GPIO
#define SOLENOID_VALVE_PIN                          7
#define FLOW_SENSOR_PIN                             6
#define BATTERY_PIN                                 1
#define SOLAR_PIN                                   5

// Variáveis globais
uint8_t txpacket[BUFFER_SIZE];  // Buffer para transmissão de dados
static RadioEvents_t RadioEvents;
volatile uint32_t pulseCount = 0;
uint32_t lastPulseCount = 0;
unsigned long lastTransmitTime = 0;
const float PULSE_TO_LITERS = 2.25 / 1000; // Fator de conversão de pulsos para litros
bool valveState = false;
bool loraIdle = true;

// Protótipos de funções
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnTxDone(void);
void OnTxTimeout(void);
void flowSensorISR();
void processCommand(uint8_t command);
float readBatteryVoltage();
float readSolarVoltage();

void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

    // Configura GPIO
    pinMode(SOLENOID_VALVE_PIN, OUTPUT);
    digitalWrite(SOLENOID_VALVE_PIN, LOW);

    pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowSensorISR, RISING);

    // Configura LoRa
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH,
                      LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

    Serial.println("Nó atuador iniciado e pronto para receber comandos.");
}

void loop() {
    if (loraIdle) {
        loraIdle = false;
        Serial.println("Esperando comandos via LoRa...");
        Radio.Rx(0);
    }
    Radio.IrqProcess();

    // Envia dados periódicos
    if (millis() - lastTransmitTime >= 10000) {  // A cada 10 segundos
        lastTransmitTime = millis();

        uint32_t currentPulseCount = pulseCount;
        uint32_t pulseDifference = currentPulseCount - lastPulseCount;
        lastPulseCount = currentPulseCount;

        float flowRate = pulseDifference * PULSE_TO_LITERS * 6; // Converte para litros/minuto

        float batteryVoltage = readBatteryVoltage();
        float solarVoltage = readSolarVoltage();

        // Prepara o pacote de transmissão
        memcpy(&txpacket[0], &flowRate, 4);
        memcpy(&txpacket[4], &batteryVoltage, 4);
        memcpy(&txpacket[8], &solarVoltage, 4);

        Radio.Send(txpacket, BUFFER_SIZE);
        Serial.printf("Dados enviados: Vazão=%.2f L/min, Bateria=%.2f V, Solar=%.2f V\n", flowRate, batteryVoltage, solarVoltage);
    }
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    Radio.Sleep();
    loraIdle = true;

    if (size == 1) {  // Espera um comando de 1 byte
        processCommand(payload[0]);
    } else {
        Serial.println("Tamanho do comando inválido.");
    }
}

void OnTxDone(void) {
    Serial.println("Transmissão concluída.");
    loraIdle = true;
}

void OnTxTimeout(void) {
    Serial.println("Erro: Tempo limite de transmissão.");
    loraIdle = true;
}

void flowSensorISR() {
    pulseCount++;
}

void processCommand(uint8_t command) {
    if (command == 1) {
        digitalWrite(SOLENOID_VALVE_PIN, HIGH);
        valveState = true;
        Serial.println("Válvula ligada.");
    } else if (command == 0) {
        digitalWrite(SOLENOID_VALVE_PIN, LOW);
        valveState = false;
        Serial.println("Válvula desligada.");
    } else {
        Serial.println("Comando inválido.");
    }
}

float readBatteryVoltage() {
    int analogValue = analogRead(BATTERY_PIN);
    return analogValue * 0.0041;  // Ajustar o fator de escala conforme necessário
}

float readSolarVoltage() {
    int analogValue = analogRead(SOLAR_PIN);
    return analogValue * 0.0041;  // Ajustar o fator de escala conforme necessário
}
