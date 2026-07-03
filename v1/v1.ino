// ============================================================================
// Communication full-duplex NRZ entre 2 ESP32 via bit-banging sur PIN 4/5
// ============================================================================

// Les includes Arduino et FreeRTOS sont déjà fournis par Arduino IDE
#include "esp_task_wdt.h"
#include "rom/ets_sys.h" // esp_rom_delay_us

// CONFIGURATION GLOBALE
#define ESP_ID 1 // 1 ou 2 -- à changer avant chaque upload

#define TX_PIN 4
#define RX_PIN 5
#define BIT_PERIOD_US 1000 // 1 bit = 1ms

#define PREAMBLE_BYTE 0x55 // 01010101 -- synchronisation
#define START_BYTE 0x7E    // 01111110 -- délimiteur début
#define END_BYTE 0x7E      // 01111110 -- délimiteur fin
#define MAX_PAYLOAD 80

#define FRAME_TYPE_DEBUT 0x01
#define FRAME_TYPE_DATA 0x02
#define FRAME_TYPE_FIN 0x03
#define FRAME_TYPE_NACK 0x04

#define ENABLE_ERROR_INJECTION 1

// Structure de trame
typedef struct
{
  uint8_t type;
  uint8_t seqNum;
  uint8_t payloadLen;
  uint8_t param;
  uint8_t payload[MAX_PAYLOAD];
  uint16_t crc;
  bool crcValid; // rempli par receiveFrame(), pas transmis sur le fil
} Frame;

// Etat de sequencement/NACK (un seul producteur/consommateur chacun -> volatile suffit)
uint8_t g_rxExpectedSeq = 1;
volatile bool g_localGapDetected = false;
volatile uint8_t g_localGapTargetSeq = 0;
volatile bool g_remoteNackReceived = false;
volatile uint8_t g_remoteNackTargetSeq = 0;
// Empeche de renvoyer un NACK a chaque nouvelle trame hors-sequence tant que
// l'episode d'erreur en cours (meme trame attendue) n'est pas resolu.
bool g_nackAlreadySentForEpisode = false;

// ============================================================================
// LOGS DIFFÉRÉS
// ============================================================================
// Le RX verrouille son horloge d'échantillonnage UNE SEULE FOIS sur le
// préambule puis l'extrapole pour TOUTE la trame. Le moindre appel bloquant
// (Serial.print) pendant la réception/émission d'une trame peut prendre
// plusieurs ms et décale irrémédiablement le reste de la trame. Donc aucun
// Serial.print ne doit avoir lieu pendant le bit-bang: on accumule le texte
// dans un buffer global et on l'affiche seulement une fois la trame terminée,
// juste avant l'attente entre deux envois/réceptions.

#define LOG_BUF_SIZE 3072
char g_txLogBuf[LOG_BUF_SIZE];
size_t g_txLogLen = 0;
char g_rxLogBuf[LOG_BUF_SIZE];
size_t g_rxLogLen = 0;

void logAppend(char *buf, size_t *len, const char *fmt, ...)
{
  if (*len >= LOG_BUF_SIZE - 1)
    return;
  va_list args;
  va_start(args, fmt);
  int written = vsnprintf(buf + *len, LOG_BUF_SIZE - *len, fmt, args);
  va_end(args);
  if (written > 0)
    *len += written;
}

void logFlush(char *buf, size_t *len)
{
  if (*len > 0)
  {
    Serial.print(buf);
    Serial.println();
    Serial.println();
  }
  buf[0] = '\0';
  *len = 0;
}

// ============================================================================
// ENCODAGE NRZ BAS NIVEAU
// ============================================================================
// Convention NRZ simple : 0 = LOW (maintenu 1ms), 1 = HIGH (maintenu 1ms)

// Horloge TX absolue: chaque sendByte() repart d'un temps de reference figé
// une seule fois par trame (voir sendFrame), pour que digitalWrite()/l'overhead
// de boucle ne s'accumule pas bit apres bit sur les trames longues (payload
// jusqu'a 80 octets = ~700 bits). Un delai relatif (delayMicroseconds apres
// digitalWrite) derive de quelques us par bit, ce qui suffit a desynchroniser
// le recepteur (horloge absolue, lui aussi figee une seule fois) sur une
// trame de plusieurs centaines de bits.
uint32_t g_txClockStart = 0;
int g_txBitIndex = 0;

void sendBit(bool bitValue)
{
  digitalWrite(TX_PIN, bitValue ? HIGH : LOW);
  uint32_t targetTime = g_txClockStart + ((g_txBitIndex + 1) * (uint32_t)BIT_PERIOD_US);
  while (micros() < targetTime)
  {
    // Spin jusqu'a la fin exacte du bit (horloge absolue, pas de derive cumulee)
  }
  g_txBitIndex++;
}

void sendByte(uint8_t b)
{
  for (int i = 7; i >= 0; i--)
  {
    sendBit((b >> i) & 1);
  }
}

// Variable globale pour synchroniser l'horloge d'échantillonnage des bits
uint32_t g_rxClockStart = 0;
int g_rxBitIndex = 0;

// Lit un bit en utilisant l'horloge globale d'échantillonnage
bool receiveBitWithGlobalClock(uint32_t bitPeriodUs, int bitIndex, bool *outBit)
{
  if (!outBit)
    return false;

  // Calculer le moment d'échantillonnage pour ce bit (au centre du bit)
  uint32_t sampleTime = g_rxClockStart + (bitIndex * bitPeriodUs) + (bitPeriodUs / 2);
  uint32_t timeoutTime = g_rxClockStart + ((bitIndex + 1) * bitPeriodUs);

  // Attendre le moment d'échantillonnage
  while (micros() < sampleTime)
  {
    // Spin jusqu'au moment d'échantillonner
  }

  *outBit = digitalRead(RX_PIN) == HIGH;

  // Attendre la fin du bit
  while (micros() < timeoutTime)
  {
    // Attendre le prochain bit
  }

  return true;
}

bool receiveByte(uint8_t *outByte, uint32_t bitPeriodUs)
{
  if (!outByte)
    return false;

  uint8_t byte = 0;
  for (int i = 0; i < 8; i++)
  {
    bool bit;
    if (!receiveBitWithGlobalClock(bitPeriodUs, g_rxBitIndex, &bit))
    {
      return false;
    }
    byte = (byte << 1) | (bit ? 1 : 0);
    g_rxBitIndex++;
  }
  *outByte = byte;
  return true;
}

// ============================================================================
// CALIBRATION DU PRÉAMBULE
// ============================================================================
// Le préambule 0x55 (01010101) alterne à chaque bit en NRZ => transition tous
// les ~1ms. Ligne idle = LOW. 0x55 commence par un bit 0 (LOW), donc il n'y a
// PAS de front au tout début du préambule : le premier front utile est la
// transition LOW->HIGH entre le bit0 (0) et le bit1 (1). On s'accroche à CE
// front précis, on calcule le centre du bit1 à partir de là, puis on relit
// les 8 bits pour valider que c'est bien 0x55. Si la validation échoue (glitch,
// front capté au milieu d'un préambule déjà en cours, bruit), on réessaie
// entièrement au lieu d'abandonner.
bool detectAndCalibrateOnPreamble(uint32_t bitPeriodUs)
{
  const int maxAttempts = 20;
  for (int attempt = 0; attempt < maxAttempts; attempt++)
  {
    // Étape A: s'assurer que la ligne est idle (LOW) avant d'armer la détection,
    // pour éviter de s'accrocher à un front au milieu d'une trame déjà en cours.
    while (digitalRead(RX_PIN) == HIGH)
    {
      esp_rom_delay_us(50);
    }

    // Étape B: attendre le front montant LOW->HIGH (transition bit0->bit1 du 0x55)
    uint32_t armStart = micros();
    while (digitalRead(RX_PIN) == LOW)
    {
      if ((micros() - armStart) > 2000000UL)
      {
        break; // rien reçu depuis 2s: on relance une nouvelle tentative propre
      }
      esp_rom_delay_us(50);
    }
    if (digitalRead(RX_PIN) == LOW)
    {
      continue; // timeout d'armement, on recommence l'étape A/B
    }

    uint32_t frontTime = micros();
    uint32_t bit1Center = frontTime + (bitPeriodUs / 2);

    // On relit les bits 1 à 7 (le bit0 est implicite = 0) pour valider le motif.
    uint8_t preambleByte = 0;
    uint32_t sampleTime = bit1Center;

    for (int i = 1; i < 8; i++)
    {
      while (micros() < sampleTime)
      {
        // Spin (précision microseconde requise pour l'échantillonnage)
      }
      bool bit = (digitalRead(RX_PIN) == HIGH);
      preambleByte = (preambleByte << 1) | (bit ? 1 : 0);
      sampleTime += bitPeriodUs;
    }

    if (preambleByte != PREAMBLE_BYTE)
    {
      continue;
    }

    // Initialiser l'horloge globale. Le bit8 (START_BYTE bit0) est le prochain
    // bit après le bit7 du préambule. sampleTime pointe déjà au centre du bit8.
    // g_rxClockStart doit satisfaire: sampleTime(bitIndex) = g_rxClockStart + bitIndex*bitPeriodUs + bitPeriodUs/2
    g_rxClockStart = sampleTime - (8 * bitPeriodUs) - (bitPeriodUs / 2);
    g_rxBitIndex = 8; // Prochain bit à lire = bit 8 (START_BYTE)

    return true;
  }

  return false;
}

// ============================================================================
// GESTION TRAME COMPLÈTE
// ============================================================================

void buildFrame(Frame *f, uint8_t type, uint8_t seq, uint8_t param, const uint8_t *payload, uint8_t len)
{
  if (!f)
    return;

  f->type = type;
  f->seqNum = seq;
  f->payloadLen = (len > MAX_PAYLOAD) ? MAX_PAYLOAD : len;
  f->param = param;
  f->crcValid = true;

  if (payload && len > 0)
  {
    memcpy(f->payload, payload, f->payloadLen);
  }

  uint8_t crcBuf[4 + MAX_PAYLOAD];
  crcBuf[0] = f->type;
  crcBuf[1] = f->seqNum;
  crcBuf[2] = f->payloadLen;
  crcBuf[3] = f->param;
  memcpy(&crcBuf[4], f->payload, f->payloadLen);
  f->crc = computeCRC16(crcBuf, 4 + f->payloadLen);
}

void sendFrame(const Frame *f)
{
  if (!f)
    return;

  g_txClockStart = micros();
  g_txBitIndex = 0;

  sendByte(PREAMBLE_BYTE);
  sendByte(START_BYTE);

  sendByte(f->type);
  sendByte(f->seqNum);
  sendByte(f->payloadLen);
  sendByte(f->param);

  for (int i = 0; i < f->payloadLen; i++)
  {
    sendByte(f->payload[i]);
  }

  sendByte(f->crc & 0xFF);
  sendByte((f->crc >> 8) & 0xFF);

  sendByte(END_BYTE);
}

bool receiveFrame(Frame *f)
{
  if (!f)
    return false;

  g_rxClockStart = 0;
  g_rxBitIndex = 0;

  uint32_t bitPeriodUs = BIT_PERIOD_US;
  if (!detectAndCalibrateOnPreamble(bitPeriodUs))
  {
    logAppend(g_rxLogBuf, &g_rxLogLen, "[RX] ERREUR: Timeout detection preambule\n");
    return false;
  }

  uint8_t startByte;
  if (!receiveByte(&startByte, bitPeriodUs) || startByte != START_BYTE)
  {
    logAppend(g_rxLogBuf, &g_rxLogLen, "[RX] ERREUR: Start byte invalide (0x%02X)\n", startByte);
    return false;
  }

  if (!receiveByte(&f->type, bitPeriodUs) ||
      !receiveByte(&f->seqNum, bitPeriodUs) ||
      !receiveByte(&f->payloadLen, bitPeriodUs) ||
      !receiveByte(&f->param, bitPeriodUs))
  {
    logAppend(g_rxLogBuf, &g_rxLogLen, "[RX] ERREUR: Timeout lecture entete\n");
    return false;
  }

  if (f->payloadLen > 0)
  {
    if (f->payloadLen > MAX_PAYLOAD)
    {
      logAppend(g_rxLogBuf, &g_rxLogLen, "[RX] ERREUR: Payload trop long\n");
      return false;
    }
    for (int i = 0; i < f->payloadLen; i++)
    {
      if (!receiveByte(&f->payload[i], bitPeriodUs))
      {
        logAppend(g_rxLogBuf, &g_rxLogLen, "[RX] ERREUR: Timeout lecture payload\n");
        return false;
      }
    }
  }

  uint8_t crcLow, crcHigh;
  if (!receiveByte(&crcLow, bitPeriodUs) || !receiveByte(&crcHigh, bitPeriodUs))
  {
    logAppend(g_rxLogBuf, &g_rxLogLen, "[RX] ERREUR: Timeout lecture CRC\n");
    return false;
  }
  f->crc = ((uint16_t)crcHigh << 8) | crcLow;

  uint8_t endByte;
  if (!receiveByte(&endByte, bitPeriodUs) || endByte != END_BYTE)
  {
    logAppend(g_rxLogBuf, &g_rxLogLen, "[RX] ERREUR: End byte invalide (0x%02X)\n", endByte);
    return false;
  }

  // Recalcul CRC apres capture complete de la trame (aucun bit-banging en cours ici)
  uint8_t crcBuf[4 + MAX_PAYLOAD];
  crcBuf[0] = f->type;
  crcBuf[1] = f->seqNum;
  crcBuf[2] = f->payloadLen;
  crcBuf[3] = f->param;
  memcpy(&crcBuf[4], f->payload, f->payloadLen);
  f->crcValid = (computeCRC16(crcBuf, 4 + f->payloadLen) == f->crc);

  return true;
}

// CRC-16-CCITT (poly 0x1021, init 0xFFFF)
uint16_t computeCRC16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++)
  {
    crc ^= ((uint16_t)data[i]) << 8;
    for (int b = 0; b < 8; b++)
    {
      crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
    }
  }
  return crc;
}

// ============================================================================
// ACQUISITION DE DONNEES SIMULEE
// ============================================================================

#define TOTAL_PAQUETS 5

const char *acquireDonnees(int index)
{
  static const char *donnees_txt[TOTAL_PAQUETS] = {
      "Ligne 1 - 2026-07-01 10:00:01 | Temp: 22.4 C | Humidite: 45.2 % | Node: 01",
      "Ligne 2 - 2026-07-01 10:05:01 | Temp: 22.5 C | Humidite: 45.1 % | Node: 01",
      "Ligne 3 - 2026-07-01 10:10:01 | Temp: 22.8 C | Humidite: 44.9 % | Node: 01",
      "Ligne 4 - 2026-07-01 10:15:01 | Temp: 23.1 C | Humidite: 44.8 % | Node: 01",
      "Ligne 5 - 2026-07-01 10:20:01 | Temp: 23.0 C | Humidite: 45.0 % | Node: 01",
  };
  if (index < 0 || index >= TOTAL_PAQUETS)
    return "";
  return donnees_txt[index];
}

int loadDonneesIntoPackets(uint8_t packets[][MAX_PAYLOAD], uint8_t *packetLens, int maxPackets)
{
  int count = (TOTAL_PAQUETS > maxPackets) ? maxPackets : TOTAL_PAQUETS;
  for (int i = 0; i < count; i++)
  {
    const char *line = acquireDonnees(i);
    int len = strlen(line);
    if (len > MAX_PAYLOAD)
      len = MAX_PAYLOAD;
    memcpy(packets[i], line, len);
    packetLens[i] = len;
  }
  return count;
}

bool g_errorAlreadyInjected = false; // n'injecter qu'une seule fois par session (pas a chaque retransmission)
uint8_t g_errorFrameSeq = 0;   // numéro de trame (1..5) qui sera corrompue

void maybeInjectBitError(Frame *f)
{
    if (!f || f->payloadLen == 0)
        return;

    if (g_errorAlreadyInjected)
        return;

    if (f->seqNum != g_errorFrameSeq)
        return;

    // Choisir un octet aléatoire
    uint8_t byteIdx = esp_random() % f->payloadLen;

    // Choisir un bit aléatoire
    uint8_t bitIdx = esp_random() % 8;

    f->payload[byteIdx] ^= (1 << bitIdx);

    g_errorAlreadyInjected = true;

    logAppend(g_txLogBuf, &g_txLogLen,
              "[TX] *** ERREUR INJECTÉE *** trame=%d octet=%d bit=%d\n",
              f->seqNum,
              byteIdx,
              bitIdx);
}

// ============================================================================
// LOGIQUE APPLICATIVE
// ============================================================================

void sendAcquisitionMessage()
{
  uint8_t packets[10][MAX_PAYLOAD];
  uint8_t packetLens[10];
  int totalPackets = loadDonneesIntoPackets(packets, packetLens, 10);

  if (totalPackets == 0)
    return;

  logAppend(g_txLogBuf, &g_txLogLen, "[TX] J'envoie %d paquets de donnees\n", totalPackets);
  g_errorAlreadyInjected = false;

  // Choisit une des trames de données (1 à totalPackets)
  if (ENABLE_ERROR_INJECTION)
      g_errorFrameSeq = (esp_random() % totalPackets) + 1;
  else
      g_errorFrameSeq = 0;

  logAppend(g_txLogBuf, &g_txLogLen,
            "[TX] Injection d'erreur prévue sur la trame %d\n",
            g_errorFrameSeq);
  g_remoteNackReceived = false; // reset: ignorer tout residu d'une session precedente
  g_localGapDetected = false;

  Frame fBegin;
  buildFrame(&fBegin, FRAME_TYPE_DEBUT, 0, totalPackets, 0, 0);
  sendFrame(&fBegin);

  int i = 0;
  while (i < totalPackets)
  {
    if (g_remoteNackReceived)
    {
      uint8_t target = g_remoteNackTargetSeq;
      g_remoteNackReceived = false;
      logAppend(g_txLogBuf, &g_txLogLen,
                "[TX] NACK recu -> retransmission a partir de la trame %d\n", target);
      i = target - 1;
      continue;
    }

    Frame fData;
    buildFrame(&fData, FRAME_TYPE_DATA, i + 1, 0, packets[i], packetLens[i]);
    logAppend(g_txLogBuf, &g_txLogLen, "[TX] Envoi trame %d: %s\n", i + 1, acquireDonnees(i));
    maybeInjectBitError(&fData);
    sendFrame(&fData);

    if (g_localGapDetected)
    {
      uint8_t target = g_localGapTargetSeq;
      g_localGapDetected = false;
      Frame fNack;
      buildFrame(&fNack, FRAME_TYPE_NACK, 0, target, 0, 0);
      sendFrame(&fNack);
      logAppend(g_txLogBuf, &g_txLogLen,
                "[TX] Erreur detectee sur la reception locale -> envoi d'un NACK pour la trame %d\n", target);
    }

    i++;
  }

  Frame fEnd;
  buildFrame(&fEnd, FRAME_TYPE_FIN, totalPackets, 0, 0, 0);
  sendFrame(&fEnd);
  logAppend(g_txLogBuf, &g_txLogLen, "[TX] Session terminee (FIN envoye)\n");
}

// ============================================================================
// TÂCHES FreeRTOS
// ============================================================================

void taskRX(void *pvParameters)
{
  vTaskDelay(pdMS_TO_TICKS(1500));

  char assembledMsg[512] = {0};
  int assembledLen = 0;

  while (1)
  {
    Frame rxFrame;
    memset(&rxFrame, 0, sizeof(Frame));

    if (!receiveFrame(&rxFrame))
    {
      logFlush(g_rxLogBuf, &g_rxLogLen);
      continue;
    }

    if (rxFrame.type == FRAME_TYPE_DEBUT)
    {
      assembledLen = 0;
      g_rxExpectedSeq = 1;
      g_localGapDetected = false;
      g_nackAlreadySentForEpisode = false;
      logAppend(g_rxLogBuf, &g_rxLogLen, "[RX] Debut: %d paquets attendus\n", rxFrame.param);
    }
    else if (rxFrame.type == FRAME_TYPE_DATA)
    {
      if (!rxFrame.crcValid)
      {
        logAppend(g_rxLogBuf, &g_rxLogLen,
                  "[RX] J'ai recu trame %d: refusee, erreur CRC detectee\n", rxFrame.seqNum);
        if (!g_nackAlreadySentForEpisode)
        {
          g_localGapTargetSeq = g_rxExpectedSeq;
          g_localGapDetected = true;
          g_nackAlreadySentForEpisode = true;
        }
      }
      else if (rxFrame.seqNum != g_rxExpectedSeq)
      {
        logAppend(g_rxLogBuf, &g_rxLogLen,
                  "[RX] J'ai recu trame %d: refusee, en attente de la trame %d\n",
                  rxFrame.seqNum, g_rxExpectedSeq);
        if (!g_nackAlreadySentForEpisode)
        {
          g_localGapTargetSeq = g_rxExpectedSeq;
          g_localGapDetected = true;
          g_nackAlreadySentForEpisode = true;
        }
      }
      else if (assembledLen + rxFrame.payloadLen + 1 <= (int)sizeof(assembledMsg) - 1)
      {
        logAppend(g_rxLogBuf, &g_rxLogLen, "[RX] J'ai recu trame %d: %.*s\n",
                  rxFrame.seqNum, rxFrame.payloadLen, rxFrame.payload);
        if (assembledLen > 0)
        {
          assembledMsg[assembledLen] = '\n';
          assembledLen++;
        }
        memcpy(&assembledMsg[assembledLen], rxFrame.payload, rxFrame.payloadLen);
        assembledLen += rxFrame.payloadLen;
        g_rxExpectedSeq++;
        g_nackAlreadySentForEpisode = false; // trame attendue recue: episode resolu
      }
    }
    else if (rxFrame.type == FRAME_TYPE_FIN)
    {
      assembledMsg[assembledLen] = '\0';
      logAppend(g_rxLogBuf, &g_rxLogLen, "[RX] Session terminee, message complet recu:\n%s\n", assembledMsg);
      memset(assembledMsg, 0, sizeof(assembledMsg));
      logFlush(g_rxLogBuf, &g_rxLogLen); // fin de session: on affiche le resume complet
    }
    else if (rxFrame.type == FRAME_TYPE_NACK)
    {
      if (!g_remoteNackReceived)
      {
        logAppend(g_rxLogBuf, &g_rxLogLen, "[RX] NACK recu pour paquet %d\n", rxFrame.param);
        g_remoteNackTargetSeq = rxFrame.param;
        g_remoteNackReceived = true;
      }
    }
    else
    {
      logAppend(g_rxLogBuf, &g_rxLogLen,
                "[RX] ERREUR: type de trame inconnu 0x%02X seq=%d len=%d param=%d\n",
                rxFrame.type, rxFrame.seqNum, rxFrame.payloadLen, rxFrame.param);
      logFlush(g_rxLogBuf, &g_rxLogLen);
    }
  }
}

void taskTX(void *pvParameters)
{
  vTaskDelay(pdMS_TO_TICKS(3000));

  while (1)
  {
    sendAcquisitionMessage();
    logFlush(g_txLogBuf, &g_txLogLen);
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

// ============================================================================
// SETUP / LOOP
// ============================================================================

void setup()
{
  Serial.begin(115200);
  delay(500);
  randomSeed(esp_random());

  // Le RX doit boucler en busy-wait (précision microseconde) sans jamais
  // rendre la main pendant toute la durée d'une trame (~700ms pour un message
  // complet), ce qui empêche la tâche IDLE du coeur 0 de tourner et déclenche
  // le Task Watchdog (reboot en boucle). On désactive donc entièrement le
  // TWDT: c'est un choix assumé pour ce protocole bit-bang temps réel strict,
  // pas un correctif de contournement d'un autre bug.
  esp_task_wdt_deinit();

  pinMode(TX_PIN, OUTPUT);
  pinMode(RX_PIN, INPUT);
  digitalWrite(TX_PIN, LOW);

  xTaskCreatePinnedToCore(taskRX, "TaskRX", 4096, 0, 1, 0, 0);
  xTaskCreatePinnedToCore(taskTX, "TaskTX", 4096, 0, 1, 0, 1);
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}
