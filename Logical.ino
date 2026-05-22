#include <Env.h>
#include <Wire.h>
#include <WiFi.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>

#include <UniversalTelegramBot.h>
#include <SensirionI2cSht4x.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <TFT_eSPI.h>

Preferences prefs;

// ── Pines ────────────────────────────────────────────────────
#define PIN_IR_TX 26
#define PIN_BL 27

// ── Intervalos (ms) ──────────────────────────────────────────
#define MS_SENSOR 1000      // lectura SHT40 cada 1 s
#define MS_IR 120000         // reenvío cada 1:30 s
#define MS_FRACASO 600000   // 10 minutos → fracaso
#define MS_WIFI_CHECK 5000  // revisión de Wi-Fi cada 5 s

// ── Diferencia fija entre umbral de alerta y normalización (1°C)
#define DELTA_TEMP 1.0f

// ── Estados del sistema ───────────────────────────────────────
enum Estado { STANDBY,
              INTENTANDO,
              FRACASO };

// ── Objetos ──────────────────────────────────────────────────
SensirionI2cSht4x sht4x;
TFT_eSPI tft;
IRsend irsend(PIN_IR_TX);

// Clientes para Telegram
WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);

// ── Estado global ────────────────────────────────────────────
float temperatura = 0.0f;
float humedad = 0.0f;
bool sensorOK = false;
bool sensorOKPrev      = false;
bool sensorAlgunaVezOK = false;
bool wifiConectado = false;
bool alertasActivadas = false;
int setLimitsStep = 0;  // 0=inactivo, 1=esperando temperatura límite
Estado estadoActual = STANDBY;
unsigned long tUltimoSensor = 0;
unsigned long tUltimoIR = 0;
unsigned long tInicioIntento = 0;
unsigned long tUltimoWiFi = 0;

float t_prev = 0.0f;           // Temperatura registrada en el último envío IR
bool bajandoMensajeEnviado = false;  // Evita repetir aviso "Temperatura bajando"

#define MS_BOT_CHECK 3000  // Revisar Telegram cada 3 segundos
unsigned long tUltimoTelegram = 0;

// Variables de umbrales que ahora no serán #define, sino modificables:
float tempEncender = 25.0f;
float tempApagar = 24.0f;


// ── Paleta de colores (RGB565) ────────────────────────────────
//  Verde (STANDBY)
#define C_FONDO_OK 0x04C0
#define C_ACENTO_OK 0x07E0
#define C_SEP_OK 0x0320

//  Amarillo (INTENTANDO)
#define C_FONDO_WARN 0xFE80
#define C_ACENTO_WARN 0xFFE0
#define C_SEP_WARN 0x6320

//  Rojo (FRACASO)
#define C_FONDO_ERR 0x9800
#define C_ACENTO_ERR 0xF800
#define C_SEP_ERR 0x8800

#define C_VALOR TFT_WHITE

// ── Geometría (172 × 320 px, portrait) ───────────────────────
#define W 172
#define CX (W / 2)

// ── Layout vertical ──────────────────────────────────────────
//  Bloque temperatura: y=20..155
//    Label  "TEMPERATURA"  y=30
//    Valor  grande         y=85
//    Unidad "°C"           y=135
//  Separador doble         y=160..162
//  Bloque humedad: y=165..310
//    Label  "HUMEDAD"      y=175
//    Valor  grande         y=238
//    Unidad "% RH"         y=288

// ── Colores activos según estado ─────────────────────────────
uint16_t colorFondo() {
  switch (estadoActual) {
    case INTENTANDO: return C_FONDO_WARN;
    case FRACASO: return C_FONDO_ERR;
    default: return C_FONDO_OK;
  }
}
uint16_t colorAcento() {
  switch (estadoActual) {
    case INTENTANDO: return C_ACENTO_WARN;
    case FRACASO: return C_ACENTO_ERR;
    default: return C_ACENTO_OK;
  }
}
uint16_t colorSep() {
  switch (estadoActual) {
    case INTENTANDO: return C_SEP_WARN;
    case FRACASO: return C_SEP_ERR;
    default: return C_SEP_OK;
  }
}

// ================================================================
void setup() {
  prefs.begin("config", false);
  tempEncender = prefs.getFloat("t_on", 25.0f);
  tempApagar = tempEncender - DELTA_TEMP;
  alertasActivadas = prefs.getBool("alerts", true);
  Serial.begin(115200);
  Serial.println("\n=== Monitor Ambiental — Iniciando ===");

  // Configurar certificado raíz para Telegram (Requisito de seguridad)
  secured_client.setCACert(TELEGRAM_CERTIFICATE_ROOT);

  ledcAttach(PIN_BL, 5000, 8);
  ledcWrite(PIN_BL, 210);

  tft.init();
  tft.setRotation(2);
  tft.setTextDatum(MC_DATUM);
  tft.fillScreen(C_FONDO_OK);

  splashScreen();

  Wire.begin(21, 22);
  sht4x.begin(Wire, SHT40_I2C_ADDR_44);

  uint32_t serialNum = 0;
  uint16_t err = sht4x.serialNumber(serialNum);
  if (err == 0) {
    sensorOK = true;
    Serial.printf("SHT40 OK — Número de serie: %u\n", serialNum);
  } else {
    Serial.println("ADVERTENCIA: SHT40 no detectado");
  }

  irsend.begin();
  Serial.printf("IR TX en GPIO %d — listo\n", PIN_IR_TX);

  // ── Inicio de conexión Wi-Fi ─────────────────────────────
  Serial.printf("Conectando a red Wi-Fi: %s ", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);

  // Bloqueo inicial corto solo para asegurar la primera conexión
  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED && intentos < 15) {
    delay(500);
    Serial.print(".");
    intentos++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ Wi-Fi conectado exitosamente en el setup.");
    Serial.print("   IP Asignada: ");
    Serial.println(WiFi.localIP());
    wifiConectado = true;
    // ── ENVÍO DE ALERTA DE CONEXIÓN INICIAL AL BOT ──
    bot.sendMessage(CHAT_ID_ADMIN, "✅ *ThermoUN en línea.*\nSistema reiniciado y conexión Wi-Fi establecida correctamente.", "Markdown");
    if (sensorOKPrev && sensorAlgunaVezOK && wifiConectado) {
      bot.sendMessage(CHAT_ID_ADMIN, "⚠️ *Error:* No se detecta el sensor de temperatura.", "Markdown");
    }
    Serial.println("Mensaje de inicio enviado por Telegram.");
  } else {
    Serial.println("\n⚠️ No se pudo conectar al inicio. El sistema intentará en segundo plano.");
  }
  // ───────────────────────────────────────────────────────────

  leerSensor();
  dibujarUI();
}

// ================================================================
void loop() {

  // ── Lógica No Bloqueante para Re-conexión Wi-Fi ────────────
  unsigned long ahora = millis();

  // Revisión de Wi-Fi cada 5 segundos (MS_WIFI_CHECK)
  if (ahora - tUltimoWiFi >= MS_WIFI_CHECK) {
    tUltimoWiFi = ahora;

    bool estaConectadoAhora = (WiFi.status() == WL_CONNECTED);

    if (estaConectadoAhora && !wifiConectado) {
      // Caso: Acaba de recuperar la conexión
      wifiConectado = true;
      Serial.println("✅ Wi-Fi Reconectado.");

      // Intentar avisar (esto fallará si no hay internet real aún,
      // pero el próximo ciclo del bot lo compensará)
      bot.sendMessage(CHAT_ID_ADMIN, "🔄 *ThermoUN:* Conexión restablecida.", "Markdown");
    } else if (!estaConectadoAhora && wifiConectado) {
      // Caso: Se acaba de perder la conexión
      wifiConectado = false;
      Serial.println("⚠️ Wi-Fi Perdido. Reintentando automáticamente...");
      // No llamamos a WiFi.reconnect(), el ESP32 ya lo está haciendo solo.
    }
  }

  if (ahora - tUltimoSensor >= MS_SENSOR) {
    tUltimoSensor = ahora;
    leerSensor();
    actualizarValores();
  }

  if (ahora - tUltimoIR >= MS_IR) {
    tUltimoIR = ahora;
    gestionarIR();
  }

  // ── Lógica No Bloqueante para Telegram ────────────
  if (wifiConectado && (ahora - tUltimoTelegram >= MS_BOT_CHECK)) {
    tUltimoTelegram = ahora;

    // Obtenemos la cantidad de mensajes sin leer
    int numNuevosMensajes = bot.getUpdates(bot.last_message_received + 1);

    while (numNuevosMensajes) {
      manejarMensajesTelegram(numNuevosMensajes);
      numNuevosMensajes = bot.getUpdates(bot.last_message_received + 1);
    }
  }
}

// ================================================================
//  SENSOR
// ================================================================
void leerSensor() {
  float t, h;
  uint16_t err = sht4x.measureHighPrecision(t, h);

  sensorOKPrev = sensorOK;  // guarda estado anterior ANTES de actualizar

  if (err == 0) {
    temperatura = t;
    humedad     = h;
    sensorOK    = true;

    // ¿Acaba de recuperarse después de haber fallado (y el sensor sí funcionó antes)?
    if (!sensorOKPrev && sensorAlgunaVezOK && wifiConectado) {
      bot.sendMessage(CHAT_ID_ADMIN, "✅ *Medición reestablecida.*", "Markdown");
    }
    sensorAlgunaVezOK = true;  // a partir de aquí, las transiciones futuras sí se notifican
    Serial.printf("  Temp = %+.2f °C   Hum = %.2f %%RH\n", t, h);

  } else {
    sensorOK = false;

    // ¿Acaba de fallar? Solo notifica si antes funcionaba
    if (sensorOKPrev && sensorAlgunaVezOK && wifiConectado) {
      bot.sendMessage(CHAT_ID_ADMIN, "⚠️ *Error:* No se detecta el sensor de temperatura.", "Markdown");
    }
    Serial.printf("  Error de lectura SHT40: código %u\n", err);
  }
}

// ================================================================
//  DISPLAY — splash
// ================================================================
void splashScreen() {
  tft.fillScreen(C_FONDO_OK);
  tft.setTextColor(C_ACENTO_OK, C_FONDO_OK);
  tft.setTextSize(1);
  tft.drawString("Monitor Ambiental", CX, 148);
  tft.drawString("ESP32 + SHT40 + IR", CX, 168);
  delay(800);
}

// ================================================================
//  DISPLAY — estructura fija
//
//  Bloque temperatura  y=20..155
//  Separador doble     y=160..162
//  Bloque humedad      y=165..310
// ================================================================
void dibujarUI() {
  uint16_t cf = colorFondo();
  uint16_t ca = colorAcento();
  uint16_t cs = colorSep();

  tft.fillScreen(cf);

  // Labels pegados a sus bloques
  tft.setTextColor(ca, cf);
  tft.setTextSize(2);
  tft.drawString("TEMPERATURA", CX, 40);
  tft.drawString("HUMEDAD", CX, 185);

  // Separador horizontal doble centrado verticalmente
  tft.drawLine(12, 160, W - 12, 160, cs);
  tft.drawLine(12, 162, W - 12, 162, cs);

  actualizarValores();
}

// ================================================================
//  DISPLAY — refresca solo los números
// ================================================================
void actualizarValores() {
  uint16_t cf = colorFondo();
  uint16_t ca = colorAcento();
  char buf[12];

  if (!sensorOK) {
    tft.fillRect(0, 50, W, 108, cf);
    tft.fillRect(0, 195, W, 108, cf);
    tft.setTextSize(2);
    tft.setTextColor(ca, cf);
    tft.drawString("-- ERROR --", CX, 95);
    tft.drawString("-- ERROR --", CX, 240);
    return;
  }

  // ── Temperatura ──────────────────────────────────────────
  tft.fillRect(0, 50, W, 108, cf);

  tft.setTextSize(4);
  tft.setTextColor(C_VALOR, cf);
  dtostrf(temperatura, 5, 1, buf);
  tft.drawString(buf, CX, 95);

  tft.setTextSize(2);
  tft.setTextColor(ca, cf);
  tft.drawString("o C", CX, 140);

  // ── Humedad ──────────────────────────────────────────────
  tft.fillRect(0, 195, W, 108, cf);

  tft.setTextSize(4);
  tft.setTextColor(C_VALOR, cf);
  dtostrf(humedad, 5, 1, buf);
  tft.drawString(buf, CX, 238);

  tft.setTextSize(2);
  tft.setTextColor(ca, cf);
  tft.drawString("% RH", CX, 283);
}

// ================================================================
//  CAMBIO DE ESTADO Y ALERTAS TELEGRAM
// ================================================================
void cambiarEstado(Estado nuevo) {
  if (nuevo == estadoActual) return;

  Estado estadoAnterior = estadoActual;  // Guardamos memoria de dónde venimos
  estadoActual = nuevo;
  dibujarUI();  // Actualiza la pantalla TFT inmediatamente

  // ── Lógica de Alertas Telegram ──
  String mensaje = "";

  // 1. STANDBY -> INTENTANDO (Subió la temperatura)
  if (estadoAnterior == STANDBY && nuevo == INTENTANDO) {
    mensaje = "⚠️ *ALERTA:* Temperatura en " + String(temperatura, 1) + "°C.\nIniciando protocolo de normalización de AC.";
  }
  // 2. INTENTANDO -> FRACASO (Arriba por mucho tiempo)
  else if (estadoAnterior == INTENTANDO && nuevo == FRACASO) {
    mensaje = "🚨 *FRACASO CRÍTICO:* 10 min sin reducir la temperatura.\nMantenida en " + String(temperatura, 1) + "°C.";
  }
  // 3. FRACASO -> INTENTANDO (Temperatura comenzando a bajar)
  else if (estadoAnterior == FRACASO && nuevo == INTENTANDO) {
    mensaje = "🌡️ *Temperatura en proceso de normalización.*\nActualmente en " + String(temperatura, 1) + "°C.";
  }
  // 4. FRACASO/INTENTANDO -> STANDBY (Temperatura reestablecida)
  else if ((estadoAnterior == FRACASO || estadoAnterior == INTENTANDO) && nuevo == STANDBY) {
    mensaje = "✅ *NORMALIZADO:* Temperatura óptima alcanzada (" + String(temperatura, 1) + "°C).\nSistema en estado normal.";
  }

  // Si se generó un mensaje y las alertas no han sido silenciadas, se envía
  if (mensaje != "" && alertasActivadas) {
    bot.sendMessage(CHAT_ID_ADMIN, mensaje, "Markdown");
    Serial.println("Telegram >> Alerta de cambio de estado enviada.");
  }
}

// ================================================================
//  IR — gestión por temperatura con histéresis y temporizador
//
//  Protocolo : LG2
//  Código HEX: 0x8800628 (28 bits)
//  address   : 0x88   command : 0x62
// ================================================================
void gestionarIR() {
  unsigned long ahora = millis();

  if (temperatura >= tempEncender) {

    // ── Transición STANDBY → INTENTANDO ──────────────────────────
    if (estadoActual == STANDBY) {
      tInicioIntento = ahora;
      t_prev = temperatura;
      bajandoMensajeEnviado = false;
      cambiarEstado(INTENTANDO);
      Serial.printf("ALERTA   : %.1f°C >= %.1f°C — Iniciando intentos\n",
                    temperatura, tempEncender);
    }

    unsigned long tiempoElevado = ahora - tInicioIntento;
    bool tempBajando = (temperatura <= t_prev - 0.1f);

    if (tiempoElevado >= MS_FRACASO && !tempBajando) {
      // ── Fracaso: >= 10 min Y temperatura sin bajar ────────────
      if (estadoActual != FRACASO) {
        cambiarEstado(FRACASO);
        Serial.printf("FRACASO  : 10 min sin reducir temperatura (%.1f°C)\n",
                      temperatura);
      }
      irsend.sendLG2(0x8800628ULL, 28);
      t_prev = temperatura;
      Serial.printf("IR >>    : LG2 0x8800628 enviado (%.1f°C)\n", temperatura);

    } else {
      // ── INTENTANDO: < 10 min, O temperatura bajando ──────────
      // (si baja la temp, no se declara fracaso sin importar el tiempo)
      if (tempBajando) {
        if (!bajandoMensajeEnviado && alertasActivadas) {
          unsigned long segs = tiempoElevado / 1000;
          String mm = String(segs / 60);
          String ss = (segs % 60 < 10 ? "0" : "") + String(segs % 60);
          String msg = (tiempoElevado >= MS_FRACASO)
            ? "🌡️ *Temperatura bajando, manténgase alerta.*\nTiempo de temperatura elevada: " + mm + ":" + ss
            : "🌡️ *Temperatura bajando.* Señal IR suspendida temporalmente.";
          bot.sendMessage(CHAT_ID_ADMIN, msg, "Markdown");
          bajandoMensajeEnviado = true;
        }
        Serial.printf("IR >>    : Suprimido — bajando (%.1f°C <= t_prev %.1f°C - 0.1)\n",
                      temperatura, t_prev);
      } else {
        bajandoMensajeEnviado = false;
        irsend.sendLG2(0x8800628ULL, 28);
        t_prev = temperatura;
        Serial.printf("IR >>    : LG2 0x8800628 enviado (%.1f°C)\n", temperatura);
      }
    }

  } else {
    // temperatura < tempEncender

    if (temperatura < tempApagar) {
      // ── Temperatura normal → OK ───────────────────────────────
      if (estadoActual != STANDBY) {
        tInicioIntento = 0;
        bajandoMensajeEnviado = false;
        Serial.printf("OK       : %.1f°C < %.1f°C — Temperatura normalizada\n",
                      temperatura, tempApagar);
        cambiarEstado(STANDBY);
      }

    } else {
      // tempApagar <= temperatura < tempEncender
      // Zona de transición: solo actuar si venimos de FRACASO
      if (estadoActual == FRACASO) {
        tInicioIntento = ahora;
        t_prev = temperatura;
        bajandoMensajeEnviado = false;
        cambiarEstado(INTENTANDO);
        Serial.printf("NORMALIZ : %.1f°C — Retomando desde FRACASO\n", temperatura);
      }
    }
  }
}

// ================================================================
//  GESTIÓN DE MENSAJES TELEGRAM
// ================================================================
void manejarMensajesTelegram(int numNuevosMensajes) {
  for (int i = 0; i < numNuevosMensajes; i++) {
    String chat_id = String(bot.messages[i].chat_id);
    String texto = bot.messages[i].text;
    String remitente = bot.messages[i].from_name;

    // ── 1. SEGURIDAD: LISTA BLANCA ──
    if (chat_id != String(CHAT_ID_ADMIN)) {
      bot.sendMessage(chat_id, "⛔ Acceso denegado. No tienes autorización para operar este monitor.", "");
      Serial.println("Intento de acceso bloqueado del usuario: " + remitente);
      continue;  // Salta al siguiente mensaje sin procesar nada más
    }

    Serial.println("Comando recibido: " + texto);

    // ── 2. PROCESAMIENTO DE COMANDOS ──

    // ── Wizard /set_limits: intercepción de pasos activos ──────────
    if (setLimitsStep == 1) {
      String inp = texto; inp.trim();
      if (inp.length() == 0 || (!isDigit(inp[0]) && inp[0] != '-')) {
        bot.sendMessage(chat_id, "⚠️ El valor debe ser un número. Operación cancelada.", "");
      } else {
        tempEncender = inp.toFloat();
        tempApagar   = tempEncender - DELTA_TEMP;
        prefs.putFloat("t_on", tempEncender);
        // t_off ya no se guarda en prefs: siempre se deriva al arrancar
        bot.sendMessage(chat_id,
          "⚙️ *Temperatura límite actualizada:*\n🔥 Encendido AC: " + String(tempEncender, 1) +
          "°C\n❄️ Normalización: " + String(tempApagar, 1) + "°C",
          "Markdown");
      }
      setLimitsStep = 0;
    }

    if (texto == "/status") {
      String mensaje = "📊 *Estado Actual del Sistema*\n\n";
      mensaje += "🌡️ Temperatura: *" + String(temperatura, 1) + " °C*\n";
      mensaje += "💧 Humedad: *" + String(humedad, 1) + " %RH*\n";

      mensaje += "⚙️ Estado AC: ";
      if (estadoActual == STANDBY) mensaje += "Normal (OK)";
      else if (estadoActual == INTENTANDO) {
        bool irSuspendido = (temperatura <= t_prev - 0.1f);
        mensaje += irSuspendido
          ? "Intentando (Temperatura bajando, IR suspendido)"
          : "Intentando (Enviando señal IR)";
      }
      else if (estadoActual == FRACASO) mensaje += "⚠️ FRACASO (No enfría)";

      mensaje += "\n\n🔒 Temperatura máxima:\nEncender: " + String(tempEncender, 1) + "°C";

      bot.sendMessage(chat_id, mensaje, "Markdown");
    }

    else if (texto == "/enable_alerts") {
      if (!alertasActivadas) {
        alertasActivadas = true;
        prefs.putBool("alerts", true);
      }
      bot.sendMessage(chat_id, "✅ *Alertas Activadas.*\nRecibirás notificaciones de los cambios de estado.", "Markdown");
    }

    else if (texto == "/disable_alerts") {
      if (alertasActivadas) {
        alertasActivadas = false;
        prefs.putBool("alerts", false);
      }
      bot.sendMessage(chat_id, "🔕 *Alertas Desactivadas.*\nEl sistema operará en silencio.", "Markdown");
    }

    else if (texto == "/login") {
      // Comando comodín: Ignorado silenciosamente por diseño.
      bot.sendMessage(chat_id, "✅ Ya te encuentras autenticado mediante Chat ID.", "");
    }

    else if (texto == "/set_limits") {
      setLimitsStep = 1;
      bot.sendMessage(chat_id,"🌡️ Ingresa la temperatura límite en °C:\n_(Se le notificará cuando la habitación supere esta temperatura)_", "Markdown");
    }

    else if (texto == "/start") {
      String bienvenida = "👋 Hola " + remitente + ", bienvenido al Monitor Ambiental.\n\nUsa el menú para interactuar con el sistema.";
      bot.sendMessage(chat_id, bienvenida, "");
    }

    else if (texto == "/info") {
      String info = "🌡️ *ThermoHigrometer*\n";
      info += "_El chat bot que te permite conocer las condiciones ambientales del clúster computacional del DIEE._\n\n";
      info += "📋 *Comandos disponibles:*\n\n";
      info += "*/status* — Muestra la temperatura, humedad y estado actual del sistema.\n\n";
      info += "*/set\\_limits* — Define la temperatura límite. El sistema encenderá el AC al superarla.\n\n";
      info += "*/reset* — Restaura los valores predeterminados de temperatura y alertas.\n\n";
      info += "*/enable\\_alerts* — Activa las notificaciones automáticas de cambio de estado.\n\n";
      info += "*/disable\\_alerts* — Silencia las notificaciones. El sistema sigue operando.\n\n";
      info += "*/info* — Muestra esta ayuda.\n\n";
      info += "*/start* — Mensaje de bienvenida.";
      bot.sendMessage(chat_id, info, "Markdown");
    }

    else if (texto == "/reset") {
      prefs.putFloat("t_on",  25.0f);
      tempEncender = 25.0f;
      tempApagar   = tempEncender - DELTA_TEMP;
      prefs.putBool("alerts", true);
      alertasActivadas = true;
      bot.sendMessage(chat_id,
        "🔄️ *Valores restablecidos:*\nTemp. límite: 25°C 🔥\nNormalización: " + String(tempApagar, 1) + "°C ❄️\nAlertas: Activas ✅",
        "Markdown");
    }
  }
}