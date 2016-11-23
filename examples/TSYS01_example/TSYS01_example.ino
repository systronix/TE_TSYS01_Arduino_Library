#include <tsys01.h>

static tsys01 m_tsys01;

void setup(void) {
  Serial.begin(9600);

  Serial.println("==== TE Connectivity ====");
  Serial.println("======== TSYS01 =========");
  Serial.println("======== Measure ========");

  m_tsys01.begin();
}

void loop(void) {
  float temperature;
  boolean connected;

  connected = m_tsys01.is_connected();
  if (connected) {
    Serial.println(connected ? "Sensor Connected" : "Sensor Disconnected");

    m_tsys01.read_temperature(&temperature);

    Serial.print("---Temperature = ");
    Serial.print(temperature, 1);
    Serial.print((char)176);
    Serial.println("C");
  } else {
    Serial.println(connected ? "Sensor Connected" : "Sensor Disconnected");
  }

  delay(1000);
}
