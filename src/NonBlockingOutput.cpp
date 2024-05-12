#include <Arduino.h>
#include "NonBlockingOutput.h"

NonBlockingOutput::NonBlockingOutput() {
  m_nLastMillis = 0;
  m_nOutputCount = 0;
  m_nCurrentIndex = 0;
}

void NonBlockingOutput::addOutput(uint8_t nPin, uint16_t nDuration) {
  if (m_nOutputCount < MAX_OUTPUTS) {
    m_arOutputPin[m_nOutputCount] = nPin;
    m_arOutputDuration[m_nOutputCount] = nDuration;
    m_nOutputCount++;
  }
}

void NonBlockingOutput::start() {
  if (m_fStart)
    return;
  m_fStart = true;
  m_nLastMillis = millis();
  m_nCurrentIndex = 0;
  digitalWrite(m_arOutputPin[m_nCurrentIndex], ON);
}

void NonBlockingOutput::update() {
    if (!m_fStart)
        return;

    if (millis() - m_nLastMillis >= m_arOutputDuration[m_nCurrentIndex]) {
        m_nLastMillis = millis();
        digitalWrite(m_arOutputPin[m_nCurrentIndex], OFF);
        m_nCurrentIndex++;
        if (m_nCurrentIndex >= m_nOutputCount) {
            m_nCurrentIndex = 0;
            m_fStart = false;
        }
        else
          digitalWrite(m_arOutputPin[m_nCurrentIndex], ON);
    }
}

void NonBlockingOutput::initOutputs() {
  for (uint8_t i = 0; i < m_nOutputCount; i++) {
    pinMode(m_arOutputPin[i], OUTPUT);
    digitalWrite(m_arOutputPin[i], OFF);
  }
}