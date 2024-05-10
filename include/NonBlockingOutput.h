#include <Arduino.h>
#define MAX_OUTPUTS 8
#define ON 1
#define OFF 0

class NonBlockingOutput {
  public:
    NonBlockingOutput();
    void addOutput(uint8_t nPin, uint16_t nDuration);
    void begin() {
      initOutputs();
    }
    void start();
    void update();
    
  private:
    void initOutputs();

    bool m_fStart;
    uint8_t m_arOutputPin[MAX_OUTPUTS];
    uint16_t m_arOutputDuration[MAX_OUTPUTS];
    uint8_t m_nOutputCount;
    uint8_t m_nCurrentIndex;
    uint32_t m_nLastMillis;
};