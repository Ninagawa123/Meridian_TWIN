#include <Meridian.h>

#define MSG_SIZE 90           // Size of Meridim array
MERIDIANFLOW::Meridian mr; //namespace specification and instantiation for Meridian

typedef union // Union for Meridim array
{
  short sval[MSG_SIZE + 2];
  unsigned short usval[MSG_SIZE + 2];
  uint8_t bval[MSG_SIZE * 2 + 4];
} UnionData;
UnionData s_spi_meridim;     // Meridim array for sending spi
UnionData r_spi_meridim;     // Meridim array for receiving spi
UnionData s_spi_meridim_dma; // Meridim array for sending udp
UnionData r_spi_meridim_dma; // Meridim array for receiving udp

void setup() {
  Serial.begin(115200);
}

void loop() {
  float value1 = 67.89;
  short value2 = 23456;

  Serial.println(mr.cksm_val(s_spi_meridim.sval, MSG_SIZE)); // calculate checksum
  Serial.println(mr.cksm_rslt(s_spi_meridim.sval, MSG_SIZE)); // evaluate checksum, OK:True, NG:False
  Serial.println(mr.float2HfShort(value1)); //Float type value multiplied by 100 to short type
  Serial.println(mr.HfShort2float(value2)); //Float type value multiplied by 100 to short type

  Serial.println("---");
}
