#include "utils.h"
#include <Arduino.h>

String Utils::floatToStr(float num) {
  char charVal[10] = "          ";
  dtostrf(num, 4, 2, charVal);

  String strVal = "";
  for (int i = 0; i < sizeof(charVal); i++) {
    if (charVal[i] != ' ') {
      strVal += charVal[i];
    } else {
      break;
    }
  }

  return strVal;
}
