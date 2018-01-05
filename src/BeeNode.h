
#ifndef BeeNode_h
#define BeeNode_h

#include <arduino.h>

typedef struct {
  byte nodeId[4];
  byte nodeAddress;
  float hiveTemps[3];
} NodeData_t;

#endif
