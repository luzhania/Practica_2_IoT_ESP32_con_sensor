#include <Arduino.h>
#include <iostream>
#include "Utilities.h"
#include "SensorClient.h"
using namespace std;

SensorClient client("Galaxy S9+7c14", "betitox007.,", 17, 19);

void setup()
{
  client.setup();
}

void loop()
{
  Utilities::nonBlockingDelay(100, []()
                              { client.loop(); });
}
