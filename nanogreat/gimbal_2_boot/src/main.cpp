// nanogreat 独立工程入口，不依赖 Nano 云台主工程的控制入口。
#include "nanogreatMount.h"

void setup()
{
    initNanogreat();
}

void loop()
{
    nanogreatLoop();
}

