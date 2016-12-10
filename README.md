# Final Project Hardware Synthesis Laboratory
## Assigment
### Using board STM32F4DISCOVERY that have following IC in it. 
1. LIS302DL: 3-axis accelerometer
2. MP45DT02: digital microphone
  * http://www.christoph-lauer.de/archives/11669
  * https://www.theunterminatedstring.com/probing-pdm/
  * http://www.st.com/content/ccc/resource/technical/document/application_note/ca/18/be/bb/f8/53/47/a5/DM00040808.pdf/files/DM00040808.pdf/jcr:content/translations/en.DM00040808.pdf
3. CS43L22: audio DAC, speaker driver
  * [Data Sheet](https://www.cirrus.com/en/pubs/proDatasheet/CS43L22_F2.pdf)
  * [Sample code + nice code explain (โคตรยาว)](http://www.design-software.de/system/attachments/attached_binaries/000/000/059/original/100_dac.c?1432329707)
  * [Some explanation how it work but use difference function.](http://www.mind-dump.net/configuring-the-stm32f4-discovery-for-audio)
  * http://www.intmath.com/trigonometric-graphs/music.php
  * http://narodstream.ru/stm-urok-48-usb-device-audio/#codelink

---

### MP45DT02: digital microphone Tutorials
import file `pdm_filter.h` and `libPDMFilter_Keil.lib` to project for using PDM_Filter

add following code to `main.c`

```c
#include "pdm_filter.h"
...
static PDMFilter_InitStruct pdm_filter;
...
int main(void) 
{
 ...
 
 pdm_filter.LP_HZ=8000;
 pdm_filter.HP_HZ=10;
 pdm_filter.Fs=FS;
 pdm_filter.Out_MicChannels=1;
 pdm_filter.In_MicChannels=1;
 
 PDM_Filter_Init(&pdm_filter);
}
```

to use pdm filter use when `pdm_fifo` is full.

---

### CS43L22: audio DAC, speaker driver
Note frequency calcutor
from [reference] http://www.intmath.com/trigonometric-graphs/music.php, C is at frequency 261.63. so calculate by `48k / 261.63` 48k is number of I2S sample size.

then we need to generate sine wave length equals number that we calculate above. This will be 1 loop.