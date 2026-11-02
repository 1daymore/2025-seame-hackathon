python3 - <<'PY'
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
ssd1306(i2c(port=1, address=0x3C)).hide()   # DISPLAY OFF
PY