#!/usr/bin/env python3
"""
INA219 전압·전류 → SSD1306 OLED 실시간 표시 스크립트
- INA219 초기화 실패·I²C 오류가 나도 루프 유지
- Ctrl-C, systemd SIGTERM 시 OLED 화면 OFF
"""

import time, sys, signal
from board import SCL, SDA
import busio
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
from PIL import ImageFont, Image

# ─────────────────────────────────────────────────────────────
# 1) INA219 센서 준비 (실패해도 프로그램은 계속)
# ─────────────────────────────────────────────────────────────
SENSOR_ADDR = 0x40            # sudo i2cdetect -y 1 결과에 맞춰 수정
try:
    from adafruit_ina219 import INA219
    i2c_bus = busio.I2C(SCL, SDA)                 # frequency 기본 100 kHz
    ina219   = INA219(i2c_bus, addr=SENSOR_ADDR)
    HAVE_INA = True
except Exception as e:
    print("[WARN] INA219 init failed:", e)
    ina219   = None
    HAVE_INA = False

# ─────────────────────────────────────────────────────────────
# 2) OLED 디바이스 준비
# ─────────────────────────────────────────────────────────────
serial  = i2c(port=1, address=0x3C)
device  = ssd1306(serial)

try:
    font = ImageFont.truetype("DejaVuSans.ttf", 20)
except IOError:
    font = ImageFont.load_default()

# 화면 OFF 용 클린업 함수
def oled_off(*_):
    try:
        device.hide()      # DISPLAY OFF
    finally:
        sys.exit(0)

# SIGINT(Ctrl-C)·SIGTERM(systemd Stop) 잡기
signal.signal(signal.SIGINT,  oled_off)
signal.signal(signal.SIGTERM, oled_off)

# ─────────────────────────────────────────────────────────────
# 3) 헬퍼 함수
# ─────────────────────────────────────────────────────────────
MAX_V = 16.8
MIN_V = 12.0
def battery_percent(v):
    if v >= MAX_V: return 100
    if v <= MIN_V: return 0
    return round((v - MIN_V) * 100 / (MAX_V - MIN_V))

# ─────────────────────────────────────────────────────────────
# 4) 메인 루프
# ─────────────────────────────────────────────────────────────
t0 = time.time()          # 경과초 표시용
while True:
    # 4-1) 센서 읽기 (예외 무시하고 0으로 대체)
    try:
        if HAVE_INA:
            voltage = ina219.bus_voltage + ina219.shunt_voltage
            current = ina219.current
            power   = ina219.power
        else:
            voltage = current = power = 0
    except Exception as e:
        print("[INA219]", e)
        voltage = current = power = 0

    # 4-2) OLED 그리기 (예외 발생 시 로그만 남기고 루프 유지)
    try:
        with canvas(device) as draw:
            draw.text((0, 0),  f"Volt: {voltage:5.2f} V", font=font, fill="white")
            draw.text((0, 20), f"Batt: {battery_percent(voltage):3d} %", font=font, fill="white")
            draw.text((0, 40), f"T+{int(time.time()-t0)} s", font=font, fill="white")
    except Exception as e:
        print("[OLED]", e)

    # (선택) 로그 파일에 남길 때만
    # print(f"{time.strftime('%H:%M:%S')}  V={voltage:.2f}  I={current:.0f}  P={power:.0f}")

    time.sleep(1)
