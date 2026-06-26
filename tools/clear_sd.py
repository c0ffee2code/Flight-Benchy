"""
Delete all flight data from the SD card.

Mounts /sd, removes every entry under /sd/flights/, then unmounts.
Runs on the Pico via:
    python -m mpremote connect COM7 run tools/clear_sd.py
"""
import os
import time
from machine import Pin, SPI

import sdcard

PIN_SD_SCK  = 18
PIN_SD_MOSI = 19
PIN_SD_MISO = 16
PIN_SD_CS   = 17

SD_MOUNT    = "/sd"
FLIGHTS_DIR = SD_MOUNT + "/flights"


def rmtree(path):
    stat = os.stat(path)
    if stat[0] & 0x4000:  # directory
        for name in os.listdir(path):
            rmtree(path + "/" + name)
        os.rmdir(path)
        print("  rmdir:", path)
    else:
        os.remove(path)
        print("  rm:   ", path)


def main():
    cs_pin = Pin(PIN_SD_CS, Pin.OUT, value=1)
    spi = SPI(0, baudrate=400_000, polarity=0, phase=0,
              sck=Pin(PIN_SD_SCK), mosi=Pin(PIN_SD_MOSI), miso=Pin(PIN_SD_MISO))
    time.sleep_ms(250)
    sd = sdcard.SDCard(spi, cs_pin, baudrate=25_000_000)
    vfs = os.VfsFat(sd)
    os.mount(vfs, SD_MOUNT)

    try:
        try:
            entries = os.listdir(FLIGHTS_DIR)
        except OSError:
            print("No flights directory found -- nothing to delete.")
            return
        print("Found {} item(s) in {}".format(len(entries), FLIGHTS_DIR))
        for name in entries:
            rmtree(FLIGHTS_DIR + "/" + name)
        print("Done -- SD card cleared.")
    finally:
        os.umount(SD_MOUNT)


main()
