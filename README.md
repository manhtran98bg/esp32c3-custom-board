# ESP32-C3 Custom Board for Xiaozhi AI

This repository contains the **esp32c3-custom-board**, a custom hardware configuration designed for the Xiaozhi AI project (78/xiaozhi-esp32).  
The board features an ESP32-C3 MCU, a 1.28'' GC9A01 round display with touch panel, and an ES8311 audio codec paired with an NS4150 amplifier and a single microphone input.

---

## **Hardware Overview**

### **Main Components**
- **MCU:** ESP32-C3  
- **Display:** GC9A01 1.28” Round LCD (SPI)  
- **Touch:** I2C Touch Panel  
- **Audio:**
  - ES8311 Codec  
  - Output: NS4150 Amplifier x 1 
  - Input: Microphone x 1 

---

## **Pin Mapping**

### **Audio (I2S + ES8311 Codec)**

| Function | ESP32-C3 Pin |
|----------|--------------|
| MCLK | GPIO 2 |
| WS / LRCK | GPIO 5 |
| BCLK / SCLK | GPIO 3 |
| I2S DOUT (Codec → ESP32C3) | GPIO 4 |
| I2S DIN (ESP32C3 → Codec) | GPIO 6 |
| PA Enable (NS4150) | *Not connected* |
| Codec I2C SDA | GPIO 0 |
| Codec I2C SCL | GPIO 1 |

---

### **Display — GC9A01 (SPI)**

| Function | ESP32-C3 Pin |
|----------|--------------|
| LCD_BL | GPIO 7 |
| LCD_CLK | GPIO 10 |
| LCD_DATA | GPIO 9 |
| LCD_CS | *Not connected* |
| LCD_DC | GPIO 20 |
| LCD_RST | GPIO 8 |

---

### **Touch Panel (I2C)**

| Function | ESP32-C3 Pin |
|----------|--------------|
| TP SDA | GPIO 0 |
| TP SCL | GPIO 1 |
| TP RST | GPIO 8 |
| TP INT | GPIO 21 |

---

## **Intergration**
###  Step 1 — Add the Board Type to `Kconfig.projbuild`
Open:

```
main/Kconfig.projbuild
```

Inside the `choice BOARD_TYPE` section, add a new entry:

```kconfig
choice BOARD_TYPE
    prompt "Board Type"
    default BOARD_TYPE_BREAD_COMPACT_WIFI
    help
        Board type. 开发板类型

    # ... other board options ...

    config BOARD_TYPE_ESP32C3_CUSTOM_BOARD
        bool "ESP32-C3 Custom Board"
        depends on IDF_TARGET_ESP32C3
endchoice
```

### Notes:
- The name **BOARD_TYPE_ESP32C3_CUSTOM_BOARD** must be:
  - ALL UPPERCASE  
  - Use underscores  
- The `depends on` line must match your chip:
  - `IDF_TARGET_ESP32C3` for ESP32‑C3  

---
### Step 2 — Add CMake Configuration in `main/CMakeLists.txt`

Find the board selection chain and add:

```cmake
elseif(CONFIG_BOARD_TYPE_ESP32C3_CUSTOM_BOARD)
    set(BOARD_TYPE "esp32c3-custom-board")  # must match folder name
    set(BUILTIN_TEXT_FONT font_puhui_basic_20_4)
    set(BUILTIN_ICON_FONT font_awesome_20_4)
    set(DEFAULT_EMOJI_COLLECTION twemoji_64)
endif()
```
### Font Size Recommendations

Choose based on screen resolution:

| Screen Size | Text Font | Icon Font |
|-------------|-----------|-----------|
| 128×64 | `font_puhui_basic_14_1` | `font_awesome_14_1` |
| 240×240 | `font_puhui_basic_16_4` | `font_awesome_16_4` |
| 240×320 | `font_puhui_basic_20_4` | `font_awesome_20_4` |
| 480×320+ | `font_puhui_basic_30_4` | `font_awesome_30_4` |

### Emoji Collections

- `twemoji_32` — smaller screens  
- `twemoji_64` — larger screens (recommended for 1.28" GC9A01)
---

## **Reference:** 
Official Xiaozhi custom board guide — **[here](https://github.com/78/xiaozhi-esp32/blob/main/docs/custom-board.md)**


---
