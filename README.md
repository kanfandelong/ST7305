# ST7305_DMA 库说明

## 简介
ST7305_DMA 是基于 ESP32 平台、支持 DMA 刷新和多缓冲区管理的 ST7305 显示驱动库，继承自 Adafruit_GFX，适用于 384x168 分辨率的单色全反射式液晶屏。支持高性能 SPI DMA 刷新、TE 同步、滑动特效、缓冲区混合等高级功能。

## 主要特性
- 支持 ESP32 硬件 SPI + DMA 高速刷新
- 多缓冲区（4 个）管理，支持缓冲区切换、复制、比较、混合
- 支持 TE（Tearing Effect）信号同步刷新，防止撕裂
- 支持滑动特效（全屏/分块滑动）
- 支持旋转、窗口裁剪、反显、休眠、功耗模式切换
- 兼容 Adafruit_GFX 基本绘图 API

## 依赖
- [Adafruit_GFX](https://github.com/adafruit/Adafruit-GFX-Library)
- ESP-IDF/Arduino 框架

## 主要接口
- `begin(bool reset = true)`：初始化显示屏
- `display(bool ignoreTE = false)`：刷新显示内容
- `clearDisplay(uint16_t color = 0xFF)`：清屏
- `drawPixel(x, y, color)`：绘制像素
- `drawbitmap/drawXBitmap(...)`：绘制位图
- `slideScreenFull(...)`：全屏滑动特效
- `swapBuffer(idx)`：切换缓冲区
- `blendBuffers(dest, src, mode)`：缓冲区混合
- `setvoltage(...)`：设置驱动电压
- `setPowerMode(...)`：切换功耗模式

## 示例
```cpp
#include "ST7305_DMA.h"

ST7305_DMA lcd(384, 168, VSPI_HOST, 18, 23, -1, 5, 17, 16, 4);

void setup() {
    lcd.begin();
    lcd.clearDisplay();
    lcd.drawPixel(10, 10, 0);
    lcd.display();
}

void loop() {
    // ...
}
```

## 典型接线
| LCD 引脚 | ESP32 GPIO |
|----------|------------|
| SCLK     | 18         |
| MOSI     | 23         |
| CS       | 5          |
| DC       | 17         |
| RST      | 16         |
| TE       | 4 (可选)   |

## 进阶用法
- **多缓冲区动画**：利用 `swapBuffer`/`blendBuffers` 实现无撕裂动画
- **TE 同步**：硬件连接 TE 引脚，`display()` 自动等待同步
- **滑动特效**：`slideScreenFull` 支持多方向滑动切换

## 许可协议
GPL-V3 License

---
如需详细 API 说明，请参考源码注释。

# ST7305

```c

#include <SPI.h>
#include "ST7305.h"

// Pin definitions
#define TFT_CS    SS
#define TFT_DC    8
#define TFT_RST   10
#define TFT_TE    -1  // Set to valid pin if using TE, or -1 if not used

// Display dimensions
#define SCREEN_WIDTH   168
#define SCREEN_HEIGHT  384

// Create display instance
ST7305 display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, TFT_CS, TFT_DC, TFT_RST, TFT_TE);

void setup() {
    Serial.begin(115200);
    Serial.println("ST7305 Test");

    // Initialize SPI
    SPI.begin();
    
    // Initialize display
    if (!display.begin()) {
        Serial.println("Failed to initialize display!");
        while (1);
    }
  
    display.setRotation(1);
    // Draw some test patterns
    display.clearDisplay();
    
    // Draw a border
    display.drawRect(0, 0, display.width(), display.height(), 1);
    
    // Draw some text
    display.setTextSize(2);
    display.setTextColor(1);
    display.setCursor(10, 10);
    display.println("Hello World!");
    
    // Draw some shapes
    display.drawRect(10, 50, 60, 40, 1);
    display.fillRect(100, 50, 60, 40, 1);
    display.drawCircle(200, 70, 20, 1);
    display.fillCircle(280, 70, 20, 1);
    display.drawRoundRect(10, 200, 30, 50, 30, 1);
    
    
    // Update display
    display.display();
}

void loop() {
    // Your code here
    delay(1000);
}


```

![st7305](st7305.jpg)
