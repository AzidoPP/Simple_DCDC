# Simple_DCDC —— 100W 程控/数控 可调 Buck 电源模块

[![GitHub](https://img.shields.io/badge/GitHub-azidopp-181717?style=flat&logo=github&logoColor=white)](https://github.com/azidopp)
[![License](https://img.shields.io/badge/License-GPL--3.0-3DA639?style=flat&logo=gnu&logoColor=white)](https://www.gnu.org/licenses/gpl-3.0)
[![YouTube](https://img.shields.io/badge/YouTube-@lyyontop-FF0000?style=flat&logo=youtube&logoColor=white)](https://www.youtube.com/@lyyontop)
[![bilibili](https://img.shields.io/badge/bilibili-1084866085-00A1D6?style=flat&logo=bilibili&logoColor=white)](https://space.bilibili.com/1084866085)

作者 QQ：1492141580  
GitHub 仓库：https://github.com/AzidoPP/Simple_DCDC  
硬件部分工程：https://oshwhub.com/azidopp/SimpleDCDC
---
<img width="1062" height="617" alt="PCBA" src="https://github.com/user-attachments/assets/e31c2d4a-a3cd-4946-aa60-985c5b1a02e7" />

## 项目概述

这是一个**低成本的可调 Buck 程控/数控 电源模块**方案：在普通 Buck 芯片的反馈回路外部引入一个“可控的参考/误差放大环节”，从而实现输出电压可调。

- **模块本体**：电源硬件板
- **示例代码**：用 STM32F103C8T6 做一个最小可用的数控电源验证
<img width="1920" height="1080" alt="封面169" src="https://github.com/user-attachments/assets/e38e25b9-6a9c-4331-ad47-008aad12c97e" />

---

## 兼容的 Buck 芯片

本项目的模块思路适用于常见的固定频率/内部补偿 Buck 芯片。实测/目标芯片：

- **RT8289**`【已验证】`
(pin-to-pin的可替换芯片):
- **RT2805A**`【未验证】`
- **TPS5450**`【未验证】`

**长时间跑大电流请加散热器**
- 以上芯片均可**峰值**输出 **`5A`** 电流
- 以上芯片均可**输入 `30V` 电压**

---

# Part 1：电源模块

核心思想：**“劫持/调制 Buck 的 FB 引脚等效反馈电压”**。

- Buck 芯片内部根据 **FB 引脚电压**和内部基准（Vref_int）调整输出电压
- 通过在 FB 回路中加入运放，把“输出电压（经分压）”与“外部给定参考（Vref_equiv）”形成误差并调制到 FB 上  
- 当外部给定变化时，运放驱动 FB 偏移，Buck 会自动调整占空比/能量，直到输出稳定到新的目标值


<img width="1135" height="726" alt="演示图片1" src="https://github.com/user-attachments/assets/5cab2245-d911-4bf4-8c1d-2b99b73e76fd" />

很多 MCU 没有 DAC：可用 **PWM + RC 低通滤波** 得到近似直流的 `Vref_equiv`。

你示例中采用了 **8 倍比例**（也就是“设定端电压 × 8 ≈ 输出电压”）：

- `Vout ≈ 8 * Vref_equiv`


反馈采样 `OFB` 口
Vout反馈分压值：
- `Rtop = 30kΩ`
- `Rbot = 1.8kΩ`

<img width="540" height="327" alt="演示图片2" src="https://github.com/user-attachments/assets/db409009-bf46-43c6-8e0a-e44e1d117860" />

---

# Part 2：示例（数控DCDC电源）

代码仓库：https://github.com/AzidoPP/Simple_DCDC 

使用市售STM32F103C8T6开发板 + 1.3寸OLED搭建

**引脚接线**
**PWM 输出（模块模拟参考输入 Asig）**
- `PA8`：`TIM1_CH1` PWM 模拟 DAC

**ADC 采样**
- `PA3`：外部 **2.5V 标定参考**（用于估算 VDDA/ADC 基准，提升精度）
- `PA4`：采样 **反馈分压点 Vfb**

**显示**
1.3寸OLED 使用u8g2库驱动 SH1106 128x64 I2C
- `PB6`：SCL
- `PB7`：SDA

**按键**
按下强下拉至GND，松开则浮空
- `PC13`：UP
- `PA15`：DOWN

---
# Part 3：测试

**测试纹波**
- 空载纹波 45mV
<img width="1348" height="711" alt="空载纹波" src="https://github.com/user-attachments/assets/5de8a578-6453-4bab-84c5-e1a83aa89a84" />

- 添加8欧姆负载后，纹波150mV
<img width="1597" height="748" alt="8欧负载纹波" src="https://github.com/user-attachments/assets/8ed0dcf9-b118-4524-bb84-fec3d55008ef" />

**调压精度**

精度 `± 20mV`
- 设定7V
<img width="1768" height="677" alt="7V" src="https://github.com/user-attachments/assets/5eaa46a9-d629-4ef2-a06b-d325d92f6208" />

- 设定6V
<img width="1760" height="653" alt="6V" src="https://github.com/user-attachments/assets/cb80c1e2-743b-40eb-bc2e-868379d15f9b" />

- 设定5V
<img width="1778" height="595" alt="5V" src="https://github.com/user-attachments/assets/19732022-069d-4414-a461-416a93c69beb" />

- 设定4V
<img width="1700" height="676" alt="4V" src="https://github.com/user-attachments/assets/ccea085c-b3b1-42a1-83b9-7e8e5b605f81" />

- 设定3V
<img width="1747" height="642" alt="3V" src="https://github.com/user-attachments/assets/8b65d2cb-83ba-4b14-9466-00f66cb30b80" />

---

## 参考
ZhuoQing, “将 LM2596-5 改造成可调输出电压”, CSDN博客, May 30 2024. https://blog.csdn.net/zhuoqingjoking97298/article/details/139327600


---

## License

GPL-3.0
