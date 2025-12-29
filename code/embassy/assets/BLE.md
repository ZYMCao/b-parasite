## 0. BLE basics
* **Operating band**: 2400 MHz – 2483.5 MHz (~ 2.4 GHz)
* **Channel bandwidth**: 2 MHz
* **Number of RF channels**: 40
* **Maximum transmit power**: 20 dBm (0.1 W)
* **Maximum application data throughput**:	1.4 Mbps
* **Maximum range at reduced data rates**: (125 & 500 kbps)	~1000 m


## 1. Breaking Down the Channel Math

The available spectrum is 83.5 MHz (from 2400 MHz to 2483.5 MHz).

* **BLE Center Frequencies:** BLE channels start at **2402 MHz** and end at **2480 MHz**.
* **The 2 MHz Spacing:** Because each channel is **2 MHz wide**, the math works out perfectly:  steps, which gives you exactly **40 channels** (index 0 to 39).
* **The "Wasted" Space:** You might wonder about the remaining 3.5 MHz. The protocol leaves small **Guard Bands** (保护频带) at the very bottom (2 MHz) and top (1.5 MHz) of the band to prevent BLE signals from "bleeding" into other regulated frequency ranges.

---

## 2. Other Wireless Protocols (Competitors & Peers)

BLE is part of a large family of **Wireless Transfer Protocols** (无线传输协议). Here are the others often found in the same "neighborhood":

| Protocol | Typical Use Case | Chinese Term     |
| --- | --- |------------------|
| **Wi-Fi (802.11)** | High-speed internet, video streaming | **无线网络 (Wi-Fi)** |
| **Zigbee** | Smart home mesh (bulbs, sensors) | **紫蜂**           |
| **Thread** | Modern smart home standard (Matter) | **Thread 协议**    |
| **Classic Bluetooth** | High-quality audio, hands-free calling | **经典蓝牙**         |
| **NFC** | Contactless payments, tap-to-pair | **近场通信**         |
| **LoRaWAN** | Long-range agricultural/industrial sensors | **远距离无线电**       |

---

## 3. Why 40 Channels Matters: The Wi-Fi Problem

The reason BLE uses 40 channels instead of just one big one is **Interference** (干扰).

Wi-Fi is much "louder" and uses much wider channels (20 MHz or more). To survive, BLE uses **Adaptive Frequency Hopping** (自适应跳频). If a Wi-Fi signal is blocking channel 10, the BLE devices will simply "hop" to channel 25 and continue their conversation without the user ever noticing a drop.


## 4. Protocol Stack Architecture
![BLE Channel Spectrum](https://academy.nordicsemi.com/wp-content/uploads/2023/04/MicrosoftTeams-image-29.png)

### 1. The Application Layer

This is where the developers write their code. It uses **Profiles**, **Services**, and **Characteristics** to define what the device actually does (e.g., a heart rate monitor or a smart lock).

### 2. The Host

The Host is the logical manager of the connection. It doesn't care about radio waves; it only cares about data.

* **GATT (Generic Attribute Profile / 通用属性规范):** Defines the hierarchy of data (Services and Characteristics).
* **GAP (Generic Access Profile / 通用访问规范):** Controls how devices find each other (Advertising) and connect.
* **ATT (Attribute Protocol / 属性协议):** The "delivery man" that moves small pieces of data between devices.
* **SMP (Security Manager Protocol / 安全管理协议):** Handles pairing and encryption to keep data private.
* **L2CAP (Logical Link Control and Adaptation Protocol / 逻辑链路控制与自适应协议):** Acts as a multiplexer, taking data from different layers and packaging it for the lower levels.

### 3. Host Controller Interface (HCI / 主机控制器接口)

This is the **physical or logical divider** between the Host and the Controller. In many systems, the Host lives on a powerful application processor (like your phone's CPU), while the Controller lives on a small, dedicated radio chip. The HCI is the "bridge" that allows them to talk to each other.

### 4. The Controller

This is the "body" of the protocol that handles the hardware-intensive work.

* **Link Layer (LL / 链路层):** The "traffic cop" of the radio. It manages the state of the radio (Standing by, Scanning, or Connected) and handles timing and error checking.
* **Physical Layer (PHY / 物理层):** The actual radio hardware. It converts digital bits (0s and 1s) into 2.4 GHz radio waves and vice-versa.

## 5. GAP (Generic Access Profile / 通用访问规范)
* **Connection-oriented communication**: When there is a dedicated connection between devices, forming bi-directional communication.

![](https://academy.nordicsemi.com/wp-content/uploads/2023/03/blefund_less1_connection_topology-1.png)

| 英文术语 | 中文翻译 |
| --- | -- |
| **Advertising** | 广播 |
| **Scanning** | 扫描 |
| **Peripheral** | 外围设备 / 从机 |
| **Central** | 中央设备 / 主机 |
| **Advertising Packet** | 广播包 |
| **Advertising Interval** | 广播间隔 |
| **Scan Response** | 扫描响应 |
| **Connectable Advertising** | 可连接广播 |

* **Broadcast communication**: When devices communicate without establishing a connection first, by broadcasting data packets, and devices within range receive them.

![](https://academy.nordicsemi.com/wp-content/uploads/2023/03/blefund_less1_broadcast_topology-2.png)

| 英文术语 | 中文翻译 |
| --- | -- |
| **Broadcaster** | 广播者 |
| **Observer** | 观察者 |
| **Peripheral** | 外围设备 / 从机 |
| **Central** | 中央设备 / 主机 |


* **Technical breakdown of why a Connected Topology achieves higher throughput than a Broadcast Topology**:

### 1. Dedicated Frequency Spectrum (Data Channels)

* **Broadcast:** Limited to the **3 advertising channels** (37, 38, and 39). Because these channels are shared by every BLE device in the vicinity for discovery, the "airtime" is highly congested, forcing longer intervals between packets to avoid collisions.
* **Connected:** Moves the communication to the **37 data channels** (0–36). By using **Adaptive Frequency Hopping**, the two connected devices use the full width of the 2.4 GHz band, allowing for much more frequent transmissions without interference from "discoverable" devices.

### 2. Packet Size and Overhead (DLE)

* **Broadcast:** Standard advertising packets have a strict limit on the **PDU (Protocol Data Unit)** size, often limited to 31 bytes in legacy advertising. A significant portion of this is consumed by headers and device addresses, leaving very little room for application data.
* **Connected:** Supports **Data Packet Length Extension (DLE)**, which allows the PDU to increase from 27 bytes up to **251 bytes** per packet. This significantly reduces the ratio of "header overhead" to "actual data," effectively increasing the bits-per-second rate.

### 3. Connection Events vs. Advertising Intervals

* **Broadcast:** A broadcaster sends a packet and then must wait for the duration of the **Advertising Interval** before sending the next one. There is no mechanism to send multiple packets back-to-back in a single "burst".
* **Connected:** Uses **Connection Events**. During a single event, the Central and Peripheral can exchange **multiple packets** back-to-back as long as the hardware buffers allow. This "bursting" capability is the primary reason connected mode can reach 1.4 Mbps while broadcasting typically stays in the low kilobits.

## 6. Advertising
### 1. Advertising Channels
![Channel_Division](https://academy.nordicsemi.com/wp-content/uploads/2023/03/blefund_less1_ad_channels.png)

* Channels 37, 38, and 39, despite being consecutive numbers, are not actually neighboring channels, as you can see in the figure above. 
* The separation between the three channels serves to avoid adjacent-band interference. 
* Additionally, these three specific channels suffer the least from noise from other technologies using the ISM band, such as Wi-Fi.

### 2. Scan Interval and Scan Window
![](https://academy.nordicsemi.com/wp-content/uploads/2022/12/blefund_less2_adv_process-1024x576.png)
* **Scan interval**: The interval at which a device scans for advertisement packets.
* **Scan window**: The time that a device spends scanning for packets at each scan interval.
* Both scan window and scan interval range from 2.5 ms to 10.24 seconds with a step increase of 0.625 ms.
* The tricky part about scanning and advertising comes from the **tradeoff** between power consumption and discover probability/possibility/speed

### 3. Advertising Types

* **Advertising Property Definitions**

| Property                             | Definition                                                              | 
|--------------------------------------|-------------------------------------------------------------------------|
| **Connectable**, **Non-connectable** | whether the central can connect to the peripheral or not                |
| **Scannable**, **non-scannable**     | whether the peripheral accepts scan requests from a scanner or not      | 
| **Directed**, **undirected**         | whether advertisement packets are targeted to a specific scanner or not | 

| Type                | Connectable | Scannable | Directed | Common Use Case                                                                                                                           |
|---------------------|-------------|-----------|----------|-------------------------------------------------------------------------------------------------------------------------------------------|
| **ADV_NONCONN_IND** |             |           |          | **Beacons:** Maximum power saving; device never turns on its receiver.                                                                    |
| N/A                 | ✔           |           |          | BLE combined connectable/undirected into ADV_IND (which is always scannable) to ensure maximum device information discovery.              |
| **ADV_SCAN_IND**    |             | ✔         |          | **Data Broadcasting:** Allows scanners to request extra info without connecting.                                                          |
| N/A                 |             |           | ✔        | BLE specifications did not define a directed beacon, as beacons are intended for any observer in range.                                   |
| **ADV_IND**         | ✔           | ✔         |          | **General Discovery:** allows discovery and connection.                                                                                   |
| N/A                 |             | ✔         | ✔        | Directing a packet to a specific device usually implies a desire to connect; non-connectable directed packets have no practical use case. |
| **ADV_DIRECT_IND**  | ✔           |           | ✔        | **Fast Reconnection:** Used by devices like mice to quickly reconnect to a known PC.                                                      |
| N/A                 | ✔           | ✔         | ✔        | Directed advertising is designed for speed; adding scan response overhead contradicts the "fast" nature of directed packets.              |

### 4. Bluetooth Address

