# CV1 位姿包格式

[English](PACKET.md) | 简体中文

自带的数据源（`sources/cv1_udp.py`）读取这个格式。它由
[docs/reference-reader.c](reference-reader.c) 发出，每帧一个 UDP 报文，
**104 字节，小端**，最高 240 Hz。

## 布局

```c
struct cv1fbt_packet {
    unsigned int   magic;         // 0x54424631, "1FBT"
    unsigned char  version;       // 1
    unsigned char  flags;
    unsigned short reserved;
    double         time_seconds;  // 采样时的 ovr_GetTimeInSeconds

    cv1fbt_pose    left_foot;
    cv1fbt_pose    right_foot;
    cv1fbt_pose    waist;

    unsigned short valid;         // 第 i 位: 第 i 个位姿曾被看到过
    unsigned short visible;       // 第 i 位: 设备 i 此刻正被看到
};

struct cv1fbt_pose {              // 28 字节
    float qx, qy, qz, qw;         // 朝向
    float px, py, pz;             // 位置,单位米
};
```

**`reserved` 是 `unsigned short`，不是 `unsigned char`。** 加上 `double`
要求的对齐，头部正好 16 字节：

```
0  magic          4
4  version        1
5  flags          1
6  reserved       2
8  time_seconds   8
16 poses          84  (3 × 28)
100 valid         2
102 visible       2
104 合计
```

写成 Python 的格式串：

```python
PACKET = struct.Struct("<IBBHd" + "7f" * 3 + "HH")
```

## 两种「不在」

这个格式把两件容易被混为一谈的事分开了：

- **`valid`** —— 这个设备曾经被看到过，它的位姿是它真实待过的一个位置
- **`visible`** —— 这个设备此刻正被看到

设备离开视野后**仍然继续发送**，只是 `visible` 清零、位姿保持最后已知的那个。
这样它会停在原地，而不是瞬间跳到原点 —— 这才是追踪器该有的行为。

位置恰好是 `(0, 0, 0)` 表示运行时根本没法定位这个设备。
**它会被过滤掉而不是被接受**，因为戴在身上的追踪器不可能正好在原点，
接受它会把追踪器拽到地板上。

## 位掩码

| 位 | 设备 |
|---|---|
| 0 | 左脚 |
| 1 | 右脚 |
| 2 | 腰 |

## 腰没有可见性标志

读取器**没法**为vr object真实地填好 `visible`。运行时对自己那次位姿查询
会返回 `LostTracking`，**即使obj被完美追踪着也一样**，所以这个标志没有任何依据。

因此自带的数据源采取的办法是：**只要物体有位姿（也就是不是（0,0,0）），就认为它可见。**

发送方当然也可以照常设置这一位；接收方对这个设备会忽略它。

## 参考读取器的命令行

```
cv1-fbt-source.exe [--dest 192.168.1.50] [--port 39570] [--hz 240] [--quiet]
```

## 顺便一提

真的有人会和我一样单独另开一台windows来追踪这玩意吗？？
我也试过很多在linux上追踪cv1的方案 并做了大量努力让他们支持vr object，他们确实也能追踪，但是追踪质量非常差，实在是用不了，只能这样了

算了，我开源这玩意就是为了让你们想拿什么当追踪器自己写，所以，交给你啦（笑）
