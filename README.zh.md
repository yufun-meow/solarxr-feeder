# solarxr-feeder

[English](README.md) | 简体中文

把任意位姿来源喂给 WiVRn 或 Monado,变成 [SolarXR] 追踪器。

WiVRn 和 Monado 把身体追踪器当作 SolarXR 设备,并通过一对 Unix socket 发现它们。
除 SlimeVR 之外,没有别的东西实现过这个协议的追踪器侧,而且没有任何文档 ——
所有细节都得从 Monado 的 C 源码里读出来。这个项目就是那一侧的完整可用实现,
于是任何知道自己在哪的东西,都能在 Linux 上驱动 VR 里的追踪器。

关于协议的更多内容，请看docs文件夹

项目自带一个数据源:Windows 机器上的 CV1 设备组,从官方 Oculus 运行时读出位姿,
再通过网络广播过来。我知道我的这个全追方案小众又猎奇，我也没指望你真的会用这个源
但是这个项目就是从它那里分离出来的,所以实在是没有必要去把这个数据源单独删掉。
协议层并不关心追踪器位姿从哪来，因此你可以让它适配任何能追踪自身位置的东西，只需要按照下文教程写一个数据源即可。

[SolarXR]: https://github.com/SlimeVR/SolarXR-Protocol

## 它长什么样

```
   你的硬件              本程序                     运行时
  ───────────           ──────────                 ──────
                    ┌─────────────────────┐
  Windows 上的 CV1 ─┤  source: cv1_udp    │
    (UDP 包)        │        │            │
                    │        ▼            │
                    │  SolarXRService     │  SlimeVRRpc    ──►  WiVRn
                    │   (两个 socket)     │  SlimeVRInput  ──►  Monado
                    └─────────────────────┘                       │
                                                                  ▼
                                                               头显
```

运行时会把追踪器放进**头显的追踪原点**,这和任何数据源自己的空间都不是一回事,
所以需要一个变换把两者对起来。协议细节见
[docs/PROTOCOL.md](docs/PROTOCOL.md),CV1 包格式见
[docs/PACKET.md](docs/PACKET.md)。

## 环境要求

- Python 3.9 或更新
- [`flatbuffers`](https://pypi.org/project/flatbuffers/) —— `pip install flatbuffers`
- WiVRn 或 Monado,并开启原始 SolarXR 追踪器:
  `SOLARXR_RAW_TRACKERS=true wivrn-server`

## 运行

```bash
pip install flatbuffers
./feeder.py
```

默认在 UDP 39570 上收 CV1 包,以 60 Hz 把位姿推给运行时。

```
--source cv1        数据源(目前只有 cv1)
--port 39570        数据源发往的 UDP 端口
--hz 60             推送给运行时的频率
--transform PATH    映射到头显空间的变换
--mirror-port 39571 把每个包复制一份到这里,供 tools/monitor.py 使用
```

**运行时启动时必须能找到这些 socket,所以要先启动本程序。**
反过来,如果你重启了本程序,**WiVRn 也要一起重启** —— 它只连接一次,不会重试。

### 位姿从哪来

自带的数据源期望收到
[docs/reference-reader.c](docs/reference-reader.c) 产生的包 ——
那是个很小的 Windows 程序,读 Oculus 运行时并发出 104 字节的 UDP 报文。
用 MinGW 编译:

```bash
x86_64-w64-mingw32-gcc -O2 -static -o cv1-fbt-source-win64.exe docs/reference-reader.c -lws2_32
cv1-fbt-source-win64.exe --dest <本机地址>
```

你不一定非用它不可。任何能发出同样报文的程序都行,而写一个新数据源并不费事
—— 见下一节。

## 写一个数据源

继承 `TrackerSource`,为你能驱动的部位返回位姿:

```python
from solarxr_feeder.service import TrackerSource

class MyTrackers(TrackerSource):
    roles = ("left_foot", "right_foot", "waist")

    def poll(self):
        ...                     # 读入新到达的数据

    def pose(self, role):
        return (position, orientation)   # 当前没被追踪就返回 None
```

**返回 `None` 表示「这一帧没有新位姿」,服务会保持上一个。**
追踪器偶尔被遮挡就掉到地上,比站在原地不动要糟糕得多。

然后在 `feeder.py` 的 `build_source()` 里加上它,并用 `--source` 选它。

## 标定

`tools/calibrate.py` 帮你测出那个变换。它需要一个能追踪到头显手柄的 OpenXR 运行时,
以及正在运行的数据源:

```bash
./tools/calibrate.py --cv1-role left_foot --hand left
```

把那个数据源设备贴紧那个手柄,按回车,然后当成一个整体移动 25 秒。

**最容易搞错的一点是「转」。** 如果只是拿着它们在房间里走来走去而不转动,
两个设备之间的固定偏移在房间里就是恒定的,拟合会把它整个吸收进平移量:
结果是**内部完美、物理错误、而且每次都不一样**。要像拧门把手一样把它转过
180° 以上,同时走遍房间。工具会测量你转了多少,低于 `--min-turn`(默认 90°)
就拒绝保存。

握持姿势:**贴紧,不要伸到一个手臂的距离**。无法避免的那个偏移就是两个设备中心
之间的距离,它会变成结果里的一个常量误差。

一次健康的结果长这样:

```
  水平转动 178°
Samples: 24000   residual: 8.3 mm
Heading    -102.0°   (tilt fixed at 0 by gravity)
Translation +0.782 +1.751 -0.444 m
  unconstrained fit would have tilted 33.7° (residual 10.2 mm); that tilt is sensor noise
Written to ~/.config/solarxr-feeder/transform.json
```

平移里的 `+1.751` 是数据源原点离地的高度 —— 拿它当个合理性检查很有用。

## 空间之间的映射

追踪器会落在头显的追踪原点里。如果你的数据源用的是另一个原点 ——
另一个房间、另一组相机 —— 那么追踪器之间的相对位置是对的,
但相对于其他一切的位置是错的。

`--transform` 指向一个 JSON 文件:

```json
{
  "rotation":    { "x": 0.0, "y": -0.777, "z": 0.0, "w": 0.630 },
  "translation": { "x": 0.782, "y": 1.751, "z": -0.444 }
}
```

不给这个文件时,位姿原样传递,这只在两个空间本来就重合时才正确。

这个问题有个很好用的性质:**两边都用加速度计测姿态,所以两边对「哪边是下」
的判断必然一致。** 于是两个空间之间的旋转只可能是绕竖直轴的一个偏航,别的都不可能是。
一个六自由度的拟合会欣然地用传感器噪声编出一个物理上不存在的倾斜;更糟的是,
当设备只是被拿着走来走去而没被转动时,多出来的自由度没有任何东西能约束它,
每次运行的结果都会飘。

`Transform.heading_only()` 施加的就是这个约束。

## 测试

```bash
python3 tests/test_protocol.py    # 编码出的字节
python3 tests/test_solve.py       # 拟合,用已知答案的数据
python3 tests/test_service.py     # 端到端,带一个假的运行时
```

**`test_service.py` 会创建真实的 socket 路径,跑之前先停掉正在运行的 feeder。**

`test_solve.py` 不需要任何硬件:它用合成数据验证,那里面答案是已知的。
真实采集数据没有基准真值,**一个只是「自洽」的拟合可以看起来完美,却仍然是错的**。

## 状态

在 CachyOS 上、NVIDIA 显卡、Quest 3S 通过 WiVRn 26.9,以三个追踪器验证过。
其他运行时应该也能用;WiVRn 底下用的就是 Monado。

已知的粗糙之处:

- **CV1 数据源对「tracked object」做了特殊处理。** 运行时对它自己的那次查询会返回
  `LostTracking`,即使位姿是新鲜的也一样,所以没有可靠的可见性标志,
  只能由位姿本身来决定。
- **`sources/cv1_udp.py` 里只实测过列出的那几个部位。** 其他部位能编码,但没验证过。

## 许可

MIT,见 [LICENSE](LICENSE)。
