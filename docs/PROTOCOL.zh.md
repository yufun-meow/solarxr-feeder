# SolarXR 追踪器协议（WiVRn 与 Monado 的用法）

[English](PROTOCOL.md) | 简体中文

这里每一条都是靠读 Monado 的 C 源码、以及观察运行时接受了什么才弄明白的。
写下来是因为**这些内容没有任何文档**，而且下面每一条，一开始都是以「静默失败」
的形式出现的。

## 两个 socket，都由追踪器侧创建

运行时是**主动连接**出去的。这和「服务端」这个词给人的直觉相反，
也意味着追踪器侧必须在运行时启动之前就已经在监听。

| 路径 | 方向 | 用途 |
|---|---|---|
| `$XDG_RUNTIME_DIR/SlimeVRRpc` | 运行时连进来 | 请求设置，然后等回复 |
| `$XDG_RUNTIME_DIR/SlimeVRInput` | 运行时连进来 | 发送它自己知道的设备 |

顺序是固定的：运行时先连 `SlimeVRRpc`，发一个请求，等一个 `SettingsResponse`，
然后才去连 `SlimeVRInput`。

由此带来的后果：

- **如果 `SlimeVRInput` 不存在**，运行时会打印 `failed to resolve socket path`
  并拒绝连接。只实现 RPC socket 是很容易犯的错，而这条报错并不指向真正的原因。
- **停掉再重启追踪器侧**，运行时会握着一批已经不存在的 socket 连接。
  它不会重连，所以运行时也必须一起重启。

## 设置回复里带着设备清单

回复设置请求的那个 bundle 同时包含一个带 `SettingsResponse` 的
`RpcMessageHeader`，以及一个 `DataFeedUpdate`。**运行时是从这个 update 里
枚举它的追踪器的，就在连接那一刻。**

- update 为空 → `Enumerated 0 SolarXR trackers`
- 只发送「此刻恰好被追踪到」的设备 → 其余设备在整个会话里都不存在，
  之后无论来多少位姿都没用

所以：**要把每个设备都介绍一遍**，没有位姿就用任意位姿（原点即可）。

## 会致命的字段顺序

`MessageBundle` 是：

```
table MessageBundle {
  data_feed_msgs: [DataFeedMessageHeader];   // 字段 0
  rpc_msgs:       [RpcMessageHeader];        // 字段 1
}
```

**字段 0 是 data feed。** 把 rpc 向量放进字段 0，消息仍然是合法的 flatbuffers、
仍然能被解析，然后**一个追踪器都不出来，且没有任何报错**。
这个问题是靠跟一个已知可用的编码器做字节级对比才发现的。

其他不能随便改的布局事实：

- **`DeviceData.trackers` 在 vtable 的 slot 4，即字段索引 3**，
  所以这个表必须声明成 4 个 slot。
- **`RpcMessageHeader` 读的是 vtable slot 1 和 2**，所以字段索引 0 是空的，
  索引用 1 和 2。
- **`TrackerData.status` 必须是 1**，位姿才会被采用。

## 分帧

回复带长度前缀，而**这个长度包含前缀自身的 4 个字节**：

```python
conn.sendall(struct.pack("<I", len(payload) + 4) + payload)
```

差 4 个字节，运行时就会一直等永远不会到来的数据。

**位姿更新帧同样要加这个前缀。** 直接发一个裸 flatbuffer，
接收方会把它开头的 4 个字节当成长度，然后等一个永远结束不了的帧。

## 位姿更新

每一帧发一个 `MessageBundle`，里面是一个 `DataFeedUpdate`，
它的 `devices` 向量列出**全部**设备。这个向量代表的是完整集合，不是增量，
所以某一帧里漏掉的设备只是保持上一次的位姿。

**60 Hz 很舒服**，运行时收到什么就用什么。

## 运行时自己的那个 socket

本项目**不读** `SlimeVRInput`。Monado 的 feeder 会把它的设备报告写进去，
把连接接受下来就足够让它满意了。但仍然必须把数据读掉 ——
一个不被读取的 socket 最终会填满，并把写入方堵死。

## 可见性，以及被追踪物体为什么特殊

对手柄来说，运行时会给出一组状态位，说明设备此刻是否被追踪。
**对tracked object则完全没有状态字**：唯一的信号是运行时自己那次位姿查询的返回值，
而**即使tracked object的位姿是新鲜且正在移动的，这个查询也会返回 `ovrError_LostTracking`**。

对数据源的实际影响：**用位姿本身来判断物体在不在**（时间戳是否已设置、
位置是否为非零），而不要看查询结果。否则物体会永远被报告为不可见，
而它明明工作得很好。

## 一个正确的样子

先启动 feeder，再启动运行时。健康时的样子：

```
[feeder] SlimeVRRpc listening on /run/user/1000/SlimeVRRpc
[feeder] SlimeVRInput listening on /run/user/1000/SlimeVRInput
[feeder] runtime connected
[feeder] sent SettingsResponse for left_foot, right_foot, waist
```

运行时那边则是：

```
INFO [solarxr_device_create_xdevs] Enumerated 3 SolarXR physical trackers
```

**如果这里是 0，说明设置 bundle 里的设备清单是空的，或者放错了字段。**
