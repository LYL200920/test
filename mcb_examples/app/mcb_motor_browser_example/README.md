# MCB Motor Browser：二进制只读服务

本应用保留现有 HTTP 马达浏览器，在 `MCB_USE_MCX514` 构建中增加兼容 `machine_controller/mcb_cmd` 的 TCP 55000 服务。本阶段只实现设备探测和轴数据查询，不能通过二进制协议驱动 DD 马达。

## 端口和支持范围

- HTTP：80，现有路由、DD 运行参数、软件零点及运动逻辑保持不变。
- 二进制 TCP：55000，仅允许一个会话，不需要登录或握手。
- gantry0：MCX514 的物理轴 0–3。

| 轴 | MCX514 索引 | 协议掩码 |
| --- | --- | --- |
| X | 0 | 1 |
| Y | 1 | 2 |
| Z | 2 | 4 |
| U | 3 | 8 |

| 命令 | ID | 掩码规则 | 返回值 |
| --- | --- | --- | --- |
| `gantry_check_st` | `0x000301` | 0–15 | 状态位；多轴按位 OR |
| `gantry_get_pos` | `0x000306` | 单轴 | 有符号逻辑位置的 32 位位模式 |
| `gantry_get_enc_pos` | `0x000307` | 单轴 | 有符号编码器位置的 32 位位模式 |
| `gantry_get_cur_speed` | `0x000507` | 单轴 | 当前速度，单位 pps |

状态查询掩码 0 返回成功且状态为 0，以兼容 gantry0 枚举。gantry1–15、所有 conveyor、未实现命令均返回 `invalid_request`。位置和编码器计数是原始脉冲计数，不是角度或毫米，也不减去 HTTP 的软件零点；负值不取绝对值。

`gantry_mcb::axis_count()` 在现有高层库中仍固定为 3。这里支持低层 `mcb_cmd` 的 U 轴请求，但尚未修改高层全轴 API，也没有将 `machine_controller` 链接进桌面 `test` 应用。设备枚举成功不代表整机、IO、触发器和点胶机 API 可用。

**全部二进制修改命令均拒绝**，包括 servo on/off、运动、停止、回零、复位、计数器/FIFO 清理、参数和输出修改。二进制 stop 也没有开放；不能将此服务当作停止或急停通道。只读后端接口没有硬件写方法。

## 帧和状态语义

普通请求/响应固定为 8 字节：

- byte0：支持的请求为 0。
- byte1：高半字节为轴掩码，低半字节为命令族。
- byte2：高半字节为 gantry 编号，低半字节为操作。
- 成功响应 byte1/byte2 保留请求身份；状态位放在 byte3，数字放在 byte4–7（大端序）。
- 无效/不支持响应：`FF 00 00 00 00 00 00 00`。
- 后端读取失败响应：`FE 00 00 00 00 00 00 00`。
- byte0 高半字节为 `E` 的变长帧：前 4 字节为头，byte2/byte3 是大端 payload 长度（不含头）。服务消费完整有界帧后返回不支持，不把 payload 当作普通命令。

状态位映射：

| bit | 含义 | 来源 |
| --- | --- | --- |
| 7 | home | 本阶段固定 false，物理原点状态未知 |
| 6 | homing | RR3 `home_search_state()!=0` |
| 5 | moving | RR0 轴驱动状态 |
| 4 | stopped | 非驱动且非回零中 |
| 3 | error | RR0/RR2/RR3 报警及驱动错误 |
| 2 | homing_ok | 本阶段固定 false，没有可信回零完成记录 |
| 1 | positive limit | RR3 正限位 |
| 0 | negative limit | RR3 负限位 |

零计数、原点输入和 HTTP 软件零点均不证明物理回零成功。多轴状态是 OR，可能同时包含 moving 和 stopped；stopped 为 true 不意味着所有选中轴都停止。寄存器按顺序采样，不保证跨寄存器的原子快照。只读查询不读取会确认/清除 IRQ 的 RR1。

## 会话限制和生命周期

- 固定 RX：3072 字节；固定 TX：128 字节（16 个响应）。
- 变长 payload 上限：1024 字节。超长帧、RX 溢出直接关闭连接。
- RX 容量可容纳最大残帧和一个 1540 字节 uIP 包，避免分片在接收挂起后无法补全。
- 每次主循环最多处理 4 帧、向 socket 提交最多 128 字节；部分/零发送保留未接纳的数据。
- 残帧超时：5 秒，从该帧首字节到达计算，续传不刷新期限；后续残帧不继承前面完整帧的旧时间。
- 无发送进展超时：10 秒；分别检查核心队列和 socket 未确认字节。
- 空闲超时：60 秒。该客户端没有自动 heartbeat，长时间闲置后应重连。
- 收发回调不进行硬件查询、响应分派或对象销毁；处理和清理都在已有单线程主循环中进行。
- uIP close 是异步的。超时后拒绝新输入，但保留 socket，直到栈报告 closed 才销毁；期间新的二进制连接会被拒绝。底层重传/FIN 超时可能使释放晚于服务的 10 秒判定。
- uIP 不支持 TCP 半关闭。客户端必须保持完整连接，收齐响应后再 disconnect，不能 `shutdown(SHUT_WR)` 后等待最终响应。
- 服务使用固件全生命周期静态对象；不能在网络仍运行且 socket 尚在 closing 时销毁服务。

uIP 的 ACK 回调增加了实际 `UIP_ACKDATA` 条件判断，防止尚未确认响应被新请求错误释放。关闭等待计时判断屏蔽 `UIP_STOPPED` 标志，并在过期时通知 socket 所有者，避免关闭后永久占用会话。适配器还忽略接收已挂起时 ACK 路径附带的未接纳 payload，让 TCP 恢复后重传，避免重复分派。这些修正应用于共享 TCP 栈，但没有更改 HTTP 路由或马达控制逻辑。

本服务不提供认证或加密，应仅部署在受控设备网络。当前 HTTP 入口仍可以运动；新增服务没有修复原有的忙等待、网络掉线运动保护或跨入口控制权问题。后续开放运动前，需要统一运动控制、安全停止及看门狗。

## 本机测试

在 `test` 仓库根目录执行，无需 wxWidgets、VTK、相机 SDK 或 RX 工具链：

```sh
cmake -S mcb_examples/app/mcb_motor_browser_example/tests \
  -B mcb_examples/build/binary-host-tests
cmake --build mcb_examples/build/binary-host-tests -j 8
ctest --test-dir mcb_examples/build/binary-host-tests --output-on-failure
```

四个测试目标：

1. `mcb_binary_service_tests`：四轴查询、负位置边界、状态、拒绝请求、固定/变长拆包粘包、预算、缓冲与时间边界。
2. `mcb_binary_server_tests`：实际适配器配合假 socket，测试部分/零发送、接收挂起、异步关闭、进展超时和重连。
3. `mcb_uip_ack_tests`：直接编译真实 `uip.c`，构造本地 TCP 报文验证旧 ACK 不释放输出、覆盖 ACK 正确释放输出。不发送网络报文。
4. `mcb_binary_client_tests`：真实 `li5000_mcb_cmd::client` 与假轴服务在 127.0.0.1 临时端口通信，验证探测、X/Y/Z/U、拒绝写命令、批量请求和重连。客户端测试需要 POSIX。

客户端测试引用已存在的 `3rd/machine_controller/mcb_cmd` 和旁边的 `utils`；不下载依赖、不修改第三方库。不同目录可设置 `-DMCB_CMD_SOURCE_DIR=/absolute/path/to/mcb_cmd`。依赖未准备好时，可用 `-DBUILD_MCB_CLIENT_TESTS=OFF` 仅运行其余三个目标。

可选 ASan/UBSan：

```sh
cmake -S mcb_examples/app/mcb_motor_browser_example/tests \
  -B mcb_examples/build/binary-host-sanitizers \
  '-DCMAKE_C_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer -g' \
  '-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer -g' \
  '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined'
cmake --build mcb_examples/build/binary-host-sanitizers -j 8
ctest --test-dir mcb_examples/build/binary-host-sanitizers --output-on-failure
```

上述测试均使用假轴，不测试实际 MCX 寄存器、以太网设备或 HTTP 并发。假 socket 不是完整 TCP 模拟；真实 uIP 报文测试仅覆盖 ACK 回归路径。

## RX71M / MCB v2 构建

本机工具链位于 `/opt/jz/gcc8-190509/rx-elf`。在仓库根目录执行：

```sh
ROOT="$(pwd)"
cmake -S "$ROOT/mcb_examples/app/mcb_motor_browser_example" \
  -B "$ROOT/mcb_examples/build/binary-rx" \
  -DCMAKE_TOOLCHAIN_FILE="$ROOT/mcb_examples/toolchain/rx/toolchain.cmake" \
  -DCMAKE_BOARD_DIR="$ROOT/mcb_examples/board/mcb_v2" \
  -DBOARD_VARIANT=RX71M_176 \
  -DBOARD_USE_MCX512=OFF \
  -DBOARD_USE_MCX514=ON \
  -DCMAKE_LIBRARIES_DIR="$ROOT/mcb_examples"
cmake --build "$ROOT/mcb_examples/build/binary-rx" \
  --target mcb_motor_browser_example.mot -j 8
```

该目标同时构建 ELF 并生成 MOT：

- `mcb_examples/build/binary-rx/mcb_motor_browser_example.elf`
- `mcb_examples/build/binary-rx/mcb_motor_browser_example.mot`

使用新的 build 子目录，不复用移动前绝对路径写入的旧缓存。构建输出在 Git 忽略目录内。不要运行 `upload_tftp`/`upload_tftp2` 目标来做本机验证；这些目标会连接和上传到设备。本阶段没有连接真实 MCB、烧录或执行硬件动作，上板并发和实轴验证仍待完成。
