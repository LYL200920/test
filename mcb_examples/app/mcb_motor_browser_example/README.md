# MCB Motor Browser：二进制只读服务与轴诊断

本应用保留现有 HTTP 马达浏览器，在 `MCB_USE_MCX514` 构建中增加兼容 `machine_controller/mcb_cmd` 的 TCP 55000 服务和 HTTP 只读错误来源诊断。本阶段只实现设备探测、轴数据查询和诊断，不能通过二进制协议驱动 DD 马达。

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

## 四轴错误来源诊断

用户已实测第一阶段的 gantry0 探测及四轴查询。四轴 `status=0x18` 表示 **stopped（0x10）+ error（0x08）**，不是运动就绪。本次增量用于查明 error 的来源，不自动清错、修改极性、使能、停止或回零；原有 HTTP 运动入口仍然保留。

新增入口：`GET /dd_motor_diagnostics?axis=0`，轴索引 0–3 对应 X/Y/Z/U。每个请求只读一轴，HTTP 和 TCP 使用同一后端及状态映射。仅接受一个规范形式的 `axis=0`、`axis=1`、`axis=2` 或 `axis=3`；缺失、重复、未知参数、编码形式和非法值返回 400，不读取硬件。非 GET 返回 405（`Allow: GET`；HEAD 按 HTTP 规则仅返回头、不发送正文），不支持的构建或读取/诊断数据不可用返回 503，序列化容量不足返回 500。响应为 `application/json`，带 `Cache-Control: no-store`。

成功 JSON 的 `schema_version` 为 1：

- `axis_index` / `axis_name` / `axis_mask`：轴身份；`binary_status` 是整数形式的现有二进制状态字节。
- `logical_position` / `encoder_position`：有符号原始计数；`speed_pps`：非负当前速度。
- `moving` / `homing` / `error` / `positive_limit` / `negative_limit`：布尔汇总。`home` 和 `homing_ok` 均为 `null`，物理原点及回零完成状态未知。
- `rr0_raw` / `rr2_raw`：16 位原始寄存器值。
- `rr3_decoded`：**不是原始 RR3**。`signal_status()` 已按驱动配置处理极性、编码器和限位交换；`rr3_interpretation` 明确标为 `driver_polarity_and_swap_normalized`。
- `home_search_state`：RR3 自动回零状态号；0 仅表示 idle，不证明成功回零。
- `error_sources`：下表七个布尔值的 OR，严格保持原有二进制 error 定义。
- `drive_flags`：RR2 的 alarm、软/硬限位停止、同步停止、STOP0/1/2 停止及正/负限位停止标志；`signal_inputs`：归一化 RR3 的 STOP0/1/2、编码器 A/B 和 in_position。

| error_sources 字段 | 采样来源 |
| --- | --- |
| `axis_error` | RR0 当前轴 error 位（bit 4+axis） |
| `alarm_input` | 归一化 RR3 bit 6 |
| `home_error` | RR2 bit 6 |
| `interpolation_error` | RR2 bit 7 |
| `emergency_input` | RR2 bit 5 |
| `emergency_stop` | RR2 bit 15 |
| `alarm_stop` | RR2 bit 14 |

RR2 的 `alarm`（bit 4）另行报告，不增加到原有七项 OR。RR2 的停止原因可能是保留的历史/锁存状态，不能与 RR3 的实时归一化限位、报警输入混为一谈。本入口不读会确认/清除 IRQ 的 RR1，不调用 finish/error 清理方法。读取寄存器需要发送选择/读寄存器命令，但不改变运动控制、计数器或参数。寄存器顺序采样不是原子快照，不把某个标志直接解释为接线损坏或安全许可。

### 上板验收

先自行烧录本次新生成的 MOT。旧固件没有此 HTTP 路由，仅 TCP 查询成功不能证明诊断接口已更新。在 `test` 仓库根目录运行（将 `<MCB-IP>` 替换为实际地址）：

```sh
python3 mcb_examples/app/mcb_motor_browser_example/tools/mcb_diagnostics.py <MCB-IP>
```

默认顺序读取四轴；可指定轴、超时和完整 JSON：

```sh
python3 mcb_examples/app/mcb_motor_browser_example/tools/mcb_diagnostics.py <MCB-IP> --axis 0 --timeout 3
python3 mcb_examples/app/mcb_motor_browser_example/tools/mcb_diagnostics.py <MCB-IP> --json
```

脚本只用 Python 标准库，只发送该诊断入口的 GET；不会跟随重定向、使用环境代理或发送控制请求。响应大小和等待时间受限，版本/类型/位映射严格校验；HTTP 错误、旧固件、连接失败或畸形数据非零退出。默认端口 80，必要时可用 `--port` 指定。用户外部 `test_py/mcb_test.py` 无需修改。

重点保存四轴 `error_sources`、RR0/RR2 和 `rr3_decoded` 输出，再结合驱动器报警、MCB 接线和既有极性配置排查。即使诊断没有 error，也不代表已具备运动条件；共用控制权、安全停止、网络掉线保护和有限运动仍是后续工作。

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

准备好 Python3 时共有六个测试目标（未找到时 CMake 明确提示跳过 Python 项）：

1. `mcb_binary_service_tests`：四轴查询、负位置边界、状态、拒绝请求、固定/变长拆包粘包、预算、缓冲与时间边界。
2. `mcb_binary_server_tests`：实际适配器配合假 socket，测试部分/零发送、接收挂起、异步关闭、进展超时和重连。
3. `mcb_uip_ack_tests`：直接编译真实 `uip.c`，构造本地 TCP 报文验证旧 ACK 不释放输出、覆盖 ACK 正确释放输出。不发送网络报文。
4. `mcb_binary_client_tests`：真实 `li5000_mcb_cmd::client` 与假轴服务在 127.0.0.1 临时端口通信，验证探测、X/Y/Z/U、拒绝写命令、批量请求和重连。客户端测试需要 POSIX。
5. `mcb_axis_diagnostics_tests`：七个错误来源的全部组合、状态与实际二进制响应一致性、四轴/回零状态、输入校验、缺失/失败诊断、极值、有界 JSON 与缓冲复用。
6. `mcb_diagnostics_python_tests`：解析 C++ 实际 JSON 样本验证版本、类型及位映射；通过本机假 HTTP 服务测试脚本的 GET-only、超时、错误/畸形/超长响应及拒绝重定向。

客户端测试引用已存在的 `3rd/machine_controller/mcb_cmd` 和旁边的 `utils`；不下载依赖、不修改第三方库。不同目录可设置 `-DMCB_CMD_SOURCE_DIR=/absolute/path/to/mcb_cmd`。依赖未准备好时，可用 `-DBUILD_MCB_CLIENT_TESTS=OFF` 跳过真实客户端目标，其余核心和诊断目标仍可运行。

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

上述测试均使用假轴，不测试实际 MCX 寄存器、以太网设备或板端 HTTP 并发。Python 假 HTTP 服务验证的是脚本和实际 C++ JSON，不运行整个固件 HTTP server；板端路由接线通过 RX 交叉编译验证，完整上板通信仍需用户验收。假 socket 不是完整 TCP 模拟；真实 uIP 报文测试仅覆盖 ACK 回归路径。

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
