#!/usr/bin/env python3
"""CAN 协议层测试：CRC-8/ATM 校验向量 + pack/parse 回环 + 篡改拦截。

与 components/can_protocol.c 的实现一一对应（协议 v2）：
  控制帧 0x101：motor(2) + servo(2) + number(1) + gimbal_state(1) + version(1) + crc(1)
  反馈帧 0x201：echo(2) + rpm(2) + number(1) + chassis_state(1) + version(1) + crc(1)
"""


def crc8(data, length):
    """标准 CRC-8/ATM：poly 0x07, init 0x00, MSB-first, 无异或输出。
    校验向量 "123456789" 应输出 0xF4。"""
    crc = 0x00
    for i in range(length):
        crc ^= data[i]
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def pack_control_frame(motor, servo, number, state, version):
    tx = bytearray(8)
    tx[0] = motor & 0xFF
    tx[1] = (motor >> 8) & 0xFF
    tx[2] = servo & 0xFF
    tx[3] = (servo >> 8) & 0xFF
    tx[4] = number
    tx[5] = state
    tx[6] = version
    tx[7] = crc8(tx, 7)
    return bytes(tx)


def parse_control_frame(rx):
    if crc8(rx, 7) != rx[7]:
        return None
    motor = rx[0] | (rx[1] << 8)
    if motor >= 0x8000:
        motor -= 0x10000
    servo = rx[2] | (rx[3] << 8)
    if servo >= 0x8000:
        servo -= 0x10000
    return (motor, servo, rx[4], rx[5], rx[6])


def pack_feedback_frame(echo, rpm, number, state, version):
    tx = bytearray(8)
    tx[0] = echo & 0xFF
    tx[1] = (echo >> 8) & 0xFF
    tx[2] = rpm & 0xFF
    tx[3] = (rpm >> 8) & 0xFF
    tx[4] = number
    tx[5] = state
    tx[6] = version
    tx[7] = crc8(tx, 7)
    return bytes(tx)


def parse_feedback_frame(rx):
    if crc8(rx, 7) != rx[7]:
        return None
    echo = rx[0] | (rx[1] << 8)
    if echo >= 0x8000:
        echo -= 0x10000
    rpm = rx[2] | (rx[3] << 8)
    if rpm >= 0x8000:
        rpm -= 0x10000
    return (echo, rpm, rx[4], rx[5], rx[6])


FAIL = 0


def check(name, cond):
    global FAIL
    print("%s: %s" % ("PASS" if cond else "FAIL", name))
    if not cond:
        FAIL += 1


def main():
    # 1. 标准校验向量（与所有 CRC 计算器可互验）
    check("CRC-8/ATM check(123456789)=0xF4",
          crc8(b"123456789", 9) == 0xF4)

    # 2. 控制帧回环（含 int16 边界与负数）
    cases = [
        (0x1234, -100, 0, 0x03, 1),
        (32767, -32768, 255, 0xFF, 1),
        (0, 0, 0, 0, 1),
        (-32768, 32767, 42, 0x06, 1),
        (500, 300, 1, 0x02, 1),
    ]
    for c in cases:
        check("ctrl roundtrip %s" % (c,),
              parse_control_frame(pack_control_frame(*c)) == c)

    # 3. 反馈帧回环
    for c in cases:
        check("fb roundtrip %s" % (c,),
              parse_feedback_frame(pack_feedback_frame(*c)) == c)

    # 4. 篡改拦截（CRC 不符必须拒收）
    frame = bytearray(pack_control_frame(500, 300, 1, 0x02, 1))
    frame[2] ^= 0x40
    check("ctrl tamper rejected", parse_control_frame(bytes(frame)) is None)

    frame = bytearray(pack_feedback_frame(500, 300, 1, 0x02, 1))
    frame[0] ^= 0x01
    check("fb tamper rejected", parse_feedback_frame(bytes(frame)) is None)

    # 5. CRC 对每字节翻转敏感（单比特错误 100% 检出验证抽样）
    base = pack_control_frame(0x1234, -100, 0, 0x03, 1)
    all_catch = all(
        parse_control_frame(bytes(base[:i] + bytes([base[i] ^ (1 << b)]) + base[i + 1:]))
        is None
        for i in range(7) for b in range(8)
    )
    check("all single-bit errors caught (56/56)", all_catch)

    print("ALL %s" % ("PASS" if FAIL == 0 else "FAIL(%d)" % FAIL))
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
