#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
telemetry_monitor.py  ——  智能小车 $CAR 遥测帧上位机

配合 XunjiCar 固件使用：单片机每 100 ms 从 USART2(PA2=TX, PA3=RX, 115200 8N1)
发出一行 ASCII 帧：

    $CAR,N=123,TRK=1F,B=11101111,D1=235,D2=412,AX=1234,...,T=25,
         S=FWD,MA=+30,EA=+3100,MB=+0,EB=+0*7E

本工具做的事：
    1. 打开串口，按行收帧；
    2. 校验帧尾的 XOR 校验和（$ 与 * 之间所有字符异或）；
    3. 把字段翻译成人能看的一行：8 路循迹用方块画出来、距离、姿态、电机、编码器；
    4. 一边显示一边存 CSV，方便事后画曲线（Excel 直接能开）；
    5. 统计帧率、坏帧数，用来判断"线接得对不对、固件活没活着"。

用法（在 xunjicar 目录下执行）：

    # 1) 先看看电脑上有哪些串口
    python tools/telemetry_monitor.py --list

    # 2) 正常监视（把 COM5 换成你的 USB-TTL 串口号）
    python tools/telemetry_monitor.py --port COM5

    # 3) 同时存 CSV
    python tools/telemetry_monitor.py --port COM5 --csv run1.csv

    # 4) 没有硬件也想先验证脚本本身能用
    python tools/telemetry_monitor.py --selftest

依赖：pyserial（pip install pyserial）
"""

import argparse
import sys
import time
from datetime import datetime

# ---------------------------------------------------------------------------
# 依赖检查
# ---------------------------------------------------------------------------

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    print("缺少依赖 pyserial，请先执行：  python -m pip install pyserial")
    sys.exit(2)


# ---------------------------------------------------------------------------
# 帧解析
# ---------------------------------------------------------------------------

def xor_checksum(payload: str) -> int:
    """$ 与 * 之间所有字符逐一异或（跳过开头的 $）"""
    x = 0
    for ch in payload[1:]:
        x ^= ord(ch)
    return x


def parse_frame(line: str):
    """解析一行遥测帧。

    返回 (ok, fields, note)：
        ok     True 表示帧头/帧尾/校验和都对
        fields 字段字典，例如 {'N': '12', 'TRK': '1F', 'B': '11101111', ...}
        note   出错原因（ok=True 时为 空串）
    """
    line = line.strip()
    if not line:
        return False, {}, "空行"

    if not line.startswith("$"):
        return False, {}, "不是以 $ 开头（可能波特率不对或线接错）"

    if "*" not in line:
        return False, {}, "没有校验和（帧被截断？）"

    body, _, tail = line.rpartition("*")
    tail = tail.strip()
    if len(tail) < 2:
        return False, {}, "校验和字段不完整"

    try:
        want = int(tail[:2], 16)
    except ValueError:
        return False, {}, "校验和不是十六进制"

    got = xor_checksum(body)
    if want != got:
        return False, {}, "校验和错（想 %02X 实得 %02X）" % (want, got)

    if not body.startswith("$CAR"):
        return False, {}, "不是 $CAR 帧：%s" % body[:12]

    fields = {}
    for item in body[1:].split(","):
        if "=" in item:
            k, _, v = item.partition("=")
            fields[k.strip()] = v.strip()

    return True, fields, ""


def track_bar(bits: str) -> str:
    """8 位循迹状态画成条形图：固件里 1 = 白底，0 = 压到黑线。"""
    chars = []
    for b in bits:
        if b == "0":
            chars.append("█")   # 压线
        elif b == "1":
            chars.append("·")   # 白底
        else:
            chars.append("?")
    return "".join(chars)


def fmt_value(raw, suffix=""):
    if raw in (None, "", "NA"):
        return "NA"          # 模块没应答
    if raw == "---":
        return "---"         # 模块正常但这次没回波
    if raw in ("ERR", "--------"):
        return "ERR"         # 没收到数据 / 超时
    return raw + suffix


def format_line(fields: dict) -> str:
    """把字段字典拼成一行人类可读文本。"""
    parts = []

    parts.append("N=%-5s" % fields.get("N", "?"))

    # ---- 循迹 ----
    if "B" in fields:
        bits = fields["B"]
        if len(bits) == 8 and set(bits) <= {"0", "1"}:
            n_black = bits.count("0")
            parts.append("循迹[%s] %d路压线" % (track_bar(bits), n_black))
        else:
            parts.append("循迹[%s]" % bits)
    elif fields.get("TRK") == "ERR":
        parts.append("循迹[ERR:模块没数据]")

    # ---- 测距 ----
    # D1/D2 是固件里固定的路号 (= I2C1 / I2C3), 与左右无关;
    # 这里的中/左/右照 line_follow.h 的 LF_TOF_LEFT / LF_TOF_RIGHT 来, 已装车实测确认:
    #   D1 = 左边模块, D2 = 右边模块。
    # 以后若把两个模块换了位置: 改 line_follow.h 那两行 + 把下面两个名字对调。见手册 15.1。
    dists = []
    for key, name in (("D1", "左"), ("D2", "右")):
        if key in fields:
            dists.append("%s%s" % (name, fmt_value(fields[key], "mm")))
    if dists:
        parts.append("测距 " + " ".join(dists))

    # ---- IMU ----
    if "IMU" in fields:
        parts.append("IMU:NA(没应答)")
    elif "AZ" in fields:
        parts.append("AZ=%s T=%sC" % (fields.get("AZ", "?"), fields.get("T", "?")))

    # ---- 循迹控制环 ----
    if "LF" in fields:
        lf = "控制=%s" % fields.get("LF", "?")
        if "ERR" in fields:
            lf += " 偏差=%s" % fields["ERR"]
        if "ST" in fields:
            lf += " 差速=%s%%" % fields["ST"]
        if "GZD" in fields:
            try:
                lf += " 角速度=%.1fdps" % (int(fields["GZD"]) / 10.0)
            except ValueError:
                pass
        if "YAW" in fields:
            try:
                yaw = int(fields["YAW"]) / 10.0
                if abs(yaw) >= 0.5:
                    lf += " 转角=%.1f度" % yaw
            except ValueError:
                pass
        if fields.get("IMS") == "0" and "AX" in fields:
            lf += " [陀螺仪无效:没校准或没接]"
        parts.append(lf)

    # ---- 电机 ----
    if "S" in fields:
        motor = "电机 %s MA=%s%% EA=%s MB=%s%% EB=%s" % (
            fields.get("S", "?"),
            fields.get("MA", "?"), fields.get("EA", "?"),
            fields.get("MB", "?"), fields.get("EB", "?"),
        )
        parts.append(motor)

    return "  ".join(parts)


# ---------------------------------------------------------------------------
# 自检（不需要硬件）
# ---------------------------------------------------------------------------

SELFTEST_FRAMES = [
    "$CAR,N=1,TRK=1F,B=11101111,D1=235,D2=412,AX=12,AY=-5,AZ=16384,"
    "GX=1,GY=-4,GZ=8,T=25,S=MANUAL,MA=+30,EA=+3100,MB=+0,EB=+0,"
    "LF=RUN,ERR=-125,ST=+6,GZD=-82,YAW=+0,IMS=1",
    "$CAR,N=2,TRK=ERR,B=--------,D1=NA,D2=---,IMU=NA,S=MANUAL,MA=+0,EA=+0,"
    "MB=+0,EB=+0,LF=BLOCK,ERR=+0,ST=+0,GZD=+0,YAW=+0,IMS=0",
]


def run_selftest():
    print("== 自检：用两条合成帧验证解析和校验和 ==")
    for body in SELFTEST_FRAMES:
        line = "%s*%02X" % (body, xor_checksum(body))
        ok, fields, note = parse_frame(line)
        print("原始: %s" % line)
        print("解析: ok=%s %s" % (ok, format_line(fields) if ok else note))
        print("-" * 70)

    # 故意改坏一个字符，应该报校验和错
    bad = "$CAR,N=3,TRK=1F,B=11101111,D1=9999*00"
    ok, _, note = parse_frame(bad)
    print("坏帧测试: ok=%s (%s)" % (ok, note))
    print("== 自检结束：上面两条正常帧能翻译、坏帧能被拦下，就说明脚本没问题 ==")


# ---------------------------------------------------------------------------
# 串口主循环
# ---------------------------------------------------------------------------

ALARM_HINTS = {
    "TRK=ERR": "循迹模块没数据：查模块 5V/GND、TX->PA10、RX->PA9、波特率是否 115200",
    "D1=NA":   "TOF1 没应答：查 I2C1 的 PB6(SCL)/PB7(SDA)、VCC 是否 3.3V、是否 VL53L1X",
    "D2=NA":   "TOF2 没应答：查 I2C3 的 PA8(SCL)/PC9(SDA)、VCC 是否 3.3V",
    "IMU=NA":  "MPU6050 没应答：查 I2C2 的 PB10(SCL)/PB11(SDA)、AD0 是否接 GND",
}


def monitor(port, baud, csv_path, duration, show_bad):
    ser = serial.Serial(port, baud, timeout=1.0)
    print("已打开 %s @ %d，等待 $CAR 帧……（Ctrl+C 退出）" % (port, baud))
    print("提示：接好 USB-TTL 后，正常应每秒看到约 10 行。")
    print("=" * 100)

    csv_file = None
    if csv_path:
        csv_file = open(csv_path, "w", encoding="utf-8-sig", newline="")
        csv_file.write(
            "pc_time,seq,track_hex,bits,black_count,d1_mm,d2_mm,"
            "ax,ay,az,gx,gy,gz,temp_c,step,ma,ea,mb,eb,lf,err,steer,gzd,yaw,imu_ok\n"
        )
        print("CSV 记录中 -> %s" % csv_path)

    good = 0
    bad = 0
    seen_hints = set()
    t_start = time.time()
    t_last_report = t_start

    try:
        while True:
            if duration and (time.time() - t_start) >= duration:
                break

            raw = ser.readline()
            if not raw:
                continue

            text = raw.decode("ascii", errors="replace").strip()
            if not text:
                continue

            ok, fields, note = parse_frame(text)

            if not ok:
                if text.startswith("$"):
                    bad += 1
                    if show_bad:
                        print("[坏帧] %-60s  %s" % (text[:60], note))
                else:
                    # 不是 $CAR 帧：可能是引脚扫描固件的打印、或模块自己吐的原始数据，
                    # 这些也要原样显示出来（排查接线时很有用）
                    print("[其他] %s" % text)
                continue

            good += 1

            # ---- 首次出现某类故障时给一次接线提示 ----
            for token, hint in ALARM_HINTS.items():
                if token in text and token not in seen_hints:
                    seen_hints.add(token)
                    print("!! 提示：%s" % hint)

            print("[%s] %s" % (datetime.now().strftime("%H:%M:%S"), format_line(fields)))

            if csv_file:
                csv_file.write(",".join([
                    datetime.now().isoformat(timespec="milliseconds"),
                    fields.get("N", ""),
                    fields.get("TRK", ""),
                    fields.get("B", ""),
                    str(fields.get("B", "").count("0")) if fields.get("B") else "",
                    fields.get("D1", ""), fields.get("D2", ""),
                    fields.get("AX", ""), fields.get("AY", ""), fields.get("AZ", ""),
                    fields.get("GX", ""), fields.get("GY", ""), fields.get("GZ", ""),
                    fields.get("T", ""), fields.get("S", ""),
                    fields.get("MA", ""), fields.get("EA", ""),
                    fields.get("MB", ""), fields.get("EB", ""),
                    fields.get("LF", ""), fields.get("ERR", ""), fields.get("ST", ""),
                    fields.get("GZD", ""), fields.get("YAW", ""), fields.get("IMS", ""),
                ]) + "\n")
                csv_file.flush()

            # ---- 每 5 秒报一次统计 ----
            now = time.time()
            if (now - t_last_report) >= 5.0:
                t_last_report = now
                span = now - t_start
                print("-" * 100)
                print("统计：好帧 %d，坏帧 %d，平均 %.1f 帧/秒（有效帧率应接近 10）"
                      % (good, bad, good / span if span > 0 else 0))
                print("-" * 100)

    except KeyboardInterrupt:
        print("\n用户中断。")
    finally:
        ser.close()
        if csv_file:
            csv_file.close()
        span = time.time() - t_start
        print("=" * 100)
        print("结束：好帧 %d，坏帧 %d，共 %.1f 秒，平均 %.1f 帧/秒"
              % (good, bad, span, good / span if span > 0 else 0))
        if good == 0:
            print("一条有效帧都没有，按顺序检查：")
            print("  1) USB-TTL 的 RX 有没有接到 PA2(USART2_TX)？TX 接 PA3；GND 是否共地？")
            print("  2) 波特率是不是 115200？串口号选对了没有（--list 看一眼）？")
            print("  3) 板子真的在跑固件吗（重新上电，D1/D2 LED 有无反应）？")


def main():
    ap = argparse.ArgumentParser(
        description="XunjiCar $CAR 遥测帧上位机",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--port", help="串口号，例如 COM5 或 /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200, help="波特率，默认 115200")
    ap.add_argument("--csv", help="把解析结果同时写入这个 CSV 文件")
    ap.add_argument("--duration", type=float, default=0, help="只跑这么多秒（0=一直跑）")
    ap.add_argument("--list", action="store_true", help="列出本机所有串口后退出")
    ap.add_argument("--show-bad", action="store_true", help="把坏帧也打印出来（排查波特率/接线用）")
    ap.add_argument("--selftest", action="store_true", help="不需要硬件，自检脚本")
    args = ap.parse_args()

    if args.selftest:
        run_selftest()
        return

    if args.list:
        ports = list(list_ports.comports())
        if not ports:
            print("没有发现任何串口。USB-TTL 插上后要装驱动（CH340/CP2102/FT232）。")
            return
        print("本机串口：")
        for p in ports:
            print("  %-8s %s" % (p.device, p.description))
        return

    if not args.port:
        ap.error("请用 --port 指定串口（先 --list 查看），或用 --selftest 自检")

    monitor(args.port, args.baud, args.csv, args.duration, args.show_bad)


if __name__ == "__main__":
    main()
