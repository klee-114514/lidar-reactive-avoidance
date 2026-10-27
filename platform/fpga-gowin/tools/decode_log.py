"""final_leida 串口日志解码器

用法：
    python tools/decode_log.py <日志文件> [--csv out.csv]

日志就是串口助手直接保存的文本（带 [时:分:秒.毫秒]收←◆ 前缀也可以）。
脚本会自动把被串口分块切断的三行拼回来，输出逐帧表：
    版本 / df / 航向 / side / w / 两轮速度 / 电机动作 / 状态 / phi / lat /
    G(影子 kL,kR) / S(决策标志) / E(障碍段两端)

字段口径（与 rtl/final_leida_top.v 第三行一致）：
    df       正前距离（已折算到车头前缘）
    Y        航向（Q16，182 计数/度）
    d        side：0=往左绕，1=往右绕
    w        avoid_fsm 原因码
    SL/SR    两轮 5ms 编码器速度（前进为正）
    A/S      电机当前段动作 / 状态机
    P=φlat   φ=本次指令角（度），lat=累计横移（mm）
    G        ★V4.1a★ 影子规划器：00<kL><kR>，EE=还没算过，F=该侧不可行
    F        ★V4.2b-dbg★ <corr><diff>：corr=航向修正命令低字节，diff=左-右实测差
    S        ★V4.0d★ 决策标志：bit7 gap_ok / bit6 side / bit5 侧窗FFFF /
             bit4 侧窗>=300 / bit3 正前清 / bit2 hist_l FFFF / bit1 hist_r FFFF / bit0 crush
"""
import re
import sys

TS = re.compile(r"\[\d\d:\d\d:\d\d\.\d\d\d\]收←◆")

FRAME = re.compile(
    r"df=([0-9A-F]{4})"                    # 1 正前
    r".*?Y=([0-9A-F]{4})"                  # 2 航向
    r"\s+d=([01])"                          # 3 side
    r"\s+n=([0-9A-F])\s+w=([0-9A-F]{2})"   # 4 lat 低4位, 5 原因码
    r".*?SL=([0-9A-F]{4})\s+SR=([0-9A-F]{4})"   # 6 7
    r".*?A=([0-9A-F])\s+S=([0-9A-F])"      # 8 9
    r".*?P=([0-9A-F]{6})\s+F=([0-9A-F]{4})\s+G=([0-9A-F]{4})"  # 10 11 12
    r"\s+V=([0-9A-Za-z.]+)\s+S=([0-9A-F]{2})"                  # 13 14
    r".*?E=([0-9A-F]{12})",                # 15
    re.S,
)


def s16(v):
    v = int(v, 16)
    return v - 0x10000 if v & 0x8000 else v


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    raw = open(sys.argv[1], encoding="utf-8", errors="replace").read()
    s = TS.sub("", raw).replace("\n", "").replace("\r", "")
    rows = FRAME.findall(s)
    if not rows:
        print("没有解析到完整帧（检查日志是否包含三行完整输出）")
        return 1
    print(f"共 {len(rows)} 帧")
    print(" #  df  yaw  d  w   SL   SR  A S  phi lat   V      F(c/d)  G   S   E")
    for i, r in enumerate(rows):
        df, yaw, side, n, why, sl, sr, act, st, p, f, g, ver, sf, e = r
        phi = int(p[0:2], 16)
        lat = int(p[2:6], 16)
        corr = int(f[0:2], 16)                  # 低字节，按有符号解释
        corr = corr - 256 if corr > 127 else corr
        diff = int(f[2:4], 16)
        diff = diff - 256 if diff > 127 else diff
        a0, d0, a1, d1 = (int(e[0:2], 16), int(e[2:6], 16),
                          int(e[6:8], 16), int(e[8:12], 16))
        print(f"{i:3d} {int(df,16):4d} {s16(yaw)/182:5.1f} {side} {why} "
              f"{s16(sl):4d} {s16(sr):4d} {act} {st} {phi:3d} {lat:3d} "
              f"{ver:>6s} {corr:+4d}/{diff:+4d} {g} {sf} "
              f"({a0},{d0})-({a1},{d1})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
