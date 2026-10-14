"""Record and decode the bench fusion frame of the dotbot sandbox app.

The frame exists only in a build with DB_BENCH_FUSION in the dotbot project's
preprocessor definitions. Record from the MQTT broker the gateway publishes to,
then decode offline:

    python fusion.py record run.jsonl 60 --broker localhost:1883 --swarm-id A001
    python fusion.py decode run.jsonl run.json

`record` writes every frame of /mari/<swarm>/to_cloud, one JSON line each with
its host time and bytes, and needs paho-mqtt. `decode` needs nothing beyond the
standard library: it prints a count per robot and, given OUT, writes the fusion
records, adverts and status frames of every robot.

Frame (payload type 0xB1, next_proto 0x11; layout in ../main.c,
_send_bench_fusion): type, tick at sending (u32, 10 ms), records, dropped, then
records oldest first: tick u8, seq u8, flags (b0 fix from lines, b1 fix valid,
b2-3 estimator status, b4-7 lines), solve x y (u16 mm), fix - solve (i8 x2,
0.25 mm, -128 invalid, +-127 saturated), photodiode x y (u16 0.1 mm, 65535 when
not tracking), heading (i16 0.01 deg), then per line: station << 4 | sweep << 3
| result, innovation (i8, 0.25 mm, -128 not gated, +-127 saturated). A line
reads rejected and not gated both while its station settles and while the
estimator is not tracking; the record's estimator status tells them apart.
"""

import argparse
import base64
import json
import struct
import time

FUSION = 0xB1
STATUS = ["seeding", "tracking", "lost", "?"]
RESULT = ["accepted", "rejected", "chained", "seeded"]


def record(out, seconds, broker, swarm_id):
    import paho.mqtt.client as mqtt

    host, _, port = broker.partition(":")
    with open(out, "w") as f:

        def on_msg(client, userdata, msg):
            try:
                raw = base64.b64decode(msg.payload, validate=True)
            except ValueError:
                raw = msg.payload
            f.write(json.dumps({"t": round(time.time(), 4), "topic": msg.topic, "hex": raw.hex()}) + "\n")

        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        c.on_message = on_msg
        c.connect(host, int(port or 1883))
        c.subscribe(f"/mari/{swarm_id}/to_cloud")
        c.loop_start()
        time.sleep(seconds)
        c.loop_stop()


def parse_fusion(pl):
    tick, n, dropped = struct.unpack_from("<IBB", pl, 1)
    off, recs = 7, []
    for _ in range(n):
        t8, seq, flags, sx, sy, dx, dy, px, py, hd = struct.unpack_from("<BBBHHbbHHh", pl, off)
        off += 15
        lines = []
        for _ in range(flags >> 4):
            b, inn = struct.unpack_from("<Bb", pl, off)
            off += 2
            result = b & 7
            lines.append(
                {
                    "station": b >> 4,
                    "sweep": (b >> 3) & 1,
                    "result": RESULT[result] if result < len(RESULT) else result,
                    "innovation_mm": None if inn == -128 else inn / 4,
                    "saturated": abs(inn) == 127,
                }
            )
        # The record's full tick: the latest tick at or before the frame's whose low byte is t8
        full = tick - ((tick - t8) & 0xFF)
        valid = bool(flags & 2)
        solve_ok = 0xFFFF not in (sx, sy)
        recs.append(
            {
                "tick": full,
                "seq": seq,
                "from_lines": bool(flags & 1),
                "fix_valid": valid,
                "status": STATUS[(flags >> 2) & 3],
                "solve": [sx, sy] if solve_ok else None,
                "fix": [sx + dx / 4, sy + dy / 4] if valid and solve_ok and -128 not in (dx, dy) else None,
                "fix_saturated": 127 in (abs(dx), abs(dy)),
                "pose": None if 0xFFFF in (px, py) else [px / 10, py / 10],
                "heading_deg": hd / 100 if (flags >> 2) & 3 == 1 else None,
                "lines": lines,
            }
        )
    return {"tick": tick, "dropped": dropped, "records": recs}


def decode(path):
    out = {}
    with open(path) as f:
        for row in f:
            r = json.loads(row)
            b = bytes.fromhex(r["hex"])
            if len(b) < 23 or b[0] != 3:
                continue
            src = b[13:21][::-1].hex().upper()
            proto, pl = b[21], b[22:]
            d = out.setdefault(src, {"fusion": [], "advert": [], "status": [], "frames": 0, "dropped": 0})
            if proto == 0x11 and pl[0] == FUSION:
                fr = parse_fusion(pl)
                d["frames"] += 1
                d["dropped"] += fr["dropped"]
                for rec in fr["records"]:
                    rec["t_frame"] = r["t"]
                    # Host time of the solve: the frame's arrival less the ticks since the record
                    rec["t"] = round(r["t"] - (fr["tick"] - rec["tick"]) / 100, 3)
                    d["fusion"].append(rec)
            elif proto == 0x11 and pl[0] == 0x06 and len(pl) >= 43:
                dr = struct.unpack_from("<h", pl, 3)[0]
                x, y = struct.unpack_from("<II", pl, 5)
                d["advert"].append({"t": r["t"], "x": x, "y": y, "dir": None if dr in (-1000, -32768, 0x7FFF) else dr})
            elif proto == 0x10 and pl[0] == 0x80 and len(pl) >= 13:
                x, y = struct.unpack_from("<II", pl, 5)
                d["status"].append({"t": r["t"], "x": x, "y": y})
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    rec = sub.add_parser("record", help="record every to_cloud frame")
    rec.add_argument("out")
    rec.add_argument("seconds", type=float)
    rec.add_argument("--broker", default="localhost:1883", help="host:port, default localhost:1883")
    rec.add_argument("--swarm-id", default="A001")
    dec = sub.add_parser("decode", help="decode a recording")
    dec.add_argument("input")
    dec.add_argument("out", nargs="?")
    a = ap.parse_args()
    if a.cmd == "record":
        record(a.out, a.seconds, a.broker, a.swarm_id)
        return
    res = decode(a.input)
    for addr, d in res.items():
        print(addr, "fusion records", len(d["fusion"]), "frames", d["frames"], "dropped", d["dropped"], "adverts", len(d["advert"]), "status", len(d["status"]))
    if a.out:
        with open(a.out, "w") as f:
            json.dump(res, f)


if __name__ == "__main__":
    main()
