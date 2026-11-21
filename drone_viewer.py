#!/usr/bin/env python3
"""
Usage:
    python drone_live_viewer_4.py
    python drone_live_viewer_4.py --host 192.168.1.1 --port 7070 --path webcam
    python drone_live_viewer_4.py --record output.mp4
    python drone_live_viewer_4.py --no-swap-qtables --sampling 1
    python drone_live_viewer_4.py --vr
    python drone_live_viewer_4.py --vr --vr-gap 30
"""

import argparse
import io
import locale
import os
import platform
import re
import socket
import struct
import subprocess
import sys
import threading
import time
import queue
import cv2
import numpy as np

LUM_Q = [
    16, 11, 10, 16, 24, 40, 51, 61,
    12, 12, 14, 19, 26, 58, 60, 55,
    14, 13, 16, 24, 40, 57, 69, 56,
    14, 17, 22, 29, 51, 87, 80, 62,
    18, 22, 37, 56, 68, 109, 103, 77,
    24, 35, 55, 64, 81, 104, 113, 92,
    49, 64, 78, 87, 103, 121, 120, 101,
    72, 92, 95, 98, 112, 100, 103, 99,
]
CHROM_Q = [
    17, 18, 24, 47, 99, 99, 99, 99,
    18, 21, 26, 66, 99, 99, 99, 99,
    24, 26, 56, 99, 99, 99, 99, 99,
    47, 66, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
]

def make_quant_tables(q):
    if q <= 0:
        q = 1
    if q > 99:
        q = 99
    scale = 5000 // q if q < 50 else 200 - q * 2

    def scale_table(base):
        return [max(1, min(255, (v * scale + 50) // 100)) for v in base]

    return scale_table(LUM_Q), scale_table(CHROM_Q)

LUM_DC_BITS = bytes([0,1,5,1,1,1,1,1,1,0,0,0,0,0,0,0])
LUM_DC_VALS = bytes(range(12))
LUM_AC_BITS = bytes([0,2,1,3,3,2,4,3,5,5,4,4,0,0,1,0x7d])
LUM_AC_VALS = bytes([
    0x01,0x02,0x03,0x00,0x04,0x11,0x05,0x12,0x21,0x31,0x41,0x06,0x13,0x51,0x61,0x07,
    0x22,0x71,0x14,0x32,0x81,0x91,0xa1,0x08,0x23,0x42,0xb1,0xc1,0x15,0x52,0xd1,0xf0,
    0x24,0x33,0x62,0x72,0x82,0x09,0x0a,0x16,0x17,0x18,0x19,0x1a,0x25,0x26,0x27,0x28,
    0x29,0x2a,0x34,0x35,0x36,0x37,0x38,0x39,0x3a,0x43,0x44,0x45,0x46,0x47,0x48,0x49,
    0x4a,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5a,0x63,0x64,0x65,0x66,0x67,0x68,0x69,
    0x6a,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7a,0x83,0x84,0x85,0x86,0x87,0x88,0x89,
    0x8a,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9a,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,
    0xa8,0xa9,0xaa,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xc2,0xc3,0xc4,0xc5,
    0xc6,0xc7,0xc8,0xc9,0xca,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,0xe1,0xe2,
    0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,
    0xf9,0xfa,
])
CHROM_DC_BITS = bytes([0,3,1,1,1,1,1,1,1,1,1,0,0,0,0,0])
CHROM_DC_VALS = bytes(range(12))
CHROM_AC_BITS = bytes([0,2,1,2,4,4,3,4,7,5,4,4,0,1,2,0x77])
CHROM_AC_VALS = bytes([
    0x00,0x01,0x02,0x03,0x11,0x04,0x05,0x21,0x31,0x06,0x12,0x41,0x51,0x07,0x61,0x71,
    0x13,0x22,0x32,0x81,0x08,0x14,0x42,0x91,0xa1,0xb1,0xc1,0x09,0x23,0x33,0x52,0xf0,
    0x15,0x62,0x72,0xd1,0x0a,0x16,0x24,0x34,0xe1,0x25,0xf1,0x17,0x18,0x19,0x1a,0x26,
    0x27,0x28,0x29,0x2a,0x35,0x36,0x37,0x38,0x39,0x3a,0x43,0x44,0x45,0x46,0x47,0x48,
    0x49,0x4a,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5a,0x63,0x64,0x65,0x66,0x67,0x68,
    0x69,0x6a,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7a,0x82,0x83,0x84,0x85,0x86,0x87,
    0x88,0x89,0x8a,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9a,0xa2,0xa3,0xa4,0xa5,
    0xa6,0xa7,0xa8,0xa9,0xaa,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xc2,0xc3,
    0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,0xca,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,
    0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,
    0xf9,0xfa,
])

ZIGZAG = [
    0, 1, 8, 16, 9, 2, 3, 10,
    17, 24, 32, 25, 18, 11, 4, 5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13, 6, 7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63,
]

def marker(code, payload=b""):
    if payload or code not in (0xD8, 0xD9):
        return bytes([0xFF, code]) + struct.pack(">H", len(payload) + 2) + payload
    return bytes([0xFF, code])

def build_dqt(table_bytes, table_id, already_zigzag=False):
    if already_zigzag:
        zz = list(table_bytes)
    else:
        zz = [0] * 64
        for i in range(64):
            zz[i] = table_bytes[ZIGZAG[i]]
    return marker(0xDB, bytes([table_id]) + bytes(zz))

def build_dht(bits, vals, table_class, table_id):
    return marker(0xC4, bytes([(table_class << 4) | table_id]) + bits + vals)

def build_sof0(width, height, sampling_type=0):
    payload = bytes([8]) + struct.pack(">HH", height, width) + bytes([3])
    y_samp = 0x21 if sampling_type % 64 == 1 else 0x22
    payload += bytes([1, y_samp, 0])
    payload += bytes([2, 0x11, 1])
    payload += bytes([3, 0x11, 1])
    return marker(0xC0, payload)

def build_sos():
    payload = bytes([3])
    payload += bytes([1, 0x00])
    payload += bytes([2, 0x11])
    payload += bytes([3, 0x11])
    payload += bytes([0, 63, 0])
    return marker(0xDA, payload)

def build_dri(interval):
    return marker(0xDD, struct.pack(">H", interval))

_RESTART_MARKER_RE = re.compile(rb'\xff[\xd0-\xd7]')

def count_restart_markers(entropy):
    return len(_RESTART_MARKER_RE.findall(entropy))

_static_header_cache = {}
_SOS_BYTES = build_sos()
_DHT_BYTES = (
    build_dht(LUM_DC_BITS, LUM_DC_VALS, 0, 0)
    + build_dht(LUM_AC_BITS, LUM_AC_VALS, 1, 0)
    + build_dht(CHROM_DC_BITS, CHROM_DC_VALS, 0, 1)
    + build_dht(CHROM_AC_BITS, CHROM_AC_VALS, 1, 1)
)

def _get_static_header(width_px, height_px, sampling_type):
    key = (width_px, height_px, sampling_type)
    cached = _static_header_cache.get(key)
    if cached is None:
        cached = build_sof0(width_px, height_px, sampling_type=sampling_type) + _DHT_BYTES
        _static_header_cache[key] = cached
    return cached

def rebuild_jpeg(fragments, q, width_px, height_px, qtables_override=None,
                  sampling_type=0, swap_qtables=True, auto_restart_interval=True,
                  declared_restart_interval=0):
    entropy = b"".join(fragments)
    if qtables_override is not None:
        lum, chrom = qtables_override
        if swap_qtables:
            lum, chrom = chrom, lum
        already_zz = True
    else:
        lum, chrom = make_quant_tables(q)
        already_zz = False

    if auto_restart_interval:
        marker_count = count_restart_markers(entropy)
        if marker_count > 0:
            mcu_w, mcu_h = (16, 8) if sampling_type % 64 == 1 else (16, 16)
            mcus_x = (width_px + mcu_w - 1) // mcu_w
            mcus_y = (height_px + mcu_h - 1) // mcu_h
            total_mcus = mcus_x * mcus_y
            restart_interval = max(1, total_mcus // (marker_count + 1))
        else:
            restart_interval = 0
    else:
        restart_interval = declared_restart_interval
    out = bytearray()
    out += marker(0xD8)
    out += build_dqt(lum, 0, already_zigzag=already_zz)
    out += build_dqt(chrom, 1, already_zigzag=already_zz)
    out += _get_static_header(width_px, height_px, sampling_type)
    if restart_interval:
        out += build_dri(restart_interval)
    out += _SOS_BYTES
    out += entropy
    out += marker(0xD9)
    return bytes(out)

class RTSPError(Exception):
    pass

def rtsp_request(sock, method, url, cseq, extra_headers=None):
    headers = [f"{method} {url} RTSP/1.0", f"CSeq: {cseq}", "User-Agent: DroneLiveViewer/1.0"]
    if extra_headers:
        headers.extend(extra_headers)
    req = ("\r\n".join(headers) + "\r\n\r\n").encode()
    sock.sendall(req)
    resp = b""
    sock.settimeout(5)
    while b"\r\n\r\n" not in resp:
        chunk = sock.recv(4096)
        if not chunk:
            break
        resp += chunk

    if b"\r\n\r\n" in resp:
        header_bytes, _, body_bytes = resp.partition(b"\r\n\r\n")
    else:
        header_bytes, body_bytes = resp, b""

    header_text = header_bytes.decode(errors="replace")

    m = re.search(r"Content-Length:\s*(\d+)", header_text, re.IGNORECASE)
    if m:
        content_length = int(m.group(1))
        while len(body_bytes) < content_length:
            chunk = sock.recv(4096)
            if not chunk:
                break
            body_bytes += chunk

    body_text = body_bytes.decode(errors="replace")
    full_text = header_text + "\r\n\r\n" + body_text

    status_line = header_text.splitlines()[0] if header_text else ""
    if " 200 " not in status_line and status_line:
        print(f"  [rtsp] warning: non-200 response to {method}: {status_line.strip()}")

    return full_text

def rtsp_setup_and_play(host, rtsp_port, path, client_rtp_port, bind_ip=None):
    url = f"rtsp://{host}:{rtsp_port}/{path}"
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    if bind_ip:
        sock.bind((bind_ip, 0))
    sock.connect((host, rtsp_port))

    cseq = 1
    print(f"  [rtsp] OPTIONS {url}")
    rtsp_request(sock, "OPTIONS", url, cseq)
    cseq += 1

    print(f"  [rtsp] DESCRIBE {url}")
    describe_resp = rtsp_request(sock, "DESCRIBE", url, cseq, ["Accept: application/sdp"])
    cseq += 1
    print("  [rtsp] --- DESCRIBE response ---")
    for line in describe_resp.splitlines():
        print(f"  [rtsp]   {line}")
    print("  [rtsp] --- end DESCRIBE response ---")

    control = None
    for line in describe_resp.splitlines():
        line = line.strip()
        if line.lower().startswith("a=control:"):
            control = line.split(":", 1)[1].strip()
            break

    candidate_urls = []
    if control and control != "*":
        if control.lower().startswith("rtsp://"):
            candidate_urls.append(control)
        else:
            candidate_urls.append(url.rstrip("/") + "/" + control.lstrip("/"))
    candidate_urls.append(url)

    setup_resp = None
    for candidate in candidate_urls:
        print(f"  [rtsp] SETUP {candidate}  (client_port={client_rtp_port}-{client_rtp_port+1})")
        resp = rtsp_request(
            sock, "SETUP", candidate, cseq,
            [f"Transport: RTP/AVP;unicast;client_port={client_rtp_port}-{client_rtp_port+1}"]
        )
        cseq += 1
        status_line = resp.splitlines()[0] if resp else ""
        if " 200 " in status_line:
            setup_resp = resp
            break

    if setup_resp is None:
        raise RTSPError(f"SETUP failed against all candidate URLs: {candidate_urls}")

    session_match = re.search(r"Session:\s*([^\s;\r\n]+)(?:;timeout=(\d+))?", setup_resp)
    if not session_match:
        raise RTSPError(f"No Session header in SETUP response:\n{setup_resp}")
    session = session_match.group(1)
    server_timeout = int(session_match.group(2)) if session_match.group(2) else None

    server_port_match = re.search(r"server_port=(\d+)-(\d+)", setup_resp)
    server_ports = server_port_match.groups() if server_port_match else (None, None)

    print(f"  [rtsp] PLAY {url}")
    rtsp_request(sock, "PLAY", url, cseq, [f"Session: {session}"])
    cseq += 1
    return sock, session, cseq, server_timeout, server_ports

def rtcp_keepalive_loop(rtcp_sock, host, rtcp_port, stop_event, interval=5):
    if rtcp_port is None:
        return
    ssrc = 0xC0FFEE01
    rr_packet = bytes([0x80, 0xC9]) + struct.pack(">H", 1) + struct.pack(">I", ssrc)
    while not stop_event.wait(interval):
        try:
            rtcp_sock.sendto(rr_packet, (host, int(rtcp_port)))
        except Exception:
            break

def rtsp_keepalive_loop(sock, url, session, cseq_start, stop_event, server_timeout=None):
    interval = max(2, server_timeout // 2) if server_timeout else 8
    cseq = cseq_start
    while not stop_event.wait(interval):
        try:
            rtsp_request(sock, "OPTIONS", url, cseq, [f"Session: {session}"])
            cseq += 1
        except Exception:
            break

class Stats:
    def __init__(self):
        self.lock = threading.Lock()
        self.total_bytes = 0
        self.total_frames = 0
        self.wifi_percent = None
        self.wifi_channel = None

def get_openwrt_wifi_stats(router_ip="192.168.1.3", user="root"):
    try:
        ssh_base = [
            "ssh",
            "-i", os.path.expanduser("~/.ssh/id_rsa"),
            "-o", "ConnectTimeout=1",
            "-o", "BatchMode=yes",
            "-o", "StrictHostKeyChecking=no",
            "-o", "HostKeyAlgorithms=+ssh-rsa",
            "-o", "PubkeyAcceptedKeyTypes=+ssh-rsa",
            f"{user}@{router_ip}"
        ]

        cmd_info = ssh_base + ["iwinfo wlan0 info 2>/dev/null || iwinfo wlan1 info"]
        output_info = subprocess.check_output(cmd_info, timeout=2.0).decode('utf-8', errors='ignore')

        channel_match = re.search(r'Channel:\s*(\d+)', output_info)
        channel = int(channel_match.group(1)) if channel_match else None

        rssi_dbm = None
        signal_match = re.search(r'Signal:\s*(-?\d+)\s*dBm', output_info)
        if signal_match:
            rssi_dbm = int(signal_match.group(1))

        if rssi_dbm is None:
            cmd_assoc = ssh_base + ["iwinfo wlan0 assoclist 2>/dev/null || iwinfo wlan1 assoclist"]
            output_assoc = subprocess.check_output(cmd_assoc, timeout=2.0).decode('utf-8', errors='ignore')
            
            rx_match = re.search(r'(?:RX|Signal)?:\s*(-?\d+)\s*dBm', output_assoc, re.IGNORECASE)
            if not rx_match:
                rx_match = re.search(r'(-?\d+)\s*dBm', output_assoc)
            if rx_match:
                rssi_dbm = int(rx_match.group(1))
        if rssi_dbm is not None:
            signal_pct = max(0, min(100, 2 * (rssi_dbm + 100)))
            rssi_str = f"{rssi_dbm} dBm ({signal_pct}%)"
        else:
            rssi_str = None

        return rssi_str, channel

    except Exception:
        return None, None

def wifi_rssi_poll_loop(stats, stop_event, interval=1.0):
    while not stop_event.is_set():
        start_time = time.time()
        percent, channel = get_openwrt_wifi_stats(router_ip="192.168.1.3")

        with stats.lock:
            stats.wifi_percent = percent
            stats.wifi_channel = channel

        elapsed = time.time() - start_time
        sleep_time = max(0.1, interval - elapsed)
        if stop_event.wait(sleep_time):
            break

def draw_signal_bars(canvas, x, y, filled_bars, total_bars=4, bar_width=4, bar_gap=2, max_height=14):
    for i in range(total_bars):
        h = max(3, int(max_height * (i + 1) / total_bars))
        bx = x + i * (bar_width + bar_gap)
        by = y + (max_height - h)
        color = (255, 255, 255) if i < filled_bars else (90, 90, 90)
        cv2.rectangle(canvas, (bx, by), (bx + bar_width, y + max_height), color, -1)
        cv2.rectangle(canvas, (bx, by), (bx + bar_width, y + max_height), (0, 0, 0), 1)
    return x + total_bars * (bar_width + bar_gap)

def draw_rssi_overlay(canvas, wifi_percent, wifi_channel, left_x, top_y):
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.5
    thickness = 1

    pct_val = None
    if isinstance(wifi_percent, str):
        pct_match = re.search(r'(\d+)%', wifi_percent)
        if pct_match:
            pct_val = int(pct_match.group(1))
        text = f"RSSI: {wifi_percent}"
    elif isinstance(wifi_percent, (int, float)):
        pct_val = int(wifi_percent)
        dbm = (pct_val / 2) - 100
        text = f"RSSI: ~{dbm:.0f} dBm"
    else:
        text = "RSSI: N/A"

    if pct_val is not None:
        filled_bars = min(4, max(0, (pct_val + 12) // 25))
    else:
        filled_bars = 0

    icon_x = left_x + 8
    icon_y = top_y + 6
    text_x = draw_signal_bars(canvas, icon_x, icon_y, filled_bars) + 8

    (tw, th), _ = cv2.getTextSize(text, font, scale, thickness)
    text_y = icon_y + th + 6
    cv2.putText(canvas, text, (text_x, text_y), font, scale, (0, 0, 0), thickness + 2, cv2.LINE_AA)
    cv2.putText(canvas, text, (text_x, text_y), font, scale, (255, 255, 255), thickness, cv2.LINE_AA)

    chan_text = f"Ch {wifi_channel}" if (wifi_channel) else f"Ch {wifi_channel if wifi_channel else 'N/A'}"
    chan_y = text_y + th + 10
    chan_x = left_x + 8
    cv2.putText(canvas, chan_text, (chan_x, chan_y), font, scale, (0, 0, 0), thickness + 2, cv2.LINE_AA)
    cv2.putText(canvas, chan_text, (chan_x, chan_y), font, scale, (255, 255, 255), thickness, cv2.LINE_AA)

    return chan_y

def rtp_receiver(udp_sock, frame_queue, stop_event, sampling_override, swap_qtables, auto_restart_interval, stats):
    current_ts = None
    current_pkts = []
    udp_sock.settimeout(1.0)

    while not stop_event.is_set():
        try:
            data, _addr = udp_sock.recvfrom(65535)
        except socket.timeout:
            continue
        except OSError:
            break

        with stats.lock:
            stats.total_bytes += len(data)

        if len(data) < 12:
            continue

        b0 = data[0]
        if (b0 >> 6) & 0x03 != 2:
            continue
        marker_bit = (data[1] >> 7) & 0x01
        timestamp = struct.unpack(">I", data[4:8])[0]

        jpeg_payload = data[12:]
        if len(jpeg_payload) < 8:
            continue

        frag_offset = (jpeg_payload[1] << 16) | (jpeg_payload[2] << 8) | jpeg_payload[3]
        type_ = jpeg_payload[4]
        q = jpeg_payload[5]
        w = jpeg_payload[6]
        h = jpeg_payload[7]
        idx = 8

        if 64 <= type_ <= 127:
            if len(jpeg_payload) < idx + 4:
                continue
            idx += 4

        qtables = None
        if frag_offset == 0 and q >= 128:
            if len(jpeg_payload) < idx + 4:
                continue
            precision = jpeg_payload[idx + 1]
            length = struct.unpack(">H", jpeg_payload[idx + 2:idx + 4])[0]
            idx += 4
            table_data = jpeg_payload[idx:idx + length]
            idx += length
            if precision == 0 and len(table_data) >= 128:
                qtables = (list(table_data[0:64]), list(table_data[64:128]))

        payload_data = jpeg_payload[idx:]

        if timestamp != current_ts:
            current_ts = timestamp
            current_pkts = []

        current_pkts.append({
            "frag_offset": frag_offset, "data": payload_data, "type": type_,
            "q": q, "w": w, "h": h, "qtables": qtables,
        })

        if marker_bit:
            pkts = sorted(current_pkts, key=lambda p: p["frag_offset"])
            fragments = [p["data"] for p in pkts]
            qtables_override = next((p["qtables"] for p in pkts if p["qtables"] is not None), None)
            sampling_type = sampling_override if sampling_override is not None else pkts[0]["type"]

            try:
                jpeg_bytes = rebuild_jpeg(
                    fragments, pkts[0]["q"], pkts[0]["w"] * 8, pkts[0]["h"] * 8,
                    qtables_override=qtables_override, sampling_type=sampling_type,
                    swap_qtables=swap_qtables, auto_restart_interval=auto_restart_interval,
                )
                try:
                    frame_queue.put_nowait(jpeg_bytes)
                except queue.Full:
                    try:
                        frame_queue.get_nowait()
                    except queue.Empty:
                        pass
                    frame_queue.put_nowait(jpeg_bytes)
                with stats.lock:
                    stats.total_frames += 1
            except Exception as e:
                print(f"  [warn] failed to rebuild a frame: {e}")

            current_ts = None
            current_pkts = []

def format_elapsed(seconds):
    seconds = int(seconds)
    h, rem = divmod(seconds, 3600)
    m, s = divmod(rem, 60)
    return f"{h:02d}:{m:02d}:{s:02d}" if h > 0 else f"{m:02d}:{s:02d}"

def draw_timer(canvas, text, right_x, top_y):
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.5
    thickness = 1
    (tw, th), _ = cv2.getTextSize(text, font, scale, thickness)
    x = right_x - tw - 8
    y = top_y + th + 8
    cv2.putText(canvas, text, (x, y), font, scale, (0, 0, 0), thickness + 2, cv2.LINE_AA)
    cv2.putText(canvas, text, (x, y), font, scale, (255, 255, 255), thickness, cv2.LINE_AA)

def letterbox_to_window(frame, win_w, win_h):
    fh, fw = frame.shape[:2]
    if win_w <= 0 or win_h <= 0 or fw <= 0 or fh <= 0:
        return frame, 0, 0, fw, fh
    scale = min(win_w / fw, win_h / fh)
    new_w = max(1, int(round(fw * scale)))
    new_h = max(1, int(round(fh * scale)))
    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    canvas = np.zeros((win_h, win_w, 3), dtype=np.uint8)
    off_x = (win_w - new_w) // 2
    off_y = (win_h - new_h) // 2
    canvas[off_y:off_y + new_h, off_x:off_x + new_w] = resized
    return canvas, off_x, off_y, new_w, new_h

def deblock_sharpen(img, strength=1.0):
    d = cv2.bilateralFilter(img, d=9, sigmaColor=150, sigmaSpace=150)
    blurred = cv2.GaussianBlur(d, (0, 0), 0.8)
    return cv2.addWeighted(d, 1 + 0.5 * strength, blurred, -0.5 * strength, 0)

def draw_stats_topleft(canvas, text, left_x, top_y):
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.5
    thickness = 1
    (tw, th), _ = cv2.getTextSize(text, font, scale, thickness)
    x = left_x + 8
    y = top_y + th + 8
    cv2.putText(canvas, text, (x, y), font, scale, (0, 0, 0), thickness + 2, cv2.LINE_AA)
    cv2.putText(canvas, text, (x, y), font, scale, (255, 255, 255), thickness, cv2.LINE_AA)
    return y

STALL_TIMEOUT = 8.0
RECONNECT_BACKOFF = 3.0

class Recorder:
    def __init__(self, path):
        self.path = path
        self.writer = None

def run_session(args, windows, recorder, global_stats):
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    bind_addr = args.bind_ip if args.bind_ip else "0.0.0.0"
    udp_sock.bind((bind_addr, args.client_port))

    rtcp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rtcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rtcp_sock.bind((bind_addr, args.client_port + 1))

    print(f"Connecting to rtsp://{args.host}:{args.rtsp_port}/{args.path} ..."
          + (f"  (bound to {args.bind_ip})" if args.bind_ip else ""))
    try:
        rtsp_sock, session, cseq, server_timeout, server_ports = rtsp_setup_and_play(
            args.host, args.rtsp_port, args.path, args.client_port, bind_ip=args.bind_ip)
    except Exception as e:
        print(f"RTSP handshake failed: {e}")
        udp_sock.close()
        rtcp_sock.close()
        return "handshake_failed"

    stop_event = threading.Event()
    url = f"rtsp://{args.host}:{args.rtsp_port}/{args.path}"
    threading.Thread(target=rtsp_keepalive_loop, args=(rtsp_sock, url, session, cseq, stop_event, server_timeout), daemon=True).start()

    rtcp_port = server_ports[1] if server_ports else None
    threading.Thread(target=rtcp_keepalive_loop, args=(rtcp_sock, args.host, rtcp_port, stop_event), daemon=True).start()

    frame_queue = queue.Queue(maxsize=3)
    with global_stats.lock:
        global_stats.total_bytes = 0
        global_stats.total_frames = 0
    threading.Thread(target=rtp_receiver, args=(udp_sock, frame_queue, stop_event, args.sampling, args.swap_qtables, args.auto_restart_interval, global_stats), daemon=True).start()

    writer_holder = recorder
    last_good = None
    stream_start_time = None
    supports_rect = {"normal": True, "vr": True}

    last_stat_time = time.time()
    last_stat_bytes = 0
    last_stat_frames = 0
    display_fps = 0.0
    display_kbps = 0.0
    show_hud = True
    last_frame_seen_time = time.time()

    interp_map = {"linear": cv2.INTER_LINEAR, "cubic": cv2.INTER_CUBIC, "lanczos4": cv2.INTER_LANCZOS4}
    interp = interp_map[args.upscale_interp]

    print("Waiting for video... (press 'q' in the video window to quit)")

    result = "stall"
    try:
        while True:
            try:
                jpeg_bytes = frame_queue.get(timeout=1.0)
                last_frame_seen_time = time.time()
            except queue.Empty:
                if time.time() - last_frame_seen_time > STALL_TIMEOUT:
                    if last_good is None:
                        print(f"No frames received within {STALL_TIMEOUT:.0f}s of connecting "
                              f"-- the drone may not be sending video. Reconnecting...")
                    else:
                        print(f"No frames received for {STALL_TIMEOUT:.0f}s -- assuming the "
                              f"drone restarted or dropped the link. Reconnecting...")
                    result = "stall"
                    break
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    result = "quit"
                    break
                elif key == ord("h"):
                    show_hud = not show_hud
                    print(f"HUD {'on' if show_hud else 'off'}")
                continue

            arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is not None:
                if args.deblock:
                    img = deblock_sharpen(img, args.deblock_strength)
                if args.upscale_height:
                    h0, w0 = img.shape[:2]
                    new_h = args.upscale_height
                    new_w = int(round(w0 * (new_h / h0)))
                    img = cv2.resize(img, (new_w, new_h), interpolation=interp)
                last_good = img
                if stream_start_time is None:
                    stream_start_time = time.time()

            if last_good is not None:
                display_frame = last_good.copy()

                now = time.time()
                if now - last_stat_time >= 0.5:
                    with global_stats.lock:
                        cur_bytes = global_stats.total_bytes
                        cur_frames = global_stats.total_frames
                    dt = now - last_stat_time
                    display_fps = (cur_frames - last_stat_frames) / dt
                    display_kbps = (cur_bytes - last_stat_bytes) / dt / 1024.0
                    last_stat_time = now
                    last_stat_bytes = cur_bytes
                    last_stat_frames = cur_frames

                if stream_start_time is not None and show_hud:
                    elapsed_text = format_elapsed(time.time() - stream_start_time)
                    fh, fw = display_frame.shape[:2]
                    draw_timer(display_frame, elapsed_text, fw, 0)
                    stats_text = f"{display_fps:.1f} fps | {display_kbps:.0f} KB/s"
                    stats_bottom_y = draw_stats_topleft(display_frame, stats_text, 0, 0)
                    with global_stats.lock:
                        wifi_percent = global_stats.wifi_percent
                        wifi_channel = global_stats.wifi_channel
                    draw_rssi_overlay(display_frame, wifi_percent, wifi_channel, 0, stats_bottom_y)

                record_frame = last_good
                vr_display_frame = None
                vr_record_frame = None
                if windows["vr"] is not None:
                    gap_w = max(0, args.vr_gap)
                    if gap_w > 0:
                        rec_h = last_good.shape[0]
                        rec_gap = np.zeros((rec_h, gap_w, 3), dtype=np.uint8)
                        vr_record_frame = np.hstack([last_good, rec_gap, last_good])

                        disp_h = display_frame.shape[0]
                        disp_gap = np.zeros((disp_h, gap_w, 3), dtype=np.uint8)
                        vr_display_frame = np.hstack([display_frame, disp_gap, display_frame])
                    else:
                        vr_record_frame = np.hstack([last_good, last_good])
                        vr_display_frame = np.hstack([display_frame, display_frame])

                    if windows["normal"] is None:
                        record_frame = vr_record_frame

                if args.record and writer_holder.writer is None:
                    h, w = record_frame.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    writer_holder.writer = cv2.VideoWriter(writer_holder.path, fourcc, 15, (w, h))
                    print(f"Recording to {writer_holder.path}")
                if writer_holder.writer is not None:
                    writer_holder.writer.write(record_frame)

                def show_in_window(frame_to_show, win_key):
                    win = windows[win_key]
                    if win is None:
                        return
                    win_w, win_h = frame_to_show.shape[1], frame_to_show.shape[0]
                    if supports_rect[win_key]:
                        try:
                            _, _, rw, rh = cv2.getWindowImageRect(win)
                            if rw > 0 and rh > 0:
                                win_w, win_h = rw, rh
                        except Exception:
                            supports_rect[win_key] = False
                    if (win_w, win_h) != (frame_to_show.shape[1], frame_to_show.shape[0]):
                        canvas, _, _, _, _ = letterbox_to_window(frame_to_show, win_w, win_h)
                    else:
                        canvas = frame_to_show
                    cv2.imshow(win, canvas)

                show_in_window(display_frame, "normal")
                show_in_window(vr_display_frame, "vr")

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                result = "quit"
                break
            elif key == ord("h"):
                show_hud = not show_hud
                print(f"HUD {'on' if show_hud else 'off'}")
    except KeyboardInterrupt:
        result = "quit"
    finally:
        stop_event.set()
        try:
            rtsp_request(rtsp_sock, "TEARDOWN", url, 999, [f"Session: {session}"])
        except Exception:
            pass
        rtsp_sock.close()
        udp_sock.close()
        rtcp_sock.close()
        time.sleep(0.3)

    return result

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="192.168.1.1", help="Drone IP address")
    ap.add_argument("--rtsp-port", type=int, default=7070, help="RTSP TCP port")
    ap.add_argument("--path", default="webcam", help="RTSP stream path")
    ap.add_argument("--client-port", type=int, default=5000, help="Local UDP port")
    ap.add_argument("--record", default=None, help="Path to save video")
    ap.add_argument("--sampling", type=int, choices=[0, 1], default=0)
    ap.add_argument("--no-swap-qtables", dest="swap_qtables", action="store_false", default=True)
    ap.add_argument("--no-auto-restart-interval", dest="auto_restart_interval", action="store_false", default=True)
    ap.add_argument("--upscale-height", type=int, default=None)
    ap.add_argument("--upscale-interp", default="lanczos4", choices=["linear", "cubic", "lanczos4"])
    ap.add_argument("--deblock", action="store_true")
    ap.add_argument("--deblock-strength", type=float, default=1.0)
    ap.add_argument("--bind-ip", default=None, help="Local IP of the specific network interface to use (e.g. the LAN-side adapter connected to the drone's router). If omitted, the OS chooses the route.")
    ap.add_argument("--vr", action="store_true", help="Side-by-side double image for VR/cardboard viewers")
    ap.add_argument("--vr-gap", type=int, default=20, help="Gap in pixels between left and right views in VR mode (default: 20)")
    ap.add_argument("--split", action="store_true", help="Open two windows at once: one normal view, one VR side-by-side view")
    args = ap.parse_args()

    if args.upscale_height:
        target_w = int(round(640 * (args.upscale_height / 480)))
        base_w, base_h = target_w, args.upscale_height
    else:
        base_w, base_h = 640, 480
    vr_w = base_w * 2 + args.vr_gap

    windows = {"normal": None, "vr": None}

    show_normal = args.split or not args.vr
    show_vr = args.split or args.vr

    if show_normal:
        name = "FQ777 FQ40 Live"
        windows["normal"] = name
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(name, base_w, base_h)

    if show_vr:
        name = "FQ777 FQ40 Live - VR" if show_normal else "FQ777 FQ40 Live"
        windows["vr"] = name
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(name, vr_w, base_h)

    global_stop_event = threading.Event()
    global_stats = Stats()
    threading.Thread(target=wifi_rssi_poll_loop, args=(global_stats, global_stop_event, 1.0), daemon=True).start()

    recorder = Recorder(args.record) if args.record else Recorder(None)

    try:
        while True:
            result = run_session(args, windows, recorder, global_stats)
            if result == "quit":
                break
            print(f"Session ended ({result}). Reconnecting in {RECONNECT_BACKOFF:.0f}s...")
            time.sleep(RECONNECT_BACKOFF)
    finally:
        global_stop_event.set()
        cv2.destroyAllWindows()
        if recorder.writer is not None:
            recorder.writer.release()

    print("Stopped.")

if __name__ == "__main__":
    sys.exit(main())