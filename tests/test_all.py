#!/usr/bin/env python3
"""
Comprehensive automated test suite for ESP32-P4 Camera Recorder project.

Tests every offline-testable aspect:
  1.  repair_avi.py — synthetic AVI repair, edge cases, header patching, idx1 generation
  2.  usb_video_viewer.py — source inspection: guards, markers, protocol
  3.  avi_audio_check.py — parsing, format detection, chunk accounting
  4.  AVI header constant cross-checks (C defines vs Python offsets)
  5.  AVI header binary layout — build header in Python, verify every field position
  6.  C code structure — duplicates, naming conventions, diagnostic toggles, config sanity
  7.  H.264 keyframe detection logic mirror — test the NAL parsing algorithm
  8.  Audio ring buffer drain logic mirror — test overrun, wrap, alignment
  9.  EIS biquad/notch filter math — verify filter coefficients and processing
  10. EIS block matching — SAD computation, median motion vector, crop clamping
  11. IMX708 register validation — crop math, PLL, digital crop consistency
  12. Build system — CMakeLists.txt structure, component dependencies
  13. AVI chunk writing logic — padding, sizing, idx1 entries
  14. ISP configuration validation — neutral defaults confirmed
  15. Pipeline & task configuration — core pinning, queue, semaphore, fsync
  16. Edge cases & defensive code — guards, signature checks

Run:  python tests/test_all.py
"""

import struct
import os
import sys
import tempfile
import re
import math
import importlib.util
import io

# Force UTF-8 output to avoid cp1252 crashes with Unicode chars
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

# Add project root to path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

passed = 0
failed = 0
errors = []


def test(name):
    """Decorator to register and run a test."""
    def decorator(fn):
        global passed, failed
        try:
            fn()
            print(f"  PASS  {name}")
            passed += 1
        except Exception as e:
            print(f"  FAIL  {name}: {e}")
            failed += 1
            errors.append((name, str(e)))
    return decorator


# ============================================================================
# HELPER: Load a Python module from path
# ============================================================================

def load_module(name, filename):
    path = os.path.join(PROJECT_ROOT, filename)
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ============================================================================
# HELPER: Parse C source for #define constants
# ============================================================================

def parse_c_defines(source):
    """Extract #define NAME VALUE from C source. Returns dict name->value."""
    defines = {}
    for m in re.finditer(r'#define\s+(\w+)\s+(.+?)(?://|/\*|$)', source, re.MULTILINE):
        name = m.group(1)
        expr = m.group(2).strip().rstrip()
        # Strip surrounding parens
        if expr.startswith('(') and expr.endswith(')'):
            expr = expr[1:-1].strip()
        # Resolve references to other defines
        resolved = expr
        for ref_name, ref_val in list(defines.items()):
            resolved = re.sub(r'\b' + ref_name + r'\b', str(ref_val), resolved)
        try:
            val = eval(resolved)
            if isinstance(val, (int, float)):
                defines[name] = int(val) if isinstance(val, float) and val == int(val) else val
        except Exception:
            pass
    return defines


def read_c_source():
    c_path = os.path.join(PROJECT_ROOT, "main", "camera_recorder.c")
    with open(c_path, "r", encoding="utf-8", errors="replace") as f:
        return f.read()


# ============================================================================
# HELPER: Build synthetic AVI files
# ============================================================================

def build_synthetic_avi(video_frames=10, audio_chunks=5, audio_samples_per_chunk=512,
                        fps=15, width=1280, height=720,
                        corrupt_riff_size=False, add_trailing_garbage=False,
                        omit_audio=False, add_idx1=False):
    """Build a minimal AVI with H.264 video + PCM audio."""
    us_per_frame = 1_000_000 // fps
    sample_rate = 16000
    channels = 1
    bits = 16

    buf = bytearray()

    def w4cc(tag):
        buf.extend(tag.encode("ascii") if isinstance(tag, str) else tag)

    def wu32(v):
        buf.extend(struct.pack("<I", v))

    def wu16(v):
        buf.extend(struct.pack("<H", v))

    # --- RIFF header ---
    w4cc("RIFF")
    riff_size_pos = len(buf)
    wu32(0)
    w4cc("AVI ")

    # --- hdrl LIST ---
    w4cc("LIST")
    hdrl_size_pos = len(buf)
    wu32(0)
    hdrl_start = len(buf)
    w4cc("hdrl")

    # avih (56 bytes)
    w4cc("avih")
    wu32(56)
    wu32(us_per_frame)
    wu32(0)
    wu32(0)
    wu32(0)
    wu32(video_frames)
    wu32(0)
    wu32(2 if not omit_audio else 1)
    wu32(0)
    wu32(width)
    wu32(height)
    wu32(0); wu32(0); wu32(0); wu32(0)

    # --- Video stream (strl LIST) ---
    w4cc("LIST")
    vid_strl_size_pos = len(buf)
    wu32(0)
    vid_strl_start = len(buf)
    w4cc("strl")

    w4cc("strh")
    wu32(56)
    w4cc("vids")
    w4cc("H264")
    wu32(0)
    wu16(0); wu16(0)
    wu32(0)
    wu32(1)
    wu32(fps)
    wu32(0)
    wu32(video_frames)
    wu32(0); wu32(0); wu32(0)
    wu16(0); wu16(0); wu16(width); wu16(height)

    w4cc("strf")
    wu32(40)
    wu32(40)
    wu32(width)
    wu32(height)
    wu16(1); wu16(24)
    w4cc("H264")
    wu32(width * height * 3)
    wu32(0); wu32(0); wu32(0); wu32(0)

    struct.pack_into("<I", buf, vid_strl_size_pos, len(buf) - vid_strl_start)

    # --- Audio stream (strl LIST) ---
    if not omit_audio:
        w4cc("LIST")
        aud_strl_size_pos = len(buf)
        wu32(0)
        aud_strl_start = len(buf)
        w4cc("strl")

        w4cc("strh")
        wu32(56)
        w4cc("auds")
        wu32(0)
        wu32(0)
        wu16(0); wu16(0)
        wu32(0)
        wu32(1)
        wu32(sample_rate)
        wu32(0)
        total_audio = audio_chunks * audio_samples_per_chunk
        wu32(total_audio)
        wu32(0); wu32(0); wu32(channels * bits // 8)
        wu16(0); wu16(0); wu16(0); wu16(0)

        w4cc("strf")
        wu32(18)
        wu16(1)
        wu16(channels)
        wu32(sample_rate)
        wu32(sample_rate * channels * bits // 8)
        wu16(channels * bits // 8)
        wu16(bits)
        wu16(0)

        struct.pack_into("<I", buf, aud_strl_size_pos, len(buf) - aud_strl_start)

    struct.pack_into("<I", buf, hdrl_size_pos, len(buf) - hdrl_start)

    # --- movi LIST ---
    w4cc("LIST")
    movi_size_pos = len(buf)
    wu32(0)
    movi_start = len(buf)
    w4cc("movi")

    idx1_entries = []
    for i in range(max(video_frames, audio_chunks)):
        if i < video_frames:
            h264_data = bytes([0x00, 0x00, 0x00, 0x01, 0x65]) + os.urandom(200 + i * 10)
            chunk_offset = len(buf) - movi_start + 4
            w4cc("00dc")
            wu32(len(h264_data))
            idx1_entries.append((b"00dc", 0x10, chunk_offset, len(h264_data)))
            buf.extend(h264_data)
            if len(h264_data) % 2:
                buf.append(0)

        if i < audio_chunks and not omit_audio:
            audio_data = os.urandom(audio_samples_per_chunk * 2)
            chunk_offset = len(buf) - movi_start + 4
            w4cc("01wb")
            wu32(len(audio_data))
            idx1_entries.append((b"01wb", 0x00, chunk_offset, len(audio_data)))
            buf.extend(audio_data)
            if len(audio_data) % 2:
                buf.append(0)

    movi_end = len(buf)
    struct.pack_into("<I", buf, movi_size_pos, movi_end - movi_start)

    if add_idx1:
        w4cc("idx1")
        wu32(len(idx1_entries) * 16)
        for tag, flags, offset, size in idx1_entries:
            buf.extend(tag)
            wu32(flags)
            wu32(offset)
            wu32(size)

    if corrupt_riff_size:
        struct.pack_into("<I", buf, riff_size_pos, 100 * 1024 * 1024)
    else:
        struct.pack_into("<I", buf, riff_size_pos, len(buf) - 8)

    if add_trailing_garbage:
        buf.extend(b'\x00' * 4096)

    return bytes(buf)


def write_temp_avi(data):
    f = tempfile.NamedTemporaryFile(suffix=".avi", delete=False)
    f.write(data)
    f.close()
    return f.name


def cleanup(*paths):
    for p in paths:
        if p and os.path.exists(p):
            os.unlink(p)


# ============================================================================
# TEST GROUP 1: repair_avi.py — Functional Tests
# ============================================================================

print("\n" + "=" * 70)
print("  GROUP 1: repair_avi.py — Functional Tests")
print("=" * 70)

@test("repair_avi imports successfully")
def _():
    mod = load_module("repair_avi", "repair_avi.py")
    assert hasattr(mod, "repair_avi"), "repair_avi function not found"

@test("repair valid AVI — frame count preserved")
def _():
    mod = load_module("repair_avi", "repair_avi.py")
    avi_data = build_synthetic_avi(video_frames=20, audio_chunks=10)
    src = write_temp_avi(avi_data)
    dst = src + "_repaired.avi"
    try:
        mod.repair_avi(src, dst, recording_duration=2.0)
        with open(dst, "rb") as f:
            repaired = f.read()
        total_frames = struct.unpack_from("<I", repaired, 48)[0]
        assert total_frames == 20, f"Frame count: expected 20, got {total_frames}"
    finally:
        cleanup(src, dst)

@test("repair valid AVI — RIFF size correct")
def _():
    mod = load_module("repair_avi", "repair_avi.py")
    avi_data = build_synthetic_avi(video_frames=10, audio_chunks=5)
    src = write_temp_avi(avi_data)
    dst = src + "_repaired.avi"
    try:
        mod.repair_avi(src, dst, recording_duration=1.5)
        with open(dst, "rb") as f:
            repaired = f.read()
        riff_size = struct.unpack_from("<I", repaired, 4)[0]
        assert riff_size == len(repaired) - 8, f"RIFF size {riff_size} != {len(repaired) - 8}"
    finally:
        cleanup(src, dst)

@test("repair AVI with corrupted RIFF size — corrected")
def _():
    mod = load_module("repair_avi", "repair_avi.py")
    avi_data = build_synthetic_avi(video_frames=5, audio_chunks=3, corrupt_riff_size=True)
    src = write_temp_avi(avi_data)
    dst = src + "_repaired.avi"
    try:
        mod.repair_avi(src, dst, recording_duration=1.0)
        with open(dst, "rb") as f:
            repaired = f.read()
        riff_size = struct.unpack_from("<I", repaired, 4)[0]
        assert riff_size == len(repaired) - 8, f"RIFF size not corrected: {riff_size}"
    finally:
        cleanup(src, dst)

@test("repair AVI strips trailing garbage")
def _():
    mod = load_module("repair_avi", "repair_avi.py")
    avi_data = build_synthetic_avi(video_frames=5, audio_chunks=3, add_trailing_garbage=True)
    src = write_temp_avi(avi_data)
    dst = src + "_repaired.avi"
    try:
        mod.repair_avi(src, dst, recording_duration=1.0)
        with open(dst, "rb") as f:
            repaired = f.read()
        assert len(repaired) < len(avi_data), "Trailing garbage not stripped"
    finally:
        cleanup(src, dst)

@test("repair AVI creates valid idx1 with correct entry count")
def _():
    mod = load_module("repair_avi", "repair_avi.py")
    avi_data = build_synthetic_avi(video_frames=8, audio_chunks=4)
    src = write_temp_avi(avi_data)
    dst = src + "_repaired.avi"
    try:
        mod.repair_avi(src, dst, recording_duration=1.0)
        with open(dst, "rb") as f:
            repaired = f.read()
        idx1_pos = repaired.find(b"idx1")
        assert idx1_pos > 0, "idx1 not found"
        idx1_size = struct.unpack_from("<I", repaired, idx1_pos + 4)[0]
        num_entries = idx1_size // 16
        assert num_entries == 12, f"Expected 12 entries (8v+4a), got {num_entries}"
    finally:
        cleanup(src, dst)

@test("repair AVI — idx1 first entry is video keyframe (flags=0x10)")
def _():
    mod = load_module("repair_avi", "repair_avi.py")
    avi_data = build_synthetic_avi(video_frames=5, audio_chunks=3)
    src = write_temp_avi(avi_data)
    dst = src + "_repaired.avi"
    try:
        mod.repair_avi(src, dst, recording_duration=1.0)
        with open(dst, "rb") as f:
            repaired = f.read()
        idx1_pos = repaired.find(b"idx1")
        entry_tag = repaired[idx1_pos + 8:idx1_pos + 12]
        entry_flags = struct.unpack_from("<I", repaired, idx1_pos + 12)[0]
        assert entry_tag == b"00dc", f"First entry: {entry_tag}"
        assert entry_flags == 0x10, f"Keyframe flag: 0x{entry_flags:x}"
    finally:
        cleanup(src, dst)

@test("repair AVI — audio idx1 entries have flags=0")
def _():
    mod = load_module("repair_avi", "repair_avi.py")
    avi_data = build_synthetic_avi(video_frames=3, audio_chunks=3)
    src = write_temp_avi(avi_data)
    dst = src + "_repaired.avi"
    try:
        mod.repair_avi(src, dst, recording_duration=1.0)
        with open(dst, "rb") as f:
            repaired = f.read()
        idx1_pos = repaired.find(b"idx1")
        idx1_size = struct.unpack_from("<I", repaired, idx1_pos + 4)[0]
        n = idx1_size // 16
        for i in range(n):
            off = idx1_pos + 8 + i * 16
            tag = repaired[off:off + 4]
            flags = struct.unpack_from("<I", repaired, off + 4)[0]
            if tag == b"01wb":
                assert flags == 0x00, f"Audio entry {i} flags=0x{flags:x}"
    finally:
        cleanup(src, dst)

@test("repair AVI — default output path uses _repaired suffix")
def _():
    mod = load_module("repair_avi", "repair_avi.py")
    avi_data = build_synthetic_avi(video_frames=2, audio_chunks=1)
    src = write_temp_avi(avi_data)
    default_dst = src.replace(".avi", "_repaired.avi")
    try:
        mod.repair_avi(src)
        assert os.path.exists(default_dst), f"Default output {default_dst} not created"
    finally:
        cleanup(src, default_dst)

@test("repair AVI — FPS = frames/duration (30 frames / 2s = 15fps)")
def _():
    mod = load_module("repair_avi", "repair_avi.py")
    avi_data = build_synthetic_avi(video_frames=30, audio_chunks=15, fps=30)
    src = write_temp_avi(avi_data)
    dst = src + "_repaired.avi"
    try:
        mod.repair_avi(src, dst, recording_duration=2.0)
        with open(dst, "rb") as f:
            repaired = f.read()
        us_per_frame = struct.unpack_from("<I", repaired, 32)[0]
        # Allow ±1 for integer rounding (66666 or 66667)
        expected_us = 1_000_000 / 15
        assert abs(us_per_frame - expected_us) <= 1, f"us_per_frame={us_per_frame}, expected~{expected_us:.0f}"
    finally:
        cleanup(src, dst)

@test("repair AVI — video strh dwLength patched correctly")
def _():
    mod = load_module("repair_avi", "repair_avi.py")
    avi_data = build_synthetic_avi(video_frames=12, audio_chunks=6)
    src = write_temp_avi(avi_data)
    dst = src + "_repaired.avi"
    try:
        mod.repair_avi(src, dst, recording_duration=1.0)
        with open(dst, "rb") as f:
            data = f.read()
        # strh is a chunk tag, then 4-byte size, then fccType="vids"
        pos = data.find(b"strh")
        assert pos >= 0, "Video strh not found"
        # strh chunk: tag(4) + size(4) + fccType(4, "vids") + fccHandler(4) + ... dwLength at +32 from data start
        fcc_type = data[pos + 8:pos + 12]
        assert fcc_type == b"vids", f"First strh fccType={fcc_type}"
        # dwLength is at offset 32 from strh data start (pos+8+32 = pos+40)
        dwLength = struct.unpack_from("<I", data, pos + 8 + 32)[0]
        assert dwLength == 12, f"Video dwLength={dwLength}, expected 12"
    finally:
        cleanup(src, dst)

@test("repair AVI — audio strh dwLength patched")
def _():
    mod = load_module("repair_avi", "repair_avi.py")
    avi_data = build_synthetic_avi(video_frames=5, audio_chunks=4, audio_samples_per_chunk=800)
    src = write_temp_avi(avi_data)
    dst = src + "_repaired.avi"
    try:
        mod.repair_avi(src, dst, recording_duration=1.0)
        with open(dst, "rb") as f:
            data = f.read()
        # Find second strh (audio), after fccType="auds"
        first_strh = data.find(b"strh")
        second_strh = data.find(b"strh", first_strh + 4)
        assert second_strh >= 0, "Audio strh not found"
        fcc_type = data[second_strh + 8:second_strh + 12]
        assert fcc_type == b"auds", f"Second strh fccType={fcc_type}"
        # dwLength at offset 32 from strh data start
        dwLength = struct.unpack_from("<I", data, second_strh + 8 + 32)[0]
        expected = 4 * 800  # 4 chunks * 800 samples
        assert dwLength == expected, f"Audio dwLength={dwLength}, expected {expected}"
    finally:
        cleanup(src, dst)

@test("repair AVI — single frame works")
def _():
    mod = load_module("repair_avi", "repair_avi.py")
    avi_data = build_synthetic_avi(video_frames=1, audio_chunks=0)
    src = write_temp_avi(avi_data)
    dst = src + "_repaired.avi"
    try:
        mod.repair_avi(src, dst, recording_duration=1.0)
        with open(dst, "rb") as f:
            repaired = f.read()
        total_frames = struct.unpack_from("<I", repaired, 48)[0]
        assert total_frames == 1
    finally:
        cleanup(src, dst)

@test("repair AVI — large file (100 video + 50 audio chunks)")
def _():
    mod = load_module("repair_avi", "repair_avi.py")
    avi_data = build_synthetic_avi(video_frames=100, audio_chunks=50, audio_samples_per_chunk=256)
    src = write_temp_avi(avi_data)
    dst = src + "_repaired.avi"
    try:
        mod.repair_avi(src, dst, recording_duration=10.0)
        with open(dst, "rb") as f:
            repaired = f.read()
        total_frames = struct.unpack_from("<I", repaired, 48)[0]
        assert total_frames == 100
        idx1_pos = repaired.find(b"idx1")
        idx1_size = struct.unpack_from("<I", repaired, idx1_pos + 4)[0]
        assert idx1_size // 16 == 150, f"Expected 150 idx1 entries, got {idx1_size // 16}"
    finally:
        cleanup(src, dst)

@test("repair AVI — movi LIST size correct after repair")
def _():
    mod = load_module("repair_avi", "repair_avi.py")
    avi_data = build_synthetic_avi(video_frames=6, audio_chunks=3)
    src = write_temp_avi(avi_data)
    dst = src + "_repaired.avi"
    try:
        mod.repair_avi(src, dst, recording_duration=1.0)
        with open(dst, "rb") as f:
            repaired = f.read()
        movi_pos = repaired.find(b"movi") - 8
        assert repaired[movi_pos:movi_pos + 4] == b"LIST"
        movi_size = struct.unpack_from("<I", repaired, movi_pos + 4)[0]
        idx1_pos = repaired.find(b"idx1")
        expected = idx1_pos - movi_pos - 8
        assert movi_size == expected, f"movi size {movi_size} != {expected}"
    finally:
        cleanup(src, dst)

@test("repair AVI — repaired file starts with RIFF...AVI")
def _():
    mod = load_module("repair_avi", "repair_avi.py")
    avi_data = build_synthetic_avi(video_frames=3, audio_chunks=2)
    src = write_temp_avi(avi_data)
    dst = src + "_repaired.avi"
    try:
        mod.repair_avi(src, dst, recording_duration=1.0)
        with open(dst, "rb") as f:
            header = f.read(12)
        assert header[:4] == b"RIFF"
        assert header[8:12] == b"AVI "
    finally:
        cleanup(src, dst)


# ============================================================================
# TEST GROUP 2: usb_video_viewer.py — Source Inspection
# ============================================================================

print("\n" + "=" * 70)
print("  GROUP 2: usb_video_viewer.py — Source Inspection")
print("=" * 70)

VIEWER_PATH = os.path.join(PROJECT_ROOT, "usb_video_viewer.py")
with open(VIEWER_PATH, "r") as _f:
    VIEWER_SOURCE = _f.read()

@test("viewer has frame_count guard on screenshot")
def _():
    assert "frame_count > 0" in VIEWER_SOURCE or "frame_count >= 1" in VIEWER_SOURCE

@test("viewer uses JPEG SOI marker (FFD8)")
def _():
    assert "\\xFF\\xD8" in VIEWER_SOURCE

@test("viewer uses JPEG EOI marker (FFD9)")
def _():
    assert "\\xFF\\xD9" in VIEWER_SOURCE

@test("viewer uses high baud rate (921600)")
def _():
    assert "921600" in VIEWER_SOURCE

@test("viewer has OpenCV display")
def _():
    assert "imshow" in VIEWER_SOURCE

@test("viewer has keyboard quit ('q' key)")
def _():
    assert "ord('q')" in VIEWER_SOURCE

@test("viewer has serial port argument handling")
def _():
    assert "sys.argv" in VIEWER_SOURCE

@test("viewer calculates FPS from timestamps")
def _():
    assert "fps" in VIEWER_SOURCE.lower() and "time" in VIEWER_SOURCE.lower()

@test("viewer has frame buffer management")
def _():
    assert "buffer" in VIEWER_SOURCE

@test("viewer cleans up serial + windows on exit")
def _():
    assert "ser.close()" in VIEWER_SOURCE
    assert "destroyAllWindows" in VIEWER_SOURCE


# ============================================================================
# TEST GROUP 3: avi_audio_check.py — Functional Tests
# ============================================================================

print("\n" + "=" * 70)
print("  GROUP 3: avi_audio_check.py — Functional Tests")
print("=" * 70)

@test("avi_audio_check imports successfully")
def _():
    mod = load_module("avi_audio_check", "avi_audio_check.py")
    assert hasattr(mod, "analyze_avi")

@test("avi_audio_check has read_fourcc, read_u32, read_u16")
def _():
    mod = load_module("avi_audio_check", "avi_audio_check.py")
    assert hasattr(mod, "read_fourcc")
    assert hasattr(mod, "read_u32")
    assert hasattr(mod, "read_u16")

@test("read_fourcc returns 4-byte tag")
def _():
    mod = load_module("avi_audio_check", "avi_audio_check.py")
    import io
    f = io.BytesIO(b"RIFF")
    assert mod.read_fourcc(f) == b"RIFF"

@test("read_u32 decodes little-endian uint32")
def _():
    mod = load_module("avi_audio_check", "avi_audio_check.py")
    import io
    f = io.BytesIO(struct.pack("<I", 0xDEADBEEF))
    assert mod.read_u32(f) == 0xDEADBEEF

@test("read_u16 decodes little-endian uint16")
def _():
    mod = load_module("avi_audio_check", "avi_audio_check.py")
    import io
    f = io.BytesIO(struct.pack("<H", 0xCAFE))
    assert mod.read_u16(f) == 0xCAFE

@test("read_u32 returns None on short data")
def _():
    mod = load_module("avi_audio_check", "avi_audio_check.py")
    import io
    assert mod.read_u32(io.BytesIO(b"\x00\x01")) is None

@test("read_u16 returns None on short data")
def _():
    mod = load_module("avi_audio_check", "avi_audio_check.py")
    import io
    assert mod.read_u16(io.BytesIO(b"\x00")) is None


# ============================================================================
# TEST GROUP 4: AVI Header Constant Cross-Checks (C ↔ Python)
# ============================================================================

print("\n" + "=" * 70)
print("  GROUP 4: AVI Header Constant Cross-Checks (C <-> Python)")
print("=" * 70)

C_SOURCE = read_c_source()
C_DEFINES = parse_c_defines(C_SOURCE)

@test("AVI_HDR_RIFF_SIZE = 4")
def _():
    assert C_DEFINES["AVI_HDR_RIFF_SIZE"] == 4

@test("AVI_HDR_USPERFRAME = 32")
def _():
    assert C_DEFINES["AVI_HDR_USPERFRAME"] == 32

@test("AVI_HDR_FLAGS = 44")
def _():
    assert C_DEFINES["AVI_HDR_FLAGS"] == 44

@test("AVI_HDR_TOTALFRAMES = 48")
def _():
    assert C_DEFINES["AVI_HDR_TOTALFRAMES"] == 48

@test("AVI_HDR_VID_RATE = 132")
def _():
    assert C_DEFINES["AVI_HDR_VID_RATE"] == 132

@test("AVI_HDR_VID_LENGTH = 140")
def _():
    assert C_DEFINES["AVI_HDR_VID_LENGTH"] == 140

@test("AVI_HDR_AUD_LENGTH = 264")
def _():
    assert C_DEFINES["AVI_HDR_AUD_LENGTH"] == 264

@test("AVIIF_KEYFRAME = 0x10")
def _():
    assert C_DEFINES["AVIIF_KEYFRAME"] == 0x10

@test("AVIF_HASINDEX_INTERLEAVED = 0x110")
def _():
    assert C_DEFINES["AVIF_HASINDEX_INTERLEAVED"] == 0x110

@test("Python repair_avi uses matching keyframe 0x10")
def _():
    with open(os.path.join(PROJECT_ROOT, "repair_avi.py"), "r") as f:
        src = f.read()
    assert "0x10" in src

@test("Python repair patches RIFF at offset 4")
def _():
    with open(os.path.join(PROJECT_ROOT, "repair_avi.py"), "r") as f:
        src = f.read()
    assert "out, 4," in src

@test("Python repair patches totalframes at offset 48")
def _():
    with open(os.path.join(PROJECT_ROOT, "repair_avi.py"), "r") as f:
        src = f.read()
    assert "out, 48," in src

@test("Python repair patches usperframe at offset 32")
def _():
    with open(os.path.join(PROJECT_ROOT, "repair_avi.py"), "r") as f:
        src = f.read()
    assert "out, 32," in src


# ============================================================================
# TEST GROUP 5: AVI Header Binary Layout Verification
# ============================================================================

print("\n" + "=" * 70)
print("  GROUP 5: AVI Header Binary Layout Verification")
print("=" * 70)

@test("AVI: RIFF tag at byte 0")
def _():
    avi = build_synthetic_avi(video_frames=1, audio_chunks=1)
    assert avi[0:4] == b"RIFF"

@test("AVI: file type 'AVI ' at byte 8")
def _():
    avi = build_synthetic_avi(video_frames=1, audio_chunks=1)
    assert avi[8:12] == b"AVI "

@test("AVI: hdrl LIST at byte 12")
def _():
    avi = build_synthetic_avi(video_frames=1, audio_chunks=1)
    assert avi[12:16] == b"LIST"
    assert avi[20:24] == b"hdrl"

@test("AVI: avih chunk at byte 24, size=56")
def _():
    avi = build_synthetic_avi(video_frames=1, audio_chunks=1)
    assert avi[24:28] == b"avih"
    assert struct.unpack_from("<I", avi, 28)[0] == 56

@test("AVI: usperframe at byte 32 correct for 30fps")
def _():
    avi = build_synthetic_avi(video_frames=1, audio_chunks=1, fps=30)
    assert struct.unpack_from("<I", avi, 32)[0] == 33333

@test("AVI: dwFlags at byte 44 is 0 initially")
def _():
    avi = build_synthetic_avi(video_frames=1, audio_chunks=1)
    assert struct.unpack_from("<I", avi, 44)[0] == 0

@test("AVI: dwTotalFrames at byte 48")
def _():
    avi = build_synthetic_avi(video_frames=42, audio_chunks=1)
    assert struct.unpack_from("<I", avi, 48)[0] == 42

@test("AVI: dwStreams=2 at byte 56")
def _():
    avi = build_synthetic_avi(video_frames=1, audio_chunks=1)
    assert struct.unpack_from("<I", avi, 56)[0] == 2

@test("AVI: dwWidth and dwHeight in avih")
def _():
    avi = build_synthetic_avi(video_frames=1, audio_chunks=1, width=1920, height=1080)
    assert struct.unpack_from("<I", avi, 64)[0] == 1920
    assert struct.unpack_from("<I", avi, 68)[0] == 1080

@test("AVI: video stream has 'vids' + 'H264'")
def _():
    avi = build_synthetic_avi(video_frames=1, audio_chunks=1)
    pos = avi.find(b"vids")
    assert pos > 0
    assert avi[pos + 4:pos + 8] == b"H264"

@test("AVI: audio stream has 'auds'")
def _():
    avi = build_synthetic_avi(video_frames=1, audio_chunks=1)
    assert avi.find(b"auds") > 0

@test("AVI: audio strf has PCM format tag (1)")
def _():
    avi = build_synthetic_avi(video_frames=1, audio_chunks=1)
    auds_pos = avi.find(b"auds")
    strf_pos = avi.find(b"strf", auds_pos)
    assert strf_pos > 0
    format_tag = struct.unpack_from("<H", avi, strf_pos + 8)[0]
    assert format_tag == 1

@test("AVI: movi LIST present with correct tag")
def _():
    avi = build_synthetic_avi(video_frames=1, audio_chunks=1)
    movi_pos = avi.find(b"movi")
    assert movi_pos > 0
    assert avi[movi_pos - 8:movi_pos - 4] == b"LIST"

@test("AVI: video chunks use '00dc' tag")
def _():
    avi = build_synthetic_avi(video_frames=1, audio_chunks=0)
    movi_pos = avi.find(b"movi")
    assert avi[movi_pos + 4:movi_pos + 8] == b"00dc"

@test("AVI: audio chunks use '01wb' tag")
def _():
    avi = build_synthetic_avi(video_frames=1, audio_chunks=1)
    assert avi.find(b"01wb") > 0


# ============================================================================
# TEST GROUP 6: C Code Structure & Configuration Sanity
# ============================================================================

print("\n" + "=" * 70)
print("  GROUP 6: C Code Structure & Configuration Sanity")
print("=" * 70)

@test("No duplicate globals (preprocessor-aware)")
def _():
    lines = C_SOURCE.split('\n')
    pp_depth = 0
    pp_branch = [0]
    globals_seen = {}
    for i, line in enumerate(lines, 1):
        stripped = line.strip()
        if stripped.startswith("#if"):
            pp_depth += 1
            pp_branch.append(0)
        elif stripped.startswith("#elif") or stripped.startswith("#else"):
            if pp_depth < len(pp_branch):
                pp_branch[pp_depth] += 1
        elif stripped.startswith("#endif"):
            if pp_depth > 0:
                pp_branch.pop()
                pp_depth -= 1
        m = re.match(r'^static\s+(?:const\s+)?(?:volatile\s+)?\w[\w\s*]+\s+(\w+)\s*[=;{\[]', line)
        if m:
            name = m.group(1)
            branch_key = tuple(pp_branch[:pp_depth + 1])
            if name in globals_seen:
                prev_num, prev_branch = globals_seen[name]
                prev_content = lines[prev_num - 1].strip()
                if "=" in prev_content and "=" in stripped and prev_branch == branch_key:
                    raise AssertionError(f"Duplicate '{name}' at lines {prev_num} and {i}")
            globals_seen[name] = (i, branch_key)

@test("avi_update_headers uses all AVI_HDR_* constants")
def _():
    func = re.search(r'static void avi_update_headers\(.*?\n\}', C_SOURCE, re.DOTALL)
    assert func
    body = func.group(0)
    for c in ["AVI_HDR_RIFF_SIZE", "AVI_HDR_USPERFRAME", "AVI_HDR_TOTALFRAMES",
              "AVI_HDR_VID_RATE", "AVI_HDR_VID_LENGTH", "AVI_HDR_AUD_LENGTH"]:
        assert c in body, f"Missing {c}"

@test("avi_finalize uses AVIF_HASINDEX_INTERLEAVED")
def _():
    func = re.search(r'static void avi_finalize\(.*?\n\}', C_SOURCE, re.DOTALL)
    assert func
    assert "AVIF_HASINDEX_INTERLEAVED" in func.group(0)

@test("STAGING_BUF_SIZE >= 96KB")
def _():
    assert C_DEFINES["STAGING_BUF_SIZE"] >= 96 * 1024

@test("Total staging memory < 8MB")
def _():
    total = C_DEFINES["NUM_STAGING_BUFS"] * C_DEFINES["STAGING_BUF_SIZE"]
    assert total <= 8 * 1024 * 1024, f"{total / 1024 / 1024:.1f}MB"

@test("NUM_CAP_BUFFERS >= 3 (DMA minimum)")
def _():
    assert C_DEFINES["NUM_CAP_BUFFERS"] >= 3

@test("H264_GOP in [1, 60]")
def _():
    assert 1 <= C_DEFINES["H264_GOP"] <= 60

@test("H264_MIN_QP < H264_MAX_QP")
def _():
    assert C_DEFINES["H264_MIN_QP"] < C_DEFINES["H264_MAX_QP"]

@test("H264 QP range within [0, 51]")
def _():
    assert 0 <= C_DEFINES["H264_MIN_QP"] <= 51
    assert 0 <= C_DEFINES["H264_MAX_QP"] <= 51

@test("H264_BITRATE reasonable (1-50 Mbps)")
def _():
    assert 1_000_000 <= C_DEFINES["H264_BITRATE"] <= 50_000_000

@test("AVI_FPS = 30")
def _():
    assert C_DEFINES["AVI_FPS"] == 30

@test("AUDIO_SAMPLE_RATE = 16000")
def _():
    assert C_DEFINES["AUDIO_SAMPLE_RATE"] == 16000

@test("AUDIO_BITS = 16")
def _():
    assert C_DEFINES["AUDIO_BITS"] == 16

@test("AUDIO_CHANNELS = 1")
def _():
    assert C_DEFINES["AUDIO_CHANNELS"] == 1

@test("SD pins correct (CMD=44 CLK=43 D0-3=39-42)")
def _():
    exp = {"SD_CMD_PIN": 44, "SD_CLK_PIN": 43, "SD_D0_PIN": 39,
           "SD_D1_PIN": 40, "SD_D2_PIN": 41, "SD_D3_PIN": 42}
    for name, val in exp.items():
        assert C_DEFINES[name] == val, f"{name}={C_DEFINES.get(name)}"

@test("I2C pins correct (SDA=7 SCL=8 400kHz)")
def _():
    assert C_DEFINES["CAM_I2C_SDA"] == 7
    assert C_DEFINES["CAM_I2C_SCL"] == 8
    assert C_DEFINES["CAM_I2C_FREQ"] == 400000

@test("PDM mic pins correct (CLK=12 DATA=9)")
def _():
    assert C_DEFINES["PDM_CLK_PIN"] == 12
    assert C_DEFINES["PDM_DATA_PIN"] == 9

@test("LED_PIN = 3")
def _():
    assert C_DEFINES["LED_PIN"] == 3

@test("AVI_SEGMENT_MAX_BYTES = 100MB")
def _():
    assert C_DEFINES["AVI_SEGMENT_MAX_BYTES"] == 100 * 1024 * 1024

@test("AVI_MAX_IDX_ENTRIES >= 4000")
def _():
    assert C_DEFINES["AVI_MAX_IDX_ENTRIES"] >= 4000

@test("SKIP_STARTUP_FRAMES = 30")
def _():
    assert C_DEFINES["SKIP_STARTUP_FRAMES"] == 30

@test("CSI_AFULL_THRESHOLD reasonable [1024, 4096]")
def _():
    assert 1024 <= C_DEFINES["CSI_AFULL_THRESHOLD"] <= 4096

@test("AUDIO_RING_SIZE = 2s of 16-bit mono at 16kHz")
def _():
    assert C_DEFINES["AUDIO_RING_SIZE"] == 16000 * 2 * 2

@test("All DIAG_ toggles default to 0")
def _():
    for t in ["DIAG_USE_720P", "DIAG_LOW_BITRATE", "DIAG_NO_AUDIO", "DIAG_USE_1080P15"]:
        assert C_DEFINES.get(t) == 0, f"{t}={C_DEFINES.get(t)}"


# ============================================================================
# TEST GROUP 7: H.264 Keyframe Detection (Logic Mirror)
# ============================================================================

print("\n" + "=" * 70)
print("  GROUP 7: H.264 Keyframe Detection (Logic Mirror)")
print("=" * 70)

def h264_is_keyframe(data, size=None):
    """Python mirror of h264_is_keyframe() from camera_recorder.c"""
    if size is None:
        size = len(data)
    limit = min(size, 128)
    i = 0
    while i + 4 < limit:
        if data[i] == 0 and data[i+1] == 0 and data[i+2] == 0 and data[i+3] == 1:
            nal_type = data[i + 4] & 0x1F
            if nal_type == 5:
                return True
            i += 4
        else:
            i += 1
    return False

@test("IDR (NAL type 5) → keyframe")
def _():
    assert h264_is_keyframe(bytes([0,0,0,1, 0x65, 0xAB])) is True

@test("P-frame (NAL type 1) → not keyframe")
def _():
    assert h264_is_keyframe(bytes([0,0,0,1, 0x41, 0xAB])) is False

@test("SPS (NAL type 7) → not keyframe")
def _():
    assert h264_is_keyframe(bytes([0,0,0,1, 0x67, 0xAB])) is False

@test("PPS (NAL type 8) → not keyframe")
def _():
    assert h264_is_keyframe(bytes([0,0,0,1, 0x68, 0xAB])) is False

@test("SPS+PPS+IDR → keyframe (IDR found after prefix)")
def _():
    data = bytes([0,0,0,1, 0x67, 0x42, 0x00, 0x1E,
                  0,0,0,1, 0x68, 0xCE, 0x38, 0x80,
                  0,0,0,1, 0x65, 0x88, 0x84, 0x00])
    assert h264_is_keyframe(data) is True

@test("IDR at byte 130+ → NOT detected (128-byte limit)")
def _():
    data = bytes(130) + bytes([0,0,0,1, 0x65])
    assert h264_is_keyframe(data) is False

@test("Empty data → False")
def _():
    assert h264_is_keyframe(b"") is False

@test("Short data (4 bytes) → False")
def _():
    assert h264_is_keyframe(bytes([0,0,0,1])) is False

@test("3-byte start code (00 00 01) → NOT matched")
def _():
    assert h264_is_keyframe(bytes([0,0,1, 0x65, 0xAB, 0xCD, 0xEF, 0x00])) is False

@test("NRI bits set (0xE5 & 0x1F = 5) → keyframe")
def _():
    assert h264_is_keyframe(bytes([0,0,0,1, 0xE5, 0xAB])) is True

@test("Forbidden bit set (0x85 & 0x1F = 5) → still matches")
def _():
    assert h264_is_keyframe(bytes([0,0,0,1, 0x85, 0xAB])) is True

@test("Multiple NALs, IDR after non-IDR → found")
def _():
    data = (bytes([0,0,0,1, 0x67]) + bytes(10) +
            bytes([0,0,0,1, 0x65]) + bytes(10))
    assert h264_is_keyframe(data) is True

@test("All zeros data → not keyframe (00 00 00 01 00 → NAL type 0)")
def _():
    data = bytes(128)
    # This has 00 00 00 01 at position 0, but NAL type = 0 (not 5)
    assert h264_is_keyframe(data) is False


# ============================================================================
# TEST GROUP 8: Audio Ring Buffer Drain (Logic Mirror)
# ============================================================================

print("\n" + "=" * 70)
print("  GROUP 8: Audio Ring Buffer Drain (Logic Mirror)")
print("=" * 70)

RING_SIZE = 64000

def audio_drain(ring, wr, rd, dst, max_bytes, ring_size=RING_SIZE):
    """Python mirror of audio_drain() from camera_recorder.c"""
    avail = wr - rd
    if avail > ring_size:
        rd = wr - ring_size
    if avail > max_bytes:
        avail = max_bytes
    avail &= ~1
    if avail == 0:
        return 0, rd
    start = rd % ring_size
    first = ring_size - start
    if first >= avail:
        dst[:avail] = ring[start:start + avail]
    else:
        dst[:first] = ring[start:start + first]
        dst[first:avail] = ring[0:avail - first]
    return avail, rd + avail

@test("drain: simple linear read")
def _():
    ring = bytearray(RING_SIZE)
    ring[0:10] = b'\xAA' * 10
    dst = bytearray(1000)
    n, _ = audio_drain(ring, 10, 0, dst, 1000)
    assert n == 10
    assert dst[:10] == b'\xAA' * 10

@test("drain: wrap-around read")
def _():
    sz = 100
    ring = bytearray(sz)
    ring[90:100] = b'\xBB' * 10
    ring[0:10] = b'\xCC' * 10
    dst = bytearray(100)
    n, _ = audio_drain(ring, 120, 90, dst, 100, ring_size=sz)
    assert n == 30  # avail=30
    assert dst[:10] == b'\xBB' * 10
    assert dst[10:20] == b'\xCC' * 10

@test("drain: overrun recovery skips to latest")
def _():
    sz = 100
    ring = bytearray(sz)
    dst = bytearray(300)
    # C code: avail=wr-rd=250, avail>ring_size → rd=200, avail(250)>max(200) → avail=200
    n, new_rd = audio_drain(ring, 300, 50, dst, 200, ring_size=sz)
    assert n == 200
    assert new_rd == 400  # rd was 200, read 200

@test("drain: odd avail rounded down to even")
def _():
    ring = bytearray(RING_SIZE)
    ring[:11] = b'\xDD' * 11
    dst = bytearray(100)
    n, _ = audio_drain(ring, 11, 0, dst, 100)
    assert n == 10

@test("drain: empty buffer → 0")
def _():
    ring = bytearray(RING_SIZE)
    dst = bytearray(100)
    n, _ = audio_drain(ring, 50, 50, dst, 100)
    assert n == 0

@test("drain: max_bytes limits output")
def _():
    ring = bytearray(RING_SIZE)
    ring[:100] = bytes(range(100))
    dst = bytearray(200)
    n, _ = audio_drain(ring, 100, 0, dst, 20)
    assert n == 20

@test("drain: single byte available → 0 (odd)")
def _():
    ring = bytearray(RING_SIZE)
    dst = bytearray(100)
    n, _ = audio_drain(ring, 1, 0, dst, 100)
    assert n == 0

@test("drain: exactly ring_size available → full drain")
def _():
    sz = 100
    ring = bytearray(sz)
    for i in range(sz):
        ring[i] = i % 256
    dst = bytearray(sz)
    n, _ = audio_drain(ring, sz, 0, dst, sz, ring_size=sz)
    assert n == sz
    assert dst == ring

@test("drain: max_bytes = 0 → returns 0")
def _():
    ring = bytearray(RING_SIZE)
    dst = bytearray(100)
    n, _ = audio_drain(ring, 100, 0, dst, 0)
    assert n == 0


# ============================================================================
# TEST GROUP 9: EIS Biquad / Notch Filter Math
# ============================================================================

print("\n" + "=" * 70)
print("  GROUP 9: EIS Biquad / Notch Filter Math")
print("=" * 70)

class BiquadFilter:
    def __init__(self):
        self.b0 = self.b1 = self.b2 = 0.0
        self.a1 = self.a2 = 0.0
        self.z1 = self.z2 = 0.0

    def init_notch(self, freq_hz, q, fs):
        w0 = 2.0 * math.pi * freq_hz / fs
        alpha = math.sin(w0) / (2.0 * q)
        cos_w0 = math.cos(w0)
        a0 = 1.0 + alpha
        self.b0 = 1.0 / a0
        self.b1 = (-2.0 * cos_w0) / a0
        self.b2 = 1.0 / a0
        self.a1 = (-2.0 * cos_w0) / a0
        self.a2 = (1.0 - alpha) / a0
        self.z1 = 0.0
        self.z2 = 0.0

    def process(self, x):
        y = self.b0 * x + self.z1
        self.z1 = self.b1 * x - self.a1 * y + self.z2
        self.z2 = self.b2 * x - self.a2 * y
        return y

@test("notch: b0 == b2 (symmetry)")
def _():
    bq = BiquadFilter()
    bq.init_notch(5.0, 2.0, 30.0)
    assert abs(bq.b0 - bq.b2) < 1e-10

@test("notch: b1 == a1 (notch property)")
def _():
    bq = BiquadFilter()
    bq.init_notch(8.0, 2.0, 30.0)
    assert abs(bq.b1 - bq.a1) < 1e-10

@test("notch: DC gain = 1.0")
def _():
    bq = BiquadFilter()
    bq.init_notch(5.0, 2.0, 30.0)
    dc_gain = (bq.b0 + bq.b1 + bq.b2) / (1.0 + bq.a1 + bq.a2)
    assert abs(dc_gain - 1.0) < 1e-6

@test("notch: attenuates center frequency (< 1% power)")
def _():
    bq = BiquadFilter()
    freq = 5.0
    bq.init_notch(freq, 2.0, 30.0)
    fs = 30.0
    output = [bq.process(math.sin(2*math.pi*freq*i/fs)) for i in range(300)]
    power = sum(y*y for y in output[-100:]) / 100
    ref = sum(math.sin(2*math.pi*freq*i/fs)**2 for i in range(200, 300)) / 100
    assert power / max(ref, 1e-10) < 0.01

@test("notch: passes non-notch frequency (> 80%)")
def _():
    bq = BiquadFilter()
    bq.init_notch(5.0, 2.0, 30.0)
    fs = 30.0
    output = [bq.process(math.sin(2*math.pi*1.0*i/fs)) for i in range(300)]
    pwr_out = sum(y*y for y in output[-100:]) / 100
    pwr_in = sum(math.sin(2*math.pi*1.0*i/fs)**2 for i in range(200, 300)) / 100
    assert pwr_out / max(pwr_in, 1e-10) > 0.8

@test("3-stage cascade attenuates all notch frequencies")
def _():
    freqs = [5.0, 8.0, 13.0]
    cascade = [BiquadFilter() for _ in range(3)]
    fs = 30.0
    for freq in freqs:
        for i, f in enumerate(freqs):
            cascade[i].init_notch(f, 2.0, fs)
        output = []
        for i in range(300):
            x = math.sin(2*math.pi*freq*i/fs)
            y = x
            for bq in cascade:
                y = bq.process(y)
            output.append(y)
        power = sum(y*y for y in output[-100:]) / 100
        assert power < 0.01, f"Freq {freq}Hz pass-through power={power:.4f}"

@test("biquad: zero input → zero output")
def _():
    bq = BiquadFilter()
    bq.init_notch(5.0, 2.0, 30.0)
    y = 0
    for _ in range(100):
        y = bq.process(0.0)
    assert abs(y) < 1e-15

@test("notch at Nyquist/2: stable coefficients")
def _():
    bq = BiquadFilter()
    bq.init_notch(7.5, 2.0, 30.0)  # 7.5 Hz = Nyquist/2 at 30 fps
    assert not math.isnan(bq.b0)
    assert not math.isinf(bq.a2)

@test("biquad: step response settles to 1.0 (DC)")
def _():
    bq = BiquadFilter()
    bq.init_notch(5.0, 2.0, 30.0)
    y = 0
    for _ in range(500):
        y = bq.process(1.0)
    assert abs(y - 1.0) < 0.01, f"Step response settled to {y}"


# ============================================================================
# TEST GROUP 10: EIS Block Matching & Motion Estimation
# ============================================================================

print("\n" + "=" * 70)
print("  GROUP 10: EIS Block Matching & Motion Estimation")
print("=" * 70)

EIS_BLOCK_SIZE = 16
EIS_SEARCH_RANGE = 8

def eis_sad(ref, cur, stride):
    sad = 0
    for y in range(EIS_BLOCK_SIZE):
        for x in range(EIS_BLOCK_SIZE):
            sad += abs(int(ref[y * EIS_BLOCK_SIZE + x]) - int(cur[y * stride + x]))
    return sad

def eis_search(ref_block, y_plane, stride, w, h, cx, cy):
    best = float('inf')
    bx, by_ = 0, 0
    half = EIS_BLOCK_SIZE // 2
    for sy in range(-EIS_SEARCH_RANGE, EIS_SEARCH_RANGE + 1):
        for sx in range(-EIS_SEARCH_RANGE, EIS_SEARCH_RANGE + 1):
            px = cx + sx - half
            py = cy + sy - half
            if px < 0 or py < 0 or px + EIS_BLOCK_SIZE > w or py + EIS_BLOCK_SIZE > h:
                continue
            s = eis_sad(ref_block, y_plane[py * stride + px:], stride)
            if s < best:
                best = s
                bx, by_ = sx, sy
    return bx, by_

@test("SAD: identical blocks = 0")
def _():
    block = bytes(range(256))
    plane = bytearray(32 * 32)
    for r in range(16):
        plane[r*32:r*32+16] = block[r*16:(r+1)*16]
    assert eis_sad(block, plane, 32) == 0

@test("SAD: max difference (0 vs 255) = 256*255")
def _():
    ref = bytes([0]*256)
    cur = bytearray(32 * 32)
    for r in range(16):
        cur[r*32:r*32+16] = bytes([255]*16)
    assert eis_sad(ref, cur, 32) == 256 * 255

@test("SAD: single pixel diff = diff value")
def _():
    ref = bytearray(256)
    plane = bytearray(32 * 32)
    plane[0] = 42
    assert eis_sad(ref, plane, 32) == 42

@test("search: static scene → (0,0)")
def _():
    w, h = 64, 64
    plane = bytearray(w * h)
    cx, cy = 32, 32
    ref = bytes(range(256))
    for r in range(16):
        plane[(cy-8+r)*w + (cx-8):(cy-8+r)*w + (cx-8+16)] = ref[r*16:(r+1)*16]
    dx, dy = eis_search(ref, plane, w, w, h, cx, cy)
    assert (dx, dy) == (0, 0)

@test("search: +3,-2 shift found correctly")
def _():
    w, h = 128, 128
    plane = bytearray(w * h)
    ref = bytes([i % 256 for i in range(256)])
    cx, cy = 64, 64
    sx, sy = 3, -2
    for r in range(16):
        plane[(cy+sy-8+r)*w + (cx+sx-8):(cy+sy-8+r)*w + (cx+sx-8+16)] = ref[r*16:(r+1)*16]
    dx, dy = eis_search(ref, plane, w, w, h, cx, cy)
    assert (dx, dy) == (sx, sy), f"Got ({dx},{dy})"

@test("EIS crop clamp: extreme values stay in [0, 2*margin]")
def _():
    mx, my = 128, 72
    for cx, cy in [(-999, -999), (999, 999), (0, 0)]:
        crop_x = max(0, min(2*mx, mx + int(max(-mx*0.9, min(mx*0.9, cx)))))
        crop_y = max(0, min(2*my, my + int(max(-my*0.9, min(my*0.9, cy)))))
        assert 0 <= crop_x <= 2*mx
        assert 0 <= crop_y <= 2*my

@test("EIS margin: 1920x1080 → 1280x720 = 320x180")
def _():
    assert (1920 - 1280) // 2 == 320
    assert (1080 - 720) // 2 == 180

@test("EIS margin: 1536x864 → 1280x720 = 128x72")
def _():
    assert (1536 - 1280) // 2 == 128
    assert (864 - 720) // 2 == 72

@test("EIS smooth alpha gives ~20 frame time constant")
def _():
    tau = 1.0 / 0.05
    assert 10 <= tau <= 100


# ============================================================================
# TEST GROUP 11: IMX708 Register Validation
# ============================================================================

print("\n" + "=" * 70)
print("  GROUP 11: IMX708 Register Validation")
print("=" * 70)

@test("1920x1080 digital crop centers in 2304x1296")
def _():
    assert (2304 - 1920) // 2 == 192  # 0xC0
    assert (1296 - 1080) // 2 == 108  # 0x6C

@test("1280x720 digital crop centers in 2304x1296")
def _():
    assert (2304 - 1280) // 2 == 512  # 0x200
    assert (1296 - 720) // 2 == 288   # 0x120

@test("1920x1080 output size regs present in C")
def _():
    assert "{0x034C, 0x07}" in C_SOURCE and "{0x034D, 0x80}" in C_SOURCE
    assert "{0x034E, 0x04}" in C_SOURCE and "{0x034F, 0x38}" in C_SOURCE

@test("1280x720 output size regs present in C")
def _():
    assert "{0x034C, 0x05}" in C_SOURCE and "{0x034D, 0x00}" in C_SOURCE
    assert "{0x034E, 0x02}" in C_SOURCE and "{0x034F, 0xD0}" in C_SOURCE

@test("2x2 binning enabled (0x0900=1, 0x0901=0x22)")
def _():
    assert "{0x0900, 0x01}" in C_SOURCE
    assert "{0x0901, 0x22}" in C_SOURCE

@test("180° rotation (0x0101=0x03)")
def _():
    assert "{0x0101, 0x03}" in C_SOURCE

@test("Bayer order BGGR for 180° rotation")
def _():
    assert "ESP_CAM_SENSOR_BAYER_BGGR" in C_SOURCE

@test("MIPI clock = 450MHz, 2 lanes")
def _():
    assert ".mipi_clk = 450000000" in C_SOURCE
    assert ".lane_num = 2" in C_SOURCE

@test("XCLK = 24MHz")
def _():
    assert ".xclk = 24000000" in C_SOURCE

@test("RAW10 pixel format")
def _():
    assert "ESP_CAM_SENSOR_PIXFORMAT_RAW10" in C_SOURCE

@test("Register terminator {0xFFFF, 0x00}")
def _():
    assert "{0xFFFF, 0x00}" in C_SOURCE

@test("Scale-only mode 0x41 (not crash-prone 0x43)")
def _():
    assert "{0x3200, 0x41}" in C_SOURCE


# ============================================================================
# TEST GROUP 12: Build System Validation
# ============================================================================

print("\n" + "=" * 70)
print("  GROUP 12: Build System Validation")
print("=" * 70)

@test("Root CMakeLists.txt has project.cmake include")
def _():
    with open(os.path.join(PROJECT_ROOT, "CMakeLists.txt"), "r") as f:
        content = f.read()
    assert "project.cmake" in content
    assert "project(" in content

@test("Root CMakeLists.txt references esp-video-components")
def _():
    with open(os.path.join(PROJECT_ROOT, "CMakeLists.txt"), "r") as f:
        content = f.read()
    assert "esp_cam_sensor" in content and "esp_video" in content

@test("main/CMakeLists.txt lists camera_recorder.c")
def _():
    with open(os.path.join(PROJECT_ROOT, "main", "CMakeLists.txt"), "r") as f:
        content = f.read()
    assert "camera_recorder.c" in content

@test("main/CMakeLists.txt has all required dependencies")
def _():
    with open(os.path.join(PROJECT_ROOT, "main", "CMakeLists.txt"), "r") as f:
        content = f.read()
    for dep in ["driver", "esp_driver_gpio", "esp_driver_i2c", "esp_driver_i2s",
                "esp_driver_ppa", "esp_driver_sdmmc", "fatfs", "sdmmc", "esp_timer"]:
        assert dep in content, f"Missing: {dep}"

@test("idf_component.yml exists")
def _():
    assert os.path.exists(os.path.join(PROJECT_ROOT, "main", "idf_component.yml"))

@test("sdkconfig.defaults exists")
def _():
    assert os.path.exists(os.path.join(PROJECT_ROOT, "sdkconfig.defaults"))

@test("components/esp-video-components/ exists")
def _():
    assert os.path.isdir(os.path.join(PROJECT_ROOT, "components", "esp-video-components"))


# ============================================================================
# TEST GROUP 13: AVI Chunk Writing Logic
# ============================================================================

print("\n" + "=" * 70)
print("  GROUP 13: AVI Chunk Writing Logic")
print("=" * 70)

@test("chunk: even data → no padding")
def _():
    chunk = b"00dc" + struct.pack("<I", 100) + bytes(100)
    assert len(chunk) == 108 and len(chunk) % 2 == 0

@test("chunk: odd data → 1 padding byte")
def _():
    chunk = b"00dc" + struct.pack("<I", 101) + bytes(101) + b'\x00'
    assert len(chunk) == 110 and len(chunk) % 2 == 0

@test("chunk: size field = data size (not padded size)")
def _():
    chunk = b"00dc" + struct.pack("<I", 101) + bytes(101) + b'\x00'
    assert struct.unpack_from("<I", chunk, 4)[0] == 101

@test("idx1 entry is exactly 16 bytes")
def _():
    entry = struct.pack("<4sIII", b"00dc", 0x10, 1234, 5678)
    assert len(entry) == 16

@test("idx1 video: AVIIF_KEYFRAME = 0x10")
def _():
    entry = struct.pack("<4sIII", b"00dc", 0x10, 0, 100)
    assert struct.unpack_from("<I", entry, 4)[0] == 0x10

@test("idx1 audio: flags = 0")
def _():
    entry = struct.pack("<4sIII", b"01wb", 0, 0, 1024)
    assert struct.unpack_from("<I", entry, 4)[0] == 0

@test("AVI FourCC tags all 4 bytes")
def _():
    for cc in [b"00dc", b"01wb", b"idx1", b"RIFF", b"AVI ", b"LIST", b"hdrl",
               b"avih", b"strl", b"strh", b"strf", b"movi", b"vids", b"H264", b"auds"]:
        assert len(cc) == 4


# ============================================================================
# TEST GROUP 14: ISP Configuration Validation
# ============================================================================

print("\n" + "=" * 70)
print("  GROUP 14: ISP Configuration Validation")
print("=" * 70)

@test("ISP WB: neutral red_gain = 1.0")
def _():
    m = re.search(r'\.red_gain\s*=\s*([\d.]+)', C_SOURCE)
    assert m, "red_gain not found"
    assert float(m.group(1)) == 1.0

@test("ISP WB: neutral blue_gain = 1.0")
def _():
    m = re.search(r'\.blue_gain\s*=\s*([\d.]+)', C_SOURCE)
    assert m, "blue_gain not found"
    assert float(m.group(1)) == 1.0

@test("ISP CCM references identity")
def _():
    assert "identity" in C_SOURCE.lower() or "1.0" in C_SOURCE

@test("ISP demosaic gradient_ratio present")
def _():
    assert "gradient_ratio" in C_SOURCE

@test("ISP sharpen control exists")
def _():
    assert "sharpen" in C_SOURCE.lower()

@test("init_isp_white_balance function exists")
def _():
    assert re.search(r'static\s+\w+\s+init_isp_white_balance\s*\(', C_SOURCE)

@test("init_isp_color function exists")
def _():
    assert re.search(r'static\s+\w+\s+init_isp_color\s*\(', C_SOURCE)


# ============================================================================
# TEST GROUP 15: Pipeline & Task Configuration
# ============================================================================

print("\n" + "=" * 70)
print("  GROUP 15: Pipeline & Task Configuration")
print("=" * 70)

@test("capture_task pinned to Core 0")
def _():
    m = re.search(r'xTaskCreatePinnedToCore\s*\([^)]*capture_task[^)]*,\s*(\d)\s*\)', C_SOURCE, re.DOTALL)
    assert m and int(m.group(1)) == 0

@test("sd_write_task pinned to Core 1")
def _():
    m = re.search(r'xTaskCreatePinnedToCore\s*\([^)]*sd_write_task[^)]*,\s*(\d)\s*\)', C_SOURCE, re.DOTALL)
    assert m and int(m.group(1)) == 1

@test("write_queue created")
def _():
    assert "write_queue" in C_SOURCE and "xQueueCreate" in C_SOURCE

@test("staging_buf_sem created")
def _():
    assert "staging_buf_sem" in C_SOURCE

@test("heap integrity check in main loop")
def _():
    assert "heap_caps_check_integrity_all" in C_SOURCE

@test("stack watermark monitoring present")
def _():
    assert "uxTaskGetStackHighWaterMark" in C_SOURCE

@test("periodic fsync for crash resilience")
def _():
    assert "fsync" in C_SOURCE

@test("file rotation checks keyframe")
def _():
    assert "AVI_SEGMENT_MAX_BYTES" in C_SOURCE and "is_keyframe" in C_SOURCE

@test("LDO power-cycle for camera reset")
def _():
    assert "esp_ldo_release_channel" in C_SOURCE and "esp_ldo_acquire_channel" in C_SOURCE

@test("M2M encoder reset exists")
def _():
    assert "reset_m2m_encoder" in C_SOURCE

@test("CSI diagnostic dump exists")
def _():
    assert "dump_csi_diagnostics" in C_SOURCE

@test("NVS flash init present")
def _():
    assert "nvs_flash_init" in C_SOURCE

@test("app_main entry point exists")
def _():
    assert re.search(r'void\s+app_main\s*\(', C_SOURCE)

@test("SD pre-allocation (lseek + ftruncate)")
def _():
    assert "lseek" in C_SOURCE and "prealloc" in C_SOURCE

@test("Audio capture task exists")
def _():
    assert "audio_capture_task" in C_SOURCE

@test("PPA EIS scaling present")
def _():
    assert "ppa_srm_handle" in C_SOURCE or "ppa_client" in C_SOURCE.lower()


# ============================================================================
# TEST GROUP 16: Edge Cases & Defensive Code
# ============================================================================

print("\n" + "=" * 70)
print("  GROUP 16: Edge Cases & Defensive Code")
print("=" * 70)

@test("Low TODO/FIXME/HACK count (< 5)")
def _():
    todos = re.findall(r'(?://|/\*).*?\b(TODO|FIXME|HACK)\b', C_SOURCE, re.IGNORECASE)
    assert len(todos) <= 5, f"Found {len(todos)} markers"

@test("repair_avi validates RIFF+AVI signature")
def _():
    with open(os.path.join(PROJECT_ROOT, "repair_avi.py"), "r") as f:
        src = f.read()
    assert "RIFF" in src and "AVI" in src

@test("repair_avi has max chunk size guards")
def _():
    with open(os.path.join(PROJECT_ROOT, "repair_avi.py"), "r") as f:
        src = f.read()
    assert "2_000_000" in src or "2000000" in src
    assert "1_000_000" in src or "1000000" in src

@test("C code includes ESP_RETURN_ON_ERROR for init functions")
def _():
    assert "ESP_RETURN_ON_ERROR" in C_SOURCE

@test("Consecutive DQBUF fail triggers diagnostics")
def _():
    assert "consecutive" in C_SOURCE.lower() and "dump_csi_diagnostics" in C_SOURCE

@test("SD write error sets sd_full flag")
def _():
    assert "sd_full" in C_SOURCE

@test("MIPI recovery: light + heavy reset paths exist")
def _():
    assert "light_sensor_reset" in C_SOURCE
    assert "heavy_pipeline_reset" in C_SOURCE

@test("EIS constants match C defines")
def _():
    assert C_DEFINES.get("EIS_BLOCK_SIZE", 0) == 16
    assert C_DEFINES.get("EIS_SEARCH_RANGE", 0) == 8
    assert C_DEFINES.get("EIS_NUM_BLOCKS", 0) == 4


# ============================================================================
# TEST GROUP 17: Actual AVI File Output (C-mirror header writer)
# ============================================================================

print("\n" + "=" * 70)
print("  GROUP 17: Actual AVI File Output (C-mirror header writer)")
print("=" * 70)

# Python mirror of camera_recorder.c avi_write_header / chunk / update / finalize.
# Builds an identical binary to what the firmware produces.

AUDIO_SAMPLE_RATE = 16000
AUDIO_CHANNELS = 1
AUDIO_BITS = 16

# C constants
_AVI_HDR_RIFF_SIZE      = 4
_AVI_HDR_USPERFRAME     = 32
_AVI_HDR_FLAGS          = 44
_AVI_HDR_TOTALFRAMES    = 48
_AVI_HDR_VID_RATE       = 132
_AVI_HDR_VID_LENGTH     = 140
_AVI_HDR_AUD_LENGTH     = 264
_AVIF_HASINDEX_INTERLEAVED = 0x110
_AVIIF_KEYFRAME         = 0x10

def _avi_write_header(buf, width, height, fps):
    """Mirror of avi_write_header(). Appends to buf, returns movi_fourcc_pos."""
    us_per_frame = 1000000 // fps
    frame_size = width * height
    audio_rate = AUDIO_SAMPLE_RATE
    audio_bytes_per_sec = audio_rate * AUDIO_CHANNELS * (AUDIO_BITS // 8)
    audio_block_align = AUDIO_CHANNELS * (AUDIO_BITS // 8)

    def w4cc(tag):
        buf.extend(tag.encode('ascii') if isinstance(tag, str) else tag)
    def wu32(v):
        buf.extend(struct.pack('<I', v & 0xFFFFFFFF))
    def wu16(v):
        buf.extend(struct.pack('<H', v & 0xFFFF))

    # RIFF header
    w4cc("RIFF"); wu32(0); w4cc("AVI ")

    # LIST hdrl
    w4cc("LIST")
    hdrl_size_pos = len(buf); wu32(0)
    hdrl_start = len(buf)
    w4cc("hdrl")

    # avih (56 bytes)
    w4cc("avih"); wu32(56)
    wu32(us_per_frame)       # dwMicroSecPerFrame
    wu32(frame_size)         # dwMaxBytesPerSec
    wu32(0)                  # dwPaddingGranularity
    wu32(0)                  # dwFlags (0 initially)
    wu32(0)                  # dwTotalFrames (placeholder)
    wu32(0)                  # dwInitialFrames
    wu32(2)                  # dwStreams
    wu32(frame_size)         # dwSuggestedBufferSize
    wu32(width)              # dwWidth
    wu32(height)             # dwHeight
    wu32(0); wu32(0); wu32(0); wu32(0)  # reserved

    # Stream 0: Video (H.264)
    w4cc("LIST")
    strl0_size_pos = len(buf); wu32(0)
    strl0_start = len(buf)
    w4cc("strl")

    w4cc("strh"); wu32(56)
    w4cc("vids"); w4cc("H264")
    wu32(0)                  # dwFlags
    wu32(0)                  # wPriority + wLanguage
    wu32(0)                  # dwInitialFrames
    wu32(1)                  # dwScale
    wu32(fps)                # dwRate
    wu32(0)                  # dwStart
    wu32(0)                  # dwLength (placeholder)
    wu32(frame_size)         # dwSuggestedBufferSize
    wu32(0)                  # dwQuality
    wu32(0)                  # dwSampleSize
    wu32(0)                  # rcFrame (left, top)
    wu32((height << 16) | width)  # rcFrame (right, bottom)

    w4cc("strf"); wu32(40)
    wu32(40)                 # biSize
    wu32(width)              # biWidth
    wu32(height)             # biHeight
    wu32((24 << 16) | 1)     # biPlanes(1) + biBitCount(24)
    w4cc("H264")             # biCompression
    wu32(frame_size)         # biSizeImage
    wu32(0); wu32(0); wu32(0); wu32(0)

    # Patch strl0 size
    struct.pack_into('<I', buf, strl0_size_pos, len(buf) - strl0_start)

    # Stream 1: Audio (PCM)
    w4cc("LIST")
    strl1_size_pos = len(buf); wu32(0)
    strl1_start = len(buf)
    w4cc("strl")

    w4cc("strh"); wu32(56)
    w4cc("auds")
    wu32(0)                  # fccHandler
    wu32(0)                  # dwFlags
    wu32(0)                  # wPriority + wLanguage
    wu32(0)                  # dwInitialFrames
    wu32(audio_block_align)  # dwScale
    wu32(audio_bytes_per_sec)# dwRate
    wu32(0)                  # dwStart
    wu32(0)                  # dwLength (placeholder)
    wu32(audio_bytes_per_sec)# dwSuggestedBufferSize
    wu32(0)                  # dwQuality
    wu32(audio_block_align)  # dwSampleSize
    wu32(0)                  # rcFrame
    wu32(0)

    w4cc("strf"); wu32(16)
    wu16(1)                  # WAVE_FORMAT_PCM
    wu16(AUDIO_CHANNELS)
    wu32(audio_rate)
    wu32(audio_bytes_per_sec)
    wu16(audio_block_align)
    wu16(AUDIO_BITS)

    # Patch strl1 size
    struct.pack_into('<I', buf, strl1_size_pos, len(buf) - strl1_start)
    # Patch hdrl size
    struct.pack_into('<I', buf, hdrl_size_pos, len(buf) - hdrl_start)

    # LIST movi
    w4cc("LIST"); wu32(0)
    movi_fourcc_pos = len(buf)
    w4cc("movi")

    return movi_fourcc_pos


def _avi_write_chunk(buf, fourcc, data):
    """Mirror of avi_write_chunk(). Returns (offset_relative_to_movi, data_size)."""
    buf.extend(fourcc.encode('ascii') if isinstance(fourcc, str) else fourcc)
    buf.extend(struct.pack('<I', len(data)))
    buf.extend(data)
    if len(data) % 2:
        buf.append(0)


def _avi_update_headers(buf, movi_start, current_pos, total_video, total_audio, fps):
    """Mirror of avi_update_headers()."""
    movi_data_size = current_pos - movi_start
    us = 1000000 // fps if fps > 0 else 100000
    struct.pack_into('<I', buf, _AVI_HDR_RIFF_SIZE, current_pos - 8)
    struct.pack_into('<I', buf, _AVI_HDR_USPERFRAME, us)
    struct.pack_into('<I', buf, _AVI_HDR_TOTALFRAMES, total_video)
    struct.pack_into('<I', buf, _AVI_HDR_VID_RATE, fps)
    struct.pack_into('<I', buf, _AVI_HDR_VID_LENGTH, total_video)
    struct.pack_into('<I', buf, _AVI_HDR_AUD_LENGTH, total_audio)
    struct.pack_into('<I', buf, movi_start - 4, movi_data_size)


def _avi_finalize(buf, movi_start, total_video, total_audio, idx_entries, fps):
    """Mirror of avi_finalize()."""
    # Write idx1
    buf.extend(b"idx1")
    buf.extend(struct.pack('<I', len(idx_entries) * 16))
    for fourcc, flags, offset, size in idx_entries:
        tag = fourcc.encode('ascii') if isinstance(fourcc, str) else fourcc
        buf.extend(tag)
        buf.extend(struct.pack('<III', flags, offset, size))
    file_end = len(buf)
    _avi_update_headers(buf, movi_start, file_end, total_video, total_audio, fps)
    struct.pack_into('<I', buf, _AVI_HDR_FLAGS, _AVIF_HASINDEX_INTERLEAVED)


def build_c_mirror_avi(width=1280, height=720, fps=30,
                       video_frames_data=None, audio_chunks_data=None):
    """Build a complete AVI using the C-mirrored functions.
    video_frames_data: list of bytes objects (H.264 frame data)
    audio_chunks_data: list of bytes objects (PCM audio data)
    Returns (bytes, movi_fourcc_pos).
    """
    if video_frames_data is None:
        # Default: 10 fake H.264 IDR frames
        video_frames_data = [bytes([0,0,0,1,0x65]) + os.urandom(500) for _ in range(10)]
    if audio_chunks_data is None:
        # Default: 5 audio chunks of 1024 bytes (512 samples)
        audio_chunks_data = [os.urandom(1024) for _ in range(5)]

    buf = bytearray()
    movi_fourcc_pos = _avi_write_header(buf, width, height, fps)
    movi_start = movi_fourcc_pos  # 'movi' tag position

    idx_entries = []
    total_audio_samples = 0
    n_vid = len(video_frames_data)
    n_aud = len(audio_chunks_data)

    for i in range(max(n_vid, n_aud)):
        if i < n_vid:
            # Offset relative to movi_fourcc_pos + 4 (the 'movi' tag)
            chunk_off = len(buf) - movi_start
            _avi_write_chunk(buf, "00dc", video_frames_data[i])
            idx_entries.append(("00dc", _AVIIF_KEYFRAME, chunk_off, len(video_frames_data[i])))
        if i < n_aud:
            chunk_off = len(buf) - movi_start
            _avi_write_chunk(buf, "01wb", audio_chunks_data[i])
            idx_entries.append(("01wb", 0, chunk_off, len(audio_chunks_data[i])))
            total_audio_samples += len(audio_chunks_data[i]) // 2

    _avi_finalize(buf, movi_start, n_vid, total_audio_samples, idx_entries, fps)
    return bytes(buf), movi_fourcc_pos


# Build a reference AVI once for the validation tests
_REF_WIDTH, _REF_HEIGHT, _REF_FPS = 1280, 720, 30
_REF_NFRAMES, _REF_NAUDIO = 10, 5
_REF_VID_DATA = [bytes([0,0,0,1,0x65]) + bytes(range(256)) + os.urandom(244) for _ in range(_REF_NFRAMES)]
_REF_AUD_DATA = [os.urandom(1024) for _ in range(_REF_NAUDIO)]
_REF_AVI, _REF_MOVI_POS = build_c_mirror_avi(
    _REF_WIDTH, _REF_HEIGHT, _REF_FPS, _REF_VID_DATA, _REF_AUD_DATA)
_REF_TOTAL_AUDIO_SAMPLES = sum(len(a) // 2 for a in _REF_AUD_DATA)

@test("C-mirror AVI: starts with RIFF...AVI")
def _():
    assert _REF_AVI[:4] == b"RIFF"
    assert _REF_AVI[8:12] == b"AVI "

@test("C-mirror AVI: RIFF size = file_size - 8")
def _():
    riff_size = struct.unpack_from('<I', _REF_AVI, _AVI_HDR_RIFF_SIZE)[0]
    assert riff_size == len(_REF_AVI) - 8, f"{riff_size} != {len(_REF_AVI) - 8}"

@test("C-mirror AVI: usperframe at offset 32 = 33333")
def _():
    v = struct.unpack_from('<I', _REF_AVI, _AVI_HDR_USPERFRAME)[0]
    assert v == 1000000 // 30, f"{v}"

@test("C-mirror AVI: flags at offset 44 = HASINDEX|INTERLEAVED (0x110)")
def _():
    v = struct.unpack_from('<I', _REF_AVI, _AVI_HDR_FLAGS)[0]
    assert v == _AVIF_HASINDEX_INTERLEAVED, f"0x{v:x}"

@test("C-mirror AVI: totalframes at offset 48 = 10")
def _():
    v = struct.unpack_from('<I', _REF_AVI, _AVI_HDR_TOTALFRAMES)[0]
    assert v == _REF_NFRAMES, f"{v}"

@test("C-mirror AVI: vid_rate at offset 132 = 30")
def _():
    v = struct.unpack_from('<I', _REF_AVI, _AVI_HDR_VID_RATE)[0]
    assert v == _REF_FPS, f"{v}"

@test("C-mirror AVI: vid_length at offset 140 = 10")
def _():
    v = struct.unpack_from('<I', _REF_AVI, _AVI_HDR_VID_LENGTH)[0]
    assert v == _REF_NFRAMES, f"{v}"

@test("C-mirror AVI: aud_length at offset 264 = total_audio_samples")
def _():
    v = struct.unpack_from('<I', _REF_AVI, _AVI_HDR_AUD_LENGTH)[0]
    assert v == _REF_TOTAL_AUDIO_SAMPLES, f"{v} != {_REF_TOTAL_AUDIO_SAMPLES}"

@test("C-mirror AVI: hdrl LIST present and size correct")
def _():
    assert _REF_AVI[12:16] == b"LIST"
    hdrl_size = struct.unpack_from('<I', _REF_AVI, 16)[0]
    assert _REF_AVI[20:24] == b"hdrl"
    # hdrl_end should be at 20 + hdrl_size
    hdrl_end = 20 + hdrl_size
    # Next thing after hdrl should be LIST movi
    assert _REF_AVI[hdrl_end:hdrl_end + 4] == b"LIST", f"at {hdrl_end}: {_REF_AVI[hdrl_end:hdrl_end+4]}"

@test("C-mirror AVI: avih dwStreams = 2")
def _():
    v = struct.unpack_from('<I', _REF_AVI, 56)[0]
    assert v == 2

@test("C-mirror AVI: avih width/height match")
def _():
    w = struct.unpack_from('<I', _REF_AVI, 64)[0]
    h = struct.unpack_from('<I', _REF_AVI, 68)[0]
    assert (w, h) == (_REF_WIDTH, _REF_HEIGHT), f"({w},{h})"

@test("C-mirror AVI: video strh has vids + H264")
def _():
    pos = _REF_AVI.find(b"strh")
    assert _REF_AVI[pos+8:pos+12] == b"vids"
    assert _REF_AVI[pos+12:pos+16] == b"H264"

@test("C-mirror AVI: video strf BITMAPINFOHEADER biCompression = H264")
def _():
    pos = _REF_AVI.find(b"strf")
    assert pos > 0
    assert _REF_AVI[pos+8:pos+12] == struct.pack('<I', 40)[:4]  # biSize = 40
    comp = _REF_AVI[pos + 24:pos + 28]  # biCompression at offset 16 from biSize
    assert comp == b"H264", f"{comp}"

@test("C-mirror AVI: audio strh has auds + PCM fields")
def _():
    first_strh = _REF_AVI.find(b"strh")
    pos = _REF_AVI.find(b"strh", first_strh + 4)
    assert _REF_AVI[pos+8:pos+12] == b"auds"
    # strh data layout: fccType(4)+fccHandler(4)+dwFlags(4)+wPri(4)+dwInit(4)+dwScale(4)+dwRate(4)
    # dwScale at data offset 20, dwRate at data offset 24
    dwscale = struct.unpack_from('<I', _REF_AVI, pos + 8 + 20)[0]
    dwrate = struct.unpack_from('<I', _REF_AVI, pos + 8 + 24)[0]
    assert dwscale == 2, f"dwScale={dwscale}"
    assert dwrate == 32000, f"dwRate={dwrate}"

@test("C-mirror AVI: audio strf WAVE_FORMAT_PCM=1, 16kHz, mono, 16-bit")
def _():
    first_strf = _REF_AVI.find(b"strf")
    pos = _REF_AVI.find(b"strf", first_strf + 4)
    assert pos > 0
    fmt_tag = struct.unpack_from('<H', _REF_AVI, pos + 8)[0]
    channels = struct.unpack_from('<H', _REF_AVI, pos + 10)[0]
    sample_rate = struct.unpack_from('<I', _REF_AVI, pos + 12)[0]
    bps = struct.unpack_from('<I', _REF_AVI, pos + 16)[0]
    block_align = struct.unpack_from('<H', _REF_AVI, pos + 20)[0]
    bits = struct.unpack_from('<H', _REF_AVI, pos + 22)[0]
    assert fmt_tag == 1, f"wFormatTag={fmt_tag}"
    assert channels == 1, f"nChannels={channels}"
    assert sample_rate == 16000, f"nSamplesPerSec={sample_rate}"
    assert bps == 32000, f"nAvgBytesPerSec={bps}"
    assert block_align == 2, f"nBlockAlign={block_align}"
    assert bits == 16, f"wBitsPerSample={bits}"

@test("C-mirror AVI: movi LIST present at expected position")
def _():
    assert _REF_AVI[_REF_MOVI_POS:_REF_MOVI_POS + 4] == b"movi"
    assert _REF_AVI[_REF_MOVI_POS - 8:_REF_MOVI_POS - 4] == b"LIST"

@test("C-mirror AVI: movi size consistent with data")
def _():
    # C code: avi_finalize calls avi_update_headers with file_end (after idx1)
    # so movi_data_size = file_end - movi_start (includes idx1 in the count,
    # matching the firmware's actual output)
    movi_size = struct.unpack_from('<I', _REF_AVI, _REF_MOVI_POS - 4)[0]
    expected = len(_REF_AVI) - _REF_MOVI_POS
    assert movi_size == expected, f"movi_size={movi_size} expected={expected}"

@test("C-mirror AVI: first video chunk is 00dc with correct size")
def _():
    pos = _REF_MOVI_POS + 4  # after 'movi'
    assert _REF_AVI[pos:pos+4] == b"00dc"
    sz = struct.unpack_from('<I', _REF_AVI, pos + 4)[0]
    assert sz == len(_REF_VID_DATA[0]), f"{sz}"

@test("C-mirror AVI: first audio chunk is 01wb")
def _():
    assert _REF_AVI.find(b"01wb") > _REF_MOVI_POS

@test("C-mirror AVI: idx1 present with correct entry count")
def _():
    idx1_pos = _REF_AVI.find(b"idx1")
    assert idx1_pos > 0
    idx1_size = struct.unpack_from('<I', _REF_AVI, idx1_pos + 4)[0]
    n = idx1_size // 16
    assert n == _REF_NFRAMES + _REF_NAUDIO, f"{n} != {_REF_NFRAMES + _REF_NAUDIO}"

@test("C-mirror AVI: idx1 video entries have KEYFRAME flag")
def _():
    idx1_pos = _REF_AVI.find(b"idx1")
    idx1_size = struct.unpack_from('<I', _REF_AVI, idx1_pos + 4)[0]
    n = idx1_size // 16
    for i in range(n):
        off = idx1_pos + 8 + i * 16
        tag = _REF_AVI[off:off+4]
        flags = struct.unpack_from('<I', _REF_AVI, off + 4)[0]
        if tag == b"00dc":
            assert flags == _AVIIF_KEYFRAME, f"entry {i} flags=0x{flags:x}"

@test("C-mirror AVI: idx1 audio entries have flags=0")
def _():
    idx1_pos = _REF_AVI.find(b"idx1")
    idx1_size = struct.unpack_from('<I', _REF_AVI, idx1_pos + 4)[0]
    n = idx1_size // 16
    for i in range(n):
        off = idx1_pos + 8 + i * 16
        tag = _REF_AVI[off:off+4]
        flags = struct.unpack_from('<I', _REF_AVI, off + 4)[0]
        if tag == b"01wb":
            assert flags == 0, f"entry {i} flags=0x{flags:x}"

@test("C-mirror AVI: idx1 offsets point to valid chunk data")
def _():
    idx1_pos = _REF_AVI.find(b"idx1")
    idx1_size = struct.unpack_from('<I', _REF_AVI, idx1_pos + 4)[0]
    n = idx1_size // 16
    for i in range(n):
        off = idx1_pos + 8 + i * 16
        chunk_off = struct.unpack_from('<I', _REF_AVI, off + 8)[0]
        chunk_sz = struct.unpack_from('<I', _REF_AVI, off + 12)[0]
        # offset relative to movi tag, so absolute = movi_pos + chunk_off
        abs_pos = _REF_MOVI_POS + chunk_off
        assert abs_pos + chunk_sz <= idx1_pos, f"entry {i}: abs_pos={abs_pos} + sz={chunk_sz} > idx1={idx1_pos}"

@test("C-mirror AVI: odd-sized video data gets padded to even")
def _():
    odd_data = bytes([0,0,0,1,0x65]) + os.urandom(100)  # 105 bytes (odd)
    assert len(odd_data) % 2 == 1
    avi, movi = build_c_mirror_avi(1280, 720, 30,
                                   video_frames_data=[odd_data],
                                   audio_chunks_data=[os.urandom(1024)])
    # After 00dc + size(4) + data(105) + pad(1) = 110 bytes
    pos = movi + 4  # after 'movi'
    assert avi[pos:pos+4] == b"00dc"
    sz = struct.unpack_from('<I', avi, pos + 4)[0]
    assert sz == 105
    next_chunk_pos = pos + 8 + sz + 1  # +1 pad
    assert next_chunk_pos % 2 == 0
    # Next chunk should be 01wb
    assert avi[next_chunk_pos:next_chunk_pos+4] == b"01wb"

@test("C-mirror AVI: header size matches C AVI_HDR_* offset expectations")
def _():
    # The movi LIST should start right after hdrl
    hdrl_size = struct.unpack_from('<I', _REF_AVI, 16)[0]
    hdrl_end = 20 + hdrl_size
    # movi LIST header: LIST(4) + size(4) + movi(4)
    assert _REF_AVI[hdrl_end:hdrl_end+4] == b"LIST"
    assert _REF_AVI[hdrl_end+8:hdrl_end+12] == b"movi"
    assert _REF_MOVI_POS == hdrl_end + 8

@test("C-mirror AVI: total header size = movi_start + 4")
def _():
    # Total header = everything before first data chunk
    header_end = _REF_MOVI_POS + 4
    first_data = _REF_AVI[header_end:header_end + 4]
    assert first_data == b"00dc", f"First chunk after header: {first_data}"

@test("C-mirror AVI: video strf biWidth/biHeight match")
def _():
    pos = _REF_AVI.find(b"strf")
    biw = struct.unpack_from('<I', pos + 12 + _REF_AVI, 0)[0] if False else 0
    # Direct read: strf(4) + size(4) + biSize(4) + biWidth(4) + biHeight(4)
    biw = struct.unpack_from('<I', _REF_AVI, pos + 12)[0]
    bih = struct.unpack_from('<I', _REF_AVI, pos + 16)[0]
    assert biw == _REF_WIDTH, f"biWidth={biw}"
    assert bih == _REF_HEIGHT, f"biHeight={bih}"

@test("C-mirror AVI: video strh rcFrame encodes width/height")
def _():
    pos = _REF_AVI.find(b"strh")
    # rcFrame at strh data offset 52 (last 4 bytes of 56-byte strh)
    rcframe = struct.unpack_from('<I', _REF_AVI, pos + 8 + 52)[0]
    w = rcframe & 0xFFFF
    h = (rcframe >> 16) & 0xFFFF
    assert w == _REF_WIDTH, f"rcFrame width={w}"
    assert h == _REF_HEIGHT, f"rcFrame height={h}"

@test("C-mirror AVI: audio strf size = 16 (no cbSize for PCM)")
def _():
    first_strf = _REF_AVI.find(b"strf")
    pos = _REF_AVI.find(b"strf", first_strf + 4)
    strf_size = struct.unpack_from('<I', _REF_AVI, pos + 4)[0]
    assert strf_size == 16, f"audio strf size={strf_size} (expected 16 for PCM)"

@test("C-mirror AVI: accepted by repair_avi (frame count preserved)")
def _():
    mod = load_module("repair_avi", "repair_avi.py")
    src = write_temp_avi(_REF_AVI)
    dst = src + "_repaired.avi"
    try:
        mod.repair_avi(src, dst, recording_duration=float(_REF_NFRAMES) / _REF_FPS)
        with open(dst, 'rb') as f:
            repaired = f.read()
        frames = struct.unpack_from('<I', repaired, 48)[0]
        assert frames == _REF_NFRAMES, f"repair_avi got {frames} frames"
    finally:
        cleanup(src, dst)

@test("C-mirror AVI: accepted by avi_audio_check (no crash)")
def _():
    mod = load_module("avi_audio_check", "avi_audio_check.py")
    src = write_temp_avi(_REF_AVI)
    try:
        # analyze_avi should not raise
        mod.analyze_avi(src)
    finally:
        cleanup(src)

@test("C-mirror AVI: repair_avi RIFF size matches original")
def _():
    mod = load_module("repair_avi", "repair_avi.py")
    src = write_temp_avi(_REF_AVI)
    dst = src + "_repaired.avi"
    try:
        mod.repair_avi(src, dst, recording_duration=float(_REF_NFRAMES) / _REF_FPS)
        with open(dst, 'rb') as f:
            repaired = f.read()
        orig_riff = struct.unpack_from('<I', _REF_AVI, 4)[0]
        rep_riff = struct.unpack_from('<I', repaired, 4)[0]
        assert rep_riff == len(repaired) - 8
        # Sizes may differ slightly (repair rebuilds idx1), but both should be valid
        assert abs(orig_riff - rep_riff) < 256, f"orig={orig_riff} repaired={rep_riff}"
    finally:
        cleanup(src, dst)

@test("C-mirror AVI: repair + re-repair is idempotent")
def _():
    mod = load_module("repair_avi", "repair_avi.py")
    src = write_temp_avi(_REF_AVI)
    dst1 = src + "_r1.avi"
    dst2 = src + "_r2.avi"
    try:
        dur = float(_REF_NFRAMES) / _REF_FPS
        mod.repair_avi(src, dst1, recording_duration=dur)
        mod.repair_avi(dst1, dst2, recording_duration=dur)
        with open(dst1, 'rb') as f:
            r1 = f.read()
        with open(dst2, 'rb') as f:
            r2 = f.read()
        assert len(r1) == len(r2), f"Sizes differ: {len(r1)} vs {len(r2)}"
        assert r1 == r2, "Re-repair changed the file"
    finally:
        cleanup(src, dst1, dst2)

@test("C-mirror AVI: update_headers without finalize leaves flags=0")
def _():
    buf = bytearray()
    movi = _avi_write_header(buf, 1280, 720, 30)
    _avi_write_chunk(buf, "00dc", bytes([0,0,0,1,0x65]) + os.urandom(100))
    _avi_update_headers(buf, movi, len(buf), 1, 0, 30)
    flags = struct.unpack_from('<I', buf, _AVI_HDR_FLAGS)[0]
    assert flags == 0, f"flags=0x{flags:x} (should be 0 before finalize)"

@test("C-mirror AVI: finalize sets HASINDEX flag")
def _():
    buf = bytearray()
    movi = _avi_write_header(buf, 1280, 720, 30)
    vid = bytes([0,0,0,1,0x65]) + os.urandom(100)
    chunk_off = len(buf) - movi
    _avi_write_chunk(buf, "00dc", vid)
    _avi_finalize(buf, movi, 1, 0, [("00dc", 0x10, chunk_off, len(vid))], 30)
    flags = struct.unpack_from('<I', buf, _AVI_HDR_FLAGS)[0]
    assert flags == _AVIF_HASINDEX_INTERLEAVED

@test("C-mirror AVI: video-only (no audio) produces valid file")
def _():
    vdata = [bytes([0,0,0,1,0x65]) + os.urandom(200) for _ in range(5)]
    avi, movi = build_c_mirror_avi(1280, 720, 30, video_frames_data=vdata, audio_chunks_data=[])
    assert avi[:4] == b"RIFF"
    riff_sz = struct.unpack_from('<I', avi, 4)[0]
    assert riff_sz == len(avi) - 8
    frames = struct.unpack_from('<I', avi, _AVI_HDR_TOTALFRAMES)[0]
    assert frames == 5
    aud_len = struct.unpack_from('<I', avi, _AVI_HDR_AUD_LENGTH)[0]
    assert aud_len == 0

@test("C-mirror AVI: large file (200 frames) has correct counts")
def _():
    vdata = [bytes([0,0,0,1,0x65]) + os.urandom(300) for _ in range(200)]
    adata = [os.urandom(640) for _ in range(100)]
    avi, _ = build_c_mirror_avi(1280, 720, 30, vdata, adata)
    frames = struct.unpack_from('<I', avi, _AVI_HDR_TOTALFRAMES)[0]
    assert frames == 200
    idx1_pos = avi.find(b"idx1")
    idx1_size = struct.unpack_from('<I', avi, idx1_pos + 4)[0]
    assert idx1_size // 16 == 300, f"idx1 entries={idx1_size // 16}"

@test("C-mirror AVI: 1920x1080 header has correct dimensions")
def _():
    avi, _ = build_c_mirror_avi(1920, 1080, 30,
                                 [bytes([0,0,0,1,0x65]) + os.urandom(100)],
                                 [os.urandom(1024)])
    w = struct.unpack_from('<I', avi, 64)[0]
    h = struct.unpack_from('<I', avi, 68)[0]
    assert (w, h) == (1920, 1080)

@test("C-mirror AVI: all chunk data bytes preserved correctly")
def _():
    known_data = bytes(range(256)) * 2  # 512 bytes, known pattern
    avi, movi = build_c_mirror_avi(1280, 720, 30,
                                    [bytes([0,0,0,1,0x65]) + known_data],
                                    [])
    pos = movi + 4  # after 'movi'
    assert avi[pos:pos+4] == b"00dc"
    sz = struct.unpack_from('<I', avi, pos + 4)[0]
    extracted = avi[pos + 8:pos + 8 + sz]
    expected = bytes([0,0,0,1,0x65]) + known_data
    assert extracted == expected, f"Data mismatch at byte {next(i for i,(a,b) in enumerate(zip(extracted,expected)) if a!=b)}"


# ============================================================================
# SUMMARY
# ============================================================================

print("\n" + "=" * 70)
total = passed + failed
print(f"  RESULTS: {passed} passed, {failed} failed out of {total} tests")
print("=" * 70)
if errors:
    print(f"\nFailed tests ({len(errors)}):")
    for name, err in errors:
        print(f"  FAIL  {name}")
        print(f"        {err}")
    sys.exit(1)
else:
    print("\nAll tests passed!")
    sys.exit(0)
