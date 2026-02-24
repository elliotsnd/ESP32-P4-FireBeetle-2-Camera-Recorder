#!/usr/bin/env python3
"""Repair an AVI file that was pre-allocated but never finalized.
Scans for valid 00dc/01wb chunks, truncates garbage, builds idx1 index,
and patches all header fields (RIFF size, frame count, movi size, FPS)."""

import struct
import sys
import os

def repair_avi(src_path, dst_path=None, recording_duration=60.0):
    if dst_path is None:
        base, ext = os.path.splitext(src_path)
        dst_path = base + "_repaired" + ext

    with open(src_path, "rb") as f:
        data = f.read()

    print(f"Source: {src_path} ({len(data)} bytes, {len(data)/1024/1024:.1f} MB)")

    # Verify RIFF AVI header
    assert data[0:4] == b"RIFF", "Not a RIFF file"
    assert data[8:12] == b"AVI ", "Not an AVI file"

    # Find movi LIST
    movi_pos = data.find(b"LISTmovi", 0, 2000)
    if movi_pos < 0:
        # Try with size bytes between LIST and movi
        for i in range(100, 1000):
            if data[i:i+4] == b"LIST" and data[i+8:i+12] == b"movi":
                movi_pos = i
                break
    assert movi_pos >= 0, "Could not find movi LIST"
    movi_data_start = movi_pos + 12  # past LIST + size + 'movi'
    print(f"movi LIST at offset {movi_pos}, data starts at {movi_data_start}")

    # Scan valid chunks
    pos = movi_data_start
    video_frames = 0
    audio_chunks = 0
    idx1_entries = []
    total_audio_samples = 0

    while pos + 8 < len(data):
        tag = data[pos:pos+4]
        size = struct.unpack_from("<I", data, pos+4)[0]

        if tag == b"00dc" and 0 < size < 2_000_000:
            # Video chunk - check first bytes look like H.264 NAL
            idx1_entries.append((b"00dc", 0x10, pos - movi_data_start + 4, size))
            video_frames += 1
            pos += 8 + size + (size % 2)
        elif tag == b"01wb" and 0 < size < 1_000_000:
            # Audio chunk
            idx1_entries.append((b"01wb", 0x00, pos - movi_data_start + 4, size))
            audio_chunks += 1
            total_audio_samples += size // 2  # 16-bit mono
            pos += 8 + size + (size % 2)
        else:
            print(f"  Stopped at offset {pos}: tag={tag!r} size={size}")
            break

    movi_end = pos
    movi_size = movi_end - movi_pos - 8  # size field excludes LIST+size (8 bytes)

    print(f"  Video frames: {video_frames}")
    print(f"  Audio chunks: {audio_chunks}")
    print(f"  Valid data ends at: {movi_end} ({movi_end/1024/1024:.1f} MB)")

    # Calculate actual FPS
    actual_fps = max(1, round(video_frames / recording_duration))
    us_per_frame = round(1_000_000 / actual_fps)
    print(f"  Estimated FPS: {actual_fps} (us_per_frame={us_per_frame})")

    # Build output: headers + valid data + idx1
    out = bytearray(data[:movi_end])

    # Append idx1
    idx1_data = b""
    for tag, flags, offset, size in idx1_entries:
        idx1_data += struct.pack("<4sIII", tag, flags, offset, size)
    out += struct.pack("<4sI", b"idx1", len(idx1_data))
    out += idx1_data

    # Patch headers
    # 1. RIFF size (offset 4)
    struct.pack_into("<I", out, 4, len(out) - 8)

    # 2. avih: us_per_frame (offset 32)
    struct.pack_into("<I", out, 32, us_per_frame)

    # 3. avih: total_frames (offset 48)
    struct.pack_into("<I", out, 48, video_frames)

    # 4. movi LIST size (offset movi_pos+4)
    struct.pack_into("<I", out, movi_pos + 4, movi_size)

    # 5. Find and patch strh dwLength for video stream
    # Walk the hdrl structure to find strh chunks
    hdrl_start = 20  # after 'hdrl' tag
    p = 24  # first chunk inside hdrl
    hdrl_end = 20 + struct.unpack_from("<I", data, 16)[0]
    stream_idx = 0
    while p < hdrl_end:
        chunk_tag = data[p:p+4]
        chunk_size = struct.unpack_from("<I", data, p+4)[0]
        if chunk_tag == b"LIST":
            list_tag = data[p+8:p+12]
            if list_tag == b"strl":
                # Find strh inside this strl
                sp = p + 12
                strl_end = p + 8 + chunk_size
                while sp < strl_end:
                    st = data[sp:sp+4]
                    ss = struct.unpack_from("<I", data, sp+4)[0]
                    if st == b"strh":
                        fcc_type = data[sp+8:sp+12]
                        # dwLength is at strh_data + 32
                        dwlen_off = sp + 8 + 32
                        if fcc_type == b"vids":
                            struct.pack_into("<I", out, dwlen_off, video_frames)
                            # Also patch dwRate (FPS) at offset 20 and dwScale at offset 16
                            struct.pack_into("<I", out, sp+8+16, 1)  # dwScale
                            struct.pack_into("<I", out, sp+8+20, actual_fps)  # dwRate
                            print(f"  Patched video strh: dwLength={video_frames}, dwRate={actual_fps}")
                        elif fcc_type == b"auds":
                            struct.pack_into("<I", out, dwlen_off, total_audio_samples)
                            print(f"  Patched audio strh: dwLength={total_audio_samples}")
                        break
                    sp += 8 + ss + (ss % 2)
                stream_idx += 1
            p += 8 + chunk_size + (chunk_size % 2)
        else:
            p += 8 + chunk_size + (chunk_size % 2)

    with open(dst_path, "wb") as f:
        f.write(out)

    print(f"\n=== REPAIRED ===")
    print(f"Output: {dst_path}")
    print(f"Size: {len(out)} bytes ({len(out)/1024/1024:.1f} MB)")
    print(f"Video: {video_frames} frames at {actual_fps} FPS")
    print(f"Audio: {audio_chunks} chunks, {total_audio_samples} samples")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python repair_avi.py <input.avi> [output.avi] [duration_seconds]")
        sys.exit(1)
    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) > 2 else None
    dur = float(sys.argv[3]) if len(sys.argv) > 3 else 60.0
    repair_avi(src, dst, dur)
