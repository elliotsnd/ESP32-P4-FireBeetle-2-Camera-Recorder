"""
Repair AVI files from ESP32-P4 camera recorder.

Fixes two bugs:
1. idx1 offsets are relative to movi_data (after 'movi' fourcc) but AVI spec
   requires them relative to the 'movi' fourcc itself → off by 4 bytes.
   Players can't find any frames.
2. Pre-allocated files have trailing zeros beyond the RIFF data.

This script scans the movi section for actual 00dc chunks and rebuilds
the idx1 from scratch with correct offsets, then truncates the file.
"""

import struct
import sys
import os
import glob


def read_u32(f):
    return struct.unpack('<I', f.read(4))[0]


def write_u32(f, val):
    f.write(struct.pack('<I', val))


def write_4cc(f, cc):
    f.write(cc.encode('ascii'))


def repair_avi(filepath):
    filesize = os.path.getsize(filepath)
    print(f"\n{'='*60}")
    print(f"Repairing: {filepath}")
    print(f"File size: {filesize:,} bytes ({filesize/1024/1024:.1f} MB)")

    with open(filepath, 'r+b') as f:
        # Verify RIFF AVI header
        riff = f.read(4)
        riff_size = read_u32(f)
        avi = f.read(4)
        if riff != b'RIFF' or avi != b'AVI ':
            print(f"  ERROR: Not a valid AVI file (got {riff} {avi})")
            return False

        print(f"  RIFF size: {riff_size:,} (data ends at {riff_size + 8:,})")

        # Find 'movi' LIST
        f.seek(12)  # after RIFF header
        movi_fourcc_pos = None
        movi_list_size_pos = None

        while f.tell() < min(riff_size + 8, filesize):
            pos = f.tell()
            tag = f.read(4)
            if len(tag) < 4:
                break
            size = read_u32(f)

            if tag == b'LIST':
                list_type = f.read(4)
                if list_type == b'hdrl':
                    # Skip header list
                    f.seek(pos + 8 + size)
                    continue
                elif list_type == b'movi':
                    movi_list_size_pos = pos + 4
                    movi_fourcc_pos = pos + 8  # position of 'movi' fourcc
                    movi_data_start = pos + 12  # position of first chunk
                    movi_data_end = pos + 8 + size  # end of movi LIST
                    print(f"  movi fourcc at: 0x{movi_fourcc_pos:X}")
                    print(f"  movi data: 0x{movi_data_start:X} - 0x{movi_data_end:X}")
                    break
            else:
                f.seek(pos + 8 + size + (size & 1))

        if movi_fourcc_pos is None:
            print("  ERROR: Could not find 'movi' LIST")
            return False

        # Scan movi section for actual 00dc chunks
        frames = []
        f.seek(movi_data_start)
        while f.tell() < movi_data_end and f.tell() < filesize:
            chunk_pos = f.tell()
            chunk_tag = f.read(4)
            if len(chunk_tag) < 4:
                break
            chunk_size = read_u32(f)

            if chunk_tag == b'00dc':
                # Verify JPEG SOI marker
                jpeg_start = f.read(2)
                is_jpeg = (jpeg_start == b'\xff\xd8')
                # offset relative to movi fourcc
                offset = chunk_pos - movi_fourcc_pos
                frames.append((offset, chunk_size, is_jpeg))
                # Skip to next chunk (data + pad)
                f.seek(chunk_pos + 8 + chunk_size + (chunk_size & 1))
            elif chunk_tag == b'\x00\x00\x00\x00':
                # Hit zero padding (pre-allocated space)
                break
            else:
                # Unknown chunk, skip
                f.seek(chunk_pos + 8 + chunk_size + (chunk_size & 1))

        total_frames = len(frames)
        jpeg_ok = sum(1 for _, _, j in frames if j)
        print(f"  Found {total_frames} frames ({jpeg_ok} valid JPEG)")

        if total_frames == 0:
            print("  ERROR: No frames found in movi section")
            return False

        # Show frame size stats
        sizes = [s for _, s, _ in frames]
        print(f"  Frame sizes: min={min(sizes)/1024:.0f}KB max={max(sizes)/1024:.0f}KB avg={sum(sizes)/len(sizes)/1024:.0f}KB")

        # Calculate actual movi end (after last frame chunk)
        last_offset, last_size, _ = frames[-1]
        actual_movi_end = movi_fourcc_pos + last_offset + 8 + last_size + (last_size & 1)
        actual_movi_data_size = actual_movi_end - movi_fourcc_pos

        print(f"  Actual movi end: 0x{actual_movi_end:X}")

        # Write new idx1 at actual movi end
        f.seek(actual_movi_end)
        write_4cc(f, 'idx1')
        write_u32(f, total_frames * 16)
        for offset, size, _ in frames:
            write_4cc(f, '00dc')
            write_u32(f, 0x10)  # AVIIF_KEYFRAME
            write_u32(f, offset)  # correct offset from movi fourcc
            write_u32(f, size)

        new_file_end = f.tell()
        print(f"  New idx1 at: 0x{actual_movi_end:X} ({total_frames} entries)")
        print(f"  New file end: 0x{new_file_end:X} ({new_file_end:,} bytes)")

        # Patch RIFF size
        f.seek(4)
        write_u32(f, new_file_end - 8)

        # Patch avih dwTotalFrames (offset 48)
        f.seek(48)
        write_u32(f, total_frames)

        # Patch strh dwLength (offset 140)
        f.seek(140)
        write_u32(f, total_frames)

        # Patch movi LIST size
        f.seek(movi_list_size_pos)
        write_u32(f, actual_movi_data_size)

        # Truncate file
        f.truncate(new_file_end)

    new_size = os.path.getsize(filepath)
    saved = filesize - new_size
    print(f"  Truncated: {filesize:,} -> {new_size:,} (saved {saved/1024/1024:.1f} MB)")
    print(f"  REPAIRED OK: {total_frames} frames")
    return True


if __name__ == '__main__':
    if len(sys.argv) > 1:
        files = sys.argv[1:]
    else:
        # Auto-find AVI files in common locations
        files = glob.glob(r"G:\_Organized_Loose_Files\esp32p4_camera_recorder\*.AVI")
        files += glob.glob(r"G:\_Organized_Loose_Files\esp32p4_camera_recorder\*.avi")
        # Deduplicate
        files = list(set(files))

    if not files:
        print("Usage: python fix_avi.py <file1.avi> [file2.avi] ...")
        print("Or place AVI files in G:\\_Organized_Loose_Files\\esp32p4_camera_recorder\\")
        sys.exit(1)

    print(f"Found {len(files)} AVI file(s) to repair")
    ok = 0
    for f in sorted(files):
        if repair_avi(f):
            ok += 1

    print(f"\n{'='*60}")
    print(f"Repaired {ok}/{len(files)} files")
