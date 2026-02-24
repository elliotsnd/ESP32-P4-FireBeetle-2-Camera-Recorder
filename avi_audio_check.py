#!/usr/bin/env python3
"""
AVI Audio Diagnostic Tool
Reads an AVI file, extracts all audio chunks (01wb), and saves as standalone WAV.
Also reports chunk sizes, gaps, and any issues found.

Usage: python avi_audio_check.py <path_to_avi>
  e.g. python avi_audio_check.py D:\vid\rec_0003.avi
"""

import sys
import struct
import os
import wave

def read_fourcc(f):
    return f.read(4)

def read_u32(f):
    data = f.read(4)
    if len(data) < 4:
        return None
    return struct.unpack('<I', data)[0]

def read_u16(f):
    data = f.read(2)
    if len(data) < 2:
        return None
    return struct.unpack('<H', data)[0]

def analyze_avi(filepath):
    print(f"Analyzing: {filepath}")
    print(f"File size: {os.path.getsize(filepath):,} bytes")
    print()

    with open(filepath, 'rb') as f:
        # Read RIFF header
        riff = read_fourcc(f)
        riff_size = read_u32(f)
        avi_tag = read_fourcc(f)
        print(f"RIFF: {riff} size={riff_size} type={avi_tag}")
        if riff != b'RIFF' or avi_tag != b'AVI ':
            print("ERROR: Not a valid AVI file!")
            return

        # Parse top-level chunks
        audio_chunks = []
        video_chunks = []
        audio_format = {}
        video_format = {}
        movi_offset = None

        def parse_list(end_pos, depth=0):
            nonlocal movi_offset
            indent = "  " * depth
            list_type = read_fourcc(f)
            print(f"{indent}LIST type: {list_type.decode('ascii', errors='replace')}")

            if list_type == b'movi':
                movi_offset = f.tell()
                print(f"{indent}  movi starts at offset {movi_offset}")
                # Parse movi chunks
                while f.tell() < end_pos:
                    chunk_id = read_fourcc(f)
                    if not chunk_id or len(chunk_id) < 4:
                        break
                    chunk_size = read_u32(f)
                    if chunk_size is None:
                        break
                    chunk_data_pos = f.tell()

                    if chunk_id == b'00dc':
                        video_chunks.append({
                            'offset': chunk_data_pos - 8,
                            'data_offset': chunk_data_pos,
                            'size': chunk_size
                        })
                    elif chunk_id == b'01wb':
                        audio_chunks.append({
                            'offset': chunk_data_pos - 8,
                            'data_offset': chunk_data_pos,
                            'size': chunk_size
                        })
                    elif chunk_id == b'LIST':
                        parse_list(chunk_data_pos + chunk_size, depth + 1)
                        continue

                    # Skip to next chunk (with padding)
                    skip = chunk_size + (chunk_size & 1)
                    f.seek(chunk_data_pos + skip)
                return

            # Parse sub-chunks in LIST
            while f.tell() < end_pos:
                chunk_id = read_fourcc(f)
                if not chunk_id or len(chunk_id) < 4:
                    break
                chunk_size = read_u32(f)
                if chunk_size is None:
                    break
                chunk_data_pos = f.tell()

                chunk_id_str = chunk_id.decode('ascii', errors='replace')

                if chunk_id == b'LIST':
                    parse_list(chunk_data_pos + chunk_size, depth + 1)
                    continue
                elif chunk_id == b'avih':
                    us_per_frame = read_u32(f)
                    f.seek(chunk_data_pos + 16)  # skip to dwTotalFrames
                    total_frames = read_u32(f)
                    f.seek(chunk_data_pos + 24)  # dwStreams
                    n_streams = read_u32(f)
                    f.seek(chunk_data_pos + 32)  # dwWidth
                    width = read_u32(f)
                    height = read_u32(f)
                    print(f"{indent}  avih: {width}x{height}, {us_per_frame}us/frame ({1000000/us_per_frame:.1f}fps), "
                          f"{total_frames} frames, {n_streams} streams")
                elif chunk_id == b'strh':
                    fcc_type = read_fourcc(f)
                    fcc_handler = read_fourcc(f)
                    f.seek(chunk_data_pos + 20)  # skip to dwScale
                    dw_scale = read_u32(f)
                    dw_rate = read_u32(f)
                    dw_start = read_u32(f)
                    dw_length = read_u32(f)
                    dw_buf = read_u32(f)
                    dw_quality = read_u32(f)
                    dw_sample_size = read_u32(f)
                    type_str = fcc_type.decode('ascii', errors='replace')
                    handler_str = fcc_handler.decode('ascii', errors='replace')
                    print(f"{indent}  strh: type={type_str} handler={handler_str} "
                          f"scale={dw_scale} rate={dw_rate} ({dw_rate/dw_scale if dw_scale else 0:.0f}/s) "
                          f"length={dw_length} sampleSize={dw_sample_size}")
                    if fcc_type == b'auds':
                        audio_format['scale'] = dw_scale
                        audio_format['rate'] = dw_rate
                        audio_format['length'] = dw_length
                        audio_format['sample_size'] = dw_sample_size
                elif chunk_id == b'strf':
                    # Check if this is audio or video strf
                    if 'tag' not in audio_format and chunk_size >= 16:
                        # Could be audio strf - check by looking for reasonable wFormatTag
                        pos = f.tell()
                        tag = read_u16(f)
                        if tag == 1 or tag == 0:  # PCM or unknown
                            if tag == 1:
                                nch = read_u16(f)
                                rate = read_u32(f)
                                byterate = read_u32(f)
                                align = read_u16(f)
                                bits = read_u16(f)
                                audio_format['tag'] = tag
                                audio_format['channels'] = nch
                                audio_format['sample_rate'] = rate
                                audio_format['byte_rate'] = byterate
                                audio_format['block_align'] = align
                                audio_format['bits'] = bits
                                print(f"{indent}  strf (audio): PCM {rate}Hz {bits}bit {nch}ch "
                                      f"byterate={byterate} align={align} (chunk_size={chunk_size})")
                            else:
                                f.seek(pos)
                                # Not audio, probably video with biSize
                                bi_size = tag | (read_u16(f) << 16)
                                bi_width = read_u32(f)
                                bi_height = read_u32(f)
                                f.seek(pos + 12)
                                bi_planes_bits = read_u32(f)
                                bi_compression = read_fourcc(f)
                                print(f"{indent}  strf (video): {bi_width}x{bi_height} "
                                      f"compression={bi_compression.decode('ascii', errors='replace')} "
                                      f"biSize={bi_size}")
                        else:
                            f.seek(pos)
                            bi_size = read_u32(f)
                            f.seek(pos)
                            bi_width = struct.unpack('<I', f.read(4))[0]
                            print(f"{indent}  strf: size={chunk_size} (first u32={bi_width})")
                    else:
                        print(f"{indent}  strf: size={chunk_size}")
                else:
                    print(f"{indent}  {chunk_id_str}: size={chunk_size}")

                skip = chunk_size + (chunk_size & 1)
                f.seek(chunk_data_pos + skip)

        # Parse top-level chunks
        file_end = 12 + riff_size
        while f.tell() < file_end:
            chunk_id = read_fourcc(f)
            if not chunk_id or len(chunk_id) < 4:
                break
            chunk_size = read_u32(f)
            if chunk_size is None:
                break
            chunk_data_pos = f.tell()
            chunk_id_str = chunk_id.decode('ascii', errors='replace')

            if chunk_id == b'LIST':
                parse_list(chunk_data_pos + chunk_size, 1)
            elif chunk_id == b'idx1':
                print(f"\n  idx1: {chunk_size} bytes ({chunk_size // 16} entries)")
                # Read first few and last few entries
                n_entries = chunk_size // 16
                for i in range(min(5, n_entries)):
                    ck = read_fourcc(f)
                    flags = read_u32(f)
                    offset = read_u32(f)
                    size = read_u32(f)
                    ck_str = ck.decode('ascii', errors='replace')
                    print(f"    [{i}] {ck_str} flags=0x{flags:04x} offset={offset} size={size}")
                if n_entries > 10:
                    f.seek(chunk_data_pos + (n_entries - 3) * 16)
                    print(f"    ...")
                    for i in range(n_entries - 3, n_entries):
                        ck = read_fourcc(f)
                        flags = read_u32(f)
                        offset = read_u32(f)
                        size = read_u32(f)
                        ck_str = ck.decode('ascii', errors='replace')
                        print(f"    [{i}] {ck_str} flags=0x{flags:04x} offset={offset} size={size}")
            else:
                print(f"  {chunk_id_str}: size={chunk_size}")

            skip = chunk_size + (chunk_size & 1)
            f.seek(chunk_data_pos + skip)

    # Report
    print(f"\n{'='*60}")
    print(f"SUMMARY")
    print(f"{'='*60}")
    print(f"Video chunks: {len(video_chunks)}")
    print(f"Audio chunks: {len(audio_chunks)}")
    if audio_format:
        print(f"Audio format: {audio_format.get('sample_rate', '?')}Hz "
              f"{audio_format.get('bits', '?')}bit "
              f"{audio_format.get('channels', '?')}ch")
        print(f"Audio strh: scale={audio_format.get('scale')}, rate={audio_format.get('rate')}, "
              f"length={audio_format.get('length')}, sampleSize={audio_format.get('sample_size')}")

    if audio_chunks:
        sizes = [c['size'] for c in audio_chunks]
        total_audio_bytes = sum(sizes)
        print(f"\nAudio chunk sizes: min={min(sizes)}, max={max(sizes)}, avg={sum(sizes)/len(sizes):.0f}")
        print(f"Total audio data: {total_audio_bytes} bytes")

        # Check for odd-sized chunks (misaligned samples)
        odd_chunks = [i for i, s in enumerate(sizes) if s % 2 != 0]
        if odd_chunks:
            print(f"WARNING: {len(odd_chunks)} chunks have ODD byte count (sample misalignment!)")
            print(f"  Chunks with odd sizes: {odd_chunks[:10]}...")
        else:
            print(f"All audio chunks have even byte count (sample-aligned OK)")

        if audio_format.get('sample_rate'):
            sr = audio_format['sample_rate']
            ba = audio_format.get('block_align', 2)
            total_samples = total_audio_bytes // ba
            duration = total_samples / sr
            print(f"Audio duration: {duration:.2f}s ({total_samples} samples)")

            if video_chunks and audio_format.get('rate') and audio_format.get('scale'):
                video_rate = audio_format.get('rate', 1) / audio_format.get('scale', 1)
                # This is wrong - video rate is in the video strh, not audio
                # Just use video chunk count info
                pass

        # Print first 10 chunk sizes
        print(f"\nFirst 10 audio chunk sizes:")
        for i, c in enumerate(audio_chunks[:10]):
            print(f"  [{i}] offset={c['offset']} size={c['size']} bytes ({c['size']//2} samples)")

        # Extract audio to WAV
        wav_path = filepath.rsplit('.', 1)[0] + '_extracted_audio.wav'
        print(f"\nExtracting audio to: {wav_path}")
        with open(filepath, 'rb') as f:
            sr = audio_format.get('sample_rate', 16000)
            nch = audio_format.get('channels', 1)
            bits = audio_format.get('bits', 16)

            with wave.open(wav_path, 'wb') as wf:
                wf.setnchannels(nch)
                wf.setsampwidth(bits // 8)
                wf.setframerate(sr)

                for chunk in audio_chunks:
                    f.seek(chunk['data_offset'])
                    data = f.read(chunk['size'])
                    wf.writeframes(data)

        print(f"Extracted {total_audio_bytes} bytes of audio to WAV")
        print(f"\nPlay both files and compare:")
        print(f"  1. {wav_path}  (extracted from AVI)")
        print(f"  2. The audio_NNNN.wav file on SD card (direct mic recording)")
        print(f"If extracted WAV crackles too -> audio DATA is corrupted (ring buffer bug)")
        print(f"If extracted WAV plays fine -> AVI CONTAINER has format issue")

        # Also check for sample-level discontinuities
        print(f"\nChecking for sample discontinuities...")
        with open(filepath, 'rb') as f:
            prev_last_sample = None
            big_jumps = 0
            for i, chunk in enumerate(audio_chunks):
                f.seek(chunk['data_offset'])
                data = f.read(min(chunk['size'], 4))  # read first 2 samples
                if len(data) >= 2:
                    first_sample = struct.unpack('<h', data[0:2])[0]
                    if prev_last_sample is not None:
                        jump = abs(first_sample - prev_last_sample)
                        if jump > 5000:
                            big_jumps += 1
                            if big_jumps <= 5:
                                print(f"  Chunk {i}: jump={jump} (prev_last={prev_last_sample}, first={first_sample})")

                # Read last sample
                f.seek(chunk['data_offset'] + chunk['size'] - 2)
                data = f.read(2)
                if len(data) >= 2:
                    prev_last_sample = struct.unpack('<h', data[0:2])[0]

            print(f"  Total large jumps (>5000): {big_jumps} out of {len(audio_chunks)} chunk boundaries")
            if big_jumps > len(audio_chunks) * 0.1:
                print(f"  WARNING: Many discontinuities! Likely ring buffer overrun or data corruption")
            else:
                print(f"  Audio data appears continuous across chunk boundaries")

    print(f"\n{'='*60}")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python avi_audio_check.py <path_to_avi>")
        print("  e.g. python avi_audio_check.py D:\\vid\\rec_0003.avi")
        sys.exit(1)
    analyze_avi(sys.argv[1])
