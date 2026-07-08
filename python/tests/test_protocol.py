"""Tests for the ESP32 -> PC wire protocol decoder."""

import protocol
from protocol import decode_line, Target, RawFrame


def test_decode_valid_frame():
    line = "R-2540,5185,0,360,0,0,0,0,0,0,0,0,3245,37"
    f = decode_line(line)
    assert isinstance(f, RawFrame)
    assert f.ts_ms == 3245
    assert f.frame_id == 37
    assert len(f.targets) == 3
    assert f.targets[0] == Target(-2540, 5185, 0, 360)
    assert f.targets[0].present
    assert not f.targets[1].present
    assert not f.targets[2].present


def test_decode_picks_present_targets_only():
    line = "R10,20,5,300,0,0,0,0,0,0,0,0,100,1"
    f = decode_line(line)
    assert f.targets[0].present
    assert not f.targets[1].present


def test_decode_rejects_non_r_lines():
    assert decode_line("ESP32 LD2450 breath detector ready") is None
    assert decode_line("Rbad,line") is None
    assert decode_line("") is None


def test_decode_rejects_wrong_field_count():
    # 11 numeric fields instead of 12 + ts + frame_id
    assert decode_line("R1,2,3,4,5,6,7,8,9,10,11,12,99") is None
