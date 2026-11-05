"""guide_points.yaml / guide_script.yaml 的纯逻辑校验测试（不依赖 ROS）。"""

from __future__ import annotations

import textwrap

import pytest

from guide_core import load_points, load_script, wrap_angle


def _write(tmp_path, name, text):
    path = tmp_path / name
    path.write_text(textwrap.dedent(text), encoding="utf-8")
    return str(path)


POINTS = """
start_poi: start_point
poi_order: [POI_05, POI_06]
tracks:
  outbound: track_line_1000
  return: track_line_1001
timeouts:
  to_start_s: 120
  per_poi_s: 90
dwell_s: 5.0
dwell_overrides:
  POI_05: 12.0
speed:
  outbound_mps: 0.3
  return_mps: 0.3
"""


def test_empty_config_is_not_marked(tmp_path):
    cfg = load_points(_write(tmp_path, "points.yaml", "start_poi: ''\npoi_order: []\n"))
    assert cfg.marked is False
    assert cfg.poi_order == ()


def test_valid_config(tmp_path):
    cfg = load_points(_write(tmp_path, "points.yaml", POINTS))
    assert cfg.marked is True
    assert cfg.start_poi == "start_point"
    assert cfg.poi_order == ("POI_05", "POI_06")
    assert cfg.tracks.outbound == "track_line_1000"
    assert cfg.dwell_for("POI_05") == 12.0
    assert cfg.dwell_for("POI_06") == 5.0


def test_duplicate_poi_rejected(tmp_path):
    with pytest.raises(ValueError, match="重复"):
        load_points(_write(tmp_path, "points.yaml",
                           POINTS.replace("[POI_05, POI_06]", "[POI_05, POI_05]")))


def test_start_in_order_rejected(tmp_path):
    with pytest.raises(ValueError, match="不应出现在讲解顺序"):
        load_points(_write(tmp_path, "points.yaml",
                           POINTS.replace("[POI_05, POI_06]",
                                          "[POI_05, start_point]")))


def test_transit_must_be_in_order(tmp_path):
    with pytest.raises(ValueError, match="不在 poi_order"):
        load_points(_write(tmp_path, "points.yaml",
                           POINTS + "transit_pois: [POI_99]\n"))


def test_unknown_key_rejected(tmp_path):
    with pytest.raises(ValueError, match="未知配置项"):
        load_points(_write(tmp_path, "points.yaml", POINTS + "poi_oder: [POI_05]\n"))


def test_script_defaults(tmp_path):
    script = load_script(_write(tmp_path, "script.yaml", """
        head:
          yaw: 0.2
          pitch: -0.1
        points:
          POI_05:
            text: "这是第一个展品"
            head_yaw: -0.4
    """))
    assert script.point("POI_05").head_yaw == -0.4
    assert script.point("POI_05").text == "这是第一个展品"
    assert script.point("POI_06").head_yaw == 0.2
    assert script.point("POI_06").head_pitch == -0.1
    assert script.point("POI_06").text == ""


def test_wrap_angle():
    assert wrap_angle(3.5) == pytest.approx(-2.7831, abs=1e-4)
    assert wrap_angle(0.0) == 0.0
