import unittest
from types import SimpleNamespace

from opendbc.car.rivian.ext_controller import ExternalController, MIN_TORQUE_FRAMES, TORQUE_HANDOFF_MIN_SPEED
from opendbc.car.rivian.interface import CarInterface
from opendbc.car.rivian.fingerprints import FW_VERSIONS
from opendbc.car.rivian.values import CAR, FW_QUERY_CONFIG, WMI, ModelLine, ModelYear


def _fake_controller_state(*, standstill: bool, v_ego_raw: float):
  return SimpleNamespace(
    out=SimpleNamespace(
      standstill=standstill,
      vEgoRaw=v_ego_raw,
      steeringPressed=False,
      steeringAngleDeg=0.0,
      steeringRateDeg=0.0,
      steeringTorque=0.0,
    ),
    eac_status=1,
    eac_error_code=0,
    hands_on_level=0,
    sccm_wheel_touch={
      "SETME_X52": 100.0,
      "SCCM_WheelTouch_CapacitiveValue": 0.0,
    },
  )


def _angle_actuators(angle_deg: float = 0.0):
  return SimpleNamespace(steeringAngleDeg=angle_deg, torque=0.0)


class TestRivian(unittest.TestCase):
  def test_custom_fuzzy_fingerprinting(self):
    for platform in CAR:
      with self.subTest(platform=platform.name):
        for wmi in WMI:
          for line in ModelLine:
            for year in ModelYear:
              for bad in (True, False):
                vin = ["0"] * 17
                vin[:3] = wmi
                vin[3] = line.value
                vin[9] = year.value
                if bad:
                  vin[3] = "Z"
                vin = "".join(vin)

                matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy({}, vin, FW_VERSIONS)
                should_match = year in platform.config.years and not bad
                assert (matches == {platform}) == should_match, "Bad match"

  def test_cooperative_torque_handoff_stays_active_at_standstill(self):
    CP = CarInterface.get_non_essential_params(CAR.RIVIAN_R1)
    erc = ExternalController(CP)
    erc.torque_active = True
    erc.torque_active_frames = MIN_TORQUE_FRAMES

    CS = _fake_controller_state(standstill=True, v_ego_raw=0.0)
    erc._update_hands_on(CS)
    erc._update_torque_active(CS, lat_active=True, actuators=_angle_actuators())

    self.assertTrue(erc.torque_active)

  def test_cooperative_torque_handoff_clears_above_min_speed(self):
    CP = CarInterface.get_non_essential_params(CAR.RIVIAN_R1)
    erc = ExternalController(CP)
    erc.torque_active = True
    erc.torque_active_frames = MIN_TORQUE_FRAMES

    CS = _fake_controller_state(standstill=False, v_ego_raw=TORQUE_HANDOFF_MIN_SPEED + 0.1)
    erc._update_hands_on(CS)
    erc._update_torque_active(CS, lat_active=True, actuators=_angle_actuators())

    self.assertFalse(erc.torque_active)
