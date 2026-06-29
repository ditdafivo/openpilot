from types import SimpleNamespace
import unittest

from opendbc.car.rivian.ext_controller import ExternalController, MIN_TORQUE_FRAMES, TORQUE_HANDOFF_MIN_SPEED
from opendbc.car.rivian.interface import CarInterface
from opendbc.car.rivian.values import CAR


class TestRivianExternalController(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.CP = CarInterface.get_non_essential_params(CAR.RIVIAN_R1)

  @staticmethod
  def _car_state(*, standstill: bool, v_ego_raw: float):
    return SimpleNamespace(
      out=SimpleNamespace(
        standstill=standstill,
        vEgoRaw=v_ego_raw,
        aEgo=0.0,
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

  @staticmethod
  def _actuators():
    return SimpleNamespace(steeringAngleDeg=0.0, torque=0.0)

  def _controller_in_torque_mode(self):
    erc = ExternalController(self.CP)
    erc.torque_active = True
    erc.torque_active_frames = MIN_TORQUE_FRAMES
    return erc

  def test_torque_handoff_stays_active_at_standstill(self):
    erc = self._controller_in_torque_mode()
    cs = self._car_state(standstill=True, v_ego_raw=0.0)

    erc.update(cs, True, self._actuators())

    self.assertTrue(erc.torque_active)

  def test_torque_handoff_clears_above_min_speed_when_settled(self):
    erc = self._controller_in_torque_mode()
    cs = self._car_state(standstill=False, v_ego_raw=TORQUE_HANDOFF_MIN_SPEED + 0.1)

    erc.update(cs, True, self._actuators())

    self.assertFalse(erc.torque_active)
