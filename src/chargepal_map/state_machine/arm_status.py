from __future__ import annotations
# libs
import rospy

# services
from chargepal_services.srv import getArmStatus, getArmStatusRequest, getArmStatusResponse


class ArmStatusServer:

    def __init__(self) -> None:
        self.arm_free = False
        self._srv = rospy.Service('get_arm_status', getArmStatus)

    def _status_callback(self, req: getArmStatusRequest) -> getArmStatusResponse:
        res = getArmStatusResponse()
        res.arm_free = self.arm_free
        return res
