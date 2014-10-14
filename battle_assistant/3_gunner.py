import BigWorld
import Math
from AvatarInputHandler.AimingSystems import SniperAimingSystem
from ProjectileMover import collideDynamicAndStatic
from AvatarInputHandler import AimingSystems




oldSniperAimingSystem_enable = SniperAimingSystem.SniperAimingSystem.enable
def SniperAimingSystem_enable(self, targetPos):
    oldSniperAimingSystem_enable(self, targetPos)
    self._shootDistance = 0.0



oldSniperAimingSystem_getDesiredShotPoint = SniperAimingSystem.SniperAimingSystem.getDesiredShotPoint
def SniperAimingSystem_getDesiredShotPoint(self):
    start = self.matrix.translation
    dir = self.matrix.applyVector(Math.Vector3(0, 0, 1))

    end = start + dir.scale(10000.0)
    point = collideDynamicAndStatic(start, end, (BigWorld.player().playerVehicleID,), skipGun=False)
    if point is not None:
        result = point[0]
        self._shootDistance = (result - start).length
    else:
        if self._shootDistance > 0.0:
            result = start + dir.scale(self._shootDistance)
        else:
            result = AimingSystems.shootInSkyPoint(start, dir)

    return result




if BigWorld._ba_config['gunner']['enabled']:
    SniperAimingSystem.SniperAimingSystem.enable = SniperAimingSystem_enable
    SniperAimingSystem.SniperAimingSystem.getDesiredShotPoint = SniperAimingSystem_getDesiredShotPoint