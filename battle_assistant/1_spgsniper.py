import BigWorld, Math, Keys                                   
import BattleReplay
import math
from debug_utils import *

from AvatarInputHandler import AimingSystems, control_modes, mathUtils, cameras
from AvatarInputHandler.DynamicCameras import StrategicCamera
from AvatarInputHandler.AimingSystems import StrategicAimingSystem
import BattleReplay
from AvatarInputHandler.cameras import FovExtended, _clampPoint2DInBox2D
from projectile_trajectory import getShotAngles, computeProjectileTrajectory
from constants import SERVER_TICK_LENGTH, SHELL_TRAJECTORY_EPSILON_CLIENT
from ProjectileMover import collideDynamicAndStatic, collideVehiclesAndStaticScene
from gun_rotation_shared import calcPitchLimitsFromDesc
from gui.WindowsManager import g_windowsManager
from gui.scaleform import Minimap


gSPGSniperEnabled = False


def _getGunMarkerPosition( shotPos, shotVec, testOnlyPlane = None ):
    player = BigWorld.player( )
    shotDescr = player.vehicleTypeDescriptor.shot
    gravity = Math.Vector3(0.0, -shotDescr['gravity'], 0.0)

    prevPos = Math.Vector3( shotPos )
    prevVelocity = Math.Vector3( shotVec )
    dt = SERVER_TICK_LENGTH
    while True:
        newPos = prevPos + prevVelocity.scale(dt) + gravity.scale(dt * dt * 0.5)
        prevVelocity += gravity.scale(dt)

        if testOnlyPlane is not None:
            if prevPos.y > testOnlyPlane and newPos.y <= testOnlyPlane:
                t = (testOnlyPlane - prevPos.y) / (newPos.y - prevPos.y)
                dir = newPos - prevPos
                testRes = (prevPos + dir.scale(t),)
            else:
                testRes = None
        else:
            testRes = BigWorld.wg_collideSegment(BigWorld.player().spaceID, prevPos, newPos, 128)
        if testRes is not None:
            endPos = testRes[0]
            factor = (endPos - prevPos).length / (newPos - prevPos).length
            prevVelocity += gravity.scale(factor * SERVER_TICK_LENGTH)
            break
        pos = player.arena.collideWithSpaceBB(prevPos, newPos)
        if pos is not None:
            endPos = pos
            factor = (endPos - prevPos).length / (newPos - prevPos).length
            prevVelocity += gravity.scale(factor * SERVER_TICK_LENGTH)
            break

        prevPos = newPos

    return endPos, prevVelocity# + gravity


oldStrategicCamera_create = StrategicCamera.StrategicCamera.create
def StrategicCamera_create(self, onChangeControlMode):
    self._prevNearPlane = BigWorld.projection().nearPlane
    self._prevFarPlane = BigWorld.projection().farPlane

    oldStrategicCamera_create(self, onChangeControlMode)

    self._StrategicCamera__aimingSystem._shellVelocity = Math.Vector3( 0.0, -1.0, 0.0 )
    self._StrategicCamera__aimingSystem._lastModeWasSniper = False
    self._StrategicCamera__aimingSystem._initialDistance = 1.0
    self._defaultSrcMat = Math.Matrix( )

oldStrategicCamera_enable = StrategicCamera.StrategicCamera.enable
def StrategicCamera_enable(self, targetPos, saveDist):
    self._prevNearPlane = BigWorld.projection().nearPlane
    self._prevFarPlane = BigWorld.projection().farPlane
    global gSPGSniperEnabled

    if gSPGSniperEnabled:
        self._StrategicCamera__aimingSystem._StrategicAimingSystem__planePosition.x = self._StrategicCamera__aimingSystem._matrix.translation.x
        self._StrategicCamera__aimingSystem._StrategicAimingSystem__planePosition.y = 0.0
        self._StrategicCamera__aimingSystem._StrategicAimingSystem__planePosition.z = self._StrategicCamera__aimingSystem._matrix.translation.z
        gSPGSniperEnabled = False

    oldStrategicCamera_enable(self, targetPos, saveDist)

oldStrategicCamera_disable = StrategicCamera.StrategicCamera.disable
def StrategicCamera_disable(self):
    oldStrategicCamera_disable(self)
    BigWorld.projection().nearPlane = self._prevNearPlane
    BigWorld.projection().farPlane = self._prevFarPlane

    global gSPGSniperEnabled
    if gSPGSniperEnabled:
        self._StrategicCamera__aimingSystem._StrategicAimingSystem__planePosition.x = self._StrategicCamera__aimingSystem._matrix.translation.x
        self._StrategicCamera__aimingSystem._StrategicAimingSystem__planePosition.y = 0.0
        self._StrategicCamera__aimingSystem._StrategicAimingSystem__planePosition.z = self._StrategicCamera__aimingSystem._matrix.translation.z
        self._StrategicCamera__aimingSystem._lastModeWasSniper = False
        gSPGSniperEnabled = False

oldStrategicCamera__cameraUpdate = StrategicCamera.StrategicCamera._StrategicCamera__cameraUpdate
def StrategicCamera__cameraUpdate( self ):
    replayCtrl = BattleReplay.g_replayCtrl

    global gSPGSniperEnabled
    if not gSPGSniperEnabled:
        srcMat = mathUtils.createRotationMatrix((0, -math.pi * 0.499, 0))
        self._StrategicCamera__cam.source = srcMat
        self._StrategicCamera__cam.target.b = self._StrategicCamera__aimingSystem.matrix

        if not replayCtrl.isPlaying:
            BigWorld.projection().nearPlane = self._prevNearPlane
            BigWorld.projection().farPlane = self._prevFarPlane
            BigWorld.projection().fov = StrategicCamera.StrategicCamera.ABSOLUTE_VERTICAL_FOV
        return oldStrategicCamera__cameraUpdate( self )


    distRange = self._StrategicCamera__cfg['distRange'][:]
    if distRange[0] < 20:
        distRange[0] = 20
    distRange[1] = 600


    player = BigWorld.player( )
    descr = player.vehicleTypeDescriptor

    shotEnd = self._StrategicCamera__aimingSystem.matrix.translation

    shellVelocity = Math.Vector3( self._StrategicCamera__aimingSystem._shellVelocity )
    shellVelocity.normalise( )

    srcMat = Math.Matrix()
    srcMat.setRotateYPR((shellVelocity.yaw, -shellVelocity.pitch, 0))
    shift = srcMat.applyVector(Math.Vector3(self._StrategicCamera__dxdydz.x, 0, -self._StrategicCamera__dxdydz.y)) * self._StrategicCamera__curSense




    if replayCtrl.isPlaying and replayCtrl.isControllingCamera:
        aimOffset = replayCtrl.getAimClipPosition()
    else:
        aimWorldPos = self._StrategicCamera__aimingSystem.matrix.applyPoint(Math.Vector3( 0, 0, 0 ))
        aimOffset = cameras.projectPoint(aimWorldPos)
        if replayCtrl.isRecording:
            replayCtrl.setAimClipPosition(Math.Vector2( aimOffset.x, aimOffset.y ))

    self._StrategicCamera__aimOffsetFunc((aimOffset.x, aimOffset.y))
    shotDescr = BigWorld.player().vehicleTypeDescriptor.shot
    BigWorld.wg_trajectory_drawer().setParams(shotDescr['maxDistance'], Math.Vector3(0, -shotDescr['gravity'], 0), self._StrategicCamera__aimOffsetFunc())
    curTime = BigWorld.time()
    deltaTime = curTime - self._StrategicCamera__prevTime
    self._StrategicCamera__prevTime = curTime

    if replayCtrl.isPlaying:
        if self._StrategicCamera__needReset != 0:
            if self._StrategicCamera__needReset > 1:
                player = BigWorld.player()
                if player.inputHandler.ctrl is not None:
                    player.inputHandler.ctrl.resetGunMarkers()
                    
            self._StrategicCamera__needReset = 0
        else:
            self._StrategicCamera__needReset += 1

        if replayCtrl.isControllingCamera:
            self._StrategicCamera__aimingSystem.updateTargetPos(replayCtrl.getGunRotatorTargetPoint())
        else:
            self._StrategicCamera__aimingSystem.handleMovement(shift.x, shift.z)
            if shift.x != 0 and shift.z != 0 or self._StrategicCamera__dxdydz.z != 0:
                self._StrategicCamera__needReset = 2
            
    self._StrategicCamera__aimingSystem.handleMovement(shift.x, shift.z)
    self._StrategicCamera__camDist -= self._StrategicCamera__dxdydz.z * float(self._StrategicCamera__curSense)
    maxPivotHeight = (distRange[1] - distRange[0]) / BigWorld._ba_config['spg']['zoomSpeed']
    self._StrategicCamera__camDist = mathUtils.clamp(0, maxPivotHeight, self._StrategicCamera__camDist)
    self._StrategicCamera__cfg['camDist'] = self._StrategicCamera__camDist
    if self._StrategicCamera__dxdydz.z != 0 and self._StrategicCamera__onChangeControlMode is not None and mathUtils.almostZero(self._StrategicCamera__camDist - maxPivotHeight):
        self._StrategicCamera__onChangeControlMode()
    self._StrategicCamera__updateOscillator(deltaTime)
    if not self._StrategicCamera__autoUpdatePosition:
        self._StrategicCamera__dxdydz = Math.Vector3(0, 0, 0)




    fov = min( 6.0 * descr.gun['shotDispersionAngle'], math.pi * 0.5 )
    zoomFactor = 1.0 / math.tan( fov * 0.5 ) / 5.0

    #old scheme
    #zoomDistance = ( self._StrategicCamera__camDist + distRange[0] ) * zoomFactor

    #new scheme
    zoomDistance = distRange[0] * zoomFactor
    fovFactor = self._StrategicCamera__camDist / maxPivotHeight
    fov = fov * (1.0 - fovFactor) + math.radians(20.0) * fovFactor

    cameraOffset = -shellVelocity.scale( zoomDistance )
    cameraPosition = shotEnd + cameraOffset

    collPoint = None if BigWorld._ba_config['spg']['ignoreObstacles'] else BigWorld.wg_collideSegment(player.spaceID, shotEnd - shellVelocity.scale(1.0 if shellVelocity.y > 0.0 else distRange[0] * zoomFactor * 0.25), cameraPosition, 128)

    if collPoint is None:
        collPoint = player.arena.collideWithSpaceBB(shotEnd, cameraPosition)
        if collPoint is not None:
            collPoint += shellVelocity
    else:
        collPoint = collPoint[0]

    recalculateDist = False
    if collPoint is not None:
        cameraPosition = collPoint
        cameraOffset = cameraPosition - shotEnd
        recalculateDist = True

    if cameraOffset.length > 700.0:
        cameraOffset.normalise()
        cameraOffset = cameraOffset.scale(700.0)
        cameraPosition = shotEnd + cameraOffset
        recalculateDist = True

    #if recalculateDist:
    #    self._StrategicCamera__camDist = cameraOffset.length / zoomFactor - distRange[0]

    #bb = BigWorld.player().arena.arenaType.boundingBox
    #cameraPositionClamped = _clampPoint2DInBox2D(bb[0] - Math.Vector2( 50.0, 50.0 ), bb[1] + Math.Vector2( 50.0, 50.0 ), Math.Vector2(cameraPosition.x, cameraPosition.z))

    #if abs( cameraPositionClamped.x - cameraPosition.x ) > 0.1 or abs( cameraPositionClamped.y - cameraPosition.z ) > 0.1:
    #    clampFactor = min( ( cameraPositionClamped.x - shotEnd.x ) / cameraOffset.x if abs( cameraOffset.x ) > 0.001 else 1.0, ( cameraPositionClamped.y - shotEnd.z ) / cameraOffset.z if abs( cameraOffset.z ) > 0.001 else 1.0 )
    #else:
    #    clampFactor = 1.0

    #if clampFactor < 0.99:
    #    cameraOffset *= clampFactor
    #    cameraPosition = shotEnd + cameraOffset
    #    self._StrategicCamera__camDist = cameraOffset.length / zoomFactor - distRange[0]

    trgMat = Math.Matrix( )
    trgMat.setTranslate( cameraPosition )

    self._StrategicCamera__cam.source = srcMat
    self._StrategicCamera__cam.target.b = trgMat
    self._StrategicCamera__cam.pivotPosition = Math.Vector3( 0, 0, 0 )

    delta = self._prevFarPlane - self._prevNearPlane

    BigWorld.projection().nearPlane = max( cameraOffset.length - delta * 0.5, 1.0 )
    BigWorld.projection().farPlane = max( cameraOffset.length + delta * 0.5, self._prevFarPlane )
    BigWorld.projection().fov = fov
    BigWorld.player().positionControl.moveTo(shotEnd)

    #LOG_ERROR( '{0} {1}'.format( cameraPosition, self._StrategicCamera__camDist ) )
    #FLUSH_LOG( )

    return 0

oldStrategicAimingSystem_updateMatrix = StrategicAimingSystem.StrategicAimingSystem._StrategicAimingSystem__updateMatrix
def StrategicAimingSystem_updateMatrix(self):
    player = BigWorld.player( )
    descr = player.vehicleTypeDescriptor

    global gSPGSniperEnabled
    if not gSPGSniperEnabled:
        if self._lastModeWasSniper:
            if BigWorld._ba_config['spg']['ignoreObstacles']:
                turretYaw, gunPitch = getShotAngles(descr, player.getOwnVehicleMatrix(), (0, 0), self._matrix.translation, True )
                currentGunMat = AimingSystems.getPlayerGunMat(turretYaw, gunPitch)
                clientShotStart = currentGunMat.translation
                clientShotVec = currentGunMat.applyVector(Math.Vector3(0, 0, descr.shot['speed']))
                self._matrix.translation, self._shellVelocity = _getGunMarkerPosition( clientShotStart, clientShotVec, None )

            self._StrategicAimingSystem__planePosition = Math.Vector3(self._matrix.translation.x, 0.0, self._matrix.translation.z)

        oldStrategicAimingSystem_updateMatrix( self )
        self._lastModeWasSniper = False
        return

    bb = BigWorld.player().arena.arenaType.boundingBox
    pos2D = _clampPoint2DInBox2D(bb[0] - Math.Vector2( 200.0, 200.0 ), bb[1] + Math.Vector2( 200.0, 200.0 ), Math.Vector2(self._StrategicAimingSystem__planePosition.x, self._StrategicAimingSystem__planePosition.z))
    self._StrategicAimingSystem__planePosition.x = pos2D[0]
    self._StrategicAimingSystem__planePosition.z = pos2D[1]

    playerPos = player.getOwnVehiclePosition()
    
    if not self._lastModeWasSniper:
        collPoint = BigWorld.wg_collideSegment(BigWorld.player().spaceID, self._StrategicAimingSystem__planePosition + Math.Vector3(0, 1000.0, 0), self._StrategicAimingSystem__planePosition + Math.Vector3(0, -1000.0, 0), 3)
        self._StrategicAimingSystem__planePosition.y = 0.0 if collPoint is None else collPoint[0][1]
        self._initialDistance = ( Math.Vector3( self._StrategicAimingSystem__planePosition.x, playerPos.y, self._StrategicAimingSystem__planePosition.z ) - playerPos ).length + 0.01

    distance = ( Math.Vector3( self._StrategicAimingSystem__planePosition.x, playerPos.y, self._StrategicAimingSystem__planePosition.z ) - playerPos ).length + 0.01
    heightFactor = distance / self._initialDistance
    aimPoint = Math.Vector3( self._StrategicAimingSystem__planePosition.x, playerPos.y * (1.0 - heightFactor) + self._StrategicAimingSystem__planePosition.y * heightFactor, self._StrategicAimingSystem__planePosition.z )

    turretYaw, gunPitch = getShotAngles(descr, player.getOwnVehicleMatrix(), (0, 0), aimPoint, True )
    currentGunMat = AimingSystems.getPlayerGunMat(turretYaw, gunPitch)
    clientShotStart = currentGunMat.translation
    clientShotVec = currentGunMat.applyVector(Math.Vector3(0, 0, descr.shot['speed']))
    
    self._matrix.translation, self._shellVelocity = _getGunMarkerPosition( clientShotStart, clientShotVec, aimPoint.y if BigWorld._ba_config['spg']['ignoreObstacles'] else None )

    self._lastModeWasSniper = True

    #LOG_ERROR( '{0} {1}'.format( gunPitch, gunPitchLimits ) )
    #FLUSH_LOG( )

oldStrategicAimingSystem_getDesiredShotPoint = StrategicAimingSystem.StrategicAimingSystem.getDesiredShotPoint
def StrategicAimingSystem_getDesiredShotPoint( self, terrainOnlyCheck = False ):
    global gSPGSniperEnabled
    if not gSPGSniperEnabled:
        return oldStrategicAimingSystem_getDesiredShotPoint( self, terrainOnlyCheck )

    return self._matrix.translation

def minimapResetCamera(cam):
    minimap = g_windowsManager.battleWindow.minimap
    if minimap is None:
        return

    if minimap._Minimap__cameraHandle is not None:
        minimap._Minimap__ownUI.delEntry(minimap._Minimap__cameraHandle)

    global gSPGSniperEnabled
    if gSPGSniperEnabled:
        #m = Math.WGStrategicAreaViewMP()
        #m.source = cam._StrategicCamera__aimingSystem._matrix
        #m.baseScale = (1.0, 1.0)
        m = cam._StrategicCamera__aimingSystem._matrix
    else:
        m = Math.WGStrategicAreaViewMP()
        m.source = BigWorld.camera().invViewMatrix
        m.baseScale = (1.0, 1.0)

    minimap._Minimap__cameraHandle = minimap._Minimap__ownUI.addEntry(m, minimap.zIndexManager.getIndexByName(Minimap.CAMERA_STRATEGIC))
    minimap._Minimap__ownUI.entryInvoke(minimap._Minimap__cameraHandle, ('gotoAndStop', [Minimap.CURSOR_STRATEGIC]))
    minimap._Minimap__parentUI.call('minimap.entryInited', [])


oldStrategicControlMode_handleKeyEvent = control_modes.StrategicControlMode.handleKeyEvent
def StrategicControlMode_handleKeyEvent( self, isDown, key, mods, event = None ):

    keyProcessed = oldStrategicControlMode_handleKeyEvent( self, isDown, key, mods, event )

    if not keyProcessed and isDown and key in eval(BigWorld._ba_config['spg']['keys']):
        global gSPGSniperEnabled
        gSPGSniperEnabled = not gSPGSniperEnabled
        BigWorld.player().positionControl.followCamera(not gSPGSniperEnabled)

        minimapResetCamera(self._cam)

        return True

    return keyProcessed



if BigWorld._ba_config['spg']['enabled']:
    StrategicCamera.StrategicCamera._StrategicCamera__cameraUpdate = StrategicCamera__cameraUpdate
    StrategicCamera.StrategicCamera.create = StrategicCamera_create
    StrategicCamera.StrategicCamera.enable = StrategicCamera_enable
    StrategicCamera.StrategicCamera.disable = StrategicCamera_disable
    StrategicAimingSystem.StrategicAimingSystem._StrategicAimingSystem__updateMatrix = StrategicAimingSystem_updateMatrix
    StrategicAimingSystem.StrategicAimingSystem.getDesiredShotPoint = StrategicAimingSystem_getDesiredShotPoint
    control_modes.StrategicControlMode.handleKeyEvent = StrategicControlMode_handleKeyEvent

    #print 'SPG Sniper Mod enabled'

    #print control_modes.StrategicControlMode.handleKeyEvent