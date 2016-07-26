#import bwpydevd
#bwpydevd.startPyDevD(ide='eclipse')


import BigWorld, Math, Keys                                   
import BattleReplay
import GUI
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
from gui.app_loader import g_appLoader
from gui.scaleform import Minimap
from gun_rotation_shared import calcPitchLimitsFromDesc
from gui import DEPTH_OF_GunMarker
from gui import g_guiResetters
import ProjectileMover
import Avatar
from gui.Scaleform.framework import ViewTypes
from gui.Scaleform.genConsts.BATTLE_VIEW_ALIASES import BATTLE_VIEW_ALIASES






class SPGAim(object):
    
    def __init__(self):
        self.enabled = False
        self._projectileModel = None
        self._projectileID = None
        self._trackProjectile = False
        self._trackProjectileStartPoint = None
        self._trackProjectileVelocity = None
        self._trackProjectileStartTime = 0.0
        self._followProjectileKey = eval(BigWorld._ba_config['spg']['followProjectileKey'])

    def _getGunMarkerPosition( self, shotPos, shotVec, testOnlyPlane = None ):
        player = BigWorld.player( )
        shotDescr = player.vehicleTypeDescriptor.shot
        gravity = Math.Vector3(0.0, -shotDescr['gravity'], 0.0)

        prevPos = Math.Vector3( shotPos )
        prevVelocity = Math.Vector3( shotVec )
        dt = SERVER_TICK_LENGTH
        flightTime = 0.0
        while True:
            newPos = prevPos + prevVelocity.scale(dt) + gravity.scale(dt * dt * 0.5)
            prevVelocity += gravity.scale(dt)
            flightTime += dt

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

        return endPos, prevVelocity, flightTime

    def onStrategicCameraCreate(self, camera):
        self._shellVelocity = Math.Vector3( 0.0, -1.0, 0.0 )
        self._lastModeWasSniper = False
        self._initialDistance = 1.0
        self._desiredShotPoint = None

    def onStrategicCameraEnable(self, camera):
        if self.enabled:
            camera._StrategicCamera__aimingSystem._StrategicAimingSystem__planePosition.x = camera._StrategicCamera__aimingSystem._matrix.translation.x
            camera._StrategicCamera__aimingSystem._StrategicAimingSystem__planePosition.y = 0.0
            camera._StrategicCamera__aimingSystem._StrategicAimingSystem__planePosition.z = camera._StrategicCamera__aimingSystem._matrix.translation.z
            self.enabled = False

    def onStrategicCameraDisable(self, camera):
        if self.enabled:
            camera._StrategicCamera__aimingSystem._StrategicAimingSystem__planePosition.x = camera._StrategicCamera__aimingSystem._matrix.translation.x
            camera._StrategicCamera__aimingSystem._StrategicAimingSystem__planePosition.y = 0.0
            camera._StrategicCamera__aimingSystem._StrategicAimingSystem__planePosition.z = camera._StrategicCamera__aimingSystem._matrix.translation.z
            self._lastModeWasSniper = False
            self.enabled = False

    def onStrategicCameraUpdate(self, camera):
        distRange = list(camera._StrategicCamera__cfg['distRange'])
        if distRange[0] < 20:
            distRange[0] = 20
        distRange[1] = 600


        player = BigWorld.player( )
        descr = player.vehicleTypeDescriptor

        shotEnd = camera._StrategicCamera__aimingSystem.matrix.translation

        shellVelocity = Math.Vector3( self._shellVelocity )
        shellVelocity.normalise( )

        srcMat = Math.Matrix()
        srcMat.setRotateYPR((shellVelocity.yaw, -shellVelocity.pitch, 0))
        shift = srcMat.applyVector(Math.Vector3(camera._StrategicCamera__dxdydz.x, 0, -camera._StrategicCamera__dxdydz.y)) * camera._StrategicCamera__curSense


        replayCtrl = BattleReplay.g_replayCtrl
        if replayCtrl.isPlaying and replayCtrl.isControllingCamera:
            aimOffset = replayCtrl.getAimClipPosition()
        else:
            aimWorldPos = camera._StrategicCamera__aimingSystem.matrix.translation
            aimOffset = cameras.projectPoint(aimWorldPos)
            aimOffset = Math.Vector2(aimOffset.x, aimOffset.y)
            if replayCtrl.isRecording:
                replayCtrl.setAimClipPosition(aimOffset)
        camera._StrategicCamera__aimOffset = aimOffset
        
        shotDescr = BigWorld.player().vehicleTypeDescriptor.shot
        BigWorld.wg_trajectory_drawer().setParams(shotDescr['maxDistance'], Math.Vector3(0, -shotDescr['gravity'], 0), aimOffset)
        curTime = BigWorld.time()
        deltaTime = curTime - camera._StrategicCamera__prevTime
        camera._StrategicCamera__prevTime = curTime

        if replayCtrl.isPlaying:
            if camera._StrategicCamera__needReset != 0:
                if camera._StrategicCamera__needReset > 1:
                    from helpers import isPlayerAvatar
                    if isPlayerAvatar():
                        if player.inputHandler.ctrl is not None:
                            player.inputHandler.ctrl.resetGunMarkers()
                    camera._StrategicCamera__needReset = 0
                else:
                    camera._StrategicCamera__needReset += 1

            if replayCtrl.isControllingCamera:
                camera._StrategicCamera__aimingSystem.updateTargetPos(replayCtrl.getGunRotatorTargetPoint())
            else:
                camera._StrategicCamera__aimingSystem.handleMovement(shift.x, shift.z)
                if shift.x != 0 and shift.z != 0 or camera._StrategicCamera__dxdydz.z != 0:
                    self._StrategicCamera__needReset = 2
        else:
            camera._StrategicCamera__aimingSystem.handleMovement(shift.x, shift.z)
            
        camera._StrategicCamera__camDist -= camera._StrategicCamera__dxdydz.z * float(camera._StrategicCamera__curSense)        
        maxPivotHeight = (distRange[1] - distRange[0]) / BigWorld._ba_config['spg']['zoomSpeed']
        camera._StrategicCamera__camDist = mathUtils.clamp(0, maxPivotHeight, camera._StrategicCamera__camDist)
        camera._StrategicCamera__cfg['camDist'] = camera._StrategicCamera__camDist
        if camera._StrategicCamera__dxdydz.z != 0 and camera._StrategicCamera__onChangeControlMode is not None and mathUtils.almostZero(camera._StrategicCamera__camDist - maxPivotHeight):
            camera._StrategicCamera__onChangeControlMode()
        camera._StrategicCamera__updateOscillator(deltaTime)
        if not camera._StrategicCamera__autoUpdatePosition:
            camera._StrategicCamera__dxdydz = Math.Vector3(0, 0, 0)



        fov = min( 6.0 * descr.gun['shotDispersionAngle'], math.pi * 0.5 )
        zoomFactor = 1.0 / math.tan( fov * 0.5 ) / 5.0

        zoomDistance = distRange[0] * zoomFactor
        fovFactor = camera._StrategicCamera__camDist / maxPivotHeight
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

        trgMat = Math.Matrix( )
        trgMat.setTranslate( cameraPosition )

        camera._StrategicCamera__cam.source = srcMat
        camera._StrategicCamera__cam.target.b = trgMat
        camera._StrategicCamera__cam.pivotPosition = Math.Vector3( 0, 0, 0 )

        delta = self._prevFarPlane - self._prevNearPlane

        BigWorld.projection().nearPlane = max( cameraOffset.length - delta * 0.5, 1.0 )
        BigWorld.projection().farPlane = max( cameraOffset.length + delta * 0.5, self._prevFarPlane )
        BigWorld.projection().fov = fov
        BigWorld.player().positionControl.moveTo(shotEnd)

        if BigWorld._ba_config['spg']['alwaysFollowProjectile'] or BigWorld.isKeyDown(self._followProjectileKey):
            if self._trackProjectile:
                time = BigWorld.time() - self._trackProjectileStartTime
                if time > 0:
                    shotDescr = descr.shot
                    gravity = Math.Vector3(0.0, -shotDescr['gravity'], 0.0)
                    shellVelocity = self._trackProjectileVelocity + gravity.scale(time)
                    srcMat.setRotateYPR((shellVelocity.yaw, -shellVelocity.pitch, 0))
                    camera._StrategicCamera__cam.source = srcMat
                    camera._StrategicCamera__cam.target.b.setTranslate(self._trackProjectileStartPoint + self._trackProjectileVelocity.scale(time) + gravity.scale(time * time * 0.5))
                    BigWorld.projection().fov = math.pi * 0.4

            elif player._PlayerAvatar__projectileMover and self._projectileID in player._PlayerAvatar__projectileMover._ProjectileMover__projectiles:
                shellVelocity = Math.Matrix(self._projectileModel.matrix).applyVector(Math.Vector3(0.0, 0.0, 1.0))
                srcMat.setRotateYPR((shellVelocity.yaw, -shellVelocity.pitch, 0))
                camera._StrategicCamera__cam.source = srcMat
                camera._StrategicCamera__cam.target.b.setTranslate(self._projectileModel.position)
                BigWorld.projection().fov = math.pi * 0.4

        return 0

    def calcTrajectoryProperties(self, aimPoint):
        player = BigWorld.player( )
        descr = player.vehicleTypeDescriptor

        finalPathTurretYaw, finalPathGunPitch = getShotAngles(descr, player.getOwnVehicleStabilisedMatrix(), (0, 0), aimPoint, True )
        currentGunMat = AimingSystems.getPlayerGunMat(finalPathTurretYaw, finalPathGunPitch)
        clientShotStart = currentGunMat.translation
        clientShotVec = currentGunMat.applyVector(Math.Vector3(0, 0, descr.shot['speed']))

        self._finalPathPoint, self._finalPathShellVelocity, finalPathFlightTime = self._getGunMarkerPosition( clientShotStart, clientShotVec, aimPoint.y if BigWorld._ba_config['spg']['ignoreObstacles'] else None )

    def onStrategicAimingSystemUpdateMatrix(self, aim):
        player = BigWorld.player( )
        descr = player.vehicleTypeDescriptor
        bb = BigWorld.player().arena.arenaType.boundingBox
        pos2D = _clampPoint2DInBox2D(bb[0] - Math.Vector2( 200.0, 200.0 ), bb[1] + Math.Vector2( 200.0, 200.0 ), Math.Vector2(aim._StrategicAimingSystem__planePosition.x, aim._StrategicAimingSystem__planePosition.z))
        aim._StrategicAimingSystem__planePosition.x = pos2D[0]
        aim._StrategicAimingSystem__planePosition.z = pos2D[1]

        playerPos = player.getOwnVehiclePosition()
    
        if not self._lastModeWasSniper:
            collPoint = BigWorld.wg_collideSegment(BigWorld.player().spaceID, aim._StrategicAimingSystem__planePosition + Math.Vector3(0, 1000.0, 0), aim._StrategicAimingSystem__planePosition + Math.Vector3(0, -1000.0, 0), 3)
            aim._StrategicAimingSystem__planePosition.y = 0.0 if collPoint is None else collPoint[0][1]
            self._initialDistance = ( Math.Vector3( aim._StrategicAimingSystem__planePosition.x, playerPos.y, aim._StrategicAimingSystem__planePosition.z ) - playerPos ).length + 0.01

        distance = ( Math.Vector3( aim._StrategicAimingSystem__planePosition.x, playerPos.y, aim._StrategicAimingSystem__planePosition.z ) - playerPos ).length + 0.01
        heightFactor = distance / self._initialDistance

        aimPoint = Math.Vector3( aim._StrategicAimingSystem__planePosition.x, playerPos.y * (1.0 - heightFactor) + aim._StrategicAimingSystem__planePosition.y * heightFactor, aim._StrategicAimingSystem__planePosition.z )

        self.calcTrajectoryProperties(aimPoint)
        aim._matrix.translation, self._shellVelocity = self._finalPathPoint, self._finalPathShellVelocity

        self._lastModeWasSniper = True

        #FLUSH_LOG( )

    def captureProjectile(self, shotID, model):
        self._projectileModel = model
        self._projectileID = shotID
        self._trackProjectile = False
        
    def predictProjectile(self):
        player = BigWorld.player( )
        if not self.enabled or self._projectileID in player._PlayerAvatar__projectileMover._ProjectileMover__projectiles:
            return
        descr = player.vehicleTypeDescriptor
        self._trackProjectile = True
        self._trackProjectileStartTime = BigWorld.time() + 2 * SERVER_TICK_LENGTH

        finalPathTurretYaw, finalPathGunPitch = getShotAngles(descr, player.getOwnVehicleStabilisedMatrix(), (0, 0), self._lastShotPoint, True)
        currentGunMat = AimingSystems.getPlayerGunMat(finalPathTurretYaw, finalPathGunPitch)
        self._trackProjectileStartPoint = currentGunMat.translation
        self._trackProjectileVelocity = currentGunMat.applyVector(Math.Vector3(0, 0, descr.shot['speed']))



spgAim = SPGAim()

oldStrategicCamera_create = StrategicCamera.StrategicCamera.create
def StrategicCamera_create(self, onChangeControlMode):
    spgAim._prevNearPlane = BigWorld.projection().nearPlane
    spgAim._prevFarPlane = BigWorld.projection().farPlane
    oldStrategicCamera_create(self, onChangeControlMode)
    spgAim.onStrategicCameraCreate(self)

oldStrategicCamera_enable = StrategicCamera.StrategicCamera.enable
def StrategicCamera_enable(self, targetPos, saveDist):
    spgAim._prevNearPlane = BigWorld.projection().nearPlane
    spgAim._prevFarPlane = BigWorld.projection().farPlane
    spgAim.onStrategicCameraEnable(self)
    oldStrategicCamera_enable(self, targetPos, saveDist)

oldStrategicCamera_disable = StrategicCamera.StrategicCamera.disable
def StrategicCamera_disable(self):
    oldStrategicCamera_disable(self)
    spgAim.onStrategicCameraDisable(self)
    BigWorld.projection().nearPlane = spgAim._prevNearPlane
    BigWorld.projection().farPlane = spgAim._prevFarPlane


oldStrategicCamera__cameraUpdate = StrategicCamera.StrategicCamera._StrategicCamera__cameraUpdate
def StrategicCamera__cameraUpdate( self ):
    replayCtrl = BattleReplay.g_replayCtrl

    if not spgAim.enabled:
        srcMat = mathUtils.createRotationMatrix((0, -math.pi * 0.499, 0))
        self._StrategicCamera__cam.source = srcMat
        self._StrategicCamera__cam.target.b = self._StrategicCamera__aimingSystem.matrix

        BigWorld.projection().nearPlane = spgAim._prevNearPlane
        BigWorld.projection().farPlane = spgAim._prevFarPlane
        BigWorld.projection().fov = StrategicCamera.StrategicCamera.ABSOLUTE_VERTICAL_FOV
        return oldStrategicCamera__cameraUpdate( self )

    return spgAim.onStrategicCameraUpdate(self)

oldStrategicAimingSystem_updateMatrix = StrategicAimingSystem.StrategicAimingSystem._StrategicAimingSystem__updateMatrix
def StrategicAimingSystem_updateMatrix(self):
    player = BigWorld.player( )
    descr = player.vehicleTypeDescriptor

    if not spgAim.enabled:
        if spgAim._lastModeWasSniper:
            if BigWorld._ba_config['spg']['ignoreObstacles']:
                turretYaw, gunPitch = getShotAngles(descr, player.getOwnVehicleStabilisedMatrix(), (0, 0), self._matrix.translation, True )
                currentGunMat = AimingSystems.getPlayerGunMat(turretYaw, gunPitch)
                clientShotStart = currentGunMat.translation
                clientShotVec = currentGunMat.applyVector(Math.Vector3(0, 0, descr.shot['speed']))
                self._matrix.translation, spgAim._shellVelocity, _ = spgAim._getGunMarkerPosition( clientShotStart, clientShotVec, None )

            self._StrategicAimingSystem__planePosition = Math.Vector3(self._matrix.translation.x, 0.0, self._matrix.translation.z)

        oldStrategicAimingSystem_updateMatrix( self )
        spgAim._lastModeWasSniper = False

        collPoint = BigWorld.wg_collideSegment(BigWorld.player().spaceID, self._StrategicAimingSystem__planePosition + Math.Vector3(0, 1000.0, 0), self._StrategicAimingSystem__planePosition + Math.Vector3(0, -1000.0, 0), 3)
        aimPoint = Math.Vector3( self._StrategicAimingSystem__planePosition.x, 0.0 if collPoint is None else collPoint[0][1], self._StrategicAimingSystem__planePosition.z )
        spgAim.calcTrajectoryProperties(aimPoint)
        return

    spgAim.onStrategicAimingSystemUpdateMatrix(self)

oldStrategicAimingSystem_getDesiredShotPoint = StrategicAimingSystem.StrategicAimingSystem.getDesiredShotPoint
def StrategicAimingSystem_getDesiredShotPoint( self, terrainOnlyCheck = False ):
    if not spgAim.enabled:
        return oldStrategicAimingSystem_getDesiredShotPoint( self, terrainOnlyCheck )

    spgAim._lastShotPoint = self._matrix.translation
    return self._matrix.translation

def minimapResetCamera(cam):
    #import pydevd; pydevd.settrace();
    minimap = g_appLoader.getDefBattleApp().containerManager.getContainer(ViewTypes.VIEW).getView().components[BATTLE_VIEW_ALIASES.MINIMAP]
    if minimap is None:
        return

    if minimap._Minimap__cameraHandle is not None:
        minimap._Minimap__ownUI.delEntry(minimap._Minimap__cameraHandle)

    if spgAim.enabled:
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
    if keyProcessed:
        return True

    if isDown and key in eval(BigWorld._ba_config['spg']['keys']):
        spgAim.enabled = not spgAim.enabled

        BigWorld.player().positionControl.followCamera(not spgAim.enabled)

        #minimapResetCamera(self._cam)

        return True

    return False

oldProjectileMover_add = ProjectileMover.ProjectileMover.add
def ProjectileMover_add(*kargs, **kwargs):
    if not spgAim.enabled:
        oldProjectileMover_add(*kargs, **kwargs)    

    attackerID = kargs[oldProjectileMover_add.func_code.co_varnames.index('attackerID')]
    if spgAim.enabled and attackerID == BigWorld.player().playerVehicleID:
        kargs = list(kargs)
        kargs[oldProjectileMover_add.func_code.co_varnames.index('startPoint')] = kargs[oldProjectileMover_add.func_code.co_varnames.index('refStartPoint')]

    oldProjectileMover_add(*kargs, **kwargs)    

    attacker = BigWorld.entity(attackerID)
    if attackerID == BigWorld.player().playerVehicleID:
        shotID = kargs[oldProjectileMover_add.func_code.co_varnames.index('shotID')]
        proj = BigWorld.player()._PlayerAvatar__projectileMover._ProjectileMover__projectiles.get(shotID, None)
        if proj:
            spgAim.captureProjectile(shotID, proj['model'])

oldPlayerAvatar__startWaitingForShot = Avatar.PlayerAvatar._PlayerAvatar__startWaitingForShot
def PlayerAvatar__startWaitingForShot(*kargs, **kwargs):
    oldPlayerAvatar__startWaitingForShot(*kargs, **kwargs)
    if spgAim.enabled:
        spgAim.predictProjectile()


if BigWorld._ba_config['spg']['enabled']:
    StrategicCamera.StrategicCamera._StrategicCamera__cameraUpdate = StrategicCamera__cameraUpdate
    StrategicCamera.StrategicCamera.create = StrategicCamera_create
    StrategicCamera.StrategicCamera.enable = StrategicCamera_enable
    StrategicCamera.StrategicCamera.disable = StrategicCamera_disable
    StrategicAimingSystem.StrategicAimingSystem._StrategicAimingSystem__updateMatrix = StrategicAimingSystem_updateMatrix
    StrategicAimingSystem.StrategicAimingSystem.getDesiredShotPoint = StrategicAimingSystem_getDesiredShotPoint
    control_modes.StrategicControlMode.handleKeyEvent = StrategicControlMode_handleKeyEvent
    ProjectileMover.ProjectileMover.add = ProjectileMover_add
    Avatar.PlayerAvatar._PlayerAvatar__startWaitingForShot = PlayerAvatar__startWaitingForShot