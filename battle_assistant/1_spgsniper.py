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
from gui.WindowsManager import g_windowsManager
from gui.scaleform import Minimap
from gun_rotation_shared import calcPitchLimitsFromDesc
from gui import DEPTH_OF_GunMarker
from gui import g_guiResetters
import ClientArena



class SPGAim(object):
    
    def __init__(self):
        self.enabled = False

    def _isOutOfLimits(self, angle, limits):
        if limits is None:
            return False
        elif abs(limits[1] - angle) < 1e-05 or abs(limits[0] - angle) < 1e-05:
            return False
        else:
            dpi = 2 * math.pi
            minDiff = math.fmod(limits[0] - angle + dpi, dpi)
            maxDiff = math.fmod(limits[1] - angle + dpi, dpi)
            return minDiff <= maxDiff

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
        BigWorld.projection().nearPlane = self._prevNearPlane
        BigWorld.projection().farPlane = self._prevFarPlane

        if self.enabled:
            camera._StrategicCamera__aimingSystem._StrategicAimingSystem__planePosition.x = camera._StrategicCamera__aimingSystem._matrix.translation.x
            camera._StrategicCamera__aimingSystem._StrategicAimingSystem__planePosition.y = 0.0
            camera._StrategicCamera__aimingSystem._StrategicAimingSystem__planePosition.z = camera._StrategicCamera__aimingSystem._matrix.translation.z
            self._lastModeWasSniper = False
            self.enabled = False

    def onStrategicCameraUpdate(self, camera):
        replayCtrl = BattleReplay.g_replayCtrl


        distRange = camera._StrategicCamera__cfg['distRange'][:]
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


        if replayCtrl.isPlaying and replayCtrl.isControllingCamera:
            aimOffset = replayCtrl.getAimClipPosition()
        else:
            aimWorldPos = camera._StrategicCamera__aimingSystem.matrix.translation
            aimOffset = cameras.projectPoint(aimWorldPos)
            if replayCtrl.isRecording:
                replayCtrl.setAimClipPosition(Math.Vector2( aimOffset.x, aimOffset.y ))

        camera._StrategicCamera__aimOffsetFunc((aimOffset.x, aimOffset.y))
        shotDescr = BigWorld.player().vehicleTypeDescriptor.shot
        BigWorld.wg_trajectory_drawer().setParams(shotDescr['maxDistance'], Math.Vector3(0, -shotDescr['gravity'], 0), camera._StrategicCamera__aimOffsetFunc())
        curTime = BigWorld.time()
        deltaTime = curTime - camera._StrategicCamera__prevTime
        camera._StrategicCamera__prevTime = curTime

        if replayCtrl.isPlaying:
            if camera._StrategicCamera__needReset != 0:
                if camera._StrategicCamera__needReset > 1:
                    player = BigWorld.player()
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

        return 0

    def calcTrajectoryProperties(self, aimPoint):
        player = BigWorld.player( )
        descr = player.vehicleTypeDescriptor

        finalPathTurretYaw, finalPathGunPitch = getShotAngles(descr, player.getOwnVehicleMatrix(), (0, 0), aimPoint, True )
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

    def onStartBattle(self):
        pass

    def onStopBattle(self):
        pass

    def _createLabel(self, x, y, depth):
        label = GUI.Text('')
        label.visible = True
        label.font = 'hpmp_panel.font'
        label.colourFormatting = True
        label.multiline = True
        label.filterType = 'LINEAR'
        label.widthMode = 'PIXEL'
        label.heightMode = 'PIXEL'
        label.verticalPositionMode = 'PIXEL'
        label.horizontalPositionMode = 'PIXEL'
        label.horizontalAnchor = 'LEFT'
        GUI.addRoot(label)
        sr = GUI.screenResolution()
        x = sr[0] * 0.5 + x
        y = sr[1] * 0.5 + y
        label.position = (x, y, depth)
        return label
        
    def onChangeScreenResolution(self):
        pass


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


oldStrategicCamera__cameraUpdate = StrategicCamera.StrategicCamera._StrategicCamera__cameraUpdate
def StrategicCamera__cameraUpdate( self ):
    replayCtrl = BattleReplay.g_replayCtrl

    if not spgAim.enabled:
        srcMat = mathUtils.createRotationMatrix((0, -math.pi * 0.499, 0))
        self._StrategicCamera__cam.source = srcMat
        self._StrategicCamera__cam.target.b = self._StrategicCamera__aimingSystem.matrix

        if not replayCtrl.isPlaying:
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
                turretYaw, gunPitch = getShotAngles(descr, player.getOwnVehicleMatrix(), (0, 0), self._matrix.translation, True )
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

    return self._matrix.translation

def minimapResetCamera(cam):
    minimap = g_windowsManager.battleWindow.minimap
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

        minimapResetCamera(self._cam)

        return True

    return False

oldClientArena_setupBBColliders = ClientArena.ClientArena._ClientArena__setupBBColliders
def ClientArena_setupBBColliders(self):
    if BigWorld.wg_getSpaceBounds().length == 0.0:
        return False
    arenaBB = self.arenaType.boundingBox
    spaceBB = ClientArena._convertToList(BigWorld.wg_getSpaceBounds())
    self._ClientArena__arenaBBCollider = ClientArena._BBCollider(arenaBB, (-500.0, 2500.0))
    self._ClientArena__spaceBBCollider = ClientArena._BBCollider(spaceBB, (-500.0, 2500.0))
    return True


oldGunControlMode_createGunMarker = control_modes._GunControlMode._GunControlMode__createGunMarker
def GunControlMode_createGunMarker(self, mode, isStrategic):
    oldGunControlMode_createGunMarker(self, mode, False)


if BigWorld._ba_config['spg']['enabled']:
    StrategicCamera.StrategicCamera._StrategicCamera__cameraUpdate = StrategicCamera__cameraUpdate
    StrategicCamera.StrategicCamera.create = StrategicCamera_create
    StrategicCamera.StrategicCamera.enable = StrategicCamera_enable
    StrategicCamera.StrategicCamera.disable = StrategicCamera_disable
    StrategicAimingSystem.StrategicAimingSystem._StrategicAimingSystem__updateMatrix = StrategicAimingSystem_updateMatrix
    StrategicAimingSystem.StrategicAimingSystem.getDesiredShotPoint = StrategicAimingSystem_getDesiredShotPoint
    control_modes.StrategicControlMode.handleKeyEvent = StrategicControlMode_handleKeyEvent
    ClientArena.ClientArena._ClientArena__setupBBColliders = ClientArena_setupBBColliders
    #control_modes._GunControlMode._GunControlMode__createGunMarker = GunControlMode_createGunMarker

    g_windowsManager.onInitBattleGUI += spgAim.onStartBattle
    g_windowsManager.onDestroyBattleGUI += spgAim.onStopBattle
    g_guiResetters.add(spgAim.onChangeScreenResolution)

    #print 'SPG Sniper Mod enabled'

    #print control_modes.StrategicControlMode.handleKeyEvent