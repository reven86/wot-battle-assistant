import BigWorld
import Avatar
import constants
import Vehicle
from gui.app_loader import g_appLoader
from chat_shared import CHAT_COMMANDS
import CommandMapping
import TriggersManager
from TriggersManager import TRIGGER_TYPE
import random
import string
from gui.battle_control import g_sessionProvider
from debug_utils import *


 
gExpertTarget = None
 
oldPlayerAvatar_targetFocus = Avatar.PlayerAvatar.targetFocus
def PlayerAvatar_targetFocus(self, entity):
    global gExpertTarget
    saveMaySeeOtherVehicleDamagedDevices = self._PlayerAvatar__maySeeOtherVehicleDamagedDevices

    self._PlayerAvatar__maySeeOtherVehicleDamagedDevices = (gExpertTarget is None and self._PlayerAvatar__maySeeOtherVehicleDamagedDevices)
    oldPlayerAvatar_targetFocus(self, entity)
    self._PlayerAvatar__maySeeOtherVehicleDamagedDevices = saveMaySeeOtherVehicleDamagedDevices
 
oldPlayerAvatar_targetBlur = Avatar.PlayerAvatar.targetBlur
def PlayerAvatar_targetBlur(self, prevEntity):
    global gExpertTarget
    saveMaySeeOtherVehicleDamagedDevices = self._PlayerAvatar__maySeeOtherVehicleDamagedDevices

    self._PlayerAvatar__maySeeOtherVehicleDamagedDevices = (gExpertTarget is None and self._PlayerAvatar__maySeeOtherVehicleDamagedDevices)
    oldPlayerAvatar_targetBlur(self, prevEntity)
    self._PlayerAvatar__maySeeOtherVehicleDamagedDevices = saveMaySeeOtherVehicleDamagedDevices
 
old_PlayerAvatar_showOtherVehicleDamagedDevices = Avatar.PlayerAvatar.showOtherVehicleDamagedDevices
def PlayerAvatar_showOtherVehicleDamagedDevices(self, vehicleID, damagedExtras, destroyedExtras):
    #print 'PlayerAvatar_showOtherVehicleDamagedDevices'
    global gExpertTarget
    if gExpertTarget is not None:
        target = gExpertTarget
        feedback = g_sessionProvider.getFeedback()
        if not isinstance(target, Vehicle.Vehicle):
            if self._PlayerAvatar__maySeeOtherVehicleDamagedDevices and vehicleID != 0:
                self.cell.monitorVehicleDamagedDevices(0)
                #print 'PlayerAvatar_showOtherVehicleDamagedDevices monitor {0}'.format(0)
                #FLUSH_LOG()
        elif target.id == vehicleID:
            feedback.showVehicleDamagedDevices(vehicleID, damagedExtras, destroyedExtras, avatar=self)
        else:
            if self._PlayerAvatar__maySeeOtherVehicleDamagedDevices:
                self.cell.monitorVehicleDamagedDevices(target.id)
                #print 'PlayerAvatar_showOtherVehicleDamagedDevices monitor {0}'.format(target.id)
                #FLUSH_LOG()
            feedback.hideVehicleDamagedDevices(vehicleID)
    else:
        old_PlayerAvatar_showOtherVehicleDamagedDevices(self, vehicleID, damagedExtras, destroyedExtras)
 

def setNewTarget(newTarget):
    global gExpertTarget
    #print 'newTarget {0} oldTarget {1}'.format(newTarget, gExpertTarget)
    if newTarget is not gExpertTarget and (newTarget is None or newTarget.isAlive()):
        gExpertTarget = newTarget
        BigWorld.player().cell.monitorVehicleDamagedDevices( gExpertTarget.id if gExpertTarget is not None else 0 )
        #print 'setNewTarget monitor {0}'.format(gExpertTarget.id if gExpertTarget is not None else 0)
        #FLUSH_LOG()
        if g_appLoader.getDefBattleApp():
            g_appLoader.getDefBattleApp().pMsgsPanel._FadingMessages__showMessage(random.choice(string.ascii_letters), 'Expert: {0}'.format(g_sessionProvider.getCtx().getPlayerFullNameParts(vID=gExpertTarget.id)[0]) if gExpertTarget is not None else 'Expert: OFF', 'default')


oldPlayerAvatar_handleKey = Avatar.PlayerAvatar.handleKey
def PlayerAvatar_handleKey(self, isDown, key, mods):
    if self._PlayerAvatar__maySeeOtherVehicleDamagedDevices:
        cmdMap = CommandMapping.g_instance
        #print 'PlayerAvatar_handleKey {0} {1} {2}'.format(cmdMap.isFired(CommandMapping.CMD_CHAT_SHORTCUT_ATTACK, key), isDown, self._PlayerAvatar__maySeeOtherVehicleDamagedDevices)
        if isDown and cmdMap.isFired(CommandMapping.CMD_CHAT_SHORTCUT_ATTACK, key):
            setNewTarget(BigWorld.target())

    return oldPlayerAvatar_handleKey(self, isDown, key, mods)


oldVehicle_stopVisual = Vehicle.Vehicle.stopVisual
def Vehicle_stopVisual(self):
    oldVehicle_stopVisual( self )

    global gExpertTarget
    if gExpertTarget is not None and self.id == gExpertTarget.id:
        setNewTarget(None)

oldVehicle__onVehicleDeath = Vehicle.Vehicle._Vehicle__onVehicleDeath
def Vehicle__onVehicleDeath(self, isDeadStarted = False):
    oldVehicle__onVehicleDeath(self, isDeadStarted)
    if gExpertTarget is not None and gExpertTarget.id == self.id:
        setNewTarget(None)


oldPlayerAvatar_showShotResults = Avatar.PlayerAvatar.showShotResults
def PlayerAvatar_showShotResults(self, results):
    oldPlayerAvatar_showShotResults(self, results)

    if not self._PlayerAvatar__maySeeOtherVehicleDamagedDevices:
        return

    VHF = constants.VEHICLE_HIT_FLAGS
    for r in results:
        vehicleID = r & 4294967295L
        flags = r >> 32 & 4294967295L
        if flags & VHF.VEHICLE_WAS_DEAD_BEFORE_ATTACK:
            continue
        if flags & VHF.VEHICLE_KILLED:
            return
        setNewTarget(BigWorld.entity(vehicleID))
        return


                                   

if BigWorld._ba_config['expert']['enabled']:
    Avatar.PlayerAvatar.targetFocus = PlayerAvatar_targetFocus
    Avatar.PlayerAvatar.targetBlur = PlayerAvatar_targetBlur
    Avatar.PlayerAvatar.showOtherVehicleDamagedDevices = PlayerAvatar_showOtherVehicleDamagedDevices
    Avatar.PlayerAvatar.handleKey = PlayerAvatar_handleKey
    Avatar.PlayerAvatar.showShotResults = PlayerAvatar_showShotResults
    Vehicle.Vehicle.stopVisual = Vehicle_stopVisual
    Vehicle.Vehicle._Vehicle__onVehicleDeath = Vehicle__onVehicleDeath
