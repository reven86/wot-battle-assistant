import BigWorld
import threading
import urllib
from gui.prb_control.events_dispatcher import EventDispatcher

if True:
    try:
        import yaml
        import ResMgr

        configPath = ResMgr.resolveToAbsolutePath('scripts/client/gui/mods/mod_battle_assistant.txt').decode('utf-8')
        with open(configPath, 'rt') as configFile:
            BigWorld._ba_config = yaml.load(configFile.read())
    except:
        print "Can't read Battle Assistant config, using defaults"
        import sys, traceback
        traceback.print_exception(*sys.exc_info())
        BigWorld._ba_config = {
            'spg': {
                'enabled':True, 
                'keys':"[Keys.KEY_MOUSE2, Keys.KEY_G]", 
                'ignoreObstacles':False, 
                'zoomSpeed':3.0, 
                'alwaysFollowProjectile':False, 
                'followProjectileKey': 'Keys.KEY_LALT',
                'activateCommandersCamera': False,
            },
            'expert':{'enabled':True}, 'gunner':{'enabled':True}
        }
    finally:
        BigWorld._ba_config['version'] = '1.3.9'
        BigWorld._ba_config['analyticsEventSent'] = False

print 'Battle Assistant: v{}'.format(BigWorld._ba_config['version'])

def sendAnalyticsEvent():
    if BigWorld._ba_config['analyticsEventSent']:
        return

    try:
        dbID = BigWorld.player( ).databaseID

        params = urllib.urlencode({'uid': dbID, 'v': BigWorld._ba_config['version']})
        f = urllib.urlopen("http://wot-utils.appspot.com/ba-analytics", params).read()
        #f = urllib.urlopen("http://localhost:8000/ba-analytics", params).read()
        BigWorld._ba_config['analyticsEventSent'] = True
    except:
        pass    


oldEventDispatcher_loadHangar = EventDispatcher.loadHangar
def EventDispatcher_loadHangar(*kargs, **kwargs):
    oldEventDispatcher_loadHangar(*kargs, **kwargs)    

    if not BigWorld._ba_config['analyticsEventSent']:
        thread = threading.Thread( target = sendAnalyticsEvent, name = 'Thread' )
        thread.start( )


EventDispatcher.loadHangar = EventDispatcher_loadHangar
