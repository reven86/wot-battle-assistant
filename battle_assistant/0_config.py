import BigWorld

if True:
    try:
        import mods.yaml as yaml
        import ResMgr

        configPath = ResMgr.resolveToAbsolutePath('scripts/client/mods/battle_assistant.txt')
        with open(configPath, 'rt') as configFile:
            BigWorld._ba_config = yaml.load(configFile.read())
    except:
        BigWorld._ba_config = {'spg':{'enabled':True, 'keys':"[Keys.KEY_MOUSE2, Keys.KEY_G]", 'zoomSpeed':3.0}, 'expert':{'enabled':True}, 'gunner':{'enabled':True}}

print 'Battle Assistant: v1.2.3'

