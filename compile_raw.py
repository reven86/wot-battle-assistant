import os
import marshal
import base64
import imp
import random
import string
import zlib
import re
import sys
import yaml
import cPickle
import types
from modulefinder import ModuleFinder, Module


name = sys.argv[1]


import os

out = open('mod_' + name + '.pyc', 'wb')
out.write(imp.get_magic())
out.write('\x00\x00\x00\x00')
template = 'import marshal;'
template += 'import sys, types;'


def packModule(name):
    importOrder = []


    importOrder = []
    finder = ModuleFinder()
    old_addModule = finder.add_module
    def addModule(fqname):
        if fqname.startswith(name):
            #print fqname
            if fqname not in importOrder:
                importOrder.append(fqname)
        return old_addModule(fqname)
    finder.add_module = addModule
    finder.import_module(name, name, None)

    # hack for yaml
    importOrder = [
        'yaml',
        'yaml.error',
        'yaml.tokens',
        'yaml.events',
        'yaml.nodes',
        'yaml.reader',
        'yaml.scanner',
        'yaml.composer',
        'yaml.constructor',
        'yaml.resolver',
        'yaml.parser',
        'yaml.loader',
        'yaml.emitter',
        'yaml.serializer',
        'yaml.representer',
        'yaml.dumper',
    ]

    global template
    template += '{0}=types.ModuleType("{0}","None");'.format(name)
    template += '{}.__dict__["__builtins__"]=__builtins__;'.format(name)
    template += 'sys.modules["{0}"]={0};'.format(name)

    for n in importOrder:
        if not n.startswith(name):
            continue

        print 'packing {}'.format(n)
        if n == name:
            continue

        filename = n.replace('.','/')+'.pyc'

        modName = filename[len(name)+1:-4]#n.replace('.','_')
        template += '{}=types.ModuleType("{}","None");'.format(modName,filename[len(name)+1:-4])
        template += '{}.__dict__["__builtins__"]=__builtins__;'.format(modName)
        template += '{}.__dict__["{}"]={};'.format(name, filename[len(name)+1:-4], modName)
        template += 'sys.modules["{0}"]={0};'.format(modName)
        with open(filename, 'rb') as f:
            data = f.read()[8:]
            template += 'exec marshal.loads({}) in {}.__dict__;'.format(repr(data), modName)

    for n in importOrder:
        filename = n.replace('.','/')+'.pyc'
        modName = filename[len(name)+1:-4]#n.replace('.','_')
        if len(modName)>0:
            template += 'del {};'.format(modName)

    with open(name+'/__init__.pyc', 'rb') as f:
        data = f.read()[8:]
        template += 'exec marshal.loads({}) in {}.__dict__;'.format(repr(data), name)

packModule('yaml')
#print template
    

files = os.listdir(name) 

for fileName in files:
    if '.py' in fileName:
        with open('{0}/{1}'.format(name, fileName), 'rt') as f:
            print 'processing {0}'.format(fileName)
            code = compile(f.read(), fileName, 'exec')
            dictName = ''.join((random.choice(string.ascii_letters) for _ in range(random.randint(2, 10)))) + '_' + fileName[:-3]
            data = marshal.dumps(code)
            template += '{1}=dict();exec marshal.loads({0}) in {1};'.format(repr(data), dictName)

out.write(marshal.dumps(compile(template, name, 'exec'))) 