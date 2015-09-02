import os
import marshal
import base64
import imp
import random
import string
import zlib
import re
import sys


name = sys.argv[1]


import os

out = open('mod_' + name + '.pyc', 'wb')
out.write(imp.get_magic())
out.write('\x00\x00\x00\x00')
template = 'import marshal;'
files = os.listdir(name) 

for fileName in files:
    if '.py' in fileName:
        with open('{0}/{1}'.format(name, fileName), 'rt') as f:
            print 'processing {0}'.format(fileName)
            code = compile(f.read(), name, 'exec')
            data = marshal.dumps(code)
            template += 'exec marshal.loads({0});'.format(repr(data))

out.write(marshal.dumps(compile(template, name, 'exec'))) 