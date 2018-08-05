# -*- coding: UTF-8 -*-
import codecs
import os
import re

isCFile = re.compile(r'c')


def transferThisFolder(path):
    everything = os.listdir(path)
    for thing in everything:
        if thing == '.git':
            pass
        elif os.path.isfile(path+'/'+thing):
            if re.search(r'\.[ch]$', thing):
                fileZouNi = path+'/'+thing
                print('zouni: ',fileZouNi)
                try:
                    f = codecs.open(fileZouNi, 'r', 'gb2312')
                    ff = f.read()
                    file_object = codecs.open(fileZouNi, 'w', 'utf-8')
                    file_object.write(ff)
                except:
                    pass
        else:
            transferThisFolder(path+'/'+thing)


transferThisFolder('.')
