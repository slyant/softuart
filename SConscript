from building import *
Import('rtconfig')

src = []
path = []
cwd   = GetCurrentDir()

if GetDepend('PKG_USING_SOFTUART'):
    src += Glob('softuart.c')
    path += [cwd]

# add src and include to group.
group = DefineGroup('softuart', src, depend = ['PKG_USING_SOFTUART'], CPPPATH = path)

Return('group')
