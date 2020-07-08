from building import *
Import('rtconfig')

src = []
path = []
cwd   = GetCurrentDir()

if GetDepend('PKG_USING_SOFTUART'):
    src += Glob('softuart.c')
    path += [cwd]
    
if GetDepend('PKG_USING_SOFTUART_STM32_DRV'):
    src += Glob('stm32_drv/*.c')
    path += [cwd + '/stm32_drv']


# add src and include to group.
group = DefineGroup('softuart', src, depend = ['PKG_USING_SOFTUART'], CPPPATH = path)

Return('group')
