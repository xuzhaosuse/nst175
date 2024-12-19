from building import *

cwd     = GetCurrentDir()
src     = Glob('*.c')
path    = [cwd]

group = DefineGroup('nst175', src, depend = ['PKG_USING_NST175'], CPPPATH = path)

Return('group')