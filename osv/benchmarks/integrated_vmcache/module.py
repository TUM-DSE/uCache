from osv.modules import api

api.require('libext')
default = api.run("/integrated_vmcache")
