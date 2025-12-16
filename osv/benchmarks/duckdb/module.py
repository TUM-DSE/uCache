from osv.modules import api

api.require('libext')
default = api.run('/benchmark_runner --help')
