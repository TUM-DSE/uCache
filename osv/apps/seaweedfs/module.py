from osv.modules import api

default = api.run(cmdline="/weed master -port 9333")
server = api.run(cmdline="/weed server -dir=/tmp -volume.max=5")
volume = api.run(cmdline="/weed volume -dir=/tmp")
