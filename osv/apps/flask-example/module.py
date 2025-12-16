from osv.modules import api

#For a proper interactive python terminal
api.require('unknown-term')

default = api.run(cmdline="--env=TERM=unknown --env=PYTHONPATH=/ --env=PYTHONHOME=/ --env=PYTHONPLATLIBDIR=lib /python /cashman/index.py")
