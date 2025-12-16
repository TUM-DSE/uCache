import os

from osv.modules import api


ROFS = ['rofs', 'virtiofs'] # List of read-only filesystems


api.require('libz')

# NOTE: This is necessary to avoid the warning about the default error log
# location. See https://trac.nginx.org/nginx/ticket/147.
error_log = ' -e stderr' if os.environ.get('fs_type') in ROFS else ''
default = api.run('/nginx.so -c /nginx/conf/nginx.conf' + error_log)
