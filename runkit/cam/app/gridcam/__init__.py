# gridcam — production grid (x, y, angle) camera application for OpenMV AE3.
#
# Version is read by the OTA bundler (host/make_bundle.py), reported in
# telemetry, and logged at boot. Bump on every release.
#
# _build.py is GENERATED at packaging time (make_bundle.py / deploy.sh)
# with the build date + git commit; it is never checked into the repo,
# so a source checkout reports build 'dev'.

VERSION = "1.32.0"

try:
    from ._build import BUILD_DATE, BUILD_COMMIT
except ImportError:
    BUILD_DATE = 'dev'
    BUILD_COMMIT = ''
