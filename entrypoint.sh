#!/bin/sh
rfkill unblock bluetooth 2>/dev/null || true
exec /usr/local/bin/odid-daemon "$@"
