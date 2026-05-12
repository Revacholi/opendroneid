#!/bin/sh
NO_BT=0
NO_WIFI=0

[ "${ODID_NO_BT:-0}"   != "0" ] && NO_BT=1
[ "${ODID_NO_WIFI:-0}" != "0" ] && NO_WIFI=1

for arg in "$@"; do
    case "$arg" in
        --no-bt)   NO_BT=1 ;;
        --no-wifi) NO_WIFI=1 ;;
    esac
done

[ "$NO_BT"   = "0" ] && rfkill unblock bluetooth 2>/dev/null || true
[ "$NO_WIFI" = "0" ] && rfkill unblock wifi     2>/dev/null || true

exec /usr/local/bin/odid-daemon "$@"
