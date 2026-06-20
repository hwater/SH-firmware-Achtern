"""PlatformIO pre-build patches for SensESP.

All patches are idempotent and re-applied on every build so they survive
library re-installs/updates.

--- Patch 1 (base_command_handler.cpp): Dash navbar entry ---
Injects a "Dash" link into /api/routes so the SensESP web-UI navbar shows a
Dash entry pointing at our custom /dash dashboard.

--- Patch 2 (base_command_handler.cpp): Case-insensitive origin check ---
SensESP 3.3 guards POST /api/device/restart (and reset) with a cross-origin
check that compares the Origin header to the stored hostname using
String::indexOf, which is case-sensitive.  If the browser accesses the device
via the AP whose SSID is uppercase (e.g. http://ACHTERNSENSORIK.local) while
the stored hostname is lowercase ("achternsensorik"), the check fails with 403
Forbidden.  Fix: copy origin and hostname to lowercase before comparing.

--- Patch 3 (wifi_provisioner.cpp): Normalize AP SSID to lowercase ---
SensESP derives the AP SSID from get_hostname() at startup, but then
WiFiProvisioner::load() may override it with an older value stored in SPIFFS
(e.g. "ACHTERNSENSORIK" saved before the hostname was migrated to lowercase).
This patch lowercases the AP SSID right after load() so it always matches the
mDNS hostname, satisfying the case-sensitive origin check in the browser's
Origin header.

--- Patch 4 (http_server.h): Enable LRU purge on the ESP-IDF httpd ---
SensESP starts the ESP-IDF http server with HTTPD_DEFAULT_CONFIG(), which sets
max_open_sockets = 7 and lru_purge_enable = false.  Browsers on the boat WiFi
(phones/tablets) keep HTTP/1.1 keep-alive sockets open; when they sleep or leave
the network the TCP socket lingers as a half-open session the server never
reclaims.  After ~7 such stale sessions accumulate (observed as the web-UI/API
"hanging" after a few days uptime) the listener can no longer accept new
connections.  Enabling lru_purge_enable makes the server drop the
least-recently-used (oldest/stale) session to admit a new connection, so it
self-heals instead of locking up.  (This framework's esp_http_server.h predates
the keep_alive_* config fields, so TCP keep-alive probes are not available here;
LRU purge is the portable fix.)
"""

import glob
import os

Import("env")  # noqa: F821  (provided by PlatformIO/SCons)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _apply_patch(content, path, marker, anchor, replacement):
    """Apply one idempotent patch. Returns (new_content, changed: bool)."""
    if marker in content:
        print("[patch_sensesp] already patched (%s): %s" % (marker, path))
        return content, False
    if anchor not in content:
        print("[patch_sensesp] WARNING: anchor not found (%s): %s"
              % (marker, path))
        return content, False
    new_content = content.replace(anchor, replacement, 1)
    print("[patch_sensesp] patched (%s): %s" % (marker, path))
    return new_content, True


def _patch_file(pattern, patches):
    libdeps = env.get("PROJECT_LIBDEPS_DIR") or os.path.join(  # noqa: F821
        env.get("PROJECT_DIR", ""), ".pio", "libdeps"  # noqa: F821
    )
    full_pattern = os.path.join(libdeps, "**", *pattern.split("/"))
    candidates = glob.glob(full_pattern, recursive=True)
    if not candidates:
        print("[patch_sensesp] source not found yet (%s); "
              "skipping (will retry on next build)" % pattern)
        return
    for path in candidates:
        with open(path, "r", encoding="utf-8") as fh:
            content = fh.read()
        changed = False
        for marker, anchor, replacement in patches:
            content, c = _apply_patch(content, path, marker, anchor, replacement)
            changed = changed or c
        if changed:
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(content)


# ---------------------------------------------------------------------------
# Patch 1 – Dash navbar entry  (base_command_handler.cpp)
# ---------------------------------------------------------------------------
_NAVBAR_ANCHOR = (
    "  for (auto it = routes.begin(); it != routes.end(); ++it) {\n"
    "    routes_json.add(it->as_json());\n"
    "  }\n"
)
_NAVBAR_REPLACEMENT = _NAVBAR_ANCHOR + (
    "\n"
    "  // [achtern02] Extra navbar entry -> custom /dash dashboard. Added to the\n"
    "  // /api/routes JSON only (not to `routes`), so no SPA catch-all handler is\n"
    "  // registered for /dash and our real handler keeps serving it. The\n"
    '  // relative href "dash" makes preact-router skip click interception\n'
    "  // (it only hijacks hrefs starting with \"/\"), giving a full-page load.\n"
    "  // componentName MUST be one the SPA knows (StatusPage/SystemPage/...);\n"
    "  // an unknown name makes the frontend throw and render only the navbar.\n"
    "  // It is otherwise inert here: the client-side route is never reached\n"
    "  // because the relative href triggers a full page load to /dash.\n"
    '  routes_json.add(RouteDefinition("Dash", "dash", "StatusPage").as_json());\n'
)

# ---------------------------------------------------------------------------
# Patch 2 – Case-insensitive origin check  (base_command_handler.cpp)
# Arduino String::indexOf is case-sensitive; toLowerCase() is void/in-place,
# so we copy to lowercase temporaries before comparing.
# ---------------------------------------------------------------------------
_ORIGIN_ANCHOR = (
    "    String hostname = SensESPBaseApp::get_hostname();\n"
    "    if (origin_str.indexOf(hostname) < 0) {\n"
)
_ORIGIN_REPLACEMENT = (
    "    String hostname = SensESPBaseApp::get_hostname();\n"
    "    // [achtern02] Case-insensitive origin check: Arduino String::indexOf is\n"
    "    // case-sensitive, so 'ACHTERNSENSORIK.local' would not match hostname\n"
    "    // 'achternsensorik'. toLowerCase() is void/in-place, so copy first.\n"
    "    String _origin_lc = origin_str; _origin_lc.toLowerCase();\n"
    "    String _host_lc = hostname;     _host_lc.toLowerCase();\n"
    "    if (_origin_lc.indexOf(_host_lc) < 0) {\n"
)

_BASE_CMD_FILE = "SensESP/src/sensesp/net/web/base_command_handler.cpp"
_BASE_CMD_PATCHES = [
    ("[achtern02] Extra navbar entry",        _NAVBAR_ANCHOR,  _NAVBAR_REPLACEMENT),
    ("[achtern02] Case-insensitive origin",   _ORIGIN_ANCHOR,  _ORIGIN_REPLACEMENT),
]

# ---------------------------------------------------------------------------
# Patch 3 – Normalize AP SSID to lowercase  (wifi_provisioner.cpp)
# After load(), the stored ap_settings_.ssid_ might be uppercase (saved before
# the hostname migration).  Lowercasing it keeps the AP SSID in sync with the
# mDNS hostname so the browser's Origin header matches.
# ---------------------------------------------------------------------------
_AP_SSID_ANCHOR = (
    "  bool config_loaded = load();\n"
    "\n"
    "  if (!config_loaded) {\n"
)
_AP_SSID_REPLACEMENT = (
    "  bool config_loaded = load();\n"
    "  // [achtern02] Normalize AP SSID to lowercase so it matches the mDNS hostname\n"
    "  // ('achternsensorik') — stored settings may have saved old uppercase value.\n"
    "  ap_settings_.ssid_.toLowerCase();\n"
    "\n"
    "  if (!config_loaded) {\n"
)

_WIFI_PROV_FILE = "SensESP/src/sensesp/net/wifi_provisioner.cpp"
_WIFI_PROV_PATCHES = [
    ("[achtern02] Normalize AP SSID to lowercase", _AP_SSID_ANCHOR, _AP_SSID_REPLACEMENT),
]

# ---------------------------------------------------------------------------
# Patch 4 – Enable LRU purge on the ESP-IDF httpd  (http_server.h)
# Without it the server stops accepting connections once max_open_sockets (7)
# stale keep-alive sessions pile up (web-UI "hangs" after a few days uptime).
# Injected right before httpd_start, after the other config_ tweaks.
# ---------------------------------------------------------------------------
_LRU_ANCHOR = (
    "    config_.uri_match_fn = httpd_uri_match_wildcard;\n"
)
_LRU_REPLACEMENT = _LRU_ANCHOR + (
    "    // [achtern02] Self-heal long-uptime socket exhaustion: when all\n"
    "    // max_open_sockets session slots are taken by stale/half-open\n"
    "    // keep-alive connections, purge the least-recently-used one to admit\n"
    "    // the new connection instead of refusing it (web-UI/API would\n"
    "    // otherwise appear to hang after a few days). See patch_sensesp_navbar.py.\n"
    "    config_.lru_purge_enable = true;\n"
)

_HTTP_SERVER_FILE = "SensESP/src/sensesp/net/http_server.h"
_HTTP_SERVER_PATCHES = [
    ("[achtern02] Self-heal long-uptime socket exhaustion", _LRU_ANCHOR, _LRU_REPLACEMENT),
]

# ---------------------------------------------------------------------------
# Apply all patches
# ---------------------------------------------------------------------------
_patch_file(_BASE_CMD_FILE, _BASE_CMD_PATCHES)
_patch_file(_WIFI_PROV_FILE, _WIFI_PROV_PATCHES)
_patch_file(_HTTP_SERVER_FILE, _HTTP_SERVER_PATCHES)
