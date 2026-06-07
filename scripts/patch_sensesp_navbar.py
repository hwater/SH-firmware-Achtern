"""PlatformIO pre-build patch: add a "Dash" item to the SensESP web-UI navbar.

The SensESP navbar is built from GET /api/routes, whose route list is hardcoded
in the library (sensesp/net/web/base_command_handler.cpp) with no app-level hook,
and SensESP registers that handler before our setup() code runs, so it cannot be
overridden from the sketch. This pre-build script injects one extra route into
the /api/routes JSON so a "Dash" entry appears in the navbar linking to our
custom /dash dashboard.

Three important details:
  * The entry is added to the JSON response only, NOT to the `routes` vector that
    the same function later uses to register SPA catch-all handlers. That keeps
    our real GET /dash handler from being shadowed by an index.html handler.
  * The href is the RELATIVE path "dash" (no leading slash). SensESP's frontend
    uses preact-router, whose click interceptor only hijacks links whose href
    starts with "/". A relative href therefore performs a normal full-page
    navigation, which resolves to /dash on the current origin and loads our page.
  * componentName MUST be one the SPA knows (StatusPage/SystemPage/...). An
    unknown name makes the frontend throw and render only the navbar. It is
    otherwise inert here: the client-side route is never reached because the
    relative href triggers a full page load.

The patch is idempotent and re-applies on every build, so it survives library
re-installs / updates (which would otherwise wipe a manual edit in .pio/libdeps).
"""

import glob
import os

Import("env")  # noqa: F821  (provided by PlatformIO/SCons)

# Stable idempotency marker (independent of the componentName used below).
PATCH_MARKER = "[achtern02] Extra navbar entry"

ANCHOR = (
    "  for (auto it = routes.begin(); it != routes.end(); ++it) {\n"
    "    routes_json.add(it->as_json());\n"
    "  }\n"
)

REPLACEMENT = ANCHOR + (
    "\n"
    "  // [achtern02] Extra navbar entry -> custom /dash dashboard. Added to the\n"
    "  // /api/routes JSON only (not to `routes`), so no SPA catch-all handler is\n"
    "  // registered for /dash and our real handler keeps serving it. The\n"
    '  // relative href "dash" makes preact-router skip click interception\n'
    "  // (it only hijacks hrefs starting with \"/\"), giving a full-page load.\n"
    '  routes_json.add(RouteDefinition("Dash", "dash", "StatusPage").as_json());\n'
)


def _patch():
    libdeps = env.get("PROJECT_LIBDEPS_DIR") or os.path.join(  # noqa: F821
        env.get("PROJECT_DIR", ""), ".pio", "libdeps"  # noqa: F821
    )
    pattern = os.path.join(
        libdeps, "**", "SensESP", "src", "sensesp", "net", "web",
        "base_command_handler.cpp",
    )
    candidates = glob.glob(pattern, recursive=True)
    if not candidates:
        print("[patch_sensesp_navbar] SensESP source not found yet; "
              "skipping (will retry on next build)")
        return
    for path in candidates:
        with open(path, "r", encoding="utf-8") as fh:
            content = fh.read()
        if PATCH_MARKER in content:
            print("[patch_sensesp_navbar] already patched:", path)
            continue
        if ANCHOR not in content:
            print("[patch_sensesp_navbar] WARNING: anchor not found, "
                  "cannot patch:", path)
            continue
        content = content.replace(ANCHOR, REPLACEMENT, 1)
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(content)
        print("[patch_sensesp_navbar] patched:", path)


_patch()
