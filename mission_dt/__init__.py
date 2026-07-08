"""Mission-DT package. Raises the file-descriptor limit on POSIX so
large fleets (100+ agents, ~3 sockets each) run on default macOS
shells (soft limit 256)."""
import sys

if sys.platform != "win32":
    import resource
    _soft, _hard = resource.getrlimit(resource.RLIMIT_NOFILE)
    if _soft < 4096:
        try:
            resource.setrlimit(resource.RLIMIT_NOFILE,
                               (min(4096, _hard), _hard))
        except (ValueError, OSError):
            pass  # keep going with whatever the OS allows
