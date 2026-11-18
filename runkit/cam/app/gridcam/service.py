# Service base class — the lifecycle contract every gridcam service follows.
#
# Lifecycle: __init__(app) -> setup() -> [async run() task] -> stop().
# Services must never let an exception escape run(); the Application
# supervisor decides whether a dead service is fatal (reboot) or not.


class Service:
    """Base class for all application services.

    name     : short identifier used in logs, telemetry and heartbeats.
    critical : if True, the watchdog is only fed while this service
               heartbeats, and the supervisor reboots if its task dies.
    """

    name = "service"
    critical = False

    def __init__(self, app):
        self.app = app
        self.log = app.log.tagged(self.name) if app is not None else None
        self._last_beat_ms = 0

    def setup(self):
        """Synchronous initialisation: read config, preallocate buffers."""
        pass

    async def run(self):
        """Long-lived asyncio task. Omit (leave as None check) for passive
        services that only expose methods to others."""
        pass

    def stop(self):
        """Release resources. Called on orderly shutdown/restart."""
        pass

    def heartbeat(self):
        """Mark this service alive; HealthService gates the WDT on this."""
        self._last_beat_ms = self.app.ticks_ms()

    def last_beat_ms(self):
        return self._last_beat_ms

    def stats(self):
        """Dict merged into /telemetry under this service's name."""
        return {}

    def healthy(self):
        """Override for service-specific health checks."""
        return True
