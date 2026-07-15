"""Entry point for the autonomous robot backend."""

from __future__ import annotations

import uvicorn

from config import RobotConfig
from network.server import create_app
from utils.logger import get_logger

logger = get_logger("main")


class RobotBackend:
    """Main application orchestrator for the robot backend."""

    def __init__(self, config: RobotConfig | None = None) -> None:
        self.config = config or RobotConfig.from_env()
        self.app = create_app(self.config)

    def run(self) -> None:
        """Run the backend as a FastAPI service."""
        logger.info("Starting robot backend on %s:%s", self.config.host, self.config.port)
        uvicorn.run(self.app, host=self.config.host, port=self.config.port)


def main() -> None:
    """Launch the robot backend."""
    backend = RobotBackend()
    backend.run()


if __name__ == "__main__":
    main()
