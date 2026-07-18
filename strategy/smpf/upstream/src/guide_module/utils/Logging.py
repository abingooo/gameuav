#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
轻量日志封装：写入指定 .log 文件，可选同步输出到 stdout。
默认路径：/home/uav/lab/muav/src/guide_module/log/run.log

用法示例：
    from utils.logging import Logging
    log = Logging()  # 写默认路径
    log.info("节点启动")
    log.error("出错: %s", err)

    # 同时输出到终端并带颜色
    log_console = Logging(also_stdout=True, use_color=True)
    log_console.warn("注意: %s", detail)

    # 指定自定义路径
    from pathlib import Path
    log_custom = Logging(log_path=Path("/tmp/demo.log"), level="debug")
    log_custom.debug("调试信息")
"""

import sys
import inspect
from pathlib import Path
from typing import Optional, Union

# 避免与本文件同名导致的标准库 logging 冲突（尤其直接运行本脚本时）
_CUR_DIR = Path(__file__).resolve().parent
_SYS_PATH_BACKUP = list(sys.path)
try:
    sys.path = [p for p in sys.path if Path(p).resolve() != _CUR_DIR]
    import logging as std_logging
finally:
    sys.path = _SYS_PATH_BACKUP


class Logging:
    """
    写日志到文件，避免依赖 ROS 日志。
    示例:
        log = Logging()
        log.info("节点启动")
        log.error("出错: %s", err)

    标准 logging 等级（数值从低到高）：

    DEBUG (10)：最详细的调试信息
    INFO (20)：正常运行的提示
    WARNING (30)：潜在问题/重要提醒
    ERROR (40)：错误，但程序还能继续
    CRITICAL/FATAL (50)：严重错误，可能导致中止
    设置 logger/handler 的 level 为某值，则只输出该值及以上的日志。
    """

    DEFAULT_PATH = Path("/home/uav/lab/muav/src/guide_module/log/run.log")


    def __init__(
        self,
        log_path: Optional[Path] = None,
        level: Union[int, str] = std_logging.INFO,
        also_stdout: bool = False,
        use_color: bool = False,
    ) -> None:
        self.log_path = Path(log_path) if log_path else self.DEFAULT_PATH
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        resolved_level = self._normalize_level(level)
        self._counters = {}

        # 为每个日志文件路径创建独立 logger，避免重复 handler。
        logger_name = f"guide_logging:{self.log_path}"
        self.logger = std_logging.getLogger(logger_name)
        self.logger.setLevel(resolved_level)
        self.logger.propagate = False
        
        if not self.logger.handlers:
            file_handler = std_logging.FileHandler(self.log_path, encoding="utf-8")
            fmt = LevelNameFormatter(
                fmt="%(asctime)s [%(levelname)s] %(message)s",
                datefmt="%Y-%m-%d %H:%M:%S",
            )
            file_handler.setFormatter(fmt)
            self.logger.addHandler(file_handler)

            if also_stdout:
                stream_handler = std_logging.StreamHandler(sys.stdout)
                if use_color:
                    stream_handler.setFormatter(ColorFormatter())
                else:
                    stream_handler.setFormatter(fmt)
                self.logger.addHandler(stream_handler)
        # 计数器：key -> 次数，用于 info_every
        self._counters: dict[str, int] = {}

    @staticmethod
    def _normalize_level(level: Union[int, str]) -> int:
        """支持传入 'debug'/'info'/'warn'/'error'/'fatal' 或对应整数。"""
        if isinstance(level, str):
            name = level.strip().upper()
            mapping = {
                "DEBUG": std_logging.DEBUG,
                "INFO": std_logging.INFO,
                "WARN": std_logging.WARNING,
                "WARNING": std_logging.WARNING,
                "ERROR": std_logging.ERROR,
                "FATAL": std_logging.FATAL,
                "CRITICAL": std_logging.FATAL,
            }
            return mapping.get(name, std_logging.INFO)
        return int(level)

    def _log(self, level: int, msg: str, *args: object, color: Optional[str] = None) -> None:
        # color 仅用于 stdout 彩色输出，写入文件仍为纯文本
        if color:
            self.logger.log(level, msg, *args, extra={"color_override": color})
        else:
            self.logger.log(level, msg, *args)

    def debug(self, msg: str, *args: object) -> None:
        self._log(std_logging.DEBUG, msg, *args)

    def info(self, msg: str, *args: object, color: Optional[str] = None) -> None:
        """可指定 color，在 stdout 输出时生效（文件仍无颜色码）。"""
        self._log(std_logging.INFO, msg, *args, color=color)

    def info_every(
        self,
        interval: int,
        msg: str,
        *args: object,
        counter_name: Optional[str] = None,
        color: Optional[str] = None,
    ) -> None:
        """
        节流info：每 interval 次输出一次（首次调用不会输出）。

        Args:
            interval: 输出间隔次数，<=0 时等价于直接 info。
            msg: 文本，支持 %-style 占位符。
            *args: 对应占位符的参数。
            counter_name: 可选自定义计数器 key，默认用 msg，便于在不同位置各自计数。
            color: 仅作用于 stdout 彩色输出，文件仍为纯文本。
        """
        if interval <= 0:
            self.info(msg, *args, color=color)
            return

        key = counter_name or self._callsite_key(msg)
        # 首次调用：立刻打印并初始化计数为 1
        if key not in self._counters:
            self.info(msg, *args, color=color)
            self._counters[key] = 1
            return

        count = self._counters[key] + 1
        if count >= interval:
            self.info(msg, *args, color=color)
            self._counters[key] = 0
        else:
            self._counters[key] = count

    def _callsite_key(self, msg: str) -> str:
        """
        基于调用方文件与行号生成唯一 key，避免同一 msg 在不同位置共享计数。
        """
        frame = inspect.stack()[2]
        return f"{frame.filename}:{frame.lineno}:{msg}"

    def warn(self, msg: str, *args: object) -> None:
        self._log(std_logging.WARNING, msg, *args)

    def error(self, msg: str, *args: object) -> None:
        self._log(std_logging.ERROR, msg, *args)

    def fatal(self, msg: str, *args: object) -> None:
        self._log(std_logging.FATAL, msg, *args)


class LevelNameFormatter(std_logging.Formatter):
    """将 CRITICAL 统一显示为 FATAL。"""

    def format(self, record: std_logging.LogRecord) -> str:
        orig_levelname = record.levelname
        try:
            if record.levelno >= std_logging.CRITICAL:
                record.levelname = "FATAL"
            return super().format(record)
        finally:
            record.levelname = orig_levelname


class ColorFormatter(LevelNameFormatter):
    """可选的彩色 stdout 输出，仅用于终端查看，文件仍为纯文本。"""

    COLORS = {
        std_logging.DEBUG: "\033[36m",  # cyan
        std_logging.INFO: "\033[32m",  # green (默认 INFO 改为绿色)
        std_logging.WARN: "\033[33m",  # yellow
        std_logging.ERROR: "\033[31m",  # red
        std_logging.FATAL: "\033[1;31m",  # bold red
    }
    NAME_COLORS = {
        "black": "\033[30m",  # 黑
        "black_bold": "\033[1;30m",  # 黑(高亮)
        "red": "\033[31m",  # 红
        "red_bold": "\033[1;31m",  # 红(高亮)
        "green": "\033[32m",  # 绿
        "green_bold": "\033[1;32m",  # 绿(高亮)
        "yellow": "\033[33m",  # 黄
        "yellow_bold": "\033[1;33m",  # 黄(高亮)
        "blue": "\033[34m",  # 蓝
        "blue_bold": "\033[1;34m",  # 蓝(高亮)
        "magenta": "\033[35m",  # 品红/紫
        "magenta_bold": "\033[1;35m",  # 品红/紫(高亮)
        "cyan": "\033[36m",  # 青
        "cyan_bold": "\033[1;36m",  # 青(高亮)
        "white": "\033[37m",  # 白
        "white_bold": "\033[1;37m",  # 白(高亮)
        "gray": "\033[90m",  # 灰/亮黑
        "gray_bold": "\033[1;90m",  # 灰/亮黑(高亮)
        "light_red": "\033[91m",  # 亮红
        "light_green": "\033[92m",  # 亮绿
        "light_yellow": "\033[93m",  # 亮黄
        "light_blue": "\033[94m",  # 亮蓝
        "light_magenta": "\033[95m",  # 亮紫
        "light_cyan": "\033[96m",  # 亮青
        "light_white": "\033[97m",  # 亮白
        "orange": "\033[38;5;208m",  # 橙
        "orange_bold": "\033[1;38;5;208m",  # 橙(高亮)
        "deep_blue": "\033[38;5;19m",  # 深蓝
        "deep_blue_bold": "\033[1;38;5;19m",  # 深蓝(高亮)
    }
    RESET = "\033[0m"
    BASE_FMT = "%(asctime)s [%(levelname)s] %(message)s"

    def __init__(self) -> None:
        super().__init__(fmt=self.BASE_FMT, datefmt="%Y-%m-%d %H:%M:%S")

    def format(self, record: std_logging.LogRecord) -> str:
        msg = super().format(record)
        color = self.COLORS.get(record.levelno, "")
        override = getattr(record, "color_override", None)
        if override:
            if override.startswith("\033"):
                color = override
            else:
                color = self.NAME_COLORS.get(override, color)
        if color and sys.stdout.isatty():
            return f"{color}{msg}{self.RESET}"
        return msg


if __name__ == "__main__":
    demo = Logging(also_stdout=True, use_color=True, level="debug")
    demo.debug("Logging demo: info to file and stdout (colored)")
    demo.info("Logging demo: info to file and stdout (colored)")
    demo.warn("Logging demo: warning example")
    demo.error("Logging demo: error example with number %d", 42)
    demo.fatal("Logging demo: error example with number %d", 42)
