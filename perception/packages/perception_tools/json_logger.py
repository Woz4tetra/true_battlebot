import logging
import time
from typing import Any

from pythonjsonlogger import jsonlogger


class CustomJsonFormatter(jsonlogger.JsonFormatter):
    def add_fields(self, log_record: dict[str, Any], record: logging.LogRecord, message_dict: dict[str, Any]):
        super(CustomJsonFormatter, self).add_fields(log_record, record, message_dict)
        if not log_record.get("time"):
            # this doesn't use record.created, so it is slightly off
            log_record["time"] = time.time()
        if log_record.get("level"):
            log_record["severity"] = log_record["level"].upper()
            log_record.pop("level")
        else:
            log_record["severity"] = record.levelname
        log_record["message"] = f"<-<{record.message}>->"
        log_record["exc_info"] = record.exc_info
        log_record["exc_text"] = record.exc_text
        log_record["stack_info"] = record.stack_info
        log_record["node"] = record.module
        log_record["logger"] = record.name
        log_record["line"] = record.lineno
        log_record["function"] = record.funcName
        log_record["thread"] = record.thread
        log_record["file"] = record.filename
