import json
import logging


class CustomJsonFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:  # type: ignore[override]
        return json.dumps(
            {
                "time": record.created,
                "level": record.levelname,
                "exc_info": record.exc_info,
                "exc_text": record.exc_text,
                "stack_info": record.stack_info,
                "node": record.name,
                "logger": record.module,
                "line": record.lineno,
                "function": record.funcName,
                "thread": record.thread,
                "file": record.filename,
                "severity": record.levelname,
                "message": f"<-<{record.getMessage()}>->",
            }
        )
