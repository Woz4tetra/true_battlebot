import json
import logging


class CustomJsonFormatter(logging.Formatter):
    def format(self, record: logging.LogRecord) -> str:
        return json.dumps(
            {
                "time": float(record.created),
                "exc_info": str(record.exc_info),
                "exc_text": str(record.exc_text),
                "stack_info": str(record.stack_info),
                "node": str(record.name),
                "logger": str(record.module),
                "line": int(record.lineno),
                "function": str(record.funcName),
                "thread": str(record.thread),
                "file": str(record.filename),
                "severity": str(record.levelname),
                "message": f"<-<{record.getMessage()}>->",
            }
        )
