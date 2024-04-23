from perception_tools.messages.field_result import FieldResult


class FieldRequestHandler:
    def has_request(self) -> bool:
        pass

    def send_response(self, field_result: FieldResult) -> None:
        pass
