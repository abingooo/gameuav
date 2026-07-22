"""Capture provider response bodies without retaining request credentials."""


def model_response_snapshot(response):
    status_code = int(getattr(response, "status_code", 0))
    try:
        body = response.json()
    except Exception:
        body = str(getattr(response, "text", ""))
    return {"status_code": status_code, "body": body}
