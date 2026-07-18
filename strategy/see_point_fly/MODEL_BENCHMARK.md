# SPF VLM Model Record

Last updated: 2026-07-15

## Current Selection

SPF currently uses `gemini-3.5-flash` through the OpenAI-compatible API:

```yaml
api_provider: "openai"
model_name: "gemini-3.5-flash"
OPENAI_BASE_URL: "https://api.zhizengzeng.com/v1"
SPF_OPENAI_WIRE_API: "chat.completions"
```

The SDK appends `/chat/completions`, so the effective request endpoint is:

```text
https://api.zhizengzeng.com/v1/chat/completions
```

The API key is inherited from `/etc/gameuav/agent.env` and is intentionally not
stored in this document.

## Test Record

- Input: one RGB1 image, 640x480, from the SPF worker log
- Instruction: `fly to the left of the door`
- SPF mode: `adaptive_mode`
- Qualitative assessment: high recognition quality in the tested scene; keep as the preferred model for now
- Result: valid point/depth response and valid SPF action
- Model/API response time observed: approximately 5-6 seconds
- One end-to-end worker request (including local processing and logging): 14.8 seconds
- No ROS goal, EGO command, or flight-control command was published during the test

The approximately 5-second figure refers to the model-side response observed
through the intermediate API. End-to-end latency can be higher because of API
queueing, image encoding, SPF parsing, and log generation.

## Compatibility Note

`gemini-3.5-flash` sometimes omits the optional `label` field while returning a
valid `point` and `depth`. The SPF Tello action parser now treats `label` as
optional and continues using the returned spatial data.

## Gemini Comparison (2026-07-15)

All models below were tested through isolated SPF workers with the same 640x480
RGB1 image and the instruction `fly to the left of the door`. Times are
end-to-end worker request times for one trial and include image encoding,
inference, parsing, and SPF logging.

| Requested model | API model id | Result | Time | Returned point (px) |
|---|---|---:|---:|---:|
| Gemini 3.5 Flash | `gemini-3.5-flash` | success | 5.94 s | (321, 153) |
| Gemini 3.1 Pro | `gemini-3.1-pro-preview` | success | 20.08 s | (268, 216) |
| Gemini 2.5 Pro | `gemini-2.5-pro` | success | 31.68 s | (160, 336) |
| Gemini 2.5 Flash | `gemini-2.5-flash` | success | 20.05 s | (144, 228) |
| Gemini 2.5 Flash Lite | `gemini-2.5-flash-lite` | success | 15.48 s | (149, 112) |
| Gemini 2.0 Flash | `gemini-2.0-flash` | rejected | 1.63 s | -- |

The API rejected `gemini-2.0-flash` with: `model official deprecated; use a new
model`. Its result is not a valid latency measurement. The fastest successful
model in this single trial was `gemini-3.5-flash`; the returned points should
still be evaluated over the actual SPF task set before flight use.

### Gemini 2.5 Flash Repeat Test

The current production worker was tested three more times with the same image
and instruction. All requests returned valid SPF actions:

| Trial | Time | Returned point (px) |
|---:|---:|---:|
| 1 | 16.83 s | (144, 252) |
| 2 | 18.97 s | (176, 240) |
| 3 | 11.53 s | (144, 228) |

Mean time was 15.77 s (range 11.53-18.97 s). The point variation shows that
latency and spatial output should be measured over multiple frames before using
this model for a closed-loop flight task.
