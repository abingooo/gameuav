import unittest

import numpy as np

from strategy.smpf.runtime.sam_client import SamClient, SamProtocolError


class _Response:
    def __init__(self, data, status_code=200, text=""):
        self._data = data
        self.status_code = status_code
        self.text = text

    def json(self):
        return self._data


class _Session:
    def __init__(self, response):
        self.response = response
        self.calls = []

    def post(self, url, json, timeout):
        self.calls.append((url, json, timeout))
        return self.response


class SmpfSamClientTest(unittest.TestCase):
    def test_default_endpoint_uses_deployed_sam_service(self):
        client = SamClient(session=_Session(_Response({"mask_count": 0, "masks": []})))
        self.assertEqual(client.endpoint, "http://10.246.1.94:5002/predict")

    def test_prediction_is_encoded_and_normalized(self):
        response = _Response(
            {
                "mask_count": 1,
                "masks": [
                    {
                        "area": 20,
                        "bounding_box": {"x1": 1, "y1": 2, "x2": 5, "y2": 7},
                        "centroid": [3, 4],
                        "random_points": [[2, 3], [4, 6]],
                    }
                ],
            }
        )
        session = _Session(response)
        prediction = SamClient(session=session).predict(np.zeros((8, 8, 3), dtype=np.uint8), "chair")
        self.assertEqual(prediction.mask_count, 1)
        self.assertEqual(prediction.best_mask.bbox_yxyx, (2.0, 1.0, 7.0, 5.0))
        self.assertEqual(prediction.best_mask.centroid_uv, (3.0, 4.0))
        self.assertEqual(session.calls[0][0], "http://10.246.1.94:5002/predict")
        self.assertEqual(session.calls[0][1]["text"], "chair")
        self.assertTrue(session.calls[0][1]["image"])

    def test_per_call_timeout_can_only_shorten_client_timeout(self):
        response = _Response({"mask_count": 0, "masks": []})
        session = _Session(response)
        client = SamClient(timeout_sec=20.0, session=session)
        client.predict(np.zeros((4, 4, 3), dtype=np.uint8), "person", timeout_sec=0.75)
        self.assertEqual(session.calls[-1][2], 0.75)
        client.predict(np.zeros((4, 4, 3), dtype=np.uint8), "person", timeout_sec=30.0)
        self.assertEqual(session.calls[-1][2], 20.0)

    def test_per_call_timeout_must_be_positive_and_finite(self):
        client = SamClient(session=_Session(_Response({"mask_count": 0, "masks": []})))
        for value in (0.0, -1.0, float("inf")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                client.predict(np.zeros((4, 4, 3), dtype=np.uint8), "person", timeout_sec=value)

    def test_empty_detection_is_valid(self):
        prediction = SamClient.parse_prediction({"mask_count": 0, "masks": []}, "chair")
        self.assertEqual(prediction.mask_count, 0)
        self.assertIsNone(prediction.best_mask)

    def test_count_mismatch_is_rejected(self):
        with self.assertRaises(SamProtocolError):
            SamClient.parse_prediction({"mask_count": 1, "masks": []})

    def test_malformed_mask_is_rejected(self):
        with self.assertRaises(SamProtocolError):
            SamClient.parse_prediction(
                {
                    "mask_count": 1,
                    "masks": [
                        {
                            "bounding_box": {"x1": 5, "y1": 2, "x2": 1, "y2": 7},
                            "centroid": [3, 4],
                            "random_points": [],
                        }
                    ],
                }
            )


if __name__ == "__main__":
    unittest.main()
