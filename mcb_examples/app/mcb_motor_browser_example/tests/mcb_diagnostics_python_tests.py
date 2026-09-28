#!/usr/bin/env python3
"""Local-only stdlib tests for the read-only diagnostics client.

Run: python3 tests/mcb_diagnostics_python_tests.py [DIAGNOSTICS_TEST_EXE] -v
The optional C++ executable is invoked only with --json-fixtures; its four
serialized JSON lines are validated without accessing hardware.
"""

import contextlib
import copy
import importlib.util
import io
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from unittest import mock


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "tools" / "mcb_diagnostics.py"
SPEC = importlib.util.spec_from_file_location("mcb_diagnostics", SCRIPT_PATH)
diagnostics = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(diagnostics)
FIXTURE_EXECUTABLE = None


def sample(axis=0, rr0=0, rr2=0, rr3=0):
    """Build an independent fake document from the C++ register schema."""
    bit = lambda value, index: bool(value & (1 << index))
    sources = {
        "axis_error": bit(rr0, axis + 4), "alarm_input": bit(rr3, 6),
        "home_error": bit(rr2, 6), "interpolation_error": bit(rr2, 7),
        "emergency_input": bit(rr2, 5), "emergency_stop": bit(rr2, 15),
        "alarm_stop": bit(rr2, 14),
    }
    moving = bit(rr0, axis)
    homing = bool((rr3 >> 9) & 63)
    positive_limit, negative_limit = bit(rr3, 7), bit(rr3, 8)
    error = any(sources.values())
    status = ((0x40 if homing else 0) | (0x20 if moving else 0)
              | (0x10 if not moving and not homing else 0) | (0x08 if error else 0)
              | (0x02 if positive_limit else 0) | (0x01 if negative_limit else 0))
    return {
        "schema_version": 1, "axis_index": axis, "axis_name": "XYZU"[axis],
        "axis_mask": 1 << axis, "binary_status": status,
        "logical_position": -(1 << 31), "encoder_position": (1 << 31) - 1,
        "speed_pps": (1 << 32) - 1, "moving": moving, "homing": homing,
        "error": error, "positive_limit": positive_limit, "negative_limit": negative_limit,
        "home": None, "homing_ok": None, "rr0_raw": rr0, "rr2_raw": rr2,
        "rr3_decoded": rr3,
        "rr3_interpretation": "driver_polarity_and_swap_normalized",
        "home_search_state": (rr3 >> 9) & 63, "error_sources": sources,
        "drive_flags": {key: bit(rr2, index) for key, index in (
            ("alarm", 4), ("sw_limit_positive", 0), ("sw_limit_negative", 1),
            ("hw_limit_positive", 2), ("hw_limit_negative", 3), ("sync_stop", 8),
            ("stop0_stop", 9), ("stop1_stop", 10), ("stop2_stop", 11),
            ("limit_positive_stop", 12), ("limit_negative_stop", 13))},
        "signal_inputs": {key: bit(rr3, index) for key, index in (
            ("stop0", 0), ("stop1", 1), ("stop2", 2),
            ("encoder_a", 3), ("encoder_b", 4), ("in_position", 5))},
    }


class FakeServer(ThreadingHTTPServer):
    daemon_threads = True
    block_on_close = False

    def __init__(self):
        super().__init__(("127.0.0.1", 0), FakeHandler)
        self.requests = []
        self.response = None
        self.port = self.server_address[1]


class FakeHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *args):
        pass

    def do_GET(self):
        self.server.requests.append((self.command, self.path))
        config = self.server.response or {}
        delay = config.get("delay", 0)
        if delay:
            time.sleep(delay)
        body = config.get("body")
        if body is None:
            axis = int(self.path[-1])
            body = json.dumps(sample(axis, rr0=65535, rr2=65535, rr3=65535)).encode()
        self.send_response(config.get("status", 200))
        headers = config.get("headers", [("Content-Type", "application/json")])
        for key, value in headers:
            self.send_header(key, value)
        if config.get("chunked"):
            self.send_header("Transfer-Encoding", "chunked")
        elif not config.get("no_length") and not any(key.lower() == "content-length" for key, _ in headers):
            self.send_header("Content-Length", str(len(body)))
        self.send_header("Connection", "close")
        try:
            self.end_headers()
            if config.get("chunked"):
                self.wfile.write(("{:X}\r\n".format(len(body))).encode() + body + b"\r\n0\r\n\r\n")
            else:
                self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass
        self.close_connection = True

    def _unexpected_method(self):
        self.server.requests.append((self.command, self.path))
        self.send_response(405)
        self.send_header("Content-Length", "0")
        self.end_headers()

    do_POST = do_PUT = do_PATCH = do_DELETE = do_HEAD = do_OPTIONS = _unexpected_method


@contextlib.contextmanager
def fake_server():
    server = FakeServer()
    thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.01}, daemon=True)
    thread.start()
    try:
        yield server
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=1)


def run_cli(server, *extra):
    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), "127.0.0.1", "--port", str(server.port)] + list(extra),
        capture_output=True, text=True, timeout=4,
    )
    return result


class SchemaTests(unittest.TestCase):
    def test_all_axes_positions_extremes_and_register_extremes(self):
        for axis in range(4):
            for registers in (0, 65535):
                for logical, encoder in ((-(1 << 31), (1 << 31) - 1),
                                         ((1 << 31) - 1, -(1 << 31))):
                    with self.subTest(axis=axis, registers=registers, positions=(logical, encoder)):
                        data = sample(axis, registers, registers, registers)
                        data.update(logical_position=logical, encoder_position=encoder)
                        self.assertIs(diagnostics.validate_diagnostics(data, axis), data)

    def test_exact_json_types_for_every_field(self):
        base = sample()
        for key, value in base.items():
            if type(value) is int:
                replacements = (True, 1.0, str(value), None)
            elif type(value) is bool:
                replacements = (0, 1, "false", None)
            elif value is None:
                replacements = (False, 0, "null")
            elif type(value) is dict:
                replacements = (None, [], "{}")
            else:
                replacements = (None, 1, [])
            for replacement in replacements:
                with self.subTest(key=key, replacement=replacement):
                    data = copy.deepcopy(base)
                    data[key] = replacement
                    with self.assertRaises(diagnostics.DiagnosticsError):
                        diagnostics.validate_diagnostics(data)
        for group in ("error_sources", "drive_flags", "signal_inputs"):
            for key in base[group]:
                for replacement in (0, 1, None, "false"):
                    with self.subTest(group=group, key=key, replacement=replacement):
                        data = copy.deepcopy(base)
                        data[group][key] = replacement
                        with self.assertRaises(diagnostics.DiagnosticsError):
                            diagnostics.validate_diagnostics(data)

    def test_version_missing_and_extra_keys(self):
        for version in (0, 2, -1, True, "1"):
            with self.subTest(version=version):
                data = sample()
                data["schema_version"] = version
                with self.assertRaises(diagnostics.DiagnosticsError):
                    diagnostics.validate_diagnostics(data)
        for group in (None, "error_sources", "drive_flags", "signal_inputs"):
            original = sample() if group is None else sample()[group]
            for key in original:
                with self.subTest(group=group, missing=key):
                    data = sample()
                    obj = data if group is None else data[group]
                    del obj[key]
                    with self.assertRaises(diagnostics.DiagnosticsError):
                        diagnostics.validate_diagnostics(data)
            data = sample()
            (data if group is None else data[group])["unexpected"] = False
            with self.assertRaises(diagnostics.DiagnosticsError):
                diagnostics.validate_diagnostics(data)

    def test_integer_ranges(self):
        ranges = {
            "axis_index": (0, 3), "axis_mask": (1, 8), "binary_status": (0, 255),
            "logical_position": (-(1 << 31), (1 << 31) - 1),
            "encoder_position": (-(1 << 31), (1 << 31) - 1),
            "speed_pps": (0, (1 << 32) - 1), "rr0_raw": (0, 65535),
            "rr2_raw": (0, 65535), "rr3_decoded": (0, 65535), "home_search_state": (0, 63),
        }
        for key, (low, high) in ranges.items():
            for value in (low - 1, high + 1):
                with self.subTest(key=key, value=value):
                    data = sample()
                    data[key] = value
                    with self.assertRaises(diagnostics.DiagnosticsError):
                        diagnostics.validate_diagnostics(data)

    def test_axis_identity_and_rr3_label(self):
        for key, value in (("axis_name", "U"), ("axis_mask", 8),
                           ("rr3_interpretation", "raw")):
            with self.subTest(key=key):
                data = sample()
                data[key] = value
                with self.assertRaises(diagnostics.DiagnosticsError):
                    diagnostics.validate_diagnostics(data)
        with self.assertRaises(diagnostics.DiagnosticsError):
            diagnostics.validate_diagnostics(sample(1), expected_axis=0)

    def test_every_register_bit_and_coherence(self):
        for axis in range(4):
            for register in ("rr0_raw", "rr2_raw", "rr3_decoded"):
                for index in range(16):
                    registers = {"rr0_raw": 0, "rr2_raw": 0, "rr3_decoded": 0}
                    registers[register] = 1 << index
                    with self.subTest(axis=axis, register=register, bit=index):
                        data = sample(axis, registers["rr0_raw"], registers["rr2_raw"], registers["rr3_decoded"])
                        diagnostics.validate_diagnostics(data, axis)
        base = sample(3, 65535, 65535, 65535)
        for key in ("moving", "homing", "error", "positive_limit", "negative_limit"):
            data = copy.deepcopy(base)
            data[key] = not data[key]
            with self.subTest(field=key), self.assertRaises(diagnostics.DiagnosticsError):
                diagnostics.validate_diagnostics(data)
        for key in ("binary_status", "home_search_state"):
            data = copy.deepcopy(base)
            data[key] ^= 1
            with self.subTest(field=key), self.assertRaises(diagnostics.DiagnosticsError):
                diagnostics.validate_diagnostics(data)
        for group in ("error_sources", "drive_flags", "signal_inputs"):
            for key in base[group]:
                data = copy.deepcopy(base)
                data[group][key] = not data[group][key]
                with self.subTest(group=group, key=key), self.assertRaises(diagnostics.DiagnosticsError):
                    diagnostics.validate_diagnostics(data)

    def test_each_error_source_maps_to_error_and_status(self):
        registers = {"axis_error": (1 << 4, 0, 0), "alarm_input": (0, 0, 1 << 6),
                     "home_error": (0, 1 << 6, 0), "interpolation_error": (0, 1 << 7, 0),
                     "emergency_input": (0, 1 << 5, 0), "emergency_stop": (0, 1 << 15, 0),
                     "alarm_stop": (0, 1 << 14, 0)}
        for key, raw in registers.items():
            with self.subTest(source=key):
                data = sample(0, *raw)
                diagnostics.validate_diagnostics(data)
                self.assertTrue(data["error"])
                self.assertEqual(data["binary_status"], 0x18)
                self.assertEqual([name for name, active in data["error_sources"].items() if active], [key])
                self.assertIn("active errors: " + key, diagnostics.format_diagnostics(data))

    def test_malformed_json(self):
        for body in (b"", b"{", b"[]", b"null", b'"text"', b'\xff',
                     b'{"schema_version":1,"schema_version":1}', b'{"x":NaN}',
                     b'{"x":Infinity}', b'{"x":-Infinity}', b'{"x":1} trailing',
                     b"[" * 2000 + b"]" * 2000):
            with self.subTest(body=body[:60]), self.assertRaises(diagnostics.DiagnosticsError):
                diagnostics.parse_diagnostics(body)

    def test_body_parser_cap(self):
        body = json.dumps(sample()).encode()
        at_cap = body + b" " * (diagnostics.MAX_BODY_BYTES - len(body))
        self.assertEqual(diagnostics.parse_diagnostics(at_cap), sample())
        with self.assertRaises(diagnostics.DiagnosticsError):
            diagnostics.parse_diagnostics(at_cap + b" ")


class HTTPTests(unittest.TestCase):
    def test_default_all_axes_sequential_get_only_full_json(self):
        with fake_server() as server:
            result = run_cli(server, "--json")
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stderr, "")
            documents = json.loads(result.stdout)
            self.assertEqual(documents, [sample(axis, 65535, 65535, 65535) for axis in range(4)])
            self.assertEqual(server.requests, [("GET", "/dd_motor_diagnostics?axis={}".format(axis)) for axis in range(4)])

    def test_selected_axes_text_and_normalized_label(self):
        with fake_server() as server:
            result = run_cli(server, "--axis", "3", "1")
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(server.requests, [("GET", "/dd_motor_diagnostics?axis=3"),
                                               ("GET", "/dd_motor_diagnostics?axis=1")])
            for value in ("Axis 3 (U)", "status=0x6B", "RR0=0xFFFF", "RR2=0xFFFF",
                          "rr3_decoded=0xFFFF", "driver_polarity_and_swap_normalized",
                          "active errors: axis_error, alarm_input", "not a motion-safety assessment"):
                self.assertIn(value, result.stdout)

    def test_malformed_version_type_and_axis_reply_fail_stderr(self):
        cases = [b'{"broken":', json.dumps(dict(sample(), schema_version=2)).encode(),
                 json.dumps(dict(sample(), logical_position=True)).encode(),
                 json.dumps(sample(1)).encode()]
        with fake_server() as server:
            for body in cases:
                with self.subTest(body=body[:60]):
                    server.response = {"body": body}
                    result = run_cli(server, "--axis", "0", "--json")
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn("mcb_diagnostics: axis 0:", result.stderr)
                    self.assertNotIn("Traceback", result.stderr)
                    self.assertEqual(result.stdout, "")

    def test_http_unavailable(self):
        with fake_server() as server:
            server.response = {"status": 503, "body": b'{"error":"diagnostics_unavailable"}'}
            result = run_cli(server, "--axis", "0")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("HTTP 503", result.stderr)
            self.assertEqual(result.stdout, "")

    def test_network_unavailable(self):
        # Keep a local TCP socket bound but not listening, ensuring no remote I/O.
        with socket.socket() as bound:
            bound.bind(("127.0.0.1", 0))
            result = subprocess.run(
                [sys.executable, str(SCRIPT_PATH), "127.0.0.1", "--port",
                 str(bound.getsockname()[1]), "--axis", "0", "--timeout", "0.2"],
                capture_output=True, text=True, timeout=2)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("HTTP/network failure", result.stderr)
            self.assertNotIn("Traceback", result.stderr)
            self.assertEqual(result.stdout, "")

    def test_redirects_refused_without_followup_request(self):
        with fake_server() as server:
            for code in (301, 302, 303, 307, 308):
                with self.subTest(status=code):
                    server.requests.clear()
                    server.response = {"status": code, "body": b"", "headers": [
                        ("Location", "http://127.0.0.1:{}/control".format(server.port))]}
                    with self.assertRaisesRegex(diagnostics.DiagnosticsError, "redirect refused"):
                        diagnostics.fetch_diagnostics("127.0.0.1", server.port, 0, 0.5)
                    self.assertEqual(server.requests, [("GET", "/dd_motor_diagnostics?axis=0")])
            result = run_cli(server, "--axis", "0")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("redirect refused", result.stderr)
            self.assertEqual(result.stdout, "")

    def test_content_type_and_encoding(self):
        bad_headers = [[], [("Content-Type", "text/plain")],
                       [("Content-Type", "application/jsonp")],
                       [("Content-Type", "application/json; charset=latin1")],
                       [("Content-Type", "application/json; unexpected=true")],
                       [("Content-Type", "application/json"), ("Content-Type", "text/plain")],
                       [("Content-Type", "application/json"), ("Content-Encoding", "gzip")]]
        with fake_server() as server:
            for headers in bad_headers:
                with self.subTest(headers=headers):
                    server.response = {"headers": headers}
                    with self.assertRaises(diagnostics.DiagnosticsError):
                        diagnostics.fetch_diagnostics("127.0.0.1", server.port, 0, 0.5)
            for content_type in ("application/json", "application/json; charset=utf-8",
                                 'Application/JSON; Charset="UTF-8"'):
                server.response = {"headers": [("Content-Type", content_type)]}
                diagnostics.fetch_diagnostics("127.0.0.1", server.port, 0, 0.5)

    def test_bounded_body_at_cap_and_oversize_in_all_framings(self):
        body = json.dumps(sample()).encode()
        body += b" " * (diagnostics.MAX_BODY_BYTES - len(body))
        with fake_server() as server:
            for framing in ({}, {"no_length": True}, {"chunked": True}):
                with self.subTest(framing=framing):
                    server.response = dict(framing, body=body)
                    self.assertEqual(diagnostics.fetch_diagnostics("127.0.0.1", server.port, 0, 0.5), sample())
                    server.response = dict(framing, body=body + b" " * 8192)
                    with self.assertRaisesRegex(diagnostics.DiagnosticsError, "exceeds"):
                        diagnostics.fetch_diagnostics("127.0.0.1", server.port, 0, 0.5)
            result = run_cli(server, "--axis", "0")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("exceeds", result.stderr)
            self.assertEqual(result.stdout, "")

    def test_invalid_and_truncated_content_length(self):
        body = json.dumps(sample()).encode()
        with fake_server() as server:
            for lengths in (("bad",), ("-1",), (str(len(body)), str(len(body))),
                            (str(len(body) + 100),)):
                with self.subTest(lengths=lengths):
                    headers = [("Content-Type", "application/json")]
                    headers += [("Content-Length", length) for length in lengths]
                    server.response = {"headers": headers, "body": body}
                    with self.assertRaises(diagnostics.DiagnosticsError):
                        diagnostics.fetch_diagnostics("127.0.0.1", server.port, 0, 0.5)

    def test_reads_at_most_cap_plus_one_and_closes_response(self):
        response = mock.MagicMock()
        response.status = 200
        response.headers.get_all.side_effect = lambda key, default: (["application/json"] if key == "Content-Type" else default)
        response.read.return_value = b" " * (diagnostics.MAX_BODY_BYTES + 1)
        response.__enter__.return_value = response
        opener = mock.Mock()
        opener.open.return_value = response
        with mock.patch.object(diagnostics.urllib.request, "build_opener", return_value=opener):
            with self.assertRaisesRegex(diagnostics.DiagnosticsError, "exceeds"):
                diagnostics.fetch_diagnostics("127.0.0.1", 80, 0, 0.5)
        response.read.assert_called_once_with(diagnostics.MAX_BODY_BYTES + 1)
        response.__exit__.assert_called_once()

    def test_timeout(self):
        with fake_server() as server:
            server.response = {"delay": 0.3}
            started = time.monotonic()
            result = run_cli(server, "--axis", "0", "--timeout", "0.05")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("timed out", result.stderr)
            self.assertNotIn("Traceback", result.stderr)
            self.assertEqual(result.stdout, "")
            self.assertLess(time.monotonic() - started, 1.5)

    def test_finite_but_unrepresentable_socket_timeout_fails_cleanly(self):
        with fake_server() as server:
            result = run_cli(server, "--axis", "0", "--timeout", "1e300")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("HTTP/network failure", result.stderr)
            self.assertNotIn("Traceback", result.stderr)
            self.assertEqual(result.stdout, "")
            self.assertEqual(server.requests, [])

    def test_external_proxy_environment_is_ignored(self):
        with fake_server() as server, fake_server() as proxy:
            proxy_url = "http://127.0.0.1:{}".format(proxy.port)
            environment = {"http_proxy": proxy_url, "HTTP_PROXY": proxy_url,
                           "https_proxy": proxy_url, "HTTPS_PROXY": proxy_url,
                           "all_proxy": proxy_url, "ALL_PROXY": proxy_url,
                           "no_proxy": "", "NO_PROXY": ""}
            with mock.patch.dict(os.environ, environment):
                result = run_cli(server, "--axis", "0", "--json")
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(server.requests, [("GET", "/dd_motor_diagnostics?axis=0")])
            self.assertEqual(proxy.requests, [])


class ArgumentTests(unittest.TestCase):
    def test_invalid_arguments_never_send_requests(self):
        cases = [("--timeout", "0"), ("--timeout", "-1"), ("--timeout", "nan"),
                 ("--timeout", "inf"), ("--timeout", "-inf"), ("--timeout", "bad"),
                 ("--port", "0"), ("--port", "65536"), ("--port", "bad"),
                 ("--axis", "4"), ("--axis", "-1"), ("--axis", "true"),
                 ("--axis", "0", "0")]
        with fake_server() as server:
            for args in cases:
                with self.subTest(args=args):
                    out, err = io.StringIO(), io.StringIO()
                    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
                        with self.assertRaises(SystemExit) as raised:
                            diagnostics.main(["127.0.0.1", "--port", str(server.port)] + list(args))
                    self.assertEqual(raised.exception.code, 2)
                    self.assertTrue(err.getvalue())
            self.assertEqual(server.requests, [])

    def test_hosts_are_not_urls_or_control_paths(self):
        for host in ("http://127.0.0.1", "127.0.0.1:80", "user@localhost", "localhost/path",
                     "localhost?axis=0", "localhost#fragment", "localhost\n", "", "-bad", "foo..bar",
                     "::1%lo?control=1", "::1%lo#control", "::1%lo@localhost", "::1%lo\n"):
            with self.subTest(host=host), self.assertRaises(diagnostics.argparse.ArgumentTypeError):
                diagnostics._host_argument(host)
        for host, expected in (("localhost", "localhost"), ("127.0.0.1", "127.0.0.1"),
                               ("::1", "[::1]"), ("[::1]", "[::1]")):
            self.assertEqual(diagnostics._host_argument(host), expected)


class CppFixtureTests(unittest.TestCase):
    def test_actual_cpp_serialized_json_fixtures(self):
        if FIXTURE_EXECUTABLE is None:
            self.skipTest("pass the diagnostics C++ test executable as the first argument")
        result = subprocess.run([FIXTURE_EXECUTABLE, "--json-fixtures"],
                                capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr.decode(errors="replace"))
        self.assertEqual(result.stderr, b"")
        lines = result.stdout.splitlines()
        self.assertEqual(len(lines), 4, "fixture command must emit exactly four JSON lines")
        documents = [diagnostics.parse_diagnostics(line, axis) for axis, line in enumerate(lines)]
        positions = {doc[key] for doc in documents for key in ("logical_position", "encoder_position")}
        self.assertIn(-(1 << 31), positions)
        self.assertIn((1 << 31) - 1, positions)
        self.assertIn((1 << 32) - 1, {doc["speed_pps"] for doc in documents})
        self.assertTrue(any(doc["rr0_raw"] == 65535 or doc["rr2_raw"] == 65535
                            or doc["rr3_decoded"] == 65535 for doc in documents))
        # Exercise the real serialized bytes through the HTTP client, too.
        with fake_server() as server:
            for axis, body in enumerate(lines):
                server.response = {"body": body}
                self.assertEqual(diagnostics.fetch_diagnostics("127.0.0.1", server.port, axis, 0.5), documents[axis])


if __name__ == "__main__":
    if len(sys.argv) > 1 and not sys.argv[1].startswith("-"):
        FIXTURE_EXECUTABLE = str(Path(sys.argv.pop(1)).resolve())
    unittest.main()
