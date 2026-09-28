#!/usr/bin/env python3
"""Read-only MCB diagnostics over HTTP; never sends control requests.

Usage: mcb_diagnostics.py HOST [--port PORT] [--axis 0 1 2 3] [--json]
Only GET /dd_motor_diagnostics?axis=N is used. Default axes: 0, 1, 2, 3.
"""

import argparse
import http.client
import ipaddress
import json
import math
import re
import sys
import urllib.error
import urllib.request


MAX_BODY_BYTES = 4096
RR3_INTERPRETATION = "driver_polarity_and_swap_normalized"
ERROR_SOURCE_KEYS = (
    "axis_error", "alarm_input", "home_error", "interpolation_error",
    "emergency_input", "emergency_stop", "alarm_stop",
)
DRIVE_BITS = {
    "alarm": 4, "sw_limit_positive": 0, "sw_limit_negative": 1,
    "hw_limit_positive": 2, "hw_limit_negative": 3, "sync_stop": 8,
    "stop0_stop": 9, "stop1_stop": 10, "stop2_stop": 11,
    "limit_positive_stop": 12, "limit_negative_stop": 13,
}
SIGNAL_BITS = {
    "stop0": 0, "stop1": 1, "stop2": 2,
    "encoder_a": 3, "encoder_b": 4, "in_position": 5,
}
TOP_KEYS = {
    "schema_version", "axis_index", "axis_name", "axis_mask", "binary_status",
    "logical_position", "encoder_position", "speed_pps", "moving", "homing",
    "error", "positive_limit", "negative_limit", "home", "homing_ok",
    "rr0_raw", "rr2_raw", "rr3_decoded", "rr3_interpretation",
    "home_search_state", "error_sources", "drive_flags", "signal_inputs",
}


class DiagnosticsError(ValueError):
    """An HTTP response or diagnostics document cannot be trusted."""


def _keys(value, required, path):
    if type(value) is not dict:
        raise DiagnosticsError("{} must be an object".format(path))
    missing = set(required) - value.keys()
    extra = value.keys() - set(required)
    if missing or extra:
        raise DiagnosticsError("{} keys mismatch (missing: {}; unexpected: {})".format(
            path, ", ".join(sorted(missing)) or "none",
            ", ".join(sorted(extra)) or "none"))


def _integer(value, low, high, path):
    # bool is a subclass of int in Python, but is not a JSON integer here.
    if type(value) is not int or not low <= value <= high:
        raise DiagnosticsError("{} must be an integer in [{}, {}]".format(path, low, high))


def _boolean(value, path):
    if type(value) is not bool:
        raise DiagnosticsError("{} must be a boolean".format(path))


def _equal(actual, expected, path):
    if actual != expected:
        raise DiagnosticsError("{} is inconsistent (expected {!r})".format(path, expected))


def validate_diagnostics(data, expected_axis=None):
    """Validate the complete version-1 C++ schema and register-derived mappings.

    Return the unmodified document on success; raise DiagnosticsError otherwise.
    expected_axis, when supplied, also binds the reply to the requested axis.
    """
    _keys(data, TOP_KEYS, "diagnostics")
    _integer(data["schema_version"], 1, 1, "schema_version")
    _integer(data["axis_index"], 0, 3, "axis_index")
    axis = data["axis_index"]
    if expected_axis is not None:
        _integer(expected_axis, 0, 3, "expected_axis")
        _equal(axis, expected_axis, "axis_index")
    if type(data["axis_name"]) is not str:
        raise DiagnosticsError("axis_name must be a string")
    _equal(data["axis_name"], "XYZU"[axis], "axis_name")
    _integer(data["axis_mask"], 1, 8, "axis_mask")
    _equal(data["axis_mask"], 1 << axis, "axis_mask")
    _integer(data["binary_status"], 0, 255, "binary_status")
    for key in ("logical_position", "encoder_position"):
        _integer(data[key], -(1 << 31), (1 << 31) - 1, key)
    _integer(data["speed_pps"], 0, (1 << 32) - 1, "speed_pps")
    for key in ("rr0_raw", "rr2_raw", "rr3_decoded"):
        _integer(data[key], 0, 65535, key)
    _integer(data["home_search_state"], 0, 63, "home_search_state")
    for key in ("moving", "homing", "error", "positive_limit", "negative_limit"):
        _boolean(data[key], key)
    for key in ("home", "homing_ok"):
        if data[key] is not None:
            raise DiagnosticsError("{} must be null (not reported by this schema)".format(key))
    if type(data["rr3_interpretation"]) is not str:
        raise DiagnosticsError("rr3_interpretation must be a string")
    _equal(data["rr3_interpretation"], RR3_INTERPRETATION, "rr3_interpretation")
    for group, keys in (("error_sources", ERROR_SOURCE_KEYS),
                        ("drive_flags", DRIVE_BITS), ("signal_inputs", SIGNAL_BITS)):
        _keys(data[group], keys, group)
        for key in keys:
            _boolean(data[group][key], group + "." + key)

    rr0, rr2, rr3 = data["rr0_raw"], data["rr2_raw"], data["rr3_decoded"]
    bit = lambda register, index: bool(register & (1 << index))
    expected_sources = {
        "axis_error": bit(rr0, axis + 4), "alarm_input": bit(rr3, 6),
        "home_error": bit(rr2, 6), "interpolation_error": bit(rr2, 7),
        "emergency_input": bit(rr2, 5), "emergency_stop": bit(rr2, 15),
        "alarm_stop": bit(rr2, 14),
    }
    for key, value in expected_sources.items():
        _equal(data["error_sources"][key], value, "error_sources." + key)
    _equal(data["error"], any(expected_sources.values()), "error")
    _equal(data["moving"], bit(rr0, axis), "moving")
    _equal(data["home_search_state"], (rr3 >> 9) & 63, "home_search_state")
    _equal(data["homing"], data["home_search_state"] != 0, "homing")
    _equal(data["positive_limit"], bit(rr3, 7), "positive_limit")
    _equal(data["negative_limit"], bit(rr3, 8), "negative_limit")
    for group, register, mapping in (("drive_flags", rr2, DRIVE_BITS),
                                     ("signal_inputs", rr3, SIGNAL_BITS)):
        for key, index in mapping.items():
            _equal(data[group][key], bit(register, index), group + "." + key)
    status = ((0x40 if data["homing"] else 0) | (0x20 if data["moving"] else 0)
              | (0x10 if not data["moving"] and not data["homing"] else 0)
              | (0x08 if data["error"] else 0)
              | (0x02 if data["positive_limit"] else 0)
              | (0x01 if data["negative_limit"] else 0))
    _equal(data["binary_status"], status, "binary_status")
    return data


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise DiagnosticsError("duplicate JSON key: " + key)
        result[key] = value
    return result


def _reject_constant(value):
    raise DiagnosticsError("non-finite JSON number: " + value)


def parse_diagnostics(body, expected_axis=None):
    """Parse bounded UTF-8 JSON, rejecting duplicate keys and non-JSON numbers."""
    if len(body) > MAX_BODY_BYTES:
        raise DiagnosticsError("response exceeds {} bytes".format(MAX_BODY_BYTES))
    try:
        data = json.loads(body.decode("utf-8"), object_pairs_hook=_unique_object,
                          parse_constant=_reject_constant)
    except (UnicodeError, json.JSONDecodeError, RecursionError, ValueError) as exc:
        raise DiagnosticsError("malformed diagnostics JSON: {}".format(exc)) from exc
    return validate_diagnostics(data, expected_axis)


class _NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, request, response, code, message, headers, new_url):
        # Returning None makes urllib raise HTTPError, without a second request.
        return None


def _host_argument(value):
    host = value[1:-1] if value.startswith("[") and value.endswith("]") else value
    # ipaddress permits arbitrary IPv6 scope text; do not let it become URL syntax.
    if any(character in host for character in "%/?#@\\") or any(character.isspace() for character in host):
        raise argparse.ArgumentTypeError("host must be a plain hostname or unscoped IP address")
    try:
        address = ipaddress.ip_address(host)
        return "[{}]".format(address) if address.version == 6 else str(address)
    except ValueError:
        pass
    if (len(host) > 253 or not host or not all(
            re.fullmatch(r"[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?", label)
            for label in host.rstrip(".").split("."))):
        raise argparse.ArgumentTypeError("host must be a hostname or IP address, without a URL or port; use --port")
    return host


def _timeout_argument(value):
    try:
        timeout = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("timeout must be a positive finite number") from exc
    if not math.isfinite(timeout) or timeout <= 0:
        raise argparse.ArgumentTypeError("timeout must be a positive finite number")
    return timeout


def _port_argument(value):
    try:
        port = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("port must be an integer in [1, 65535]") from exc
    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError("port must be an integer in [1, 65535]")
    return port


def fetch_diagnostics(host, port, axis, timeout):
    """Fetch one bounded response directly, without proxies or redirects."""
    # Apply the same constraints to library callers as to CLI arguments.
    host = _host_argument(host)
    port = _port_argument(str(port))
    timeout = _timeout_argument(str(timeout))
    _integer(axis, 0, 3, "axis")
    url = "http://{}:{}/dd_motor_diagnostics?axis={}".format(host, port, axis)
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), _NoRedirect())
    request = urllib.request.Request(url, headers={"Accept": "application/json"}, method="GET")
    try:
        with opener.open(request, timeout=timeout) as response:
            if response.status != 200:
                raise DiagnosticsError("HTTP {} (expected 200)".format(response.status))
            content_types = response.headers.get_all("Content-Type", [])
            if len(content_types) != 1 or not re.fullmatch(
                    r'application/json(?:\s*;\s*charset\s*=\s*(?:utf-8|"utf-8"))?',
                    content_types[0].strip(), re.IGNORECASE):
                raise DiagnosticsError("Content-Type must be application/json with optional UTF-8 charset")
            encodings = response.headers.get_all("Content-Encoding", [])
            if encodings and encodings != ["identity"]:
                raise DiagnosticsError("encoded response bodies are not supported")
            lengths = response.headers.get_all("Content-Length", [])
            if lengths:
                if len(lengths) != 1 or not re.fullmatch(r"[0-9]+", lengths[0].strip()):
                    raise DiagnosticsError("invalid Content-Length")
                if int(lengths[0]) > MAX_BODY_BYTES:
                    raise DiagnosticsError("response exceeds {} bytes".format(MAX_BODY_BYTES))
            body = response.read(MAX_BODY_BYTES + 1)
            if lengths and len(body) != int(lengths[0]):
                raise DiagnosticsError("truncated response body")
            return parse_diagnostics(body, axis)
    except urllib.error.HTTPError as exc:
        exc.close()
        if 300 <= exc.code < 400:
            raise DiagnosticsError("HTTP {} redirect refused".format(exc.code)) from exc
        raise DiagnosticsError("HTTP {} {}".format(exc.code, exc.reason)) from exc
    except (urllib.error.URLError, OSError, http.client.HTTPException, ValueError, OverflowError) as exc:
        if isinstance(exc, DiagnosticsError):
            raise
        raise DiagnosticsError("HTTP/network failure: {}".format(exc)) from exc


def format_diagnostics(data):
    active = [key for key in ERROR_SOURCE_KEYS if data["error_sources"][key]]
    return "\n".join((
        "Axis {axis_index} ({axis_name}): status=0x{binary_status:02X}".format(**data),
        "  logical_position={logical_position} encoder_position={encoder_position} speed_pps={speed_pps}".format(**data),
        "  moving={moving} homing={homing} positive_limit={positive_limit} negative_limit={negative_limit}".format(**data),
        "  home=unknown homing_ok=unknown home_search_state={home_search_state}".format(**data),
        "  active errors: " + (", ".join(active) if active else "none reported"),
        "  RR0=0x{rr0_raw:04X} RR2=0x{rr2_raw:04X} rr3_decoded=0x{rr3_decoded:04X}".format(**data),
        "  rr3 interpretation: " + RR3_INTERPRETATION + " (not a raw wire register)",
    ))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host", type=_host_argument, help="board hostname or IP address (no URL)")
    parser.add_argument("--port", type=_port_argument, default=80, help="HTTP port (default: 80)")
    parser.add_argument("--timeout", type=_timeout_argument, default=3.0,
                        help="positive finite HTTP socket timeout in seconds (default: 3)")
    parser.add_argument("--axis", nargs="+", type=int, choices=range(4), default=list(range(4)),
                        help="axes to read in the given order (default: 0 1 2 3)")
    parser.add_argument("--json", action="store_true", help="print full validated documents as a JSON array")
    args = parser.parse_args(argv)
    if len(set(args.axis)) != len(args.axis):
        parser.error("--axis must not contain duplicates")
    try:
        documents = []
        for axis in args.axis:
            try:
                documents.append(fetch_diagnostics(args.host, args.port, axis, args.timeout))
            except DiagnosticsError as exc:
                raise DiagnosticsError("axis {}: {}".format(axis, exc)) from exc
    except DiagnosticsError as exc:
        print("mcb_diagnostics: {}".format(exc), file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps(documents, indent=2, allow_nan=False))
    else:
        print("\n\n".join(format_diagnostics(data) for data in documents))
        print("\nRead-only diagnostics; this snapshot is not a motion-safety assessment.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
