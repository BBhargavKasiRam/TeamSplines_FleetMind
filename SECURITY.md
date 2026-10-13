# Security Policy

## Supported Versions

The following versions of the Edge-AI Distributed Fleet Coordination stack are currently supported with critical security updates:

| Version | Supported | ROS 2 Distribution | Status |
| :--- | :--- | :--- | :--- |
| **1.0.x** | :white_check_mark: | ROS 2 Jazzy Jalisco | Active Development |
| < 1.0.0 | :x: | Legacy releases | Unsupported |

---

## Reporting a Vulnerability

The project team takes security vulnerabilities seriously, especially concerning autonomous cyber-physical robotics systems and inter-agent peer-to-peer networking.

If you discover a potential security vulnerability in any communication protocol, telemetry endpoint, or simulation node:

1. **Do not create a public GitHub issue.**
2. Send an email to the project maintainers at [sih_team@bel.gov.in](mailto:sih_team@bel.gov.in) with:
   * A detailed description of the vulnerability.
   * Specific components affected (e.g., P2P mesh topic injection, WebSocket gateway authentication, REST API parameter sanitization).
   * Step-by-step instructions or proof-of-concept scripts to reproduce the issue.
   * Assessment of the potential impact on autonomous fleet safety.

---

## Response & Disclosure Process

* **Acknowledgment:** Maintainers will acknowledge receipt of your vulnerability report within 48 hours.
* **Assessment & Fix:** The team will triage the issue, develop a patch in a private branch, and validate safety in simulation.
* **Release:** A security advisory and patched release will be published once the fix is verified.
